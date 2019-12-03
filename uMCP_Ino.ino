// Lightweight communication protocol with guaranteed delivery 
// Implementation for Arduino
//
// uMCP - a free view on the DDCMP protocol, especially designed to work with small baudrates and channels with high error probability - for example,
// underwater acoustic channel etc. 
// uMCP is designed for better usability of uWAVE and RedLINE underwater acoustic modems (but not only!).
//
// It is very similar to DDCMP, but has some differencies:
//
//    smaller overhead
//    possible communication between tributary stations
//    no NAKs, all error situations are managed by timeouts to decrease traffic
//    easy pipelining
//
// timeouts are causing transmission of REP (reply to message), remote system answers with ACK (which, in fact, contains remote system state)
//
//    if error is due to lost ACK, it will be corrected by ACK retransmission from the remote system
//    if error is due to lost DATA message, it will be corrected by DATA message retransmission to the remote system Retransmission performed without pipelining.
//
// Message framing
// 
// SIGN  : 8 bit (start signature) = 0xAD SID 
// SID   : 8 bit (sender ID)
// TID   : 8 bit (target ID) 
// PTYPE : 8 bit (packet type)
// 
// if PTYPE == STR || PTYPE == STA
// 
//     no extra data
// 
// else if PTYPE == REP 
//
// TCNT : 8 bit (TX Counter)
// 
// else if PTYPE == ACK
//
// TCNT : 8 bit (TX Counter)
// RCNT : 8 bit (RX Counter)
// 
// else if PTYPE = DTA || PTYPE == DTE
// 
// TCNT : 8 bit (TX Counter)
// RCNT : 8 bit (RX Counter)
// 
// endif
//
// HCHK : 8 bit (header checksum)
//
// if PTYPE == DTA || PTYPE == DTE 
//
// DCNT : 8 bit (data counter)
// DATA : 8 bit * DCNT
//
// endif


#define HAS_SERIAL1

#ifndef HAS_SERIAL1
#include <SoftwareSerial.h>
#define SOFTWARE_RX_PIN (2)
#define SOFTWARE_TX_PIN (3)
SoftwareSerial linePort(SOFTWARE_RX_PIN, SOFTWARE_TX_PIN);
#endif

/// *************************************************** uMCP_PacketType
typedef enum
{
  uMCP_PTYPE_ACK = (1 | 32),      // Acknowledgement
  uMCP_PTYPE_REP = (2 | 32),      // Reply to message number
  uMCP_PTYPE_STA = (4 | 32),      // Start acknowledged
  uMCP_PTYPE_STR = (8 | 32),      // Start
  uMCP_PTYPE_DTA = (1 | 16),      // Data
  uMCP_PTYPE_DTE = (1 | 16 | 32), // Data, with SELECT flag transmitted
  uMCP_PTYPE_INVALID
} uMCP_PacketType;

// uMCP_State
typedef enum
{
  uMCP_STATE_HALTED,  // Protocol halted
  uMCP_STATE_ISTART,  // Initiated start
  uMCP_STATE_ASTART,  // Acknowledged start
  uMCP_STATE_RUNNING, // Running
  uMCP_STATE_INVALID

} uMCP_State;

// uMCP_Timer_ID
typedef enum
{
  uMCP_Timer_TMO = 0,
  uMCP_Timer_SELECT = 1,
  uMCP_Timer_TX = 2,
  uMCP_Timer_INVALID
} uMCP_Timer_ID;

// DCID_Enum
typedef enum
{
  DC_1 = 0,
  DC_2 = 1,
  DC_3 = 2,
  DC_4 = 3,
  DC_5 = 4,
  DC_6 = 5,
  DC_INVALID
} DCID_Enum;
/// *************************************************** 

/// ***************************************************  Basic settings
#define CFG_HOST_DC_ID                   (DC_1)
#define CFG_LINE_DC_ID                   (DC_2)
#define CFG_HOST_PORT_BAUDRATE           (9600)
#define CFG_LINE_PORT_BAUDRATE           (9600)

#define CFG_DEFAULT_TID                  (0)
#define CFG_DEFAULT_SID                  (0)

// Default state of SELECT flag. If it is true, node will regain the flag by timeout anyway
#define CFG_SELECT_DEFAULT_STATE         (true)

// Default SELECT interval
#define CFG_SELECT_INTERVAL_MS           (4000)

// Default timeout interval
#define CFG_TIMEOUT_INTERVAL_MS          (1000)

// Line transmission speed in bits per second (80 for uWAVE and RedLINE modems)
#define CFG_LINEBAUDRATE                 (9600)

// Default data packet size
#define CFG_DATABLOCK_SIZE               (32) // Preferable size of a datablock (32 is set for Arduino due to poor memory)

// Nagle algorithm delay
#define CFG_NAGLE_DELAY_MS               (100)
/// *************************************************** 

/// *************************************************** 
#define UMCP_SIGN                   (0xAD)  // Packet start signature
#define UMCP_MAX_DATA_SIZE          (32)    // Maximum number of bytes in a data packet (64 by default, 32 for Arduino due to poor memory)
#define UMCP_MAX_OVERHEAD           (9)     // Maximum size of protocol overhead
#define UMCP_MAX_PACKETSIZE         (UMCP_MAX_DATA_SIZE + UMCP_MAX_OVERHEAD)
#define UMCP_FIXED_TX_DELAY_MS      (10)    // 
#define UMCP_MAX_SENT_BLOCKS        (8)     // Maximum number of unacknowledged packets to send
#define UMCP_PIPELINE_LIMIT         (4)     // Maximum number of packets to send by pipelining

#define CR_RING_SIZE                (UMCP_MAX_SENT_BLOCKS * UMCP_MAX_DATA_SIZE)
#define CR_DC_TX_BUFFER_SIZE        (255)   

#define UMCP_NBUFFER_SIZE           (512)
#define UMCP_LBUFFER_SIZE           (UMCP_MAX_PACKETSIZE)

#define IS_BYTE(value)              (((value) >= 0) && ((value) <= 255))
#define IS_VALID_TINT(value)        (((value) >= 100) && ((value) <= 120000))

// buffer from HOST
byte ih_ring[CR_RING_SIZE];
unsigned int ih_rPos = 0;
unsigned int ih_wPos = 0;
unsigned int ih_Cnt = 0;
unsigned int ih_TS = 0;

// buffer to HOST
unsigned char oh_buffer[UMCP_NBUFFER_SIZE];
unsigned int oh_cnt = 0;
bool oh_ready = false;

// buffer to LINE
byte ol_buffer[UMCP_LBUFFER_SIZE];
unsigned int ol_cnt = 0;
bool ol_ready = false;

// incoming packet
byte ip_datablock[UMCP_MAX_DATA_SIZE];
bool ip_start = false;
int ip_pos = 0;
byte ip_sid = 0;
byte ip_tid = 0;
uMCP_PacketType ip_type = uMCP_PTYPE_INVALID;
byte ip_tcnt = 0;
byte ip_rcnt = 0;
byte ip_ahchk = 0;
byte ip_dcnt = 0;
bool ip_ready = false;

byte R, N, A;
bool selectDefaultState = CFG_SELECT_DEFAULT_STATE;
bool select = CFG_SELECT_DEFAULT_STATE;
byte SID = CFG_DEFAULT_SID;
byte TID = CFG_DEFAULT_TID;
byte sid, tid, tcnt, rcnt, dcnt;
uMCP_PacketType pType;
uMCP_State state = uMCP_STATE_HALTED;
bool sack = false;
bool srep = false;
bool isTimerPendingOnTxFinish = false;
int lineBaudRate = CFG_LINEBAUDRATE;

unsigned long iTimer_Interval_ms[uMCP_Timer_INVALID];
unsigned long iTimer_ExpTime_ms[uMCP_Timer_INVALID];
unsigned long iTimer_State[uMCP_Timer_INVALID];

byte sentBlocksSize[UMCP_MAX_SENT_BLOCKS];
byte sentBlocksRPos[UMCP_MAX_SENT_BLOCKS];
byte sentBlocksCnt = 0;

/// *************************************************** 


/// *************************************************** CRC8 related stuff
const byte CRC8Table[] = { 0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
                           0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
                           0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
                           0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
                           0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
                           0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
                           0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
                           0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
                           0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
                           0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
                           0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
                           0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
                           0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
                           0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
                           0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
                           0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
                           0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
                           0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
                           0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
                           0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
                           0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
                           0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
                           0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
                           0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
                           0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
                           0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
                           0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
                           0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
                           0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
                           0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
                           0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
                           0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC };

byte CRC8_Update(byte prev, byte next)
{
  return CRC8Table[prev ^ next];
}

byte CRC8_Get(byte* buffer, int sIdx, int cnt)
{
  int i;
  byte crc = 0xff;
  for (i = 0; i < cnt; i++)
    crc = CRC8Table[crc ^ buffer[i + sIdx]];
  return crc;
}
/// ***************************************************

/// ***************************************************

// writers
void ff_fill_u8(byte* dst, byte value, unsigned int size)
{
  unsigned int blkCnt = size;
  while(blkCnt > 0u)
  {
    *dst++ = value;
    blkCnt--;
  }
}

void ff_copy_u8(byte* src, byte* dst, unsigned int size)
{
  unsigned int blkCnt = size;  
  while(blkCnt > 0u)
  {
    *dst++ = (*src++);
    blkCnt--;
  }
}

void Str_WriterInit(byte* buffer, unsigned int* srcIdx, unsigned int size)
{
  *srcIdx = 0;
  ff_fill_u8(buffer, 0, size);
}

void StrB_WriteByte(byte* buffer, unsigned int* srcIdx, byte b)
{
  buffer[*srcIdx] = b;
  (*srcIdx)++;
}

//



// uMCP

// Interval timers management routines
void uMCP_ITimer_Init(uMCP_Timer_ID timerID, unsigned int interval_ms, bool istate)
{
  iTimer_Interval_ms[timerID] = interval_ms;
  uMCP_ITimer_StateSet(timerID, istate);
}

void uMCP_ITimer_StateSet(uMCP_Timer_ID timerID, bool value)
{
  if (value)
  {
    iTimer_ExpTime_ms[timerID] = millis() + iTimer_Interval_ms[timerID];
    iTimer_State[timerID] = true;
  }
  else
    iTimer_State[timerID] = false;
}

void uMCP_ITimer_Expired(uMCP_Timer_ID timerID)
{
  if (timerID == uMCP_Timer_TMO)
  {
    if (state == uMCP_STATE_ISTART)
      uMCP_CtrlSend(uMCP_PTYPE_STR, 0, 0, true);
    else if (state == uMCP_STATE_ASTART)
      uMCP_CtrlSend(uMCP_PTYPE_STA, 0, 0, true);
    else if (state == uMCP_STATE_RUNNING)
    {
      srep = true;
      uMCP_Protocol_Perform();
    }
  }
  else if (timerID == uMCP_Timer_SELECT)
    uMCP_SELECT_Set(CFG_SELECT_DEFAULT_STATE);
  else if (timerID == uMCP_Timer_TX)
  {
    if (isTimerPendingOnTxFinish && !iTimer_State[uMCP_Timer_TMO])
    {
      isTimerPendingOnTxFinish = false;
      uMCP_ITimer_StateSet(uMCP_Timer_TMO, true);
    }
    else
      uMCP_Protocol_Perform();
  }
}

void uMCP_ITimers_Process()
{
  int tIdx = 0;
  for (tIdx = 0; tIdx < uMCP_Timer_INVALID; tIdx++)
  {
    if (iTimer_State[tIdx])
      if (millis() >= iTimer_ExpTime_ms[tIdx])
      {
        iTimer_State[tIdx] = false;
        uMCP_ITimer_Expired((uMCP_Timer_ID)tIdx);
      }
  }
}


// Utils
bool uMCP_IsByteInRangeExclusive(byte st, byte nd, byte val)
{
  bool result = false;
  byte idx = st;
  byte _nd = nd;
  _nd--;
  while ((idx != _nd) && (!result))
  {
    idx++;
    if (idx == val)
      result = true;
  }

  return result;
}


// uMCP state management routines
void uMCP_SELECT_Set(bool value)
{
  if (selectDefaultState)
    uMCP_ITimer_StateSet(uMCP_Timer_SELECT, value != selectDefaultState);

  select = value;

  if (select)
    uMCP_Protocol_Perform();
}

void uMCP_STATE_Set(uMCP_State value)
{
  if ((value == uMCP_STATE_ISTART) ||
    (value == uMCP_STATE_HALTED))
  {
    R = 0;
    N = 0;
    A = 0;
    sentBlocksCnt = 0;
  }
  state = value;
}


// basic protocol routines
void uMCP_DataSend(bool isDTE, byte tcnt, byte rcnt, unsigned int rPos, unsigned int cnt, bool isStartTimer)
{
  uMCP_PacketType ptype = uMCP_PTYPE_DTA;
  if (isDTE)
    ptype = uMCP_PTYPE_DTE;

  Str_WriterInit(ol_buffer, &ol_cnt, UMCP_LBUFFER_SIZE);
  StrB_WriteByte(ol_buffer, &ol_cnt, UMCP_SIGN);
  StrB_WriteByte(ol_buffer, &ol_cnt, ptype);
  StrB_WriteByte(ol_buffer, &ol_cnt, SID);
  StrB_WriteByte(ol_buffer, &ol_cnt, TID);
  StrB_WriteByte(ol_buffer, &ol_cnt, tcnt);
  StrB_WriteByte(ol_buffer, &ol_cnt, rcnt);
  StrB_WriteByte(ol_buffer, &ol_cnt, CRC8_Get(ol_buffer, 0, ol_cnt));

  unsigned int dstart_idx = ol_cnt;
  StrB_WriteByte(ol_buffer, &ol_cnt, (byte)cnt);

  int i;
  unsigned int rposc = rPos;
  for (i = 0; i < cnt; i++)
  {
    ol_buffer[ol_cnt++] = ih_ring[rposc];
    rposc = (rposc + 1) % CR_RING_SIZE;
  }

  StrB_WriteByte(ol_buffer, &ol_cnt, CRC8_Get(ol_buffer, dstart_idx, cnt + 1));

  ol_ready = true;
  isTimerPendingOnTxFinish = isStartTimer;

  uMCP_ITimer_Init(uMCP_Timer_TX, UMCP_FIXED_TX_DELAY_MS + 1000 * (int)(((float)(ol_cnt * 8)) / CFG_LINEBAUDRATE), true);

  uMCP_SELECT_Set(!isDTE);
}

void uMCP_NextDataBlockSend()
{
  bool isDTE = sentBlocksCnt >= UMCP_PIPELINE_LIMIT;
  byte pcnt = CFG_DATABLOCK_SIZE;
  if (ih_Cnt < CFG_DATABLOCK_SIZE)
  {
    pcnt = ih_Cnt;
    isDTE = true;
  }

  uMCP_DataSend(isDTE, N, R, ih_rPos, pcnt, isDTE);

  sentBlocksRPos[N] = ih_rPos;
  sentBlocksSize[N] = pcnt;
  sentBlocksCnt++;

  ih_rPos = (ih_rPos + pcnt) % CR_RING_SIZE;
  ih_Cnt -= pcnt;
}

void uMCP_DataBlockResend(byte blockId, bool isDTE, bool isStartTimer)
{
  if (sentBlocksSize[blockId] != 0)
    uMCP_DataSend(isDTE, blockId, R, sentBlocksRPos[blockId], sentBlocksSize[blockId], isStartTimer);
  // else
  //    wtf???
}

void uMCP_CtrlSend(uMCP_PacketType ptype, byte tcnt, byte rcnt, bool isStartTimer)
{
  Str_WriterInit(ol_buffer, &ol_cnt, UMCP_LBUFFER_SIZE);
  StrB_WriteByte(ol_buffer, &ol_cnt, UMCP_SIGN);
  StrB_WriteByte(ol_buffer, &ol_cnt, ptype);
  StrB_WriteByte(ol_buffer, &ol_cnt, SID);
  StrB_WriteByte(ol_buffer, &ol_cnt, TID);

  if ((ptype == uMCP_PTYPE_REP) || (ptype == uMCP_PTYPE_ACK))
  {
    StrB_WriteByte(ol_buffer, &ol_cnt, tcnt);
      if (ptype == uMCP_PTYPE_ACK)
        StrB_WriteByte(ol_buffer, &ol_cnt, rcnt);
  }

  StrB_WriteByte(ol_buffer, &ol_cnt, CRC8_Get(ol_buffer, 0, ol_cnt));
  ol_ready = true;
  isTimerPendingOnTxFinish = isStartTimer;

  uMCP_ITimer_Init(uMCP_Timer_TX, UMCP_FIXED_TX_DELAY_MS + 1000 * (int)(((float)(ol_cnt * 8)) / CFG_LINEBAUDRATE), true);
  uMCP_SELECT_Set(false);
}

void uMCP_AcknowledgeSentItems(int to)
{
  byte a;
  byte from = A;
  for (a = from; a != to; a++, A++)
  {
    sentBlocksSize[A] = 0; // sent block is free
    sentBlocksCnt--;
  }
}

void uMCP_Protocol_Perform()
{
   if (state == uMCP_STATE_RUNNING)
   {
    if ((!iTimer_State[uMCP_Timer_TX]) &&
      (!iTimer_State[uMCP_Timer_TMO]) &&
      (select))
    {
      if (ih_Cnt == 0)
      {
        if (srep)
        {
          uMCP_CtrlSend(uMCP_PTYPE_REP, N, 0, true);
          srep = false;
        }
        else if (sentBlocksCnt > 0)
        {
          uMCP_DataBlockResend(A + 1, true, true);
        }
        else if ((!selectDefaultState) || (sack))
        {
          uMCP_CtrlSend(uMCP_PTYPE_ACK, N, R, false);
          sack = false;
        }
      }
      else
      {
        if ((ih_Cnt >= CFG_DATABLOCK_SIZE) || (ih_TS + CFG_NAGLE_DELAY_MS < millis()))
        {
          N++;
          uMCP_NextDataBlockSend();
        }
      }
    }
  }
}

void uMCP_OnIncomingPacket()
{
  sid = ip_sid;
  tid = ip_tid;
  tcnt = ip_tcnt;
  rcnt = ip_rcnt;
  dcnt = ip_dcnt;
  pType = ip_type;
  ip_ready = false;

  switch (pType)
  {
  case uMCP_PTYPE_STR:
  {
    uMCP_SELECT_Set(true);
    if ((state == uMCP_STATE_HALTED) || (state == uMCP_STATE_ISTART))
    {
      uMCP_STATE_Set(uMCP_STATE_ASTART);
      uMCP_ITimer_StateSet(uMCP_Timer_TMO, false);
      uMCP_CtrlSend(uMCP_PTYPE_STA, 0, 0, true);
    }
    else if (state == uMCP_STATE_ASTART)
    {
      uMCP_STATE_Set(uMCP_STATE_RUNNING);
      uMCP_ITimer_StateSet(uMCP_Timer_TMO, false);
      uMCP_CtrlSend(uMCP_PTYPE_ACK, 0, 0, false);
    }
    else
    {
      uMCP_STATE_Set(uMCP_STATE_HALTED);
    }
    break;
  }
  case uMCP_PTYPE_STA:
  {
    uMCP_SELECT_Set(true);
    if ((state == uMCP_STATE_ISTART) || (state == uMCP_STATE_ASTART) || (state == uMCP_STATE_RUNNING))
    {
      uMCP_STATE_Set(uMCP_STATE_RUNNING);
      uMCP_ITimer_StateSet(uMCP_Timer_TMO, false);
      uMCP_CtrlSend(uMCP_PTYPE_ACK, 0, 0, false);
    }
    break;
  }
  case uMCP_PTYPE_REP:
  {
    if (state == uMCP_STATE_RUNNING)
    {
      sack = true;
      srep = false;
      uMCP_SELECT_Set(true);
    }
    break;
  }
  case uMCP_PTYPE_ACK:
  {
    if (state == uMCP_STATE_ASTART)
    {
      if (rcnt == 0)
      {
        uMCP_ITimer_StateSet(uMCP_Timer_TMO, false);
        uMCP_STATE_Set(uMCP_STATE_RUNNING);
      }
    }
    else if (state == uMCP_STATE_RUNNING)
    {
      uMCP_ITimer_StateSet(uMCP_Timer_TMO, false);
      if ((rcnt == N) || (uMCP_IsByteInRangeExclusive(A, N, rcnt)))
        uMCP_AcknowledgeSentItems(rcnt);
    }

    uMCP_SELECT_Set(true);
    break;
  }
  case uMCP_PTYPE_DTA:
  case uMCP_PTYPE_DTE:
  {
    if (state == uMCP_STATE_RUNNING)
    {
      if (tcnt <= R + 1)
      {
        if (tcnt == R + 1)
        {
          R++;
          ff_copy_u8(ip_datablock, oh_buffer, dcnt);
          oh_cnt = dcnt;
          oh_ready = true;
        }
        sack = true;
      }

      iTimer_State[uMCP_Timer_TMO] = false;

      if ((rcnt == N) || (uMCP_IsByteInRangeExclusive(A, N, rcnt)))
        uMCP_AcknowledgeSentItems(rcnt);

      if (pType == uMCP_PTYPE_DTA)
        uMCP_ITimer_StateSet(uMCP_Timer_TMO, true);

      uMCP_SELECT_Set(pType == uMCP_PTYPE_DTE);
    }
    break;
  }
  default:
    break;
  }
}


// Events
void uMCP_DC_Output_Process()
{  
  if (oh_ready)
  {    
    Serial.write(oh_buffer, oh_cnt);
    oh_ready = false;
  }
  
  if (ol_ready)
  {
#ifndef HAS_SERIAL1
    linePort.write(ol_buffer, ol_cnt);
#else
    Serial1.write(ol_buffer, ol_cnt);
#endif
    ol_ready = false;
  }
}

void uMCP_DC_Input_Process()
{
  if (Serial.available())
  {
    uMCP_OnNewByte(CFG_HOST_DC_ID, Serial.read());
  }

#ifndef HAS_SERIAL1
  if (linePort.available())
  {
    uMCP_OnNewByte(CFG_LINE_DC_ID, linePort.read()); 
  }  
#else
  if (Serial1.available())
  {
    uMCP_OnNewByte(CFG_LINE_DC_ID, Serial1.read()); 
  }  
#endif
}

void uMCP_OnNewByte(DCID_Enum chID, byte c)
{
  if (chID == CFG_HOST_DC_ID)
  {
    if (ih_Cnt <= CR_RING_SIZE)
    {
      ih_ring[ih_wPos] = c;
      ih_wPos = (ih_wPos + 1) % CR_RING_SIZE;
      ih_Cnt++;
      ih_TS = millis();
    }
    else
    {
      Serial.print("TX OVERFLOW");
    }
  }
  else if (chID == CFG_LINE_DC_ID)
  {
    if (ip_start)
    {
      if (ip_pos == 1)
      {
        ip_ahchk = CRC8_Update(ip_ahchk, c);
        ip_type = (uMCP_PacketType)c;
        if (ip_type == uMCP_PTYPE_INVALID)
          ip_start = false;
        else
          ip_pos++;
      }
      else if (ip_pos <= 3)
      {
        ip_ahchk = CRC8_Update(ip_ahchk, c);
        if (ip_pos == 3)
          ip_tid = c;
        else if (ip_pos == 2)
          ip_sid = c;
        ip_pos++;
      }
      else
      {
        if ((ip_type == uMCP_PTYPE_STR) || (ip_type == uMCP_PTYPE_STA))
        {
          if (ip_ahchk == c) // CRC OK
          {
            ip_start = false;
            ip_ready = true;
          }
        }
        else
        {
          if (ip_pos == 4)
          {
            ip_tcnt = c;
            ip_pos++;
            ip_ahchk = CRC8_Update(ip_ahchk, c);
          }
          else
          {
            if (ip_type == uMCP_PTYPE_REP)
            {
              if (ip_ahchk == c)
                ip_ready = true;
              ip_start = false;
            }
            else
            {
              if (ip_pos == 5)
              {
                ip_rcnt = c;
                ip_pos++;
                ip_ahchk = CRC8_Update(ip_ahchk, c);
              }
              else
              {
                if (ip_pos == 6) // header checksum
                {
                  if (ip_ahchk == c)
                  {
                    if (ip_type == uMCP_PTYPE_ACK)
                    {
                      ip_ready = true;
                      ip_start = false;
                    }
                    else
                      ip_pos++;
                  }
                  else
                    ip_start = false;
                }
                else
                {
                  if (ip_pos == 7) // dcnt
                  {
                    ip_dcnt = c;
                    if (ip_dcnt >= 1)
                    {
                      ip_ahchk = CRC8_Update(0xff, c);
                      ip_pos++;
                    }
                    else
                    {
                      ip_start = false;
                      ip_ready = true;
                      ip_type = uMCP_PTYPE_ACK;
                    }
                  }
                  else if (ip_pos == 8 + ip_dcnt)
                  {
                    if (ip_ahchk == c) // data block CRC ok
                      ip_ready = true;
                    else
                      ip_type = uMCP_PTYPE_ACK;
                    ip_start = false;
                  }
                  else
                  {
                    ip_datablock[ip_pos - 8] = c;
                    ip_ahchk = CRC8_Update(ip_ahchk, c);
                    ip_pos++;
                  }
                }
              }
            }
          }
        }
      }
    }
    else if (c == UMCP_SIGN)
    {
      ip_start = true;
      ip_pos = 1;
      ip_ahchk = CRC8_Update(0xff, UMCP_SIGN);
      ip_type = uMCP_PTYPE_INVALID;
      ip_dcnt = 0;
    }
  }
}

/// ***************************************************


void setup() {
  // Interval timers
  uMCP_ITimer_Init(uMCP_Timer_SELECT, CFG_SELECT_INTERVAL_MS, false);
  uMCP_ITimer_Init(uMCP_Timer_TMO, CFG_TIMEOUT_INTERVAL_MS, false);
  
  Serial.begin(CFG_HOST_PORT_BAUDRATE); 
  while (!Serial) { }

#ifndef HAS_SERIAL1
  linePort.begin(CFG_LINE_PORT_BAUDRATE);
#else
  Serial1.begin(CFG_LINE_PORT_BAUDRATE);
#endif
}

void loop() {
    uMCP_DC_Input_Process();
    uMCP_DC_Output_Process();
    uMCP_ITimers_Process();

    if (ip_ready)
      uMCP_OnIncomingPacket();

    // Protocol autostart if buffer is not empty
    if ((state == uMCP_STATE_HALTED) && (ih_Cnt > 0))
    {
      uMCP_STATE_Set(uMCP_STATE_ISTART);
      uMCP_CtrlSend(uMCP_PTYPE_STR, 0, 0, true);
    }
    else
      uMCP_Protocol_Perform();
}
