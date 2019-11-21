/* 
 *   Necessary hardware:
 *      - Arduino UNO board
 *      - uWAVE underwater acoustic modem (2 is better)
 *      
 *   Connect uWAVE's TX wire to Arduino's PIN 2
 *   Connect uWAVE's RX wire to Arduino's PIN 3
 *   Connect uWAVE's CMD wire to Arduino's PIN 4
 *   Connect uWAVE's GND wire to any Arduino's GND PIN
 *   Connect uWAVE's VCC wire to Arduino's 5V PIN
 *   
 *   Connect Arduino board to a PC with USB cable
 *   
 *   If you have another uWAVE modem, turn it on and make sure its tx and rx channels IDs set to 0
 *   
 *   Open Arduino IDE
 *   Open this scketch in the IDE
 *   Open Port Monitor in Main menu -> Tools -> Port monitor
 *   
 *   Program your board
 *   
 *   This sketch will put your uWAVE in cmd mode first by pulling PIN 4 to HIGH
 *   After that, it will:
 *   - query for uWAVE's device info
 *   - update modem's settings
 *   - update modem's ambient data settings, to make modem send ambient data after any modem's output
 *   - sequentaly send all remote requests from RC_PING to RC_USR_CMD_008
 *   
*/


#include <SoftwareSerial.h>

#define UWAVE_CMD_PIN (4)
#define SOFTWARE_RX_PIN (2)
#define SOFTWARE_TX_PIN (3)
#define UNDEFINED_FLOAT_VAL (-32768)

SoftwareSerial uWAVEPort(SOFTWARE_RX_PIN, SOFTWARE_TX_PIN);

#define UWAVE_MAX_IN_PACKET_SIZE  (255)
#define UWAVE_MAX_OUT_PACKET_SIZE (64)
#define UWAVE_TIMEOUT_MS          (2000)
#define UWAVE_REMOTE_TIMEOUT_MS   (5000)

unsigned long uWAVE_time = 0;
unsigned long uWAVE_rtime = 0;
bool uWAVE_is_cmd_mode = false;
bool uWAVE_is_devInfo_updated = false;
bool uWAVE_settings_updated = false;
bool uWAVE_amb_settings_updated = false;
bool uWAVE_is_busy = false;
bool uWAVE_waiting_for_remote = false;

byte uWAVE_serial[24];
byte uWAVE_core_moniker[32];
short uWAVE_core_version = 0;
byte uWAVE_system_moniker[32];
short uWAVE_system_version = 0;
float uWAVE_ac_baudrate = UNDEFINED_FLOAT_VAL;
byte uWAVE_txChID = 0;
byte uWAVE_rxChID = 0;
byte uWAVE_totalChannels = 0;
float uWAVE_salinityPSU = 0;
bool uWAVE_is_PTS_present = false;
bool uWAVE_is_cmd_mode_byDefault = false;

float uWAVE_amb_prs_mBar = UNDEFINED_FLOAT_VAL;
float uWAVE_amb_tmp_C = UNDEFINED_FLOAT_VAL;
float uWAVE_amb_dpt_m = UNDEFINED_FLOAT_VAL;
float uWAVE_amb_batV = UNDEFINED_FLOAT_VAL;
bool uWAVE_amb_updated = false;

byte uWAVE_in_packet[UWAVE_MAX_IN_PACKET_SIZE];
byte uWAVE_in_packet_idx = 0;
bool uWAVE_in_packet_ready = false;
bool uWAVE_in_packet_started = false;

byte uWAVE_out_packet[UWAVE_MAX_OUT_PACKET_SIZE];
byte uWAVE_out_packet_idx = 0;
bool uWAVE_out_packet_ready = false;

#define NMEA_SNT_STR '$'
#define NMEA_SNT_END '\n'
#define NMEA_PAR_SEP ','
#define NMEA_CHK_SEP '*'

// UWV Proprietary sentences
#define UWV_PREFIX_LEN          (4)
#define UWV_PREFIX              "PUWV\0"

#define IC_D2H_ACK              '0'        // $PUWV0,cmdID,errCode
#define IC_H2D_SETTINGS_WRITE   '1'        // $PUWV1,rxChID,txChID,styPSU,isCmdMode,isACKOnTXFinished
#define IC_H2D_RC_REQUEST       '2'        // $PUWV2,txChID,rxChID,rcCmdID
#define IC_D2H_RC_RESPONSE      '3'        // $PUWV3,txChID,rcCmdID,propTime_seс,snr,[value],[azimuth]
#define IC_D2H_RC_TIMEOUT       '4'        // $PUWV4,txChID,rcCmdID
#define IC_D2H_RC_ASYNC_IN      '5'        // $PUWV5,rcCmdID,snr,[azimuth]
#define IC_H2D_AMB_DTA_CFG      '6'        // $PUWV6,isWriteInFlash,periodMs,isPrs,isTemp,isDpt,isBatV
#define IC_D2H_AMB_DTA          '7'        // $PUWV7,prs_mBar,temp_C,dpt_m,batVoltage_V

#define IC_H2D_DINFO_GET        '?'        // $PUWV?,reserved
#define IC_D2H_DINFO            '!'        // $PUWV!,serialNumber,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,totalCh,salinityPSU,isPTS,isCmdMode

#define IC_D2H_UNKNOWN          '-'

// Local error type definition
typedef enum
{
    LOC_ERR_NO_ERROR              = 0,
    LOC_ERR_INVALID_SYNTAX        = 1,
    LOC_ERR_UNSUPPORTED           = 2,
    LOC_ERR_TRANSMITTER_BUSY      = 3,
    LOC_ERR_ARGUMENT_OUT_OF_RANGE = 4,
    LOC_ERR_INVALID_OPERATION     = 5,
    LOC_ERR_UNKNOWN_FIELD_ID      = 6,
    LOC_ERR_VALUE_UNAVAILIBLE     = 7,
    LOC_ERR_RECEIVER_BUSY         = 8,
    LOC_ERR_TX_BUFFER_OVERRUN     = 9,
    LOC_ERR_CHKSUM_ERROR          = 10,
    LOC_ACK_TX_FINISHED           = 11,
    LOC_ACK_BEFORE_STANDBY        = 12,
    LOC_ACK_AFTER_WAKEUP          = 13,
    LOC_ERR_SVOLTAGE_TOO_HIGH     = 14,

    LOC_ERR_UNKNOWN
} LocalError_Enum;

// RC_CODES_Enum
typedef enum
{
  RC_PING        = 0,
  RC_PONG        = 1,
  RC_DPT_GET     = 2,
  RC_TMP_GET     = 3,
  RC_BAT_V_GET   = 4,
  RC_ERR_NSUP    = 5,
  RC_ACK         = 6,
  RC_USR_CMD_000 = 7,
  RC_USR_CMD_001 = 8,
  RC_USR_CMD_002 = 9,
  RC_USR_CMD_003 = 10,
  RC_USR_CMD_004 = 11,
  RC_USR_CMD_005 = 12,
  RC_USR_CMD_006 = 13,
  RC_USR_CMD_007 = 14,
  RC_USR_CMD_008 = 15,
  RC_INVALID

} RC_CODES_Enum;

byte rcID;
RC_CODES_Enum requestID = RC_PING;

// ****************************************** Utils
void fill_bytes(byte* buffer, byte size)
{  
  for (byte i = 0; i < size; i++)
     buffer[i] = 0;
}




// ******************************************  Readers
float Str_ParseFloat(const byte* buffer, byte stIdx, byte ndIdx)
{
  int i, dotIdx = ndIdx - 1;
  float sign = 1.0f, fract = 0.0f;

  if (buffer[stIdx] == '-')
  { 
    sign = -1.0f;
    stIdx++;
  }

  for (i = stIdx; i <= ndIdx; i++) 
  { 
    if (buffer[i] == '.') 
    {
      dotIdx = i;
    }
  }

  float result = 0.0f;
  float multiplier = 1.0f;

  for (i = dotIdx - 1; i >= stIdx; i--)
  {
    result += ((float)((buffer[i] - '0'))) * multiplier;
    multiplier *= 10.0f;
  }

  multiplier = 0.1f;

  for (i = dotIdx + 1; i <= ndIdx; i++)
  {
    fract += ((float)((buffer[i] - '0'))) * multiplier;
    multiplier /= 10.0f;
  }

  result += fract;
  return result * sign;
}

int Str_ParseIntDec(const byte* buffer, byte stIdx, byte ndIdx)
{
  byte i;
  int sign = 1;

  if (buffer[stIdx] == '-')
  { 
    sign = -1;
    stIdx++;
  }

  int result = 0;
  int multiplier = 1;

  for (i = ndIdx; i >= stIdx; i--)
  {
    result += ((int)((buffer[i] - '0'))) * multiplier;
    multiplier *= 10;
  }

  return result;
}

byte Str_ParseHexByte(const byte* buffer, byte stIdx)
{
  byte c1 = buffer[stIdx];
  byte c2 = buffer[stIdx + 1];
  if (c1 >= 0x41) 
  { 
    c1 -= 'A'; 
    c1 += 10; 
  }
  else 
  {
    c1 -= '0';
  }
  if (c2 >= 0x41) 
  { 
    c2 -= 'A'; 
    c2 += 10; 
  } 
  else
  {
    c2 -= '0';
  }
  return c1 * 16 + c2;
}

void Str_ReadString(const byte* src_buffer, byte* dst_buffer, byte stIdx, byte ndIdx)
{
  for (byte i = stIdx; i <= ndIdx; i++) 
  {
    dst_buffer[i - stIdx] = src_buffer[i];
  }
}




// ******************************************  Writers
void Str_WriteByte(byte* buffer, byte* srcIdx, byte c)
{
  buffer[*srcIdx] = c;
  (*srcIdx)++;
}

void Str_WriteHexByte(byte* buffer, byte* srcIdx, byte c)
{
  byte b1 = c / 16;
  byte b2 = c % 16;
  if (b1 > 9) b1 += ('A' - 10); else b1 += '0';
  if (b2 > 9) b2 += ('A' - 10); else b2 += '0';
  buffer[*srcIdx] = b1;
  (*srcIdx)++;
  buffer[*srcIdx] = b2;
  (*srcIdx)++;
}

void Str_WriteIntDec(byte* buffer, byte* srcIdx, int src, byte zPad)
{
  int x = src, len = 0, i;

  do { x /= 10; len++; } while (x >= 1);

  x = 1;
  for (i = 1; i < len; i++) x *= 10;

  if (zPad > 0) i = zPad;
  else i = len;

  do
  {
    if (i > len) buffer[*srcIdx] = '0';
    else
    {
      buffer[*srcIdx] = (byte)((src / x) + '0');
      src -= (src / x) * x;
      x /= 10;
    }
    (*srcIdx)++;
  } while (--i > 0);
}

void Str_WriteFloat(byte* buffer, byte* srcIdx, float f, byte dPlaces, byte zPad)
{
  float ff = f;

  if (ff < 0)
  {
    Str_WriteByte(buffer, srcIdx, '-');
    ff = -f;
  }

  int dec = (int)ff, mult = 1, i;
  for (i = 0; i < dPlaces; i++) mult *= 10;
  int frac = (int)((ff - dec) * (float)mult);

  Str_WriteIntDec(buffer, srcIdx, dec, zPad);
  Str_WriteByte(buffer, srcIdx, '.');
  Str_WriteIntDec(buffer, srcIdx, frac, dPlaces);
}

void Str_WriteString(byte* buffer, byte* srcIdx, byte* src)
{
  byte c;
  c = *src;
  while (c != '\0') 
  { 
    buffer[(*srcIdx)++] = c;
    c = *++src; 
  }
}




// ****************************************** NMEA stuff

bool NMEA_Checksum_Check(const byte* buffer, byte size)
{
  byte i = 0;
  byte acc = 0;
  bool result = false;

  while ((i < size) && (buffer[i] != NMEA_CHK_SEP))
  {
    if (buffer[i] == NMEA_SNT_STR) 
    {
      acc = 0;
    }
    else
    { 
      acc ^= buffer[i];
    }
    i++;
  }

  if (buffer[i] == NMEA_CHK_SEP) 
  {
    result = (acc == Str_ParseHexByte(buffer, i + 1));
  }
  else
  {
    result = true;
  }

  return result;
}

void NMEA_CheckSum_Update(byte buffer[], byte size)
{
  byte i;
  byte acc = 0, b1, b2;
  for (i = 0; i < size; i++)
  {
    if (buffer[i] == NMEA_SNT_STR)
    {
      acc = 0;
    }
    else if (buffer[i] == NMEA_CHK_SEP)
    {
      b1 = acc / 16;
      if (b1 > 9)
      {
        b1 += ('A' - 10);
      }
      else
      {
        b1 += '0';      
      }
      b2 = acc % 16;
      
      if (b2 > 9)
      {
        b2 += ('A' - 10);
      }
      else
      {
        b2 += '0';
      }
      
      buffer[i + 1] = b1;
      buffer[i + 2] = b2;
    }
    else
    {
      acc ^= buffer[i];
    }
  }
}

void NMEA_ProcessByte(byte* buffer, byte size, bool* isReady, bool* isStarted, byte* idx, byte newByte)
{
  if (!(*isReady))
  {
    if (newByte == NMEA_SNT_STR)
    {
      *isStarted = true;
      fill_bytes(buffer, size);
      *idx = 0;
      buffer[(*idx)++] = newByte;
    }
    else
    {
      if (*isStarted)
      {
        if (newByte == NMEA_SNT_END)
        {
          *isStarted = false;
          *isReady = true;
        }
        else
        {
          buffer[(*idx)++] = newByte;
          if (*idx >= size) 
          {
            *isStarted = false;
          }
        }
      }
    }      
  }
}

bool NMEA_GetNextParameter(const byte* buffer, byte fromIdx, byte size, byte* stIdx, byte* ndIdx)
{
  byte i = fromIdx + 1;
  *stIdx = fromIdx;
  *ndIdx = *stIdx;

  while ((i <= size) && (*ndIdx == *stIdx))
  {
    if ((buffer[i] == NMEA_PAR_SEP) || 
        (buffer[i] == NMEA_CHK_SEP) ||
        (buffer[i] == '\r') ||
        (i == size))
    {
      *ndIdx = i;           
    }
    else
    {
      i++;
    }
  }
  return ((buffer[i] != NMEA_CHK_SEP) && (i != size) && (buffer[i] != '\r'));
}







// ******************************************  uWAVE-related items

// ******************************************  uWAVE sentences parsers
void uWAVE_ACK_Parse(const byte* buffer, byte size)
{
  // $PUWV0,sndID,errCode
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;

  LocalError_Enum errCode = LOC_ERR_UNKNOWN;
  char sntID = IC_D2H_UNKNOWN;
  
  do
  {
    result = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    switch (pIdx)
    {
      case 1:
        sntID = char(buffer[stIdx]);
        break;
      case 2:
        errCode = (LocalError_Enum)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result); 

  uWAVE_is_busy = false;

  Serial.println("\r\n\r\n[ACK]");
  if (errCode == LOC_ERR_NO_ERROR)
  {    
    if (sntID == IC_H2D_RC_REQUEST)
    {
      uWAVE_waiting_for_remote = true;
      uWAVE_rtime = millis();      
    }
    else if (sntID == IC_H2D_SETTINGS_WRITE)
    {
      uWAVE_settings_updated = true;      
    }
    else if (sntID == IC_H2D_AMB_DTA_CFG)
    {
      uWAVE_amb_settings_updated = true;      
    }
  }
  else
  {
    Serial.print("Sentence ");
    Serial.print(sntID);
    Serial.print(" caused ");
    Serial.println(errCode);
  }
}

void uWAVE_RC_RESPONSE_Parse(const byte* buffer, byte size)
{
 // $PUWV3,txChID,rcCmdID,propTime_seс,msr,[value],[azimuth]
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;

  byte txChID = 255;
  RC_CODES_Enum rcCmdID = RC_INVALID;
  float pTime = UNDEFINED_FLOAT_VAL;
  float msr = UNDEFINED_FLOAT_VAL;
  float value = UNDEFINED_FLOAT_VAL;
  float azimuth = UNDEFINED_FLOAT_VAL;
  
  do
  {
    result = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    switch (pIdx)
    {
      case 1:
        txChID = (byte)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        rcCmdID = (RC_CODES_Enum)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 3:
        pTime = Str_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 4:
        msr = Str_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 5:
        if (stIdx < ndIdx)
        {
          value = Str_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 6:
        if (stIdx < ndIdx)
        {
          azimuth = Str_ParseFloat(buffer, stIdx, ndIdx);              
        }
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result); 

  uWAVE_waiting_for_remote = false;

  Serial.println("\r\n\r\n[RC_RESPONSE]");
  Serial.print(rcCmdID);
  Serial.print(" received from ");
  Serial.print(txChID);
  Serial.print(" with MSR = ");
  Serial.print(msr);
  Serial.print(" dB ");

  Serial.print("\r\nPropagation time=");
  Serial.print(pTime, 4);
  Serial.print(" sec");
  
  if (value != UNDEFINED_FLOAT_VAL)
  {
    Serial.print("\r\nreceived value = ");
    Serial.print(value);  
  }
  if (azimuth != UNDEFINED_FLOAT_VAL)
  {
    Serial.print("\r\nazimuth = ");
    Serial.print(azimuth);
    Serial.print("°");
  }
  Serial.println(); 
}

void uWAVE_RC_TIMEOUT_Parse(const byte* buffer, byte size)
{
// $PUWV4,txChID,rcCmdID
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;

  byte txChID = 255;
  RC_CODES_Enum rcCmdID = RC_INVALID;
  
  do
  {
    result = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    switch (pIdx)
    {
      case 1:
        txChID = (byte)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        rcCmdID = (RC_CODES_Enum)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result); 

  uWAVE_waiting_for_remote = false;

  Serial.println("\r\n\r\n[RC_TIMEOUT]");
  Serial.print(rcCmdID);
  Serial.print(" to ");
  Serial.print(txChID);
  Serial.println(" caused remote timeout");
}

void uWAVE_RC_ASYNC_IN_Parse(const byte* buffer, byte size)
{
  // $PUWV5,rcCmdID,msr,[azimuth]
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;
  
  RC_CODES_Enum rcCmdID = RC_INVALID;
  float msr = UNDEFINED_FLOAT_VAL;
  float azimuth = UNDEFINED_FLOAT_VAL;
  
  do
  {
    result = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    switch (pIdx)
    {
      case 1:
        rcCmdID = (RC_CODES_Enum)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        msr = Str_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 3:
        if (stIdx != ndIdx)
        {
          azimuth = Str_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result); 

  uWAVE_waiting_for_remote = false;

  Serial.println("\r\n\r\n[RC_ASYNC_IN]");
  Serial.print(rcCmdID);
  Serial.print(" received with MSR = ");
  Serial.print(msr);
  Serial.print(" dB ");
  if (azimuth != UNDEFINED_FLOAT_VAL)
  {
    Serial.print(", azimuth = ");
    Serial.print(azimuth);
    Serial.print("°");
  }
  Serial.println();
}

void uWAVE_AMB_DTA_Parse(const byte* buffer, byte size)
{
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;
  float val;
 
  do
  {        
    result = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    if (stIdx < ndIdx)
    {
      val = Str_ParseFloat(buffer, stIdx, ndIdx);
      switch (pIdx)
      {
        case 1:    
          uWAVE_amb_prs_mBar = val;
          break;
        case 2:
          uWAVE_amb_tmp_C = val;
          break;
        case 3:
          uWAVE_amb_dpt_m = val;
          break;
        case 4:
          uWAVE_amb_batV = val;
          break;     
        default:
          break;
      }
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result);

  uWAVE_amb_updated = true;

  Serial.println("\r\n\r\n[AMB_DTA]");

  if (uWAVE_amb_prs_mBar != UNDEFINED_FLOAT_VAL)
  {
    Serial.print("Pressure, mBar=");
    Serial.println(uWAVE_amb_prs_mBar, 1);
  }

  if (uWAVE_amb_tmp_C != UNDEFINED_FLOAT_VAL)
  {
    Serial.print("Temperature, °C=");
    Serial.println(uWAVE_amb_tmp_C, 1);
  }

  if (uWAVE_amb_dpt_m != UNDEFINED_FLOAT_VAL)
  {
    Serial.print("Depth, m=");
    Serial.println(uWAVE_amb_dpt_m, 3);
  }

  if (uWAVE_amb_batV != UNDEFINED_FLOAT_VAL)
  {
    Serial.print("Supply voltage, V=");
    Serial.println(uWAVE_amb_batV, 1);
  }         
}

void uWAVE_DINFO_Parse(const byte* buffer, byte size)
{
  // $PUWV!,serialNumber,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,totalCh,salinityPSU,isPTS,isCmdMode
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;
  
  do
  {        
    result = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    switch (pIdx)
    {
      case 1:      
        Str_ReadString(buffer, uWAVE_serial, stIdx, ndIdx);
        break;
      case 2:
        Str_ReadString(buffer, uWAVE_core_moniker, stIdx, ndIdx);
        break;
      case 3:
        uWAVE_core_version = Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 4:
        Str_ReadString(buffer, uWAVE_system_moniker, stIdx, ndIdx);
        break;
      case 5:
        uWAVE_system_version = Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 6:
        uWAVE_ac_baudrate = Str_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 7:
        uWAVE_txChID = Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 8:
        uWAVE_rxChID = Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 9:
        uWAVE_totalChannels = Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 10:
        uWAVE_salinityPSU = Str_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 11:
        uWAVE_is_PTS_present = (bool)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 12:
        uWAVE_is_cmd_mode_byDefault = (bool)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      default:
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result);

  uWAVE_is_busy = false;

  Serial.println("\r\n\r\n[DINFO]");
  Serial.print("S/N=");
  Serial.write(uWAVE_serial, 24);
  Serial.print("\r\nCore moniker=");
  Serial.write(uWAVE_core_moniker, 32);
  Serial.print("\r\nCore version=");
  Serial.print(uWAVE_core_version >> 0x08);
  Serial.print(".");
  Serial.println(uWAVE_core_version & 0xff);
  Serial.print("System moniker=");
  Serial.write(uWAVE_system_moniker, 32);
  Serial.print("\r\nSystem version=");
  Serial.print(uWAVE_system_version >> 0x08);
  Serial.print(".");
  Serial.println(uWAVE_system_version & 0xff);
  Serial.print("Acoustic baudrate, bit/s=");
  Serial.println(uWAVE_ac_baudrate, 2);
  Serial.print("Tx channel ID=");
  Serial.println(uWAVE_txChID);
  Serial.print("Rx channel ID=");
  Serial.println(uWAVE_rxChID);
  Serial.print("Total channels=");
  Serial.println(uWAVE_totalChannels);
  Serial.print("Salinity, PSU=");
  Serial.println(uWAVE_salinityPSU, 1);
  Serial.print("PTS present=");
  Serial.println(uWAVE_is_PTS_present);
  Serial.print("Command mode by default=");
  Serial.println(uWAVE_is_cmd_mode_byDefault);
  
  uWAVE_is_devInfo_updated = true;
}



// ******************************************  uWAVE sentences writers
void uWAVE_DINFO_GET_Write()
{
  uWAVE_out_packet_idx = 0;
  fill_bytes(uWAVE_out_packet, UWAVE_MAX_OUT_PACKET_SIZE);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_SNT_STR);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)UWV_PREFIX);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, IC_H2D_DINFO_GET);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, 0, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_CHK_SEP);
  Str_WriteHexByte(uWAVE_out_packet, &uWAVE_out_packet_idx, 0);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)"\r\n");
  NMEA_CheckSum_Update(uWAVE_out_packet, uWAVE_out_packet_idx);
  
  uWAVE_out_packet_ready = true;
  uWAVE_is_busy = true;
  uWAVE_time = millis();
}

void uWAVE_SETTINGS_WRITE_Write(byte rxID, byte txID, float salinityPSU, bool isCmdModeByDefault, bool isACKOnTxFinished)
{
  uWAVE_out_packet_idx = 0;
  fill_bytes(uWAVE_out_packet, UWAVE_MAX_OUT_PACKET_SIZE);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_SNT_STR);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)UWV_PREFIX);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, IC_H2D_SETTINGS_WRITE);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, rxID, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, txID, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteFloat(uWAVE_out_packet, &uWAVE_out_packet_idx, salinityPSU, 0, 1);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isCmdModeByDefault, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isACKOnTxFinished, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_CHK_SEP);
  Str_WriteHexByte(uWAVE_out_packet, &uWAVE_out_packet_idx, 0);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)"\r\n");
  NMEA_CheckSum_Update(uWAVE_out_packet, uWAVE_out_packet_idx);
  
  uWAVE_out_packet_ready = true;
  uWAVE_is_busy = true;
  uWAVE_time = millis();
}

void uWAVE_RC_REQUEST_Write(byte txID, byte rxID, RC_CODES_Enum rcCmdID)
{
  uWAVE_out_packet_idx = 0;
  fill_bytes(uWAVE_out_packet, UWAVE_MAX_OUT_PACKET_SIZE);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_SNT_STR);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)UWV_PREFIX);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, IC_H2D_RC_REQUEST);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, txID, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, rxID, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, rcCmdID, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_CHK_SEP);
  Str_WriteHexByte(uWAVE_out_packet, &uWAVE_out_packet_idx, 0);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)"\r\n");
  NMEA_CheckSum_Update(uWAVE_out_packet, uWAVE_out_packet_idx);
  
  uWAVE_out_packet_ready = true;
  uWAVE_is_busy = true;
  uWAVE_time = millis();
}

void uWAVE_AMB_DTA_CFG_Write(bool isWriteInFlash, int periodMs, bool isPrs, bool isTemp, bool isDpt, bool isBatV)
{
  uWAVE_out_packet_idx = 0;
  fill_bytes(uWAVE_out_packet, UWAVE_MAX_OUT_PACKET_SIZE);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_SNT_STR);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)UWV_PREFIX);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, IC_H2D_AMB_DTA_CFG);  
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isWriteInFlash, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, periodMs, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isPrs, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isTemp, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isDpt, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_PAR_SEP);
  Str_WriteIntDec(uWAVE_out_packet, &uWAVE_out_packet_idx, isBatV, 0);
  Str_WriteByte(uWAVE_out_packet, &uWAVE_out_packet_idx, NMEA_CHK_SEP);
  Str_WriteHexByte(uWAVE_out_packet, &uWAVE_out_packet_idx, 0);
  Str_WriteString(uWAVE_out_packet, &uWAVE_out_packet_idx, (byte*)"\r\n");
  NMEA_CheckSum_Update(uWAVE_out_packet, uWAVE_out_packet_idx);
  
  uWAVE_out_packet_ready = true;
  uWAVE_is_busy = true;
  uWAVE_time = millis();
}


void uWAVE_SetCmdMode(bool isCmdMode)
{
  if (isCmdMode)
  {
    digitalWrite(UWAVE_CMD_PIN, HIGH);
    uWAVE_is_cmd_mode = true;
  }
  else
  {
    digitalWrite(UWAVE_CMD_PIN, LOW);
    uWAVE_is_cmd_mode = false;
  }
}

void uWAVE_Init()
{
  pinMode(UWAVE_CMD_PIN, OUTPUT);
  uWAVE_SetCmdMode(true);
  delay(500);
  uWAVEPort.begin(9600);
  uWAVE_time = millis();
}

void uWAVE_Input_Process()
{
  if (uWAVEPort.available() > 0)
  {
    byte newByte = uWAVEPort.read();
    NMEA_ProcessByte(uWAVE_in_packet, UWAVE_MAX_IN_PACKET_SIZE, &uWAVE_in_packet_ready, &uWAVE_in_packet_started, &uWAVE_in_packet_idx, newByte);    
  }

  if (uWAVE_in_packet_ready)
  {    
    if (NMEA_Checksum_Check(uWAVE_in_packet, uWAVE_in_packet_idx))
    {      
      if ((uWAVE_in_packet[1] == UWV_PREFIX[0]) && 
          (uWAVE_in_packet[2] == UWV_PREFIX[1]) && 
          (uWAVE_in_packet[3] == UWV_PREFIX[2]) &&
          (uWAVE_in_packet[4] == UWV_PREFIX[3]))
      {
         switch (uWAVE_in_packet[5])
         {
           case IC_D2H_ACK:             
             uWAVE_ACK_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);             
             break;           
           case IC_D2H_RC_RESPONSE:             
             uWAVE_RC_RESPONSE_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);
             break;
           case IC_D2H_RC_TIMEOUT:             
             uWAVE_RC_TIMEOUT_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);             
             break;
           case IC_D2H_RC_ASYNC_IN:
             uWAVE_RC_ASYNC_IN_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);             
             break;
           case IC_D2H_AMB_DTA:
             uWAVE_AMB_DTA_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);
             break;
           case IC_D2H_DINFO:             
             uWAVE_DINFO_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);
             break;
         }
      }      
    }
    uWAVE_in_packet_ready = false;
  }
}

void setup() 
{ 
  Serial.begin(9600);
  while (!Serial) { }
  uWAVE_Init();
}

void loop() 
{
  uWAVE_Input_Process();
  
  if (uWAVE_out_packet_ready)
  {    
    uWAVEPort.write(uWAVE_out_packet, uWAVE_out_packet_idx);
    uWAVE_out_packet_ready = false;
  }

  if (!uWAVE_is_busy && uWAVE_is_cmd_mode)
  {
    if (!uWAVE_is_devInfo_updated)
    {
      Serial.println("\r\n[DINFO_GET]");
      uWAVE_DINFO_GET_Write();
    }
    else if (!uWAVE_settings_updated)
    {
      Serial.println("\r\n[SETTINGS_WRITE]");
      uWAVE_SETTINGS_WRITE_Write(0, 0, 0.0, false, false);
    }
    else if (!uWAVE_amb_settings_updated)
    {
      Serial.println("\r\n[AMB_DTA_CFG]");
      uWAVE_AMB_DTA_CFG_Write(false, 1, true, true, true, true);
    }
    else if (!uWAVE_waiting_for_remote)
    {
      if (requestID != RC_INVALID)
      {      
        Serial.println("\r\n[RC_REQUEST]");
        Serial.print(requestID);
        Serial.println(" ...");
        uWAVE_RC_REQUEST_Write(0, 0, requestID);

        rcID = (byte)requestID;
        rcID++;
        requestID = (RC_CODES_Enum)rcID;
      }
    }
  }

  if (uWAVE_is_busy && (millis() >= uWAVE_time + UWAVE_TIMEOUT_MS))
  {
    Serial.println("\r\nDevice timeout!");
    uWAVE_is_busy = false;    
    uWAVE_waiting_for_remote = false;
  }

  if (uWAVE_waiting_for_remote && (millis() >= uWAVE_rtime + UWAVE_REMOTE_TIMEOUT_MS))
  {
    uWAVE_waiting_for_remote = false;    
  }
}
