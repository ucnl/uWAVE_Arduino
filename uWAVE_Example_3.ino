
/*
 *  This sketch parses $G*RMC sentence from a GNSS receiver,
 *  if GNSS position is defined, it requests a remote uWAVE modem by local uWAVE modem every 10 seconds
 *  and after receiving remote response (or remote timeout) send following message via RF link to the host:
 *  
 *  $PVLBL,ownLat,ownLon,ownDepth,ownBatV,targetDataID,targetDataValue,propagationTime,MSR
 *  
 *  where
 *    - ownLat - own latitude (from GNSS) in degrees as a signed real value 
 *    - ownLon - own longitude (from GNSS) in degrees as a signed real value 
 *    - ownDepth - local uWAVE's depth in meters
 *    - ownBatV - local uWAVE's supply voltage in volts
 *    - targetDataID, which can be one of the following values:
 *       2 - Depth
 *       3 - Temperature
 *       4 - Supply voltage
 *    - targetDataValue - the actual value of the specified parameter
 *    - propagationTime between the local and the remote uWAVEs, in seconds
 *    - MSR - Main lobe to side peak ratio in dB
 *    
 *  Example:
 *  
 *  $PVLBL,48.123456,44.123456,1.25,5.1,2,28.45,0.0123,24.6*6D<CR><LF>
 * 
 * 
 * 
 *  uWAVE modem wiring:
 *    uWAVE TX wire -> pin 3
 *    uWAVE RX wire -> pin 2
 *    uWAVE CMD wire -> pin 4
 *    
 *  GNSS wiring:
 *    GNSS TX wire -> pin 0
 *    
 *  RF module wiring:
 *    RF module RX wire -> pin 1 
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
#define UWAVE_REQUEST_PERIOD_MS   (10000)
#define GNSS_MAX_IN_PACKET_SIZE   (255)
#define MAX_OUT_PACKET_SIZE       (64)
#define GNSS_FIX_OBSOLETE_MS      (2000)

unsigned long uWAVE_request_time = 0;
unsigned long uWAVE_time = 0;
unsigned long uWAVE_rtime = 0;
bool uWAVE_is_cmd_mode = false;
bool uWAVE_is_devInfo_updated = false;
bool uWAVE_settings_updated = false;
bool uWAVE_amb_settings_updated = false;
bool uWAVE_is_busy = false;
bool uWAVE_waiting_for_remote = false;
bool uWAVE_response_received = false;


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

byte GNSS_in_packet[GNSS_MAX_IN_PACKET_SIZE];
byte GNSS_in_packet_idx = 0;
bool GNSS_in_packet_ready = false;
bool GNSS_in_packet_started = false;

byte out_packet[MAX_OUT_PACKET_SIZE];
byte out_packet_idx = 0;
byte out_packet_ready = false;

#define NMEA_SNT_STR '$'
#define NMEA_SNT_END '\n'
#define NMEA_PAR_SEP ','
#define NMEA_CHK_SEP '*'
#define NMEA_DATA_NOT_VALID 'V'
#define NMEA_SOUTH_SIGN 'S'
#define NMEA_WEST_SIGN 'W'

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

#define REQUEST_IDS_NUMBER 8
const RC_CODES_Enum requestIDs[REQUEST_IDS_NUMBER] = { RC_DPT_GET, RC_TMP_GET, RC_BAT_V_GET, RC_DPT_GET, RC_DPT_GET, RC_DPT_GET, RC_DPT_GET, RC_DPT_GET };
byte requestIDIdx = 0;

float ss_lat = UNDEFINED_FLOAT_VAL;
float ss_lon = UNDEFINED_FLOAT_VAL;
float ss_second;
byte ss_hour, ss_minute, ss_date, ss_month;
int ss_year;
unsigned long GNSS_fix_time = 0;

RC_CODES_Enum target_DataID = RC_INVALID;
float target_DataVal = UNDEFINED_FLOAT_VAL;
float target_pTime = UNDEFINED_FLOAT_VAL;
float target_msr = UNDEFINED_FLOAT_VAL;



// ****************************************** Utils
void fill_bytes(byte* buffer, byte size)
{  
  for (byte i = 0; i < size; i++)
     buffer[i] = 0;
}
// ******************************************



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
// ******************************************



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
// ******************************************



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
// ******************************************



// ****************************************** GNSS related items
#define C2B(b)             ((b - '0'))
#define CC2B(b1, b2)       ((10 * (b1 - '0') + (b2 - '0')))
#define CCC2B(b1, b2, b3)  ( (100 * (b1 - '0') + 10 * (b2 - '0') + (b3 - '0')))

void GNSS_RMC_Parse(const byte* buffer, byte size)
{
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;

  byte hour, minute, year, month, date;
  float second;
  float lat, lon, latSign, lonSign;

  bool result = true;
  bool isNotLastParam = true;
 
  do
  {        
    isNotLastParam = NMEA_GetNextParameter(buffer, lastDIdx, size, &stIdx, &ndIdx);
    stIdx++;
    ndIdx--;
    if (stIdx <= ndIdx)
    {      
      switch (pIdx)
      {
        case 1:    
          hour = CC2B(buffer[stIdx], buffer[stIdx + 1]);
          minute = CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
          second = Str_ParseFloat(buffer, stIdx + 4, ndIdx);
          result = (hour >= 0) && (hour <= 23) && (minute >= 0) && (minute <= 59) && (second >= 0) && (second < 60);
          break;
        case 2:
          result = (buffer[stIdx] != NMEA_DATA_NOT_VALID);          
          break;
        case 3:          
          lat = ((float)CC2B(buffer[stIdx], buffer[stIdx + 1])) + Str_ParseFloat(buffer, stIdx + 2, ndIdx) / 60.0f;         
          break;
        case 4:
          if (buffer[stIdx] == NMEA_SOUTH_SIGN) latSign = -1.0f;            
          else latSign = 1.0f;                      
          break;
        case 5:          
            lon = ((float)CCC2B(buffer[stIdx], buffer[stIdx + 1], buffer[stIdx + 2])) + Str_ParseFloat(buffer, stIdx + 3, ndIdx) / 60.0f;           
            break;
        case 6:
            if (buffer[stIdx] == NMEA_WEST_SIGN) lonSign = -1.0f;
            else lonSign = 1.0f;           
            break;
        case 9:
            date = CC2B(buffer[stIdx], buffer[stIdx + 1]);
            month = CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
            year = CC2B(buffer[stIdx + 4], buffer[stIdx + 5]);
            result = (date > 0) && (date <= 31) && (month > 0) && (month <= 12) && (year >= 00) && (year <=99);         
            break;
        case 12:        
            result = (buffer[stIdx] != NMEA_DATA_NOT_VALID);           
            break;      
        default:
          break;
      }
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (isNotLastParam);

  if (result)
  {
    lat *= latSign;
    lon *= lonSign;

    ss_lat = lat;
    ss_lon = lon;
    ss_year = year;
    ss_month = month;
    ss_date = date;
    ss_hour = hour;
    ss_minute = minute;
    ss_second = second;
    GNSS_fix_time = millis();
  }  
}
// ******************************************






// ******************************************  uWAVE-related items

// ******************************************  uWAVE sentences parsers
void uWAVE_ACK_Parse(const byte* buffer, byte size)
{
  // $PUWV0,sndID,errCode
  byte lastDIdx = 0;
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = false;

  byte errCode = 255;
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
        errCode = (byte)Str_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result); 

  uWAVE_is_busy = false;

  if (errCode == 0)
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
        if (stIdx <= ndIdx)
        {
          value = Str_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 6:
        if (stIdx <= ndIdx)
        {
          azimuth = Str_ParseFloat(buffer, stIdx, ndIdx);              
        }
        break;
    }

    lastDIdx = ndIdx + 1;
    pIdx++;
  } while (result); 

  uWAVE_waiting_for_remote = false;
  uWAVE_request_time = millis();

  target_pTime = pTime;
  target_DataID = rcCmdID;
  target_DataVal = value;
  target_msr = msr;

  uWAVE_response_received = true;
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

  target_DataID = RC_INVALID;
  target_DataVal = UNDEFINED_FLOAT_VAL;
  target_pTime = UNDEFINED_FLOAT_VAL;
  target_msr = UNDEFINED_FLOAT_VAL;

  uWAVE_response_received = true;
  uWAVE_waiting_for_remote = false;
  uWAVE_request_time = 0;
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
    if (stIdx <= ndIdx)
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
}




// ******************************************  necessary uWAVE sentences writers
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
  if (uWAVEPort.available())
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
           case IC_D2H_AMB_DTA:
             uWAVE_AMB_DTA_Parse(uWAVE_in_packet, uWAVE_in_packet_idx);
             break;           
         }
      }      
    }
    uWAVE_in_packet_ready = false;
  }
}

void GNSS_Input_Process()
{
  if (Serial.available())
  {
    byte newByte = Serial.read();
    NMEA_ProcessByte(GNSS_in_packet, GNSS_MAX_IN_PACKET_SIZE, &GNSS_in_packet_ready, &GNSS_in_packet_started, &GNSS_in_packet_idx, newByte);    
  }

  if (GNSS_in_packet_ready)
  {    
    if (NMEA_Checksum_Check(GNSS_in_packet, GNSS_in_packet_idx))
    {
      // look for '$G*RMC,...'
      if ((GNSS_in_packet[3] == 'R') &&
          (GNSS_in_packet[4] == 'M') &&
          (GNSS_in_packet[5] == 'C'))
      {  
        GNSS_RMC_Parse(GNSS_in_packet, GNSS_in_packet_idx);
      }
    }
    GNSS_in_packet_ready = false;
  }
}

// ******************************************************************

void PVLBL_Write(float ownLat, float ownLon, float ownDpt, float ownBatV, RC_CODES_Enum targetDataID, float targetDataVal, float pTime, float msr)
{
  out_packet_idx = 0;
  fill_bytes(out_packet, MAX_OUT_PACKET_SIZE);
  Str_WriteString(out_packet, &out_packet_idx, (byte*)"$PVLBL,\0");

  if (ownLat != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, ownLat, 6, 0);  
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);

  if (ownLon != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, ownLon, 6, 0);
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);

  if (ownDpt != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, ownDpt, 2, 0);
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);

  if (ownBatV != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, ownBatV, 1, 0);
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);

  if (targetDataID != RC_INVALID)
    Str_WriteIntDec(out_packet, &out_packet_idx, targetDataID, 0);    
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);
  
  if (targetDataVal != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, targetDataVal, 3, 0);    
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);

  if (pTime != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, pTime, 4, 0);  
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_PAR_SEP);

  if (msr != UNDEFINED_FLOAT_VAL)
    Str_WriteFloat(out_packet, &out_packet_idx, msr, 4, 0);  
  Str_WriteByte(out_packet, &out_packet_idx, NMEA_CHK_SEP);

  Str_WriteHexByte(out_packet, &out_packet_idx, 0);
  Str_WriteString(out_packet, &out_packet_idx, (byte*)"\r\n");
  NMEA_CheckSum_Update(out_packet, out_packet_idx);
  
  out_packet_ready = true;
}




// ******************************************************************

void setup()
{
  Serial.begin(9600);
  while (!Serial) { }
  uWAVE_Init();
}

void loop() 
{
  // Process incoming messages from the local uWAVE modem
  uWAVE_Input_Process();

  // Process incoming messages from the GNSS receiver
  GNSS_Input_Process();

  // If packet to be sent to the local uWAVE modem is ready, send it 
  if (uWAVE_out_packet_ready)
  {    
    uWAVEPort.write(uWAVE_out_packet, uWAVE_out_packet_idx);
    uWAVE_out_packet_ready = false;
  }

  // If packet to be sent via RF link to the host system is ready, send it
  if (out_packet_ready)
  {
    Serial.write(out_packet, out_packet_idx);
    out_packet_ready = false;
  }

  if (!uWAVE_is_busy && uWAVE_is_cmd_mode)
  {
    if (!uWAVE_settings_updated)
    {      
      uWAVE_SETTINGS_WRITE_Write(0, 0, 0.0, false, false);
    }
    else if (!uWAVE_amb_settings_updated)
    {      
      uWAVE_AMB_DTA_CFG_Write(false, 1, false, false, true, true);
    }
    else if ((!uWAVE_waiting_for_remote) && 
             (millis() < GNSS_fix_time + GNSS_FIX_OBSOLETE_MS) &&
             (millis() >= uWAVE_request_time + UWAVE_REQUEST_PERIOD_MS))
    {      
        uWAVE_RC_REQUEST_Write(0, 0, requestIDs[requestIDIdx]);
        requestIDIdx = (requestIDIdx + 1) % REQUEST_IDS_NUMBER;
    }
  }

  if (uWAVE_is_busy && (millis() >= uWAVE_time + UWAVE_TIMEOUT_MS))
  {    
    uWAVE_is_busy = false;    
    uWAVE_waiting_for_remote = false;
  }

  if (uWAVE_waiting_for_remote && (millis() >= uWAVE_rtime + UWAVE_REMOTE_TIMEOUT_MS))
  {
    uWAVE_waiting_for_remote = false;
    uWAVE_request_time = 0;
  }

  if (uWAVE_response_received)
  {
    PVLBL_Write(ss_lat, ss_lon, uWAVE_amb_dpt_m, uWAVE_amb_batV, target_DataID, target_DataVal, target_pTime, target_msr);    
    uWAVE_response_received = false;
  }
}
