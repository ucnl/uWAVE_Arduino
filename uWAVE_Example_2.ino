
#include <SoftwareSerial.h>

#define UWAVE_CMD_PIN (4)
#define SOFTWARE_RX_PIN (2)
#define SOFTWARE_TX_PIN (3)

byte tByte;
bool uWAVE_is_cmd_mode;

SoftwareSerial uWAVEPort(SOFTWARE_RX_PIN, SOFTWARE_TX_PIN);

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
  uWAVE_SetCmdMode(false);
  delay(500);
  uWAVEPort.begin(9600);  
}

void setup() 
{ 
  Serial.begin(9600);
  while (!Serial) { }
  uWAVE_Init();
}

void loop() 
{  
  while (Serial.available())
  {
    tByte = Serial.read();
    uWAVEPort.write(tByte);    
  }

  while (uWAVEPort.available())
  {
    tByte = uWAVEPort.read();
    Serial.write(tByte);    
  }
  
  
}
