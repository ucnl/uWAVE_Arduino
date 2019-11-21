/* 
 *   Necessary hardware:
 *      - Arduino UNO board
 *      - uWAVE underwater acoustic modem
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
 *   Now you can use Arduino IDE's port monitor as a Chat terminal to chat with another uWAVE modem,
 *   which is suppose to be connected either the same way with another Arduino board or via UART-USB converter
 *   to a PC with any terminal software intalled.
 */

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
