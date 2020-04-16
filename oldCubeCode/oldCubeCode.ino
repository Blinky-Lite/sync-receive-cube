#include <SPI.h>
#include "RH_RF95.h"
 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define LED 13
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.775
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

String appName = "Clock Receiver V3";
unsigned long startTime = 0;
unsigned long deltaTime = 0;
unsigned long maxTime = 0;
int channelPin[] = {11,10,6,5};
unsigned long channelBeginTime[] = {1000, 1000, 1000, 1000};
unsigned long channelEndTime[] = {2000, 2000, 2000, 2000};
byte channelStateMask[] = {0, 0, 0, 0};
boolean channelActive[] = {false, false, false, false};
boolean channelHigh[] = {false, false, false, false};
byte transAddr = 23;
byte stateMask = 0; 
int ic;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
int signalStrength;
boolean goodCommand = false;
boolean validData = false;

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(LED, HIGH);     
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  for( ic = 0; ic < 4; ++ic ) pinMode(channelPin[ic], OUTPUT);
  for( ic = 0; ic < 4; ++ic ) digitalWrite(channelPin[ic], LOW);
 
  Serial.begin(9600);
  delay(200);
   // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128); 
  rf95.setModeRx();
}

void loop() 
{
  if (rf95.available())
  {
    if (rf95.recv(buf, &len))
    {
      startTime = micros();
      if (transAddr == (byte)buf[0])
      {
        maxTime = 0;
        for( ic = 0; ic < 4; ++ic )
        {
          if ( (((byte)buf[1]) & channelStateMask[ic]) > 0)
          {
            channelActive[ic] = true;
            channelHigh[ic] = false;
            if (maxTime < channelEndTime[ic]) maxTime = channelEndTime[ic];
          }
        }
        maxTime = maxTime + 1000;
        deltaTime = 0;
        while (deltaTime < maxTime)
        {
          deltaTime = micros() - startTime;
          for( ic = 0; ic < 4; ++ic )
          {
            if (channelActive[ic])
            {
              if (!channelHigh[ic])
              {
                if (deltaTime > channelBeginTime[ic])
                {
                  digitalWrite(channelPin[ic], HIGH);
                  channelHigh[ic] = true;
                  digitalWrite(12, HIGH);
                }
              }
              else
              {
                if (deltaTime > channelEndTime[ic])
                {
                  digitalWrite(channelPin[ic], LOW);
                  digitalWrite(12, LOW);
                }
              }
            }
          }
        }
        for( ic = 0; ic < 4; ++ic )
        {
          channelActive[ic] = false;
          digitalWrite(channelPin[ic], LOW);
        }
      }
      checkSerial();  
    }
//    signalStrength = (int)rf95.lastRssi();
//    rf95.setModeRx();

  }
}
void checkSerial()
{
  if (Serial.available())
  {
    String serialRead = readSerial();
    goodCommand = false;
    if (serialRead.lastIndexOf("aboutGet")      > -1) 
    {
      echoStringSetting("aboutGet", appName);
      goodCommand = true;
    }
/*    
    if (serialRead.lastIndexOf("signalGet")     > -1) 
    {
      echoIntSetting("signalGet", signalStrength);
      goodCommand = true;
    }
*/
    if (serialRead.lastIndexOf("channelSet")    > -1) 
    {
      validData = readChannelSetting(serialRead);
      if (validData)
      {
        printStringToSerial("Success: " + serialRead + "\n");
      }
      else
      {
        printStringToSerial("Error Bad Data: " + serialRead + "\n");
      }
      goodCommand = true;
    }
    if (serialRead.lastIndexOf("addressSet")      > -1) 
    {

      transAddr = (byte) newIntSetting("addressSet", serialRead, 0, 255);
      printStringToSerial("Success: " + serialRead + "\n");
      goodCommand = true;
    }
    if (!goodCommand)
    {
      printStringToSerial("Error: " + serialRead + "\n");
      delay(100);
      Serial.end();
      delay(1);
      Serial.begin(9600);
    }
  }
}
String readSerial()
{
  String inputString = "";
  while(Serial.available() > 0)
  {
    char lastRecvd = Serial.read();
    delay(1);
    if (lastRecvd == '\n'){return inputString;} else {inputString += lastRecvd;}
  }
  return inputString;
}
void echoStringSetting(String getParse, String setting)
{
  printStringToSerial(getParse + " " + setting + "\n");
}
void echoIntSetting(String getParse, int setting)
{
  printStringToSerial(getParse + " " + String(setting) + "\n");
}
int newIntSetting(String setParse, String input, int low, int high)
{
   String newSettingString = input.substring(input.lastIndexOf(setParse) + setParse.length() + 1,input.length());
   int newSetting = constrain(newSettingString.toInt(),low,high);
   return newSetting;
}
void printStringToSerial(String inputString)
{
  Serial.print(inputString);
  Serial.flush();
}
boolean readChannelSetting(String serialRead)
{
  String arg;
  String remainder;
  int i1;
  remainder = serialRead;
  remainder.trim();
  i1 = remainder.indexOf(" ");
  if (i1 < 0) return false;
  remainder = remainder.substring(i1);
  remainder.trim();
  i1 = remainder.indexOf(" ");
  if (i1 < 0) return false;
  arg = remainder.substring(0, i1);
  arg.trim();
  int channel = arg.toInt();
  remainder = remainder.substring(i1);
  remainder.trim();
  i1 = remainder.indexOf(" ");
  if (i1 < 0) return false;
  arg = remainder.substring(0, i1);
  arg.trim();
  int byteMask = arg.toInt();
  remainder = remainder.substring(i1);
  remainder.trim();
  i1 = remainder.indexOf(" ");
  if (i1 < 0) return false;
  arg = remainder.substring(0, i1);
  arg.trim();
  int startTime = arg.toInt();
  remainder = remainder.substring(i1);
  remainder.trim();
  i1 = remainder.indexOf(" ");
  int endTime;
  if (i1 < 0)
  {
    endTime = remainder.toInt();
  }
  else
  {
    arg = remainder.substring(0, i1);
    arg.trim();
    endTime = arg.toInt();
  }
// Make sure times are multiples of 4 uS
  startTime = startTime / 4;
  endTime = endTime / 4;
  startTime = startTime * 4;
  endTime = endTime * 4;
  if (channel < 1) return false;
  if (channel > 4) return false;
  if (startTime < 1) return false;
  if (endTime < startTime) return false;
  if (byteMask < 0) return false;
  if (byteMask > 255) return false;
  channelBeginTime[channel - 1] = startTime;
  channelEndTime[channel - 1] = endTime;
  channelStateMask[channel - 1] = (byte) byteMask;
  
  return true;
}
