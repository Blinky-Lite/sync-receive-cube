#include <SPI.h>
#include "RH_RF95.h"

#define BAUD_RATE 9600
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
 
boolean printDiagnostcs = false;
boolean testPulses = false;

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};

RH_RF95 rf95(RFM95_CS, RFM95_INT);
unsigned long newTime = 0;
unsigned long lastWriteTime = 0;
int channelPin[] = {11,10,6,5};
boolean channelHigh[] = {false, false, false, false};
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
byte event = 0;
boolean pin12Value = true;

struct TransmitData
{
  int signalStrength;
};
struct ReceiveData
{
  int statusLedChannel = 0;
  float rfFreq = 433.3;
  int transAddr = 23;
  int modemConfigIndex = 1;
  unsigned long channelBeginTime[4];
  unsigned long channelEndTime[4];
  byte channelStateMask[4];
};

void setupPins(TransmitData* tData, ReceiveData* rData)
{
  if (printDiagnostcs) Serial.begin(9600);
  pinMode(12, OUTPUT);
  digitalWrite(12, pin12Value);
  pinMode(9, OUTPUT);
  digitalWrite(9, false);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  for( int ic = 0; ic < 4; ++ic ) pinMode(channelPin[ic], OUTPUT);
  for( int ic = 0; ic < 4; ++ic ) digitalWrite(channelPin[ic], LOW);
 
  delay(200);
   // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(rData->rfFreq);
  rf95.setModemConfig(modeConfig[rData->modemConfigIndex]); 
  rf95.setModeRx();

  for (int ic = 0; ic < 4; ++ic)
  {
    rData->channelBeginTime[ic] = 1000;
    rData->channelEndTime[ic] = 2000;
    rData->channelStateMask[ic] = 0;
  }
  if (testPulses)
  {
  // Test data
    rData->channelStateMask[0] = 31;
    rData->channelBeginTime[0] = 1000;
    rData->channelEndTime[0] = 6000;
  }

}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
  rData->statusLedChannel = newData->statusLedChannel;
  rData->transAddr = newData->transAddr;
  
  for (int ic = 0; ic < 4; ++ic)
  {
    rData->channelBeginTime[ic] = newData->channelBeginTime[ic];
    rData->channelEndTime[ic] = newData->channelEndTime[ic];
    rData->channelStateMask[ic] = newData->channelStateMask[ic];
  }

  if (newData->rfFreq != rData->rfFreq)
  {
    rData->rfFreq = newData->rfFreq;
    rf95.setFrequency(rData->rfFreq);
  }

  if (newData->modemConfigIndex != rData->modemConfigIndex)
  {
    rData->modemConfigIndex = newData->modemConfigIndex;
    rf95.setModemConfig(modeConfig[rData->modemConfigIndex]); 
  }

  if (printDiagnostcs) 
  {
    Serial.print("rfFreq: ");
    Serial.println(rData->rfFreq);
    Serial.print("modemConfigIndex: ");
    Serial.println(rData->modemConfigIndex);
    Serial.print("statusLedChannel: ");
    Serial.println(rData->statusLedChannel);
    Serial.print("transAddr: ");
    Serial.println(rData->transAddr);
    Serial.print("channelBeginTime1: ");
    Serial.println(rData->channelBeginTime[0]);
    Serial.print("channelEndTime1: ");
    Serial.println(rData->channelEndTime[0]);
    Serial.print("channelStateMask1: ");
    Serial.println(rData->channelStateMask[0]);
    Serial.println();
  }
}
boolean processData(TransmitData* tData, ReceiveData* rData)
{
  boolean timeLineRestart = false;
  newTime = micros();
  unsigned long deltaT = newTime - lastWriteTime;
  if (rf95.recv(buf, &len))
  {
    if (rData->transAddr == (byte)buf[0])
    {
      newTime = rf95.packetTime;
      lastWriteTime = newTime;
      deltaT = 0;
      event = (byte) buf[1];
      if ((byte)buf[2] == 1)
      {
        pin12Value = !pin12Value;
        digitalWrite(12, pin12Value);
        tData->signalStrength = rf95.lastRssi();
        timeLineRestart = true;
//        if (printDiagnostcs) Serial.println("Timeline Restart");
      }
    }
  }
  for (int ic = 0; ic < 4; ++ic)
  {
    channelHigh[ic] = false;
    if ( (event & rData->channelStateMask[ic]) > 0)
    {
      if ((rData->channelBeginTime[ic] <= deltaT) && (deltaT <= rData->channelEndTime[ic]))
      {
        channelHigh[ic] = true;
      }
    }
    digitalWrite(channelPin[ic], channelHigh[ic]);
  }  
  digitalWrite(9, channelHigh[rData->statusLedChannel]);
  return timeLineRestart;
}

const int microLEDPin = 13;
const int commLEDPin = 13;
boolean commLED = true;

struct TXinfo
{
  int cubeInit = 1;
  int newSettingDone = 0;
};
struct RXinfo
{
  int newSetting = 0;
};

struct TX
{
  TXinfo txInfo;
  TransmitData txData;
};
struct RX
{
  RXinfo rxInfo;
  ReceiveData rxData;
};
TX tx;
RX rx;
ReceiveData settingsStorage;

int sizeOfTx = 0;
int sizeOfRx = 0;
unsigned long lastSerialCheck = micros();

void setup()
{
  setupPins(&(tx.txData), &settingsStorage);
  pinMode(microLEDPin, OUTPUT);    
  pinMode(commLEDPin, OUTPUT);  
  digitalWrite(commLEDPin, commLED);
//  digitalWrite(microLEDPin, commLED);

  sizeOfTx = sizeof(tx);
  sizeOfRx = sizeof(rx);
  Serial1.begin(BAUD_RATE);
}
void loop()
{
  processData(&(tx.txData), &settingsStorage);
  if ((micros() - lastSerialCheck) > 1000000)
  {
//    Serial.println("Checking for data");
    lastSerialCheck = micros();
    tx.txInfo.newSettingDone = 0;
    if(Serial1.available() > 0)
    { 
//      Serial.println("Data Available");
      commLED = !commLED;
      digitalWrite(commLEDPin, commLED);
      Serial1.readBytes((uint8_t*)&rx, sizeOfRx);
      
      if (rx.rxInfo.newSetting > 0)
      {
//        Serial.println("New Setting");
        processNewSetting(&(tx.txData), &settingsStorage, &(rx.rxData));
        tx.txInfo.newSettingDone = 1;
        tx.txInfo.cubeInit = 0;
      }
    }
    Serial1.write((uint8_t*)&tx, sizeOfTx);
  }
  
}
