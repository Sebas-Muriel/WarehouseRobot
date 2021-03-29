#include <QTRSensors.h>
#include <Wire.h>
QTRSensors qtr;
QTRSensors qtr2;
QTRSensors qtr3;
QTRSensors qtr4;

#define SLAVE_ADDRESS 0x09
const uint8_t SensorCount = 8;
uint16_t sensorValuesUp[SensorCount], sensorValuesDown[SensorCount], sensorValuesLeft[SensorCount], sensorValuesRight[SensorCount];
byte I2Cbuffer[4] = {0,0,0,0};


uint8_t readSensors(uint16_t values[]);
void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  qtr.setEmitterPin(10);

//  qtr2.setTypeRC();
//  qtr2.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A6, A7, 11, 12}, SensorCount);
//  qtr2.setEmitterPin(13);

//  qtr3.setTypeRC();
//  qtr3.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A6, A7, 11, 12}, SensorCount);
//  qtr3.setEmitterPin(13);

//  qtr4.setTypeRC();
//  qtr4.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A6, A7, 11, 12}, SensorCount);
//  qtr4.setEmitterPin(13);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(I2CrequestHandler);
  Serial.begin(9600);
}


void loop()
{
  // read raw sensor values
  qtr.read(sensorValuesUp);
//  qtr2.read(sensorValuesDown);
//  qtr3.read(sensorValuesLeft);
//  qtr4.read(sensorValuesRight);
  
  I2Cbuffer[0] = readSensors(sensorValuesUp);
  //I2Cbuffer[1] = readSensors(sensorValuesDown);
  //I2Cbuffer[2] = readSensors(sensorValuesLeft);
  //I2Cbuffer[3] = readSensors(sensorValuesRight);
  

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
    {Serial.print("Up:\t"); Serial.print(sensorValuesUp[i]); Serial.println('\t');}
  for (uint8_t i = 0; i < SensorCount; i++)
    {Serial.print("Down:\t"); Serial.print(sensorValuesDown[i]); Serial.println('\t');}
  for (uint8_t i = 0; i < SensorCount; i++)
    {Serial.print("Left:\t"); Serial.print(sensorValuesLeft[i]); Serial.println('\t');}
  for (uint8_t i = 0; i < SensorCount; i++)
    {Serial.print("Right:\t"); Serial.print(sensorValuesRight[i]); Serial.println('\t');}
    
//  Serial.println();
//  Serial.print(I2Cbuffer[0], BIN);
//  Serial.println();
  delay(100);

}

uint8_t readSensors(uint16_t values[])
{
  uint8_t returnValue = 0x00;
  uint8_t adder = 0x00;
  for (int i =0; i < 8; i++)
  {
    switch(i)
    {
      case 0: adder = 0x01; break;
      case 1: adder = 0x02; break;
      case 2: adder = 0x04; break;
      case 3: adder = 0x08; break;
      case 4: adder = 0x10; break;
      case 5: adder = 0x20; break;
      case 6: adder = 0x40; break;
      case 7: adder = 0x80; break;
    }
    if (values[i] > 2400)
      returnValue += adder;
  }
  return returnValue;
}

void I2CrequestHandler()
{
  Wire.write(I2Cbuffer,4);
}
