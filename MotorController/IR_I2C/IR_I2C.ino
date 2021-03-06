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
//  qtr.setSensorPins((const uint8_t[]){51, 49, 47, 45, 43, 41, 39, 37}, SensorCount);
  qtr.setSensorPins((const uint8_t[]){37, 39, 41, 43, 45, 47, 49, 51}, SensorCount);
  qtr.setEmitterPin(53);

  qtr2.setTypeRC();
//  qtr2.setSensorPins((const uint8_t[]){50, 48, 46, 44, 42, 40, 38, 36}, SensorCount);
  qtr2.setSensorPins((const uint8_t[]){36, 38, 40, 42, 44, 46, 48, 50}, SensorCount);
  qtr2.setEmitterPin(52);

  qtr3.setTypeRC();
//  qtr3.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5, A6, A7, A8}, SensorCount);
  qtr3.setSensorPins((const uint8_t[]){A8, A7, A6, A5, A4, A3, A2, A1}, SensorCount);
  qtr3.setEmitterPin(A0);


  qtr4.setTypeRC();
//  qtr4.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount);
  qtr4.setSensorPins((const uint8_t[]){10, 9, 8, 7, 6, 5, 4, 3}, SensorCount);
  qtr4.setEmitterPin(11);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(I2CrequestHandler);
  Serial.begin(9600);
  
//  pinMode(LED_BUILTIN, OUTPUT);
//  digitalWrite(LED_BUILTIN, HIGH);
//  for (uint16_t i = 0; i < 400; i++)
//  {
//    qtr.calibrate();
//    qtr2.calibrate();
//    qtr3.calibrate();
//    qrt4.calibrate();
//  }
//  digitalWrite(LED_BUILTIN, LOW);
}


void loop()
{
  // read raw sensor values
  qtr.read(sensorValuesUp);
  qtr2.read(sensorValuesDown);
  qtr3.read(sensorValuesLeft);
  qtr4.read(sensorValuesRight);
  
  I2Cbuffer[0] = readSensors(sensorValuesUp);
  I2Cbuffer[1] = readSensors(sensorValuesDown);
  I2Cbuffer[2] = readSensors(sensorValuesLeft);
  I2Cbuffer[3] = readSensors(sensorValuesRight);
  

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
//  Serial.println("\nUp:\t");
//  for (uint8_t i = 0; i < SensorCount; i++)
//    {Serial.print(sensorValuesUp[i]); Serial.print('\t');}
//  Serial.println();
//  Serial.println(I2Cbuffer[0], BIN);
//  
//   
//  Serial.println("\nDown:\t");
//  for (uint8_t i = 0; i < SensorCount; i++)
//    { Serial.print(sensorValuesDown[i]); Serial.print('\t');}
//  Serial.println();
//  Serial.println(I2Cbuffer[1], BIN);
//  
//  
//  Serial.println("\nLeft:\t");
//  for (uint8_t i = 0; i < SensorCount; i++)
//    { Serial.print(sensorValuesLeft[i]); Serial.print('\t');}
//  Serial.println();             
//  Serial.println(I2Cbuffer[2], BIN);
//  
//  Serial.println("\nRight:\t");
//  for (uint8_t i = 0; i < SensorCount; i++)
//    { Serial.print(sensorValuesRight[i]); Serial.print('\t');}
//  Serial.println();
//  Serial.println(I2Cbuffer[3], BIN);
//  delay(500);
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
    if (values[i] >= 2500)
      returnValue += adder;
  }
  return returnValue;
}

void I2CrequestHandler()
{
  Wire.write(I2Cbuffer,4);
}
