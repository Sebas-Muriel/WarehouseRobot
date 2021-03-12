#include <QTRSensors.h>
#include <Wire.h>
QTRSensors qtr;

#define SLAVE_ADDRESS 0x09
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint8_t value = 5;


uint8_t readSensors(uint16_t values[]);
void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 10}, SensorCount);
  qtr.setEmitterPin(10);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(I2CrequestHandler);
  Serial.begin(9600);
}


void loop()
{
  // read raw sensor values
  qtr.read(sensorValues);
  value = readSensors(sensorValues);
  

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.print(value, BIN);
  Serial.println();
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
  Wire.write((byte*)&value,1);
}
