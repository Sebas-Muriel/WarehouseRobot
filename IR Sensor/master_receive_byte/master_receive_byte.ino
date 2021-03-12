#include <Time.h>
#include <TimeLib.h>
// Libraries:-
#include <Wire.h>

// Defines:-
#define SLAVE_ADDRESS1 0x08
#define SLAVE_ADDRESS2 0x09

#define Left 0x01
#define Right 0x02
#define Up 0x03
#define Down 0x04
#define Stop 0x00


uint8_t address1Val = 0;
uint8_t address2Val = 0;
uint8_t intersectionFLG = 0;
time_t current, prev;

char dir = '0';

void setup()
{
    Serial.begin(9600);
    Wire.begin();                               // Join I2C bus.
    pinMode(13, OUTPUT);
}

void loop()
{

    Wire.requestFrom(SLAVE_ADDRESS1, 1);     


    // Receive the 'int' from I2C and print it:-
    if (Wire.available() >= 1){
        address1Val = Wire.read();
        //Serial.println(address1Val,BIN);               
    }

    Wire.requestFrom(SLAVE_ADDRESS2, 1);

    if (Wire.available() >= 1){
        address2Val = Wire.read();
        //Serial.println(address2Val,BIN);                 
    }

    //If there is an intersection send a 1 to the py if not send a 0
    if ((address2Val == 0xFF || address1Val == 0xFF) && intersectionFLG == 0){
      //Stop movement
      intersectionFLG = 1;
      //dir = Stop;
      Serial.println("1");
    }
    else{
      Serial.println("0");
      dir = '0';
    }

    if (intersectionFLG == 1)
    {
      if (Serial.available()> 0)
      {
        dir = Serial.read();
        Serial.println(dir);
      }
    }

    if (dir == 35){
      digitalWrite(13, LOW);
    }
    else
    {
      digitalWrite(13, HIGH);
    }

    if (address2Val != 0xFF){
      intersectionFLG = 0;
    }
    

    //PID CONTROL
    delay(2000);
}
