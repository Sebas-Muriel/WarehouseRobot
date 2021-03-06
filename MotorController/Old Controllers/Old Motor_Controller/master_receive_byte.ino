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

#define nodeMode 0b00000000
#define tickMode 0b00000001

uint8_t address1Val = 0;
uint8_t address2Val = 0;
uint8_t intersectionFLG = 0;
uint8_t tickIntersectionFLG = 0;
time_t current, prev;

char dir = '0';
char mode = 0x10;
char reset = '0';
uint8_t data;


void convertSerialRead(char);
void setup()
{
    Serial.begin(9600);
    Wire.begin();                               // Join I2C bus.
    //pinMode(13, OUTPUT);
}

void loop()
{

  if (mode == 0x40)
  {
    while(1)
    {
      tickIntersectionFLG = 0;
      intersectionFLG = 0;
      dir = '0';
      Serial.println("Reset Mode");
      if (Serial.available() >= 1)
      {
        data = Serial.read();
        dir = data&0b00001110;
        mode = data&0b01000001;
        reset = data&0b10000000;
      }
      if (mode == 0b00000000 || mode == 0b00000001)
      {
        Serial.println("20");
        break;
      }
    }
  }
    


  
    Wire.requestFrom(SLAVE_ADDRESS1, 1);     

    // Receive the 'int' from I2C and print it:-
    if (Wire.available() >= 1){
        address1Val = Wire.read();
        Serial.println(address1Val,BIN);               
    }

    Wire.requestFrom(SLAVE_ADDRESS2, 1);

    if (Wire.available() >= 1){
        address2Val = Wire.read();
        Serial.println(address2Val,BIN);                 
    }

    if (mode == nodeMode && reset != 0b10000000)
    {
      //If there is an intersection send a 1 to the py if not send a 0
      if ((address2Val == 0xFF || address1Val == 0xFF) && intersectionFLG == 0){
        //Stop movement
        intersectionFLG = 1;
        Serial.println("1");
      }
      else{
        Serial.println("0");
        //dir = '0';
      }
  
      if (intersectionFLG == 1)
      {
        if (Serial.available() >= 1)
        { 
          data = Serial.read();
          dir = data&0b00001110;
          mode = data&0b01000001;
          reset = data&0b10000000;
        }
      }
  
      if (address2Val != 0xFF){
        intersectionFLG = 0;
      }
      Serial.println("IN NODE MODE");
    }
    else if (mode == tickMode && reset != 0b10000000)
    {
      if ((address2Val == 0b11110000 || address2Val == 0b11111000) && tickIntersectionFLG == 0)
      {
        tickIntersectionFLG = 1;
        Serial.println("1");
      }
      else
      {
        Serial.println("0");
      }

      if (tickIntersectionFLG == 1)
      {
        if (Serial.available() >= 1)
        { 
          data = Serial.read();
          dir = data&0b00001110;
          mode = data&0b01000001;
          reset = data&0b10000000;

          
        }
      }

        
      if (address2Val != 0xF8 && address2Val != 0xF0){
        tickIntersectionFLG = 0;
      }
      Serial.println("IN TICK MODE");
    }

    if (Serial.available() >= 1)
    {
      data = Serial.read();
      dir = data&0b00001110;
      mode = data&0b01000001;
      reset = data&0b10000000;
    }
    Serial.print("Mode: ");
    Serial.println(mode);
    //PID CONTROL
    //delay(2000);
}
