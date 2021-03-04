#include <Time.h>
#include <TimeLib.h>
// Libraries:-
#include <Wire.h>

// Defines:-
#define SLAVE_ADDRESS1 0x08
#define SLAVE_ADDRESS2 0x09

uint8_t address1Val = 0;
uint8_t address2Val = 0;
uint8_t sender = 0;
time_t current, prev;
uint8_t response;
void setup()
{
    Serial.begin(9600);
    Wire.begin();                               // Join I2C bus.
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
    if (address2Val == 0xFF || address1Val == 0xFF){
      //Stop movement
      
      //busy wait for the pi to send back an ack
      while(response != sender){
        prev = now();
        sender = 1;
        Serial.print(sender);
        if (Serial.available() > 0){
          response = Serial.read();
            if (response == sender) break; 
        }
        //wait three senconds before sending again
        while( (second(current) - second(prev)) >= 3){
          current = now();
        }
        
      }
    }
    else{
      sender = 0;
      Serial.print(sender);
    }
    

    //PID CONTROL
    delay(1000);
}
