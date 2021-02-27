// Libraries:-
#include <Wire.h>

// Defines:-
#define SLAVE_ADDRESS1 0x08
#define SLAVE_ADDRESS2 0x09

uint8_t address1Val = 0;
uint8_t address2Val = 0;
void setup()
{
    Serial.begin(115200);
    Wire.begin();                               // Join I2C bus.
}

void loop()
{

    Wire.requestFrom(SLAVE_ADDRESS1, 1);     // Request 2 bytes from the slave device.


    // Receive the 'int' from I2C and print it:-
    if (Wire.available() >= 1)                  // Make sure there are two bytes.
    {
        address1Val = Wire.read();
        Serial.println(address1Val,BIN);                 // Print the result.
    }

    Wire.requestFrom(SLAVE_ADDRESS2, 1);

    if (Wire.available() >= 1)                  // Make sure there are two bytes.
    {
        address2Val = Wire.read();
        Serial.println(address2Val,BIN);                 // Print the result.
    }
    Serial.println();

    //PID CONTROL
    delay(1000);
}
