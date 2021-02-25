#include <Wire.h>
#include <SoftwareSerial.h>

#define SLAVE_ADDRESS 0x04
SoftwareSerial softSerial(8, 9);  // RX, TX 
int number = 0;
int whatToSend = 0;

void setup()
{
  Wire.begin(SLAVE_ADDRESS);     // join i2c bus with address #4
  Serial.begin(9600);           // start serial for output
  softSerial.begin(9600);
  Wire.onReceive(recieveData); // register event
  Wire.onRequest(sendData);

  Serial.print("Ready");
}

void loop()
{
  softSerial.write(5); 
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receieveData(int byteCount)
{
  while(Wire.available() // loop through all but the last
  {
    number = Wire.read(); // receive byte as a character
    Serial.print("Data Recieved");         // print the character
    Serial.println(number);

    if (number == 1){
      whatToSend = 5;
      //get IR data
    }
  }
}

void sendData()
{
  Wire.write(whatToSend);
}
