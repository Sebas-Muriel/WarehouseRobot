  // Include the Software Serial library 
  #include <SoftwareSerial.h>
  // Define a Software Serial object and the used pins 
  SoftwareSerial softSerial(8, 9);
  
  void setup()  { 
      softSerial.begin(9600); 
      Serial.begin(9600);
  } 
  void loop()  { 
    //Check if there is anything in the soft Serial Buffer 
    if (softSerial.available())  { 
      int rec = softSerial.read();
      Serial.print(rec);
    } 
  }
