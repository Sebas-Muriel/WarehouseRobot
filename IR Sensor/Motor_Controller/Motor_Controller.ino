#include <Time.h>
#include <TimeLib.h>
// Libraries:-
#include <Wire.h>

//Slave Addresses for I2C
#define SLAVE_ADDRESS1 0x08
#define SLAVE_ADDRESS2 0x09

//Defines for movement
#define STOP 0x00
#define LEFT 0X01
#define RIGHT 0X02
#define UP 0x03
#define DOWN 0x04

//Defines for Modes
#define RESET_MODE 0x00
#define NODE_MODE 0x01
#define TICK_MODE 0x02

//defines for pins
#define MRDY 0x02
#define SRDY 0x03

#define LED_LEFT 10
#define LED_RIGHT 11
#define LED_UP 12
#define LED_DOWN 13
#define LED_MODE_BIT0 8
#define LED_MODE_BIT1 9

uint8_t address1Val = 0;
uint8_t address2Val = 0;
uint8_t NodeIntersectionFLG = 1, TickIntersectionFLG = 1;
time_t current, prev;

uint8_t dir = STOP;
uint8_t mode = RESET_MODE;
uint8_t UART_RX;
uint8_t MRDY_VAL;
uint8_t sending = 0, recieving = 0;


void getIRData(int SLAVE_ADDRESS, int rec_data);
void UART_REC();
void UART_TX(uint8_t message);
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(MRDY, INPUT);
  attachInterrupt(digitalPinToInterrupt(MRDY), UART_REC, FALLING);
  pinMode(SRDY, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_UP, OUTPUT);
  pinMode(LED_DOWN, OUTPUT);
  pinMode(LED_MODE_BIT0, OUTPUT);
  pinMode(LED_MODE_BIT1, OUTPUT);
  LED_ALL_OFF();
  digitalWrite(SRDY, HIGH);
}

void loop() { 
  LEDControl();
  
 // Get IR sensor Data
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
  if (mode == RESET_MODE)
  {
    dir = STOP;
    NodeIntersectionFLG = 1;
    TickIntersectionFLG = 1;
  }
  else if(mode == NODE_MODE)
  {
    if ((address2Val == 0xFF || address1Val == 0xFF) && NodeIntersectionFLG == 0){
      NodeIntersectionFLG = 1;
      UART_TX('1');
    }

    if (address2Val != 0xFF){
      NodeIntersectionFLG = 0;
    }
  }
  else if(mode == TICK_MODE)
  {
     if ((address2Val == 0b11110000 || address2Val == 0b11111000) && TickIntersectionFLG == 0)
      {
        TickIntersectionFLG = 1;
        UART_TX('1');
      }
        
      if (address2Val != 0xF8 && address2Val != 0xF0){
        TickIntersectionFLG = 0;
      }
  }

}

void UART_REC()
{
  digitalWrite(LED_MODE_BIT0, LOW);
  digitalWrite(LED_MODE_BIT1, LOW);
  
  digitalWrite(SRDY, LOW);
  
  while(digitalRead(MRDY) == LOW)
  {
    if (Serial.available() >= 1)
    {
       UART_RX = Serial.read();
       dir = (UART_RX&0b00011100)>>2;
       mode = (UART_RX&0x03);
    }
  }
 digitalWrite(SRDY, HIGH);
}

void UART_TX(char message)
{
  Serial.println(message);  
}

void LED_ALL_OFF()
{
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  digitalWrite(LED_UP, HIGH);
  digitalWrite(LED_DOWN, HIGH);
  digitalWrite(LED_MODE_BIT0, HIGH);
  digitalWrite(LED_MODE_BIT1, HIGH);
}

void LEDControl()
{
  if (mode == RESET_MODE)
  {
    digitalWrite(LED_MODE_BIT0, HIGH);
    digitalWrite(LED_MODE_BIT1, HIGH);
  }
  else if(mode == NODE_MODE)
  {
    digitalWrite(LED_MODE_BIT0, LOW);
    digitalWrite(LED_MODE_BIT1, HIGH);
  }
  else if(mode == TICK_MODE)
  {
    digitalWrite(LED_MODE_BIT0, HIGH);
    digitalWrite(LED_MODE_BIT1, LOW);
  }

  
  if (dir == LEFT)
  {
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_UP, HIGH);
    digitalWrite(LED_DOWN, HIGH);
    digitalWrite(LED_LEFT, LOW);
  }
  else if(dir == RIGHT)
  {
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, HIGH);
    digitalWrite(LED_DOWN, HIGH);
  }
  else if(dir == UP)
  {
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_UP, LOW);
    digitalWrite(LED_DOWN, HIGH);
  }
  else if(dir == DOWN)
  {
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_UP, HIGH);
    digitalWrite(LED_DOWN, LOW);
  }
  else if(dir == STOP)
  {
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, LOW);
    digitalWrite(LED_DOWN, LOW);
  }
}
