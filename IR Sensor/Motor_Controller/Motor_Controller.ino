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

//defines for Motor Movement
#define MOTOR_LEVEL1 0x00
#define MOTOR_LEVEL2 0x01
#define MOTOR_LIFT 0x02

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
uint8_t motor = MOTOR_LEVEL1;
uint8_t addr2Arr[8];

void getIRData(int SLAVE_ADDRESS, int rec_data);
void UART_REC();
void UART_TX(uint8_t message);
void binToArray(uint8_t bin, uint8_t * arr);
void IR_PID(uint8_t *arr);
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
        Serial.println(address2Val, BIN);
        binToArray(address2Val, addr2Arr);
        for(int i =0 ; i < 8; i++)
        {
          Serial.print("[");
          Serial.print(i);
          Serial.print("] ");
          Serial.println(addr2Arr[i]);
        }
        //Serial.println(address2Val,BIN);                 
    }
    delay(2000);
  if (mode == RESET_MODE)
  {
    dir = STOP;
    NodeIntersectionFLG = 1;
    TickIntersectionFLG = 1;
    motor = MOTOR_LEVEL1;
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

    if (motor == MOTOR_LEVEL1)
    {
      //bring motor to level1
      //use encoder and if the value is at the correct value then don't move
    }
    else if(motor == MOTOR_LEVEL2)
    {
      //bring motor to level2
      //use encoder and if the value is at the correct value then don't move
    }
    else if(motor == MOTOR_LIFT)
    {
      //use the current encoder value and lift it up a certain distance
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

void binToArray(uint8_t bin, uint8_t * arr)
{
  uint8_t mask = 1;
  uint8_t revArr[8];
  for (int i = 0, j =7; i < 8; i++)
  {
    arr[j] = (bin&mask)>>i;
    mask = mask<<1;
    j--;
  }
}

void IR_PID(uint8_t *arr)
{
  int16_t error = 0;
  int16_t sum = 0;
  for (int i =0 ;i < 8; i++)
  {
    if(arr[i] == 1)
    {
      switch(i)
      {
        case 0: sum+= 0; break;
        case 1: sum+= -2; break;
        case 2: sum+= -1; break;
        case 3: sum += 0; break;
        case 4: sum += 0; break;
        case 5: sum += 1; break;
        case 6: sum += 2; break;
        case 7: sum += 0; break;
      }
    }
  }
  
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
