#include <Time.h>
#include <TimeLib.h>
// Libraries:-
#include <Wire.h>

//Slave Addresses for I2C
#define I2C_UP_DOWN 0x08
#define I2C_LEFT_RIGHT 0x09

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

uint8_t I2C_Vertical[2] = {0,0};
uint8_t I2C_Horizontal[2] = {0,0};
uint8_t v_up[8], v_down[8], h_left[8], h_right[8];

uint8_t NodeIntersectionFLG = 1, TickIntersectionFLG = 1;
time_t current, prev;

uint8_t dir = STOP;
uint8_t mode = RESET_MODE;
uint8_t UART_RX;
uint8_t MRDY_VAL;
uint8_t motor = MOTOR_LEVEL1;

void getIRData(int SLAVE_ADDRESS, int rec_data);
void UART_REC();
void UART_TX(uint8_t message);
void binToArray(uint8_t bin, uint8_t * arr);
void IR_PID(uint8_t *frontSensor, uint8_t *backSensor);
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(MRDY, INPUT);
  attachInterrupt(digitalPinToInterrupt(MRDY), UART_REC, FALLING);
  pinMode(SRDY, OUTPUT);
  digitalWrite(SRDY, HIGH);
}

void loop() { 
  
    // Get IR sensor Data
    Wire.requestFrom(I2C_UP_DOWN, 2);     
    if (Wire.available() >= 1){
      for (int i = 0; i < 2; i++)
        I2C_Vertical[0] = Wire.read();

      binToArray(I2C_Vertical[0], v_up);
      binToArray(I2C_Vertical[1], v_down);
    }

    Wire.requestFrom(I2C_LEFT_RIGHT, 2);
    if (Wire.available() >= 1){
      for (int i = 0; i < 2; i++)
        I2C_Horizontal[i] = Wire.read();
        Serial.println(I2C_Horizontal[0], BIN);
        
      binToArray(I2C_Horizontal[0], h_left);
      binToArray(I2C_Horizontal[1], h_right);                 
    }

//    switch(dir)
//    {
//      case UP:    IR_PID(v_up,v_down); break;
//      case DOWN:  IR_PID(v_down,v_up); break;
//      case LEFT:  IR_PID(h_left, h_right); break;
//      case RIGHT: IR_PID(h_right,h_left); break;
//    }
    
    delay(500);
  if (mode == RESET_MODE)
  {
    dir = STOP;
    NodeIntersectionFLG = 1;
    TickIntersectionFLG = 1;
    motor = MOTOR_LEVEL1;
  }
//  else if(mode == NODE_MODE)
//  {
//    if ((address2Val[0] == 0xFF || address1Val[0] == 0xFF) && NodeIntersectionFLG == 0){
//      NodeIntersectionFLG = 1;
//      UART_TX('1');
//    }
//
//    if (address2Val[0] != 0xFF){
//      NodeIntersectionFLG = 0;
//    }
//  }
//  else if(mode == TICK_MODE)
//  {
//    if ((address2Val[0] == 0b11110000 || address2Val[0] == 0b11111000) && TickIntersectionFLG == 0)
//    {
//      TickIntersectionFLG = 1;
//      UART_TX('1');
//    }
//    
//    if (address2Val[0] != 0xF8 && address2Val[0] != 0xF0){
//      TickIntersectionFLG = 0;
//    }
//
//    if (motor == MOTOR_LEVEL1)
//    {
//      //bring motor to level1
//      //use encoder and if the value is at the correct value then don't move
//    }
//    else if(motor == MOTOR_LEVEL2)
//    {
//      //bring motor to level2
//      //use encoder and if the value is at the correct value then don't move
//    }
//    else if(motor == MOTOR_LIFT)
//    {
//      //use the current encoder value and lift it up a certain distance
//    }
//      
//    
//  }

}

void UART_REC()
{
  
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
