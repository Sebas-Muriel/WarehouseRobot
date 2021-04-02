#include "robot.h" // low level functions
#include <Time.h>
#include <TimeLib.h>
// Libraries:-
#include <Wire.h>

//Slave Addresses for I2C
#define IR_I2C 0x09

//Defines for movement
#define STOP 0x00
#define LEFT 0X01
#define RIGHT 0X02
#define UP 0x03
#define DOWN 0x04

#define RESET_MODE 0x00
//Defines for Modes 
#define NODE_MODE 0x01
#define TICK_MODE 0x02

//defines for pins
#define MRDY 0x04
#define SRDY 0x05

//defines for Motor Movement
#define MOTOR_LEVEL1 0x00
#define MOTOR_LEVEL2 0x01
#define MOTOR_LIFT 0x02

unsigned long ticks = 0; 

float goal[2] = {5.0, 0.0};

float vxMax = 0.15;
float vyMax = 0.15;
float thetaDotMax = 1.00;

float count = 1;
float errorEnc0 = 0, errorEnc1 = 0;
float SumErrorEnc0 = 0, SumErrorEnc1 = 0;
float avgErrorEnc0 = errorEnc0, avgErrorEnc1 = errorEnc1;

uint8_t I2C_IR_Values[4] = {0,0,0,0};
uint8_t v_up[8], v_down[8], h_left[8], h_right[8], s_stop[8];
time_t start, current;
float xposStart = 0, xposCurrent = 0, yposStart = 0, yposCurrent = 0;
uint8_t NodeIntersectionFLG = 0, TickIntersectionFLG = 0;

uint8_t dir = STOP;
uint8_t mode = RESET_MODE;
uint8_t UART_RX;
uint8_t MRDY_VAL;

void UART_REC();
void binToArray(uint8_t bin, uint8_t * arr);
void UART_TX(uint8_t message);
void Control (uint8_t *front, char d, uint8_t *back);
void CopyArr(uint8_t *src, uint8_t *des);
bool lineLogic(uint8_t IR_Sens);
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(MRDY, INPUT);
  attachInterrupt(digitalPinToInterrupt(MRDY), UART_REC, FALLING);
  pinMode(SRDY, OUTPUT);
  digitalWrite(SRDY, HIGH);
  startTimer(TC1, 0, TC3_IRQn, 100); //TC1 channel 0, the IRQ for that channel and the desired frequency
  encoder_init();
  pwm_init();
  // init sensor stuff

// Set pins connected to motor drivers as outputs (DUE version of DDRC)
  REG_PIOC_OWER = 0x00000FFF;     // Enable writes to lowest 4-bits of PortD's 32-bit, Output Data Status (ODSR) register: (B00000000000000000000111111111111)
  REG_PIOC_OER =  0x00000FFF;     // Set the lowest 4-bits of PortD to outputs: (B00000000000000000000111111111111)
  
  dir = STOP;
}


void loop() {

  switch(dir)
  {
    case UP:    des_robot_vel[0] = vxMax; des_robot_vel[1] = 0; break;
    case DOWN:  des_robot_vel[0] = -vxMax; des_robot_vel[1] = 0; break;
    case LEFT:  des_robot_vel[0] = 0.0; des_robot_vel[1] = vyMax; break;
    case RIGHT: des_robot_vel[0] = 0.0; des_robot_vel[1] = -vyMax; break;
    case STOP:  des_robot_vel[0] = 0.0; des_robot_vel[1] = 0.0; break;
  }



 
  
 
  
  // Joint Position and Velocity Error 
//  Serial.print("ErrorPosition[0]: "); Serial.print(des_joint_pos[0]-joint_pos[0]); Serial.print("\t");
//  Serial.print("ErrorPosition[1]: "); Serial.print(des_joint_pos[1]-joint_pos[1]); Serial.print("\t");
//  Serial.print("ErrorPosition[2]: "); Serial.print(des_joint_pos[2]-joint_pos[2]); Serial.print("\t");
//  Serial.print("ErrorPosition[3]: "); Serial.print(des_joint_pos[3]-joint_pos[3]); Serial.print("\t");
  


  
  // Print Duty Cycle 
//  Serial.print("Duty0: "); Serial.print(duty_cycle[0]); Serial.print("\t");
//  Serial.print("Duty1: "); Serial.print(duty_cycle[1]); Serial.print("\t");
//  Serial.print("Duty2: "); Serial.print(duty_cycle[2]); Serial.print("\t");
//  Serial.print("Duty3: "); Serial.print(duty_cycle[3]); Serial.print("\t");
  
  
  /*
  // Desired Voltage
  Serial.print("Des Vol[0]: "); Serial.print(des_Vol[0]); Serial.print("\t");
  Serial.print("Des Vol[1]: "); Serial.print(des_Vol[1]); Serial.print("\t");
  Serial.print("Des Vol[2]: "); Serial.print(des_Vol[2]); Serial.print("\t");
  Serial.print("Des Vol[3]: "); Serial.print(des_Vol[3]); Serial.print("\t");
  */




count++;


}

/*****************************************************************************/


/*************************  Control Function        ***************************/

void Control (uint8_t *front, char d, uint8_t *back)
{
  int flagRad; // flag to decide the type of required turn

  // wait 3 seconds to start moving
  if ( (ticks <= 300) ) {
    des_robot_vel[0] = 0.0;
    des_robot_vel[1] = 0.0;
  }

  GetCurrentStatus();
  Motor_Control(front, d, back);
  ticks++;
  

}
void UART_REC()
{
  
  digitalWrite(SRDY, LOW);
  NodeIntersectionFLG = 0;
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
  uint8_t tester[8] = {0,0,0,1,1,0,0,0};
  uint8_t count = 0;

  for (int i = 0, j =7; i < 8; i++)
  {
    arr[j] = (bin&mask)>>i;
    mask = mask<<1;
    j--;
  }

  for (int i = 2; i < 6; i++)
  {
    if (arr[i] == tester[i])
      count++;
  }
  if (count == 4)
  {
    for (int i =0 ; i < 8; i++)
      arr[i] = tester[i];
  }
}

void CopyArr(uint8_t *src, uint8_t *des)
{
  for (int i =0 ; i < 8; i++)
  {
    des[i] = src[i];
  }
}

bool lineLogic(uint8_t IR_Sens)
{
  for (int i =0 ; i < 16; i++)
  {
    if (IR_Sens == (0xf0 + i) || IR_Sens == (0xE0 + i))
      return true;
  }
  return false;
}
/*****************************************************************************/
// 100 Hz timer
void TC3_Handler()
{
  TC_GetStatus(TC1, 0);
//*************************************************************************//
//******************************GET IR SENSOR DATA*************************//
//*************************************************************************//


  /*Sends in order
   * 0 Up sensor
   * 1 Back sensor
   * 2 Left sensor
   * 3 Right sensor
  */
  Wire.requestFrom(IR_I2C, 4);
  if (Wire.available() >= 1){
    for (int i = 0; i < 4; i++)
    {
      I2C_IR_Values[i] = Wire.read();
    }
      
    binToArray(I2C_IR_Values[0], v_up);
    binToArray(I2C_IR_Values[1], v_down);
    binToArray(I2C_IR_Values[2], h_left);
    binToArray(I2C_IR_Values[3], h_right);                 
  }

//*************************************************************************//
//******************************Mode Reset**********************************//
//*************************************************************************//
  if (mode == RESET_MODE)
  {
    dir = STOP;
    for (int i =0 ; i < 8; i++)
      s_stop[i] = 0;
    NodeIntersectionFLG = 0;
    TickIntersectionFLG = 0;
    //motor = MOTOR_LEVEL1;
    xposStart = 0;
    xposCurrent = 0;
    yposStart = 0;
    yposCurrent = 0;
  }
//*************************************************************************//
//******************************Start NODE MODE****************************//
//*************************************************************************//
  else if(mode == NODE_MODE)
  {
    if (dir == LEFT)
    {
      //Start Stopping process once a black line is hit
      if ((I2C_IR_Values[2] == 0xFF || I2C_IR_Values[2] == 0xFE || I2C_IR_Values[2] == 0x7F) && NodeIntersectionFLG == 0)
      {
        xposStart = robot_pos[1];                         
        NodeIntersectionFLG = 1;
      }
      //Ignore Ticks
      if (I2C_IR_Values[2] == 0x1F || I2C_IR_Values[2] == 0x0E || I2C_IR_Values[2] == 0x0F)
      {
        I2C_IR_Values[2] = 0b00011000;
        binToArray(I2C_IR_Values[2], h_left);
      }
      xposCurrent = robot_pos[1];
      //Stop the robot once the stopping process begins and after the robot travels its distance in the x direction
      if (xposCurrent - xposStart >= ((robotLength)/ 2) && NodeIntersectionFLG == 1)
      {
        dir = STOP;
        CopyArr(h_left, s_stop);
        UART_TX('1'); 
      }
    }
    else if(dir == RIGHT)
    {
      //Start Stopping process once a black line is hit
      if (I2C_IR_Values[3] == 0xFF /*|| I2C_IR_Values[3] == 0xFE || I2C_IR_Values[3] == 0x7F)*/ && NodeIntersectionFLG == 0)
      {
        xposStart = robot_pos[1];                                                                                           
        NodeIntersectionFLG = 1;
      }
      //Ignore Ticks
      if (I2C_IR_Values[3] <= 0x1F)
      {
//        UART_TX('1');
        I2C_IR_Values[3] = 0b00011000;
        binToArray(I2C_IR_Values[3], h_right);
      }
      xposCurrent = robot_pos[1];
      //Stop the robot once the stopping process begins and after the robot travels its distance in the x direction
      if (abs(xposCurrent) - abs(xposStart) >=  ((robotLength)/2) && NodeIntersectionFLG == 1)
      {
//        dir = STOP;
//        CopyArr(h_right, s_stop);
        UART_TX('1');
      }   
    }
    else if(dir == UP)
    {
      //Start Stopping process once a black line is hit
      if (I2C_IR_Values[0] == 0xFF /*|| I2C_IR_Values[0] == 0xFE || I2C_IR_Values[0] == 0x7F)*/ && NodeIntersectionFLG == 0)
      {
        yposStart = robot_pos[0];
        NodeIntersectionFLG = 1;
      }
      yposCurrent = robot_pos[0];
      //Stop the robot once the stopping process begins and after the robot travels its distance in the x direction
      if (yposCurrent - yposStart >= ((robotLength)/ 2) && NodeIntersectionFLG == 1)
      {
        dir = STOP;
        CopyArr(v_up, s_stop);
        UART_TX('1');
      }
    }
    else if(dir == DOWN)
    {
      //Start Stopping process once a black line is hit
      if ((I2C_IR_Values[1] == 0xFF || I2C_IR_Values[1] == 0xFE || I2C_IR_Values[1] == 0x7F) && NodeIntersectionFLG == 0)
      {
        yposStart = robot_pos[0];
        NodeIntersectionFLG = 1;
      }
      yposCurrent = robot_pos[0];
      //Stop the robot once the stopping process begins and after the robot travels its distance in the x direction
      if (abs(yposCurrent) - abs(yposStart) >= ((robotLength)/ 2) && NodeIntersectionFLG == 1)
      {
        dir = STOP;
        CopyArr(v_down, s_stop);
        UART_TX('1');
      }
    }
  }
//*************************************************************************//
//******************************Start Tick Mode****************************//
//*************************************************************************//
  else if(mode == TICK_MODE)
  {
    if (dir == LEFT)
    {
      //Start Stopping process once a tick is hit
      if ((I2C_IR_Values[2] == 0x1F || I2C_IR_Values[2] == 0x0E || I2C_IR_Values[2] == 0x0F) && NodeIntersectionFLG == 0)
      {
        xposStart = robot_pos[1];
        NodeIntersectionFLG = 1;
        I2C_IR_Values[2] = 0b00011000;
        binToArray(I2C_IR_Values[2], h_left);
      }
      xposCurrent = robot_pos[1];
      //Stop the robot once the stopping process begins and after the robot travels its distance in the x direction
      if (xposCurrent - xposStart >= ((robotLength)/ 2.0) && NodeIntersectionFLG == 1)
      {
        dir = STOP;
        CopyArr(h_left, s_stop);
        UART_TX('1');
      }
    }
    else if(dir == RIGHT)
    {
      //Start Stopping process once a tick is hit
      if ((I2C_IR_Values[3] == 0x1F || I2C_IR_Values[3] == 0x0E || I2C_IR_Values[3] == 0x0F) && NodeIntersectionFLG == 0)
      {
        xposStart = robot_pos[1];
        NodeIntersectionFLG = 1;
        I2C_IR_Values[3] = 0b00011000;
        binToArray(I2C_IR_Values[3], h_right);
      }
      xposCurrent = robot_pos[1];
      //Stop the robot once the stopping process begins and after the robot travels its distance in the x direction
      if (abs(xposCurrent )- abs(xposStart) >= ((robotLength)/ 2.0) && NodeIntersectionFLG == 1)
      {
        dir = STOP;
        CopyArr(h_right, s_stop);
        UART_TX('1');
      }   
    }
  }
//*************************************************************************//
//******************************Controller*********************************//
//*************************************************************************//
  switch(dir)
  {
    case LEFT: Control(h_left, 'l', h_right); break;
    case RIGHT: Control(h_right, 'l', h_left); break;
    case UP: Control(v_up, 'u', v_down); break;
    case DOWN: Control(v_down, 'u', v_up); break;
    case STOP: Control(s_stop,'u', s_stop); break;
  }
}
/********************************************************************/
