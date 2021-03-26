#include "robot.h" // low level functions
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

uint8_t dir = STOP;
uint8_t mode = RESET_MODE;
uint8_t UART_RX;
uint8_t MRDY_VAL;

void UART_REC();
void setup() {
  Serial.begin(9600);
  //Wire.begin();
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

  Serial.begin(9600);

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
  Serial.print("ErrorPosition[0]: "); Serial.print(des_joint_pos[0]-joint_pos[0]); Serial.print("\t");
  Serial.print("ErrorPosition[1]: "); Serial.print(des_joint_pos[1]-joint_pos[1]); Serial.print("\t");
  Serial.print("ErrorPosition[2]: "); Serial.print(des_joint_pos[2]-joint_pos[2]); Serial.print("\t");
  Serial.print("ErrorPosition[3]: "); Serial.print(des_joint_pos[3]-joint_pos[3]); Serial.print("\t");
  


  /*
  // Print Duty Cycle 
  Serial.print("Duty0: "); Serial.print(duty_cycle[0]); Serial.print("\t");
  Serial.print("Duty1: "); Serial.print(duty_cycle[1]); Serial.print("\t");
  Serial.print("Duty2: "); Serial.print(duty_cycle[2]); Serial.print("\t");
  Serial.print("Duty3: "); Serial.print(duty_cycle[3]); Serial.print("\t");
  */
  
  /*
  // Desired Voltage
  Serial.print("Des Vol[0]: "); Serial.print(des_Vol[0]); Serial.print("\t");
  Serial.print("Des Vol[1]: "); Serial.print(des_Vol[1]); Serial.print("\t");
  Serial.print("Des Vol[2]: "); Serial.print(des_Vol[2]); Serial.print("\t");
  Serial.print("Des Vol[3]: "); Serial.print(des_Vol[3]); Serial.print("\t");
  */
    

  
 Serial.println();

count++;


}

/*****************************************************************************/


/*************************  Control Function        ***************************/

void Control (void)
{
  int flagRad; // flag to decide the type of required turn

  // wait 3 seconds to start moving
  if ( (ticks <= 300) ) {
    des_robot_vel[0] = 0.0;
    des_robot_vel[1] = 0.0;
  }

  GetCurrentStatus();
  Motor_Control();
  ticks++;
  

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

/*****************************************************************************/
// 100 Hz timer
void TC3_Handler()
{
        TC_GetStatus(TC1, 0);
        Control();
}
//ISR(TIMER1_COMPA_vect)
//{      // timer compare interrupt service routine
//  Control();
//}
/********************************************************************/
