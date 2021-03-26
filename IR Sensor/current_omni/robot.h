// function prototypes

void pwm_init(void);                                     
void pwm_set_duty(int channel, int duty);     
void command_motor(int channel, int duty);    
void fwdKinematics(void);                     
void invKinematics(void);                     
void timer_init(void);                        
void GetCurrentStatus(void);                  
void Motor_Control(void);                     

// Encoder Functions
void encoder1CHB(void);   
void encoder1CHA(void);   
void encoder2CHB(void);   
void encoder2CHA(void);   
void encoder3CHB(void);   
void encoder3CHA(void);   
void encoder4CHB(void);   
void encoder4CHA(void);   
void encoder_read(void);
void encoder_init(void); 

//Sonar sensors
void readSonars(void);


#define pi          3.141516
#define PPR         5736.0     // Pulses per revolution (w/ gear box included)
#define del_T       0.01       // [s]
#define wheelRad    0.1        // [m]
#define robotWidth  0.4953     // [m] (19.5 inches)
#define robotLength 0.4953    // [m] (19.5 inches)

// distance from center of robot to wheel
float R = -sqrt(pow(robotWidth/2.0, 2) + pow(robotLength/2.0, 2));

// controller constants
//#define  Kp 50.0
//#define  Kd 10.0
float Kp[4] = {50.0, 50.0, 50.0, 50.0};
float Kd[4] = {10.0, 10.0, 10.0, 10.0};

#define Vcc 12.0

float des_Vol[4] = {0.,0.,0.,0.}; // desired voltage to reach desired duty cycle
int   duty_cycle[4];


// pins for encoder 0
const byte interruptPINfrontLeft1 = 28;   // yellow encoder wire
const byte interruptPINfrontLeft2 = 27;   // white encoder wire

// pins for encoder 1
const byte interruptPINfrontRight1 = 46;  // yellow encoder wire
const byte interruptPINfrontRight2 = 47;  // white encoder wire

// pins for encoder 2
const byte interruptPINbackRight1 = 50;   // yellow encoder wire
const byte interruptPINbackRight2 = 51;   // white encoder wire

// pins for encoder 3
const byte interruptPINbackLeft1 = 48;    // yellow encoder wire
const byte interruptPINbackLeft2 = 49;    // white encoder wire


long int encoder_val[4]= {0,0,0,0}; 

/*
//Pins for sonar sensors
const int anPinRight = 0; // right sonar
const int anPinLeft = 1; // left sonar
//float distanceSonars[R2];
float obstaclePos[2]; // global position of the closest obstacle 
*/

float    joint_pos[4]     = {0.0,0.0,0.0,0.0};  // acual wheel position in rad, 0: frontLeft, 1: frontRight,  2: backRight, 3: backLeft
float    joint_vel[4]     = {0.0,0.0,0.0,0.0};  // actual wheel velocity in rad/s, 0: frontLeft, 1: frontRight,  2: backRight, 3: backLeft
float    robot_vel[3]     = {0.0,0.0,0.0};      // robot forward velocity (x) [m/s], robot side velocity (y) [m/s], and angular velocity[rad/s]
float    robot_pos[3]     = {0.0,0.0,0.0};      // robot gloabal pose X[m],Y[m], Theta[rad]
byte     state1 = 0; // for encoders
byte     state2 = 0;
byte     state3 = 0;
byte     state4 = 0;

float des_joint_pos[4] = {0.0,0.0,0.0,0.0};       // desired wheel positions
float prev_des_joint_pos[4] = {0.0,0.0,0.0,0.0};  // previous desired wheel positions
float des_joint_vel[4] = {0.0,0.0,0.0,0.0};       // desired wheel velocities
float des_robot_vel[3] = {0.0,0.0,0.0};           // desired robot velocities, 0: fwd vel, 1: side vel, 2: angular vel
   

// initialize timer
void timer_init(void)
{

  // initialize timer1
     noInterrupts();           // disable all interrupts

      PMC->PMC_PCER0 |= PMC_PCER0_PID27;                        
      TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK2    
                              | TC_CMR_ABETRG               
                              | TC_CMR_LDRA_RISING          
                              | TC_CMR_LDRB_FALLING;        
      TC0->TC_CHANNEL[0].TC_IER |= TC_IER_LDRAS | TC_IER_LDRBS; 
      TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;  
      NVIC_EnableIRQ(TC0_IRQn);   
        
     interrupts();                            
}


// initialize pwm
void pwm_init(void)
{
   
    // Sets PWM pins as HIGH using functions
    pinMode(2, OUTPUT);   //TIOA0   // motor 0
    pinMode(13, OUTPUT);  //TIOB0   // motor 1
    pinMode(3, OUTPUT);   //TIOA7   // motor 2
    pinMode(12, OUTPUT);  //TIOB8   // motor 3

}

// set duty cycle of motors
void pwm_set_duty(int channel, int duty)
{

    // motor 0
    if (channel==0)
       analogWrite(2, duty);

    // motor 1   
    else if (channel ==1)
       analogWrite(3, duty);

    // motor 2   
    else if (channel == 2)
      analogWrite(12, duty);

    // motor 3  
    else if (channel == 3)
      analogWrite(13, duty);

}


// controls the direction of the motor and calls pwm_sety_duty to run motors at specified duty
void command_motor(int channel, int duty)
{
     
    if(channel == 0){
      if (duty > 0) REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111111111011) | 0b00000000000000000000000000000010; // set C1 to 1 and C2 to 0
      else          REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111111111101) | 0b00000000000000000000000000000100; // set C1 to 0 and C2 to 1
    }
    else if (channel == 1) {
      if (duty > 0) REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111111101111) | 0b00000000000000000000000000001000; // set C3 to 1 and C4 to 0
      else          REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111111110111) | 0b00000000000000000000000000010000; // set C3 to 0 and C4 to 1
    }
    else if (channel == 2){
      if (duty > 0) REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111011111111) | 0b00000000000000000000000010000000; // set C7 to 1 and C8 to 0
      else          REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111101111111) | 0b00000000000000000000000100000000; // set C7 to 0 and C8 to 1
    }
    else if (channel == 3){
      if (duty > 0) REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111110111111) | 0b00000000000000000000000000100000; // set C5 to 1 and C6 to 0
      else          REG_PIOC_ODSR = (REG_PIOC_ODSR & 0b1111111111111111111111111011111) | 0b00000000000000000000000001000000; // set C5 to 0 and C6 to 1
    }
    
       
   
   pwm_set_duty(channel, abs(duty)); 

}

// start the timer
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
        TC_SetRA(tc, channel, rc/2); //50% high, 50% low
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}


// initialize encoders
// encoders are connected to specified pins
 void encoder_init( void)
 {
   
   // encoder 1
   pinMode(interruptPINfrontLeft1, INPUT_PULLUP);
   pinMode(interruptPINfrontLeft2, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(interruptPINfrontLeft1), encoder1CHA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(interruptPINfrontLeft2), encoder1CHB, CHANGE);



    
   // encoder 2
   pinMode(interruptPINfrontRight1, INPUT_PULLUP);
   pinMode(interruptPINfrontRight2, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(interruptPINfrontRight1), encoder2CHA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(interruptPINfrontRight2), encoder2CHB, CHANGE);


   // encoder 3
   pinMode(interruptPINbackRight1, INPUT_PULLUP);
   pinMode(interruptPINbackRight2, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(interruptPINbackRight1), encoder3CHA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(interruptPINbackRight2), encoder3CHB, CHANGE);


   // encoder 4
   pinMode(interruptPINbackLeft1, INPUT_PULLUP);
   pinMode(interruptPINbackLeft2, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(interruptPINbackLeft1), encoder4CHA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(interruptPINbackLeft2), encoder4CHB, CHANGE); 
 }




//// Encoder 1 Functions
//Encoder ENC1(interruptPINfrontLeft1, interruptPINfrontLeft2);

void encoder1CHA(void) 
{

if ((state1 == 0) && (digitalRead(interruptPINfrontLeft1) ==1))
     {
      state1 =2;
      encoder_val[0] -=1;
     }
else if ((state1==2) && (digitalRead(interruptPINfrontLeft1) ==0))
  {
    state1 =0;
    encoder_val[0] +=1;
    }

   else if ((state1==1) && (digitalRead(interruptPINfrontLeft1) ==1))
    {
    state1 =3;
    encoder_val[0] +=1;
    }
   else if ((state1==3) && (digitalRead(interruptPINfrontLeft1) ==0))
   {
      state1 = 1;
      encoder_val[0] -=1;
    }

}


void encoder1CHB(void)
{
  
   if ((state1 == 0) && (digitalRead(interruptPINfrontLeft2) ==1))
     {
      state1 = 1;
      encoder_val[0] +=1;
     }
  else if ((state1 == 3) && (digitalRead(interruptPINfrontLeft2) ==0))
  {
    state1 =2;
    encoder_val[0] +=1;
    }

  else if ((state1==2) && (digitalRead(interruptPINfrontLeft2) ==1))
  //else if ((state2==2) && (digitalReadFast(interruptPIN19) ==1)) 
    {
    state1 = 3;
    encoder_val[0] -=1;
    }
   
  else if ((state1==1) && (digitalRead(interruptPINfrontLeft2) ==0))
    {
      state1 =0;
      encoder_val[0] -=1;
    }

}



// Encoder 2 functions
void encoder2CHA(void) 
{

if ((state2 == 0) && (digitalRead(interruptPINfrontRight1) ==1))
//if ((state2 == 0) && (digitalReadFast(interruptPIN18) ==1))


     {
      state2 =2;
      encoder_val[1] -=1;
     }
else if ((state2==2) && (digitalRead(interruptPINfrontRight1) ==0))
//  else if ((state2==2) && (digitalReadFast(interruptPIN18) ==0))
  
  {
    state2 =0;
    encoder_val[1] +=1;
    }

   else if ((state2==1) && (digitalRead(interruptPINfrontRight1) ==1))
//   else if ((state2==1) && (digitalReadFast(interruptPIN18) ==1))
   
    {
    state2 =3;
    encoder_val[1] +=1;
    }
   else if ((state2==3) && (digitalRead(interruptPINfrontRight1) ==0))
   //else if ((state2==3) && (digitalReadFast(interruptPIN18) ==0))
   
   {
      state2 = 1;
      encoder_val[1] -=1;
    }

}


void encoder2CHB(void)
{
  
   if ((state2 == 0) && (digitalRead(interruptPINfrontRight2) ==1))
   //if ((state2 == 0) && (digitalReadFast(interruptPIN19) ==1))
  
     {
      state2 = 1;
      encoder_val[1] +=1;
     }
  else if ((state2 == 3) && (digitalRead(interruptPINfrontRight2) ==0))
  //else if ((state2 == 3) && (digitalReadFast(interruptPIN19) ==0))
  
  {
    state2 =2;
    encoder_val[1] +=1;
    }

  else if ((state2==2) && (digitalRead(interruptPINfrontRight2) ==1))
  //else if ((state2==2) && (digitalReadFast(interruptPIN19) ==1)) 
    {
    state2 = 3;
    encoder_val[1] -=1;
    }
   
  else if ((state2==1) && (digitalRead(interruptPINfrontRight2) ==0))
  //else if ((state2==1) && (digitalReadFast(interruptPIN19) ==0))
   
    {
      state2 =0;
      encoder_val[1] -=1;
    }

}


// Encoder 3 Functions
void encoder3CHA(void) 
{

//digitalReadFast(interruptPIN2);
if ((state3 == 0) && (digitalRead(interruptPINbackRight1) ==1))
 
//if ((state1 == 0) && (digitalReadFast(interruptPIN2) ==1))
   
   {
      state3 =2;
      encoder_val[2] -=1;
     }
 else if ((state3==2) && (digitalRead(interruptPINbackRight1) ==0))
 //else if ((state1==2) && (digitalReadFast(interruptPIN2) ==0))
 
 
  {
    state3 =0;
    encoder_val[2] +=1;
    }

  else if ((state3==1) && (digitalRead(interruptPINbackRight1) ==1))
 //else if ((state1==1) && (digitalReadFast(interruptPIN2) ==1))  
    {
    state3 =3;
    encoder_val[2] +=1;
    }
   else if ((state3==3) && (digitalRead(interruptPINbackRight1) ==0))
  //else if ((state1==3) && (digitalReadFast(interruptPIN2) ==0))
    
   
    {
      state3 =1;
      encoder_val[2] -=1;
    }

}


void encoder3CHB(void)
{

  
 if ((state3 == 0) && (digitalRead(interruptPINbackRight2) ==1))
 //if ((state1 == 0) && (digitalReadFast(interruptPIN3) ==1))
 
     {
      state3 =1;
      encoder_val[2] +=1;
     }
  else if ((state3==3) && (digitalRead(interruptPINbackRight2) ==0))
  //else if ((state1==3) && (digitalReadFast(interruptPIN3) ==0))
  
  {
    state3 =2;
    encoder_val[2] +=1;
    }

   else if ((state3==2) && (digitalRead(interruptPINbackRight2) ==1))
  // else if ((state1==2) && (digitalReadFast(interruptPIN3) ==1))
    
    
    {
    state3 =3;
    encoder_val[2] -=1;
    }
    else if ((state3==1) && (digitalRead(interruptPINbackRight2) ==0))
   //else if ((state1==1) && (digitalReadFast(interruptPIN3) ==0))
    
    {
      state3 =0;
      encoder_val[2] -=1;
    }
}


// Encoder 4 Functions
void encoder4CHA(void) 
{

//digitalReadFast(interruptPIN2);
if ((state4 == 0) && (digitalRead(interruptPINbackLeft1) ==1))
 
//if ((state1 == 0) && (digitalReadFast(interruptPIN2) ==1))
   
   {
      state4 =2;
      encoder_val[3] -=1;
     }
 else if ((state4==2) && (digitalRead(interruptPINbackLeft1) ==0))
 //else if ((state1==2) && (digitalReadFast(interruptPIN2) ==0))
 
 
  {
    state4 =0;
    encoder_val[3] +=1;
    }

  else if ((state4==1) && (digitalRead(interruptPINbackLeft1) ==1))
 //else if ((state1==1) && (digitalReadFast(interruptPIN2) ==1))  
    {
    state4 =3;
    encoder_val[3] +=1;
    }
   else if ((state4==3) && (digitalRead(interruptPINbackLeft1) ==0))
  //else if ((state1==3) && (digitalReadFast(interruptPIN2) ==0))
    
   
    {
      state4 =1;
      encoder_val[3] -=1;
    }

}


void encoder4CHB(void)
{

  
 if ((state4 == 0) && (digitalRead(interruptPINbackLeft2) ==1))
 //if ((state1 == 0) && (digitalReadFast(interruptPIN3) ==1))
 
     {
      state4 =1;
      encoder_val[3] +=1;
     }
  else if ((state4==3) && (digitalRead(interruptPINbackLeft2) ==0))
  //else if ((state1==3) && (digitalReadFast(interruptPIN3) ==0))
  
  {
    state4 =2;
    encoder_val[3] +=1;
    }

   else if ((state4==2) && (digitalRead(interruptPINbackLeft2) ==1))
  // else if ((state1==2) && (digitalReadFast(interruptPIN3) ==1))
    
    
    {
    state4 =3;
    encoder_val[3] -=1;
    }
    else if ((state4==1) && (digitalRead(interruptPINbackLeft2) ==0))
   //else if ((state1==1) && (digitalReadFast(interruptPIN3) ==0))
    
    {
      state4 =0;
      encoder_val[3] -=1;
    }    

}


// determine the current position of the robot's wheels
 void GetCurrentStatus(void)
  {
    static float prev_pos[4] = {0.0, 0.0, 0.0, 0.0}; 
    
    //joint_pos[0] = encoder_val[0]*2*pi/(Gear_Ratio*400);
    joint_pos[0] = encoder_val[0]*2*pi/(PPR);
    
    
    joint_vel[0] = (joint_pos[0] - prev_pos[0])/del_T; 
    
    prev_pos[0] = joint_pos[0];      

    //joint_pos[1] = 1.0*encoder_val[1]*2*pi/(Gear_Ratio*400);
    joint_pos[1] = 1.0*(encoder_val[1])*2*pi/(PPR);

    joint_vel[1] = (joint_pos[1] - prev_pos[1])/del_T; 
    
    prev_pos[1] = joint_pos[1];     

    // joint_pos[2] = -1.0*encoder_val[1]*2*pi/(Gear_Ratio*400);
    joint_pos[2] = encoder_val[2]*2*pi/(PPR); 
    
    joint_vel[2] = (joint_pos[2] - prev_pos[2])/del_T; 
    
    prev_pos[2] = joint_pos[2];    


    // joint_pos[1] = -1.0*encoder_val[1]*2*pi/(Gear_Ratio*400);
    joint_pos[3] = encoder_val[3]*2*pi/(PPR); 
    
    joint_vel[3] = (joint_pos[3] - prev_pos[3])/del_T; 
    
    prev_pos[3] = joint_pos[3];
    
 
    fwdKinematics(); // compute robot velocities and update robot global pose

    //globalLocalization();
    //globalLocalization(del_T);
    
    
  }


// uses the real encoder values for the speed of the wheels to calculate the actual robot velocities and then position
void fwdKinematics(void){
 
  // forward kinematics to determine actual robot forward, lateral, and angular velocity
  robot_vel[0] = wheelRad / 4.0 * ( joint_vel[0] - joint_vel[1] - joint_vel[2] + joint_vel[3]) / sin(pi/4.0); // forward velocity update
  robot_vel[1] = wheelRad / 4.0 * (-joint_vel[0] - joint_vel[1] + joint_vel[2] + joint_vel[3]) / cos(pi/4.0); // lateral velocity update
  robot_vel[2] = wheelRad / 4.0 * (joint_vel[0] + joint_vel[1] + joint_vel[2] + joint_vel[3]) / R;    // angular velocity update

  // update robot real time position
  robot_pos[0] = robot_pos[0] + robot_vel[0]*cos(robot_pos[2])*del_T - robot_vel[1]*sin(robot_pos[2])*del_T;  // x position update
  robot_pos[1] = robot_pos[1] + robot_vel[0]*sin(robot_pos[2])*del_T + robot_vel[1]*cos(robot_pos[2])*del_T;  // y position update
  robot_pos[2] = robot_pos[2] + robot_vel[2] * del_T;      // heading angle update

  robot_pos[2] = atan2(sin(robot_pos[2]), cos(robot_pos[2])); // constrain between [-pi,pi) 
  
 
}


// determines the desired angular velocities of each wheel to reach the specified speed in the main loop
void invKinematics(void){
  
  des_joint_vel[0] = (1/wheelRad)*( des_robot_vel[0]*cos(pi/4.0) - des_robot_vel[1]*sin(pi/4.0) + R*des_robot_vel[2]); // front left wheel  (omega 1)
  des_joint_vel[1] = (1/wheelRad)*(-des_robot_vel[0]*cos(pi/4.0) - des_robot_vel[1]*sin(pi/4.0) + R*des_robot_vel[2]); // front right wheel (omega 2)
  des_joint_vel[2] = (1/wheelRad)*(-des_robot_vel[0]*cos(pi/4.0) + des_robot_vel[1]*sin(pi/4.0) + R*des_robot_vel[2]); // back right wheel  (omega 3)
  des_joint_vel[3] = (1/wheelRad)*( des_robot_vel[0]*cos(pi/4.0) + des_robot_vel[1]*sin(pi/4.0) + R*des_robot_vel[2]); // back left wheel   (omega 4)


//  des_joint_vel[0] = 1.4;
//  des_joint_vel[1] = 1.4;
//  des_joint_vel[2] = 1.4;
//  des_joint_vel[0] = 1.4;

}



void  Motor_Control(void) {

  int i;
    
  // map desired robot velocities to desired wheel velocities
  invKinematics();

  // PD control Law
  for(i = 0;i<4;i++){
    des_joint_pos[i] = prev_des_joint_pos[i] + des_joint_vel[i]*del_T;
    prev_des_joint_pos[i]= des_joint_pos[i];
    des_Vol[i] = Kp[i]*(des_joint_pos[i]-joint_pos[i])+ Kd[i]*(des_joint_vel[i] - joint_vel[i]);
    
   // duty_cycle[i] = (int)(des_Vol[i])/Vcc*255;
    duty_cycle[i] = (des_Vol[i])/Vcc*255;
   
    if (duty_cycle[i]>255)
      duty_cycle[i] = 255;
    else if (duty_cycle[i]<-255)
      duty_cycle[i] = -255;
      
  }


  // control direction and speed of motors with specified duty_cycle
  
  command_motor(0,duty_cycle[0]);
  command_motor(1,duty_cycle[1]);
  command_motor(2,duty_cycle[2]);
  command_motor(3,duty_cycle[3]);
  
  /*
  command_motor(0,100);
  command_motor(1,100);
  command_motor(2,100);
  command_motor(3,100);
  */
                      
}
