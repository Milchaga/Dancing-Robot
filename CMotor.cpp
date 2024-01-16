#include "Arduino.h"
#include "CMotor.h"

//Constructor
CMotor::CMotor(int enPin, int dirPin, int SA, int SB) : motorEnable(enPin), motorDirection(dirPin), mEncA(SA), mEncB(SB)
{   
  // Declare OUTPUT PinModes
  pinMode(motorEnable, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  
  // Declare INPUT PinModes
  pinMode(mEncA, INPUT);
  pinMode(mEncB, INPUT);
}

CMotor::~CMotor(){
  
}

void CMotor::calculate(int theta_i)
{
  // PID constants
  kp = 4.2;   //2.5
  kd = 0.25;   //0.1
  ki = 1.6; //0.012
  
  // Read the position
  noInterrupts(); // disable interrupts temporarily while reading
  theta = theta_i;
  interrupts(); // turn interrupts back on
  
  // Compute deltaT in seconds
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  prevT = currT;
  
  // Control signal
  int e = theta - target;
  float dedt = (e-ePrev)/(deltaT);
  ePrev = e;
  eintegral = eintegral + e*deltaT;
  float u = kp*e + kd*dedt + ki*eintegral;
  
  // Motor Power
  pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  
  // Motor Direction
  dir = 1;
  if(u<0){
    dir = -1;
  }
  /*
  Serial.print(target);
  Serial.print(" ");
  Serial.print(theta);
  Serial.println();
  */  
}

void CMotor::setMotor(int dir, int pwmVal)
{
  analogWrite(motorEnable, pwmVal); // Motor speed
  if (dir == 1)
  {
      // Turn one way
      digitalWrite(motorDirection, LOW);
  }
  else
  {
      // no turning
      digitalWrite(motorDirection, HIGH);
  }
}
