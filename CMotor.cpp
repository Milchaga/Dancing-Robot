#include "Arduino.h"
#include "CMotor.h"

// Constructor
CMotor::CMotor(int enPin, int dirPin, int SA, int SB) : motorEnable(enPin), motorDirection(dirPin), mEncA(SA), mEncB(SB)
{   
  // Declare OUTPUT PinModes
  pinMode(motorEnable, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  
  // Declare INPUT PinModes
  pinMode(mEncA, INPUT);
  pinMode(mEncB, INPUT);
}
// Clear objects 
CMotor::~CMotor(){
  
}

// Set motor velocity and direction
void CMotor::setMotor(int dir, int pwmVal)
{
  analogWrite(motorEnable, pwmVal); // Motor speed
  digitalWrite(motorDirection, (dir == 1) ? LOW : HIGH);  // Motor direction
}
