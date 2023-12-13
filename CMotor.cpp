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
    
    // Read the angular position 
    theta = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    theta = theta_i;  // Encoder Pulses
    interrupts(); // turn interrupts back on
    

    // Compute velocity 
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;    //dt in seconds
    float omega = (theta - thetaPrev) / deltaT;         //speed of the wheel in counts/s
    thetaPrev = theta; 
    prevT = currT;

    // Convert count/s to RPM
    omega = omega / encoderTicksPerRev * 60.0;
    
    // Low-pass filter (25 Hz cutoff)
    omegaFilt = 0.854 * omegaFilt + 0.0728 * omega + 0.0728 * omegaPrev;
    omegaPrev = omega;
    
    // CALCULATE PID
    if (omegaTarget==0)
    {
      kp = 1;   //1
      ki = 0.025; //0.5
      kd = 0;   //0.005
    }
    else{
      kp = .75;   //1
      ki = 0.1; //0.1; //0.5
      kd = 0.05;   //0.005
    }
       
    float e = omegaTarget - omegaFilt;              //Proporional
    dedt = (e-ePrev)/(deltaT);                      //Derivative
    eintegral = eintegral + e * deltaT;             //Integral
    u = kp * e + ki * eintegral + dedt * kd;        //Calculate PID
    ePrev = e;
    
    // Set the motor speed and direction
    dir = 1;
    if (u < 0)
    {
        dir = -1;
    }
    power = (int)fabs(u);
    if (power > 255)
    {
        power = 255;
    }
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
