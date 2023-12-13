#ifndef CMotor_h
#define CMotor_h

#include "Arduino.h"

class CMotor{
  private:
    long prevT = 0;
    int thetaPrev = 0;
    // Use the "volatile" directive for variables
    // used in an interrupt
    //volatile float velocity_i = 0;
    //volatile long prevT_i = 0;
    float omegaFilt = 0;
    float omegaPrev = 0;
    float eintegral = 0;
    float ePrev = 0;
    float dedt = 0;
    int encoderTicksPerRev = 314;

    int motorEnable;
    int motorDirection;
    int mEncA;
    int mEncB;
    int theta_i;
    float kd;
    float ki;
    float kp;

  public:
    int dir;
    int power;
    int theta;
    CMotor(int enPin, int dirPin, int SA, int SB);
    ~CMotor();
    void calculate(int theta_i);
    void setMotor(int dir, int pwmVal);
    float omegaTarget;
    float u;
};

#endif 
