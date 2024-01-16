#ifndef CMotor_h
#define CMotor_h

#include "Arduino.h"

class CMotor{
  private:
    long prevT = 0;
    int thetaPrev = 0;
    float eintegral = 0;
    float ePrev = 0;
    float dedt = 0;

    int motorEnable;
    int motorDirection;
    int mEncA;
    int mEncB;
    int encoderTicksPerRev = 314;
    
    int theta_i;
    float kd;
    float ki;
    float kp;

  public:
    int dir;
    float pwr;
    int theta;
    CMotor(int enPin, int dirPin, int SA, int SB);
    ~CMotor();
    void calculate(int theta_i);
    void setMotor(int dir, int pwmVal);
    float target;
    float u;
};

#endif 
