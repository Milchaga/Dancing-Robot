#ifndef CMotor_h
#define CMotor_h

#include "Arduino.h"

class CMotor{
  private:
    int motorEnable;
    int motorDirection;
    int mEncA;
    int mEncB;
    int encoderTicksPerRev = 314;

  public:
    CMotor(int enPin, int dirPin, int SA, int SB);
    ~CMotor();
    void setMotor(int dir, int pwmVal);
    int dir;
    float pwr;
};

#endif 
