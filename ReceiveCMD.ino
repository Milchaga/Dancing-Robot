#include "Arduino.h"
#include "Wire.h"
#include <MPU6050_light.h>

//Read the YAW angle from the GYRO
void receiveAngle() {
  //mpu.update();
  //currentAngle = mpu.getAngleY();
  //Serial.println((String)"Angle is: " + currentAngle);
}


/// Transfer Funciton: Output = Wheel speed/position; Input = Angle 
void TTF() {
  //motorA.target = 200* sin(currentAngle* (PI / 180.0));
  //motorB.target = 200* sin(currentAngle* (PI / 180.0));
  //Serial.println((String)"Target Speed is: " + motorA.vTarget); 
}
