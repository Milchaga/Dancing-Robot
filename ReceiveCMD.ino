#include "Arduino.h"
#include "Wire.h"
#include <MPU6050_light.h>

//Read the YAW angle from the GYRO
void receiveTargetVelocity() {
  mpu.update();
  currentAngle = mpu.getAngleY();
  //Serial.println((String)"Angle is: " + currentAngle);
}


/// Transfer Funciton: Output = Wheel speed; Input = Angle 
  /// Handled by ESP32 ??

  //motorA.vTarget = 200* sin(currentAngle* (PI / 180.0));
  //motorB.vTarget = 200* sin(currentAngle* (PI / 180.0));
  //Serial.println((String)"Target Speed is: " + motorA.vTarget);
