#include "Wire.h"
#include <MPU6050_light.h>
#include <Arduino.h>
#include "CMotor.h"

MPU6050 mpu(Wire);

// Check which PINs on the ARDUINO MEGA have Interrupts
const int EncA_A = 18; //Encoder A - Channel A
const int EncA_B = 52;  //Encoder B - Channel B
const int EncB_A = 19; //Encoder A - Channel A
const int EncB_B = 28;  //Encoder B - Channel B

const int MotorAOut = 5;   //Enable Motor A
const int MotorADIR = 48;  //Direction Motor A
const int MotorBOut = 6;   //Enable Motor B
const int MotorBDIR = 26;  //Direction Motor B

// 314 pulses per revolution
volatile int EncAPulse = 0;
volatile int EncBPulse = 0;

float CNVRT_pulses_to_meters = 0.00072;    // meters per pulse OR (~)1,400 pulses every meter

float EncA_Pos = 0;
float EncB_Pos = 0;

// CREATING OBJECTS
CMotor motorA(MotorAOut, MotorADIR, EncA_A, EncA_B);
CMotor motorB(MotorBOut, MotorBDIR, EncB_A, EncB_B);

float currentAngle = 0;

void setup() {
  // Declare Interrupts
  attachInterrupt(digitalPinToInterrupt(EncA_A), EncA_Counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncB_A), EncB_Counter, CHANGE);

  // Start Serial communication
  Serial.begin(9600);
  Serial.setTimeout(100);
  
  // Initialize GYRO
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ Serial.println(mpu.begin()); } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true);   // gyro and accelero
  mpu.setFilterGyroCoef(0.98);  // Use primarily GYRO
  Serial.println("Done!\n");
}

void loop() 
{ 
  mpu.update();
  currentAngle = mpu.getAngleY(); 
  
  // Set a Position Target 
  //target = 150;                        // Constant
  //target = 100*sin(prevT/1e6);       // Sine wave
  //target = 150*(sin(prevT/1e6)>0);   // Square wave
  motorA.target = currentAngle;   // Angle reference
  
  motorA.calculate(EncAPulse);
  motorA.setMotor(motorA.dir, motorA.pwr);
  //motorB.setMotor(motorA.dir, motorA.pwr);

  
  /*
  //receiveAngle();    //Get the angle of tilt
  
  //calculateWheelSpeed();      //Calculates the velocity the wheel must turn according to receiveAngle (TARGET WHEEL SPEED)
    
  EncA_Pos = EncAPulse * CNVRT_pulses_to_meters;
  EncB_Pos = EncBPulse * CNVRT_pulses_to_meters;
  
  motorA.calculate(EncAPulse);
  motorB.calculate(EncBPulse);
  
  motorA.setMotor(motorA.dir, motorA.power);
  motorB.setMotor(motorB.dir, motorB.power);
  Serial.println((String)" Motor A speed: " + motorA.power + "; Motor B speed: " + motorB.power);
  */
}
