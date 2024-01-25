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

// 317 pulses per revolution
volatile int EncAPulse = 0;
volatile int EncBPulse = 0;

float CNVRT_pulses_to_meters = 0.00089;    // meters per pulse OR (~)1,100 pulses every meter

float EncA_Pos = 0;
float EncB_Pos = 0;


// CREATING OBJECTS
CMotor motorA(MotorAOut, MotorADIR, EncA_A, EncA_B);
CMotor motorB(MotorBOut, MotorBDIR, EncB_A, EncB_B);


float currT, prevT = 0;
float prevTheta, prevX = 0;
float K[4] = {-22.3607, -14.5333, -29.7255, -4.0219};


void setup() {
  // Declare Interrupts
  attachInterrupt(digitalPinToInterrupt(EncA_A), EncA_Counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncB_A), EncB_Counter, CHANGE);

  // Start Serial communication
  Serial.begin(9600);
  
  // Initialize GYRO
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() 
{
  /*
  digitalWrite(MotorADIR, 1);
  analogWrite(MotorAOut,120);

  Serial.println(EncAPulse);
  */
  // Get dt 
  currT = micros();
  float dt = (currT - prevT)/1.0e6;
  prevT = currT;

  // Read sensor data
  mpu.update();
  float theta = mpu.getAngleY()*0.0174;                // Pendulum angle (+offset)
  float theta_dot = (theta - prevTheta)/dt;           // Pendulum angular velocity
  float x = EncAPulse*CNVRT_pulses_to_meters;         // Cart displacement
  float x_dot = (x - prevX)/dt;                       // Cart velocity
  
  // Store previous values
  prevTheta = theta;
  prevX = x;    

  // Calculate control input
  float controlInput = -K[2] * theta - K[3] * theta_dot - K[0] * x - K[1] * x_dot;

  int motorSpeed = (fabs(controlInput)*(255-0)/6) + 0;
  
  int dir = 1;
  if(controlInput > 0){
    dir = 0;
  }
  
  motorA.setMotor(dir, motorSpeed);
  motorB.setMotor(!dir, motorSpeed);

  Serial.print(theta);
  Serial.print(" ");
  Serial.print(controlInput);
  Serial.print(" ");
  Serial.print(motorSpeed);
  Serial.println();
  
  /*
  //receiveAngle();    //Get the angle of tilt
  
  //calculateWheelSpeed();      //Calculates the velocity the wheel must turn according to receiveAngle (TARGET WHEEL SPEED)


  // Set a Position Target 
  //target = 150;                        // Constant
  //target = 100*sin(prevT/1e6);       // Sine wave
  //target = 150*(sin(prevT/1e6)>0);   // Square wave
  motorA.target = GyZ/500;   // Angle reference
  EncA_Pos = EncAPulse * CNVRT_pulses_to_meters;
  EncB_Pos = EncBPulse * CNVRT_pulses_to_meters;
  
  motorA.calculate(EncAPulse);
  motorB.calculate(EncBPulse);
  
  motorA.setMotor(motorA.dir, motorA.power);
  motorB.setMotor(motorB.dir, motorB.power);
  Serial.println((String)" Motor A speed: " + motorA.power + "; Motor B speed: " + motorB.power);
  */
}
