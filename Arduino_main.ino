#include "Wire.h"
#include <MPU6050_light.h>
#include <Arduino.h>
#include "CMotor.h"

MPU6050 mpu(Wire);

// Define motor pin connections
const int EncA_A = 18;    //Encoder A - Channel A
const int EncA_B = 52;    //Encoder A - Channel B
const int EncB_A = 19;    //Encoder B - Channel A
const int EncB_B = 28;    //Encoder B - Channel B

const int MotorAOut = 5;   //Enable Motor A
const int MotorADIR = 48;  //Direction Motor A
const int MotorBOut = 6;   //Enable Motor B
const int MotorBDIR = 26;  //Direction Motor B

// 317 pulses per revolution
volatile int EncAPulse = 0;
volatile int EncBPulse = 0;

float CNVRT_pulses_to_meters = 0.00089;    // meters per pulse OR (~)1,100 pulses every meter

// CREATING OBJECTS
CMotor motorA(MotorAOut, MotorADIR, EncA_A, EncA_B);
CMotor motorB(MotorBOut, MotorBDIR, EncB_A, EncB_B);

// State Space variables
float currT, prevT = 0;
float prevTheta, prevX = 0;
float factor = 20;
float K[4] = {-0.2, -0.2, 1.2, 0.2};    // x, x_dot, theta, theta_dot
float Nbar = -3.16; 
float ref = 0;

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
  // Get dt 
  currT = micros();
  float dt = (currT - prevT)/1.0e6;
  prevT = currT;

  // Read sensor data
  mpu.update();
  // Get all 4 states
  float theta = -(3.2*mpu.getAngleY()*0.0174) + 0.0;        // Pendulum angle [rad] (+offset)
  float theta_dot = LPF(theta,prevTheta,dt);           // Pendulum angular velocity [rad/s]
  float x = EncAPulse*CNVRT_pulses_to_meters;          // Body translational displacement [m]
  float x_dot = LPF(x,prevX,dt);                       // Body translational velocity [m/s]
  
  // Store previous values
  prevTheta = theta;
  prevX = x;    

  // Calculate control input u = -Kx + Nbar*ref
  float controlInput = -K[2] * theta - K[3] * theta_dot - K[0] * x - K[1] * x_dot + Nbar*ref;
  // Limit the maximum output in voltage
  controlInput = (controlInput > 4.45) ? 4.45 : controlInput;
  controlInput = (controlInput < -4.45) ? -4.45 : controlInput;

  // Assign correct analog value to the required voltage with a deadzone
  int motorSpeed = (fabs(controlInput)*(255-52)/4.5) + 52;

  // Assign correct direction based on the required u
  int dir = (controlInput > 0) ? 0 : 1;
  
  motorA.setMotor(!dir, motorSpeed);
  motorB.setMotor(dir, motorSpeed);

  /*
  //  Monitor theta; x; and u   // 
  mpu.update();
  Serial.print(-mpu.getAngleY());
  Serial.println();
  Serial.print(" ");
  Serial.print(theta_dot*57.29);
  Serial.print(" ");
  Serial.print(10*x);
  Serial.print(" ");
  Serial.print(10*x_dot);
  Serial.println();
  */
  
}
