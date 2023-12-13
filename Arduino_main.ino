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

float CNVRT_pulses_to_meters = 0.00072;    //meters per pulse OR (~)1,400 pulses every meter

float EncA_Pos = 0;
float EncB_Pos = 0;

// CREATING OBJECTS
CMotor motorA(MotorAOut, MotorADIR, EncA_A, EncA_B);
CMotor motorB(MotorBOut, MotorBDIR, EncB_A, EncB_B);

float currentAngle;

long prevT = 0;
float posPrev = 0;
float eprev = 0;
float eintegral = 0;
float velocityFilt = 0;
float velocityPrev = 0;
int dirB = 1;

void setup() {
  // Declare INPUT PinModes
  pinMode(EncA_A, INPUT_PULLUP);
  pinMode(EncA_B, INPUT_PULLUP);
  pinMode(EncB_A, INPUT_PULLUP);
  pinMode(EncB_B, INPUT_PULLUP);
  

  // Declare Interrupts
  //attachInterrupt(digitalPinToInterrupt(EncA_A), EncA_Counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncB_A), EncB_Counter, CHANGE);

  // Declare OUTPUT PinModes
  pinMode(MotorADIR, OUTPUT); // direction pin motor A - LOW = counterclockwise
  pinMode(MotorBDIR, OUTPUT); // direction pin motor B - LOW = counterclockwise
  pinMode(MotorAOut, OUTPUT); 
  pinMode(MotorBOut, OUTPUT);  

  // Start Serial communication
  Serial.begin(9600);
  Serial.setTimeout(100);
  
  // Initialize GYRO
  /*
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ Serial.println(mpu.begin()); } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true);   // gyro and accelero
  mpu.setFilterGyroCoef(0.98);  // Use primarily GYRO
  Serial.println("Done!\n");
  */
 
}

void loop() 
{  
    // Read the position
    int pos = 0; 
    noInterrupts(); // disable interrupts temporarily while reading
    pos = EncBPulse;
    interrupts(); // turn interrupts back on
    
    // Compute velocity
    long currT = micros();
    float deltaT = ((float) (currT-prevT))/1.0e6;
    float velocity = (pos - posPrev)/deltaT;
    posPrev = pos;
    prevT = currT;

    velocity = velocity/314*60.0;

    // Low-pass filter (25 Hz cutoff)
    velocityFilt = 0.854*velocityFilt + 0.0728*velocity + 0.0728*velocityPrev;  //0.0728
    velocityPrev = velocity;

    // Set a target
    //float vTarget = 150;                        // Constant
    float vTarget = 15*sin(currT/1e6);       // Sine wave
    //float vTarget = 15*(sin(currT/1e6)>0);   // Square wave
    

    // PID constants
    float kp = 3.25;   //2.5
    float kd = 0.115;   //0.1
    float ki = 0.06; //0.012
    
    // Control signal
    int e = vTarget - velocityFilt;
    float dedt = (e-eprev)/(deltaT);
    eprev = e;
    eintegral = eintegral + e*deltaT;
    
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    float pwrB = fabs(u);
    if( pwrB > 255 ){
      pwrB = 255;
    }
  
    // motor direction
    dirB = 1;
    if(u<0){
      dirB = -1;
    }

    Serial.print(vTarget);
    Serial.print(" ");
    Serial.print(velocityFilt);
    Serial.println();
    motorB.setMotor(dirB, pwrB);
    
          
    /*
    //receiveTargetVelocity();    //Get TARGET WHEEL SPEED from ESP32
    
    //calculateWheelSpeed();      //Calculates the velocity the wheel must turn according to receiveAngle (TARGET WHEEL SPEED)
    motorA.omegaTarget = 80;    // RPM !!!
    motorB.omegaTarget = 80;    // RPM !!!
    
    EncA_Pos = EncAPulse * CNVRT_pulses_to_meters;
    EncB_Pos = EncBPulse * CNVRT_pulses_to_meters;
    
    motorA.calculate(EncAPulse);
    motorB.calculate(EncBPulse);
    
    motorA.setMotor(motorA.dir, motorA.power);
    motorB.setMotor(motorB.dir, motorB.power);
    Serial.println((String)" Motor A speed: " + motorA.power + "; Motor B speed: " + motorB.power);
    */
}
