Arduino Handover

Hardware:
- All the pin connections can be verified from the Arduino_main.ino file. Every pin is defined and has an assignned number to it.
- In the electrical schematic, it is should that the ESP32 is connected using UART, but that is not the case. The communicaiton protocol is changed to I2C.
- The ESP32 and the MPU6050 (Gyro & Acc) can be connected to the same SCL and SDA pins on the Arduino.
- *** If changes are going to be made for the wiring, keep in mind that not all pins are suitalbe for every task. For example, the encoder channel A of both motors MUST be connected to a pin on the Arduino which has interrupts inbuilt. 
- The Pmod HB5 (H-Bridge) on motor B doesnt not work (burned). It has to be changed.
- The motors require 6V to operate. However, those 6V are split between the motors and the encoders. So, the max operating voltage of the motors is 4.45V (measured at full speed)
- When initializing, the robot has to be staying still in vertical position, so that when the robot is upstraight, the sensor (Gyro) will measure 0 degrees.

Software:
- The controller is a full-state feedback. It uses K values for each state. 
- PID was another option, but the performance was worse.
- The communication with the ESP32 is not implemented. 
- The reference following is not tested / working.
