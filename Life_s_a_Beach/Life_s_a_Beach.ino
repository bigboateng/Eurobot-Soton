#include "Wire.h"
#include <Servo.h>

// Amount of time per round
const unsigned int TIMEOUT = 90000;//90sec*1000millisec
const unsigned short int DELAY = 250; // Some delay time. 
const int ROBOT_LENGTH = 210; // Seperation of the two wheels in mm
const int WIRE_ADR = 0x58; //Wire transmission address: 7-bit address of the device to transmit to

void setup() {
  //Initialize Wire and set acceleration settings
  Wire.begin();
  
  delay(DELAY);
  reset_encoders();
  delay(DELAY);
  
  Wire.beginTransmission(WIRE_ADR);
  Wire.write(0xE);
  Wire.write(0x2);
  Wire.endTransmission(true);

  Serial.begin(9600);
}

void loop() {
 //main code!
}

// returns false if there round has not finished
// else it stops the motors and enters an infinate loop 
boolean is_the_end(){
  if (millis() > TIMEOUT){
    Serial.println("time is up!");
    stop_motors();
    while(1){};
  }
}

