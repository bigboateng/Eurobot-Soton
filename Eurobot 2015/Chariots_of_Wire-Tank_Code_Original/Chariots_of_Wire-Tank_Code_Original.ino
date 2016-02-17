//Chariots of Wire
// test
// 'The libraries used in this code have been released under a Creative Commons Attribution-ShareAlike 3.0 License, allowing users to adapt and share freely, even for commercial purposes. All credit for creation of the libraries used rests with the respective authors.'
// Abi Basham, Anders Mundal, Liam Nash, Jamie Nash, Simon Kirby


#include "Wire.h"
#include <Servo.h>

Servo yellow_clapper;
Servo green_clapper;
Servo port_magnet;
Servo starboard_magnet;
Servo port_carpetholder;
Servo starboard_carpetholder;

//Forward declarations
void checkpoint_reached();
int move_forward(int mm, bool avoid);
void rotate(int deg);
float turn(int deg, int r);
long encoder();
void reset_encoders();


//Globals
const unsigned short int delay_time = 250;
const unsigned short int encoder_delay = 10;
const unsigned short int motor_sync_delay = 0;
const unsigned char butt_pin = 5;
const unsigned char pull_pin = 4;
const unsigned char green_clapper_pin = 6;
const unsigned char yellow_clapper_pin = 9;
const unsigned char starboard_clap_bottom_pin = 7;
const unsigned char port_clap_bottom_pin = 2;
const unsigned char starboard_clap_top_pin = 10;
const unsigned char port_clap_top_pin = 8;

const unsigned char red_pin = 12;

const float count_length = 0.488692f;
const float wheel_dist = 210.0f;

const int obstacle_distance[4] = {0,10,10,0};
const int forward_speed = 0x94;
const int drive_speed = 0xA8;
const int antidrive_speed = 0x58;
const int reverse_speed = 0x6C;

//Globals which should be optimized, in order.
const int max_turn_step = 17; // decrease if robot is detracking.
const int retrack_distance = 140; //increase if robot is not retracking fully.
const float turn_factor = 0.70f; // increase if robot is overturning.
const float stop_distance = 0.0f; // increase if robot is falling short on distance.
const float slip_factor = 1.0f;
const float logstop_distance = 30.0f;

unsigned long timeout = 0;
int pull = LOW;
int butt = LOW;
int pos = 0;
bool obstacle = false;
int motoramps;

void setup() {
  // put your setup code here, to run once:

  //Initialize Arduino pins
  pinMode(red_pin, OUTPUT);
  pinMode(butt_pin, INPUT);
  pinMode(pull_pin, INPUT);



  //Initialize Wire and set acceleration settings
  Wire.begin();
  delay(delay_time);
  reset_encoders();
  delay(delay_time);
  Wire.beginTransmission(0x58);
  Wire.write(0xE);
  Wire.write(0x2);
  Wire.endTransmission(true);

  Serial.begin(9600);

}

void loop() {
  bool yellow = false;
  bool pull_primed = true;
  int odometer = 0;
  digitalWrite(red_pin, LOW);
  while (pull_primed) {
    //robot should wait here until button pressed

    pull = digitalRead(pull_pin);
    if (pull == HIGH) {
      //button is using pull-down switch so low voltage = closed.
      pull_primed = false;
    }

    butt = digitalRead(butt_pin);
    if (butt == LOW) {
      if (yellow) {
        digitalWrite(red_pin, LOW);
      }
      else {
        digitalWrite(red_pin, HIGH);
      }
      yellow = !yellow;
    }
    delay(100);
  }
  timeout = millis() + 90000;


  // starboard_carpetholder.attach(starboard_clap_top_pin);
  // port_carpetholder.attach(port_clap_top_pin);

  // starboard_carpetholder.write(140);
  // port_carpetholder.write(10);

  // delay(5000);
  if (yellow) {
    halting_forward(830);
    clever_turn(-90);
    halting_forward(300);
    move_forward(140, false);
    port_carpetholder.attach(port_clap_top_pin);
    port_carpetholder.write(0);
    move_forward(400, false);
    port_magnet.attach(port_clap_bottom_pin);
    for(int i = 0; i < 8; i++) {
      rotate(1, 30);
      port_magnet.write(20); //lol jk
      delay(20);
      rotate(2, 30);
      delay(20);
      move_forward(-10, false);
      delay(20);
      rotate(1, -15);
      delay(20);
      rotate(2, -15);
      delay(20);
    }
    move_forward(-200, false);
    starboard_carpetholder.attach(starboard_clap_top_pin);
    starboard_carpetholder.write(150);
    move_forward(200, false);
    starboard_magnet.attach(starboard_clap_bottom_pin);
    starboard_magnet.write(110);
    move_forward(40, false);


    // halting_forward(100);
    // clever_turn(99);
    // halting_forward(600);
    // move_forward(100,false);
    // delay(delay_time);
    // clever_nurt(-45);
    // move_forward(200,false);
  }
  else {
    halting_forward(800);
    clever_turn(100);
    halting_forward(300);
    move_forward(150, false);
    starboard_carpetholder.attach(starboard_clap_top_pin);
    starboard_carpetholder.write(150);
    move_forward(400, false);
    starboard_magnet.attach(starboard_clap_bottom_pin);
    for(int i = 0; i < 8; i++) {
      rotate(2, 30);
      starboard_magnet.write(110);
      delay(20);
      rotate(1, 25);
      delay(20);
      move_forward(-15, false);
      delay(20);
      rotate(2, -15);
      delay(20);
      rotate(1, -15);
      delay(20);
    }
    move_forward(-300, false);
    port_carpetholder.attach(port_clap_top_pin);
    port_carpetholder.write(0);
    move_forward(300, false);
    port_magnet.attach(port_clap_bottom_pin);
    port_magnet.write(20);
    move_forward(50, false);

    // starboard_carpetholder.attach(starboard_clap_top_pin);
    // starboard_carpetholder.write(180);
    // delay(1000);
    // starboard_carpetholder.write(160);

    // halting_forward(50);
    // clever_turn(-90);
    // halting_forward(700);
    // clever_nurt(90);
    // move_forward(-670, false);
    // delay(delay_time);
    // move_forward(700, false);
    // clever_turn(50);
    // move_forward(100,false);
    // halting_forward(400);
    // clever_turn(50);
    // halting_forward(650);
    // delay(delay_time);
    // move_forward(600, false);

  }
  pull_primed = true;
  while (pull_primed) {
    //robot should wait here until button pressed
    butt = digitalRead(butt_pin);
    if (butt == LOW) {
      //button is using pull-down switch so low voltage = closed.
      pull_primed = false;
    }
    delay(500);
  }
  delay(1000);
}
//this function finishes quickly before the time limit, but loops indefinitely afterwards.
void time_check() {
  unsigned long time = millis();
  // Serial.print("Time remaining = ");
  // Serial.println(timeout - time);
  while(time > timeout){
    Serial.println("time is up!");
    //Stop both motors
    Wire.beginTransmission(0x58);
    Wire.write(0x0);
    Wire.write(0x80);
    Wire.endTransmission(true);

    Wire.beginTransmission(0x58);
    Wire.write(0x1);
    Wire.write(0x80);
    Wire.endTransmission(true);
    delay(100000);
  }
}

//this function should turn the robot in a method sympathetic to the tracks.
//positive = clockwise
void clever_turn(int deg) {
  int odometer = 0;
  int turn_remaining = deg;
  int retrack_remaining = 0;
  int max_turn_thresh = static_cast<int>(max_turn_step * turn_factor);
  while (turn_remaining != 0) {
        //even numbered steps below
    if (turn_remaining > max_turn_thresh) {
      rotate(2,-max_turn_step);
      turn_remaining = turn_remaining - max_turn_thresh;
    }
    else if (turn_remaining < -max_turn_thresh) {
      rotate(1,-max_turn_step);
      turn_remaining = turn_remaining + max_turn_thresh;
    }
    else if (turn_remaining > 0){
      rotate(2,-static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    else if (turn_remaining < 0){
      rotate(1,static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    delay(delay_time);

    //odd numbered steps below
    if (turn_remaining > max_turn_thresh) {
      rotate(1,max_turn_step);
      turn_remaining = turn_remaining - max_turn_thresh;
      //now move forward and back to help the tracks back on.
      retrack_remaining = retrack_distance;
      while(retrack_remaining > 0) {
        delay(delay_time);
        odometer = move_forward(retrack_remaining, true);
        delay(delay_time);
        move_forward(-odometer, false);
        retrack_remaining -= odometer;
      }
    }
    else if (turn_remaining < -max_turn_thresh) {
      rotate(2,max_turn_step);
      turn_remaining = turn_remaining + max_turn_thresh;
      //now move forward and back to help the tracks back on.
      retrack_remaining = retrack_distance;
      while(retrack_remaining > 0) {
        delay(delay_time);
        odometer = move_forward(retrack_remaining, true);
        delay(delay_time);
        move_forward(-odometer, false);
        retrack_remaining -= odometer;
      }
    }
    else if (turn_remaining > 0){
      rotate(1,static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    else if (turn_remaining < 0){
      rotate(2,-static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    delay(delay_time);


  }
}

//this function should turn the robot in a method sympathetic to the tracks.
//positive = clockwise
void clever_nurt(int deg) {
  int odometer = 0;
  int turn_remaining = deg;
  int max_turn_thresh = static_cast<int>(max_turn_step * turn_factor);
  while (turn_remaining != 0) {
        //even numbered steps below
    if (turn_remaining > max_turn_thresh) {
      rotate(2,-max_turn_step);
      turn_remaining = turn_remaining - max_turn_thresh;
    }
    else if (turn_remaining < -max_turn_thresh) {
      rotate(1,-max_turn_step);
      turn_remaining = turn_remaining + max_turn_thresh;
    }
    else if (turn_remaining > 0){
      rotate(2,-static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    else if (turn_remaining < 0){
      rotate(1,-static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    delay(delay_time);

    //odd numbered steps below
    if (turn_remaining > max_turn_thresh) {
      rotate(1,max_turn_step);
      turn_remaining = turn_remaining - max_turn_thresh;
      //now move forward and back to help the tracks back on.
      odometer = move_forward(retrack_distance, true);
      delay(delay_time);
      move_forward(-odometer, false);
      delay(delay_time);
    }
    else if (turn_remaining < -max_turn_thresh) {
      rotate(2,max_turn_step);
      turn_remaining = turn_remaining + max_turn_thresh;
      //now move forward and back to help the tracks back on.
      odometer = move_forward(retrack_distance, true);
      delay(delay_time);
      move_forward(-odometer, false);
      delay(delay_time);
    }
    else if (turn_remaining > 0){
      rotate(1,static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    else if (turn_remaining < 0){
      rotate(2,static_cast<int>(turn_remaining / turn_factor));
      turn_remaining = 0;
    }
    delay(delay_time);
  }
}

// void play_tunez() {
  // Wire.requestFrom(2,6);

  // while(Wire.available())
  // {
    // char c = Wire.read(); // receive a byte as character
    // Serial.print(c);         // print the character
  // }
// }

//Move forward the given distance in mm
//use avoid = True to allow obstacle avoidance
//use avoid = False for a dumb movement
int move_forward(int mm, bool avoid) {
  int motor_speed = drive_speed;
  if (mm < 0) {
    motor_speed = antidrive_speed;
  }
  obstacle = false;
  reset_encoders();

  //Add fudgefactors and change from mm to wheel rotation counts
  float new_mm = (abs(mm) * slip_factor) - stop_distance;

  int required_counts = static_cast<int>(new_mm / count_length);

  //Start motor 1
  Wire.beginTransmission(0x58);
  Wire.write(0x0);
  Wire.write(motor_speed);
  Wire.endTransmission(true);

  delay(motor_sync_delay);

  //Start motor 2
  Wire.beginTransmission(0x58);
  Wire.write(0x1);
  Wire.write(motor_speed);
  Wire.endTransmission(true);

  long count1 = 0;
  long count2 = 0;

  //Run motors until desired count is reached
  while((count1 < required_counts) && (count2 < required_counts) && (!obstacle)) {
    count2 = count1;
    count1 = abs(encoder(2));
    delay(encoder_delay);
    for(int i = 0; i < 4; i++) {
      if ((proximity(i) < obstacle_distance[i]) && avoid) {
        obstacle = true;
      }
    }

  }

  //Stop both motors
  Wire.beginTransmission(0x58);
  Wire.write(0x0);
  Wire.write(0x80);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x58);
  Wire.write(0x1);
  Wire.write(0x80);
  Wire.endTransmission(true);

  int odometer = ((int)count1);
  odometer = static_cast<int>(odometer * count_length);
  return odometer;
}

//Rotate by leaving one side stationary and turning other side.
//side argument is the track to turn, deg is the amount of degrees to turn.
void rotate(int side, int deg) {
  reset_encoders();

  int turn_velocity = forward_speed;
  int reg = 0x0;
  if (side == 2) {
    reg = 0x1;
  }
  if (deg < 0) {
    turn_velocity = reverse_speed;
  }

  //Convert degrees to distance and then wheel counts
  float rad = (abs(deg) * 3.141593f) / 180.0f;
  float dist = rad * wheel_dist;
  int required_counts = static_cast<int>(dist / count_length);


  //Start motor
  Wire.beginTransmission(0x58);
  Wire.write(reg);
  Wire.write(turn_velocity);
  Wire.endTransmission(true);
  long count1 = 0;
  long count2 = 0;

  if (deg < 0) {
    //Wait until desired count is reached
    while((count1 > -required_counts) && (count2 > -required_counts)) {
      count2 = count1;
      count1 = encoder(side);
      // Serial.print(side);
      // Serial.print("    -->     ");
      // Serial.println(count1);
      delay(encoder_delay);
    }
  }

  else {
    //Wait until desired count is reached
    while((count1 < required_counts) && (count2 < required_counts)) {
      count2 = count1;
      count1 = encoder(side);
      delay(encoder_delay);
    }
  }

  //Stop both motors
  Wire.beginTransmission(0x58);
  Wire.write(0x0);
  Wire.write(0x80);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x58);
  Wire.write(0x1);
  Wire.write(0x80);
  Wire.endTransmission(true);
}

//this short function is instructions to move forward with very basic obstacle avoidance.
void halting_forward(int mm) {
  int odometer = 0;
  int last_step = 0;
  float correction = 0.0f;
  while(odometer < mm) {
    last_step = move_forward(mm - odometer, true);
    correction = logstop_distance * (1 - pow(2.71828, ((-0.1054 * last_step) / 40.0)));
    odometer += last_step + static_cast<int>(correction);
    delay(delay_time);
  }
}

//Get the encoder values.
//side = 1 for motor 1 (port)
//side = 2 for motor 2 (starboard)
long encoder(int side) {
  int reg = 0x6;
  if (side == 1) {
    reg = 0x2;
  }

  //Get Position of encoder 1 for counting
  Wire.beginTransmission(0x58);
  Wire.write(reg);
  Wire.endTransmission(true);

  Wire.requestFrom(0x58, 4);
  while(Wire.available() < 4) {}

  time_check();


  //Read in the values
  long pass1;
  pass1 = Wire.read();
  pass1 <<= 8;
  pass1 += Wire.read();
  pass1 <<= 8;
  pass1 += Wire.read();
  pass1 <<= 8;
  pass1 += Wire.read();
  //Return the values
  return pass1;

  if (side == 1){
    // Serial.print(pass1);
    // Serial.print("\t");
    // Serial.println(millis());
  }
}

//Get the motor amps.
//side = 1 for motor 1 (port)
//side = 2 for motor 2 (starboard)
int amps(int side) {
  int reg = 0xC;
  if (side == 1) {
    reg = 0xB;
  }

  Wire.beginTransmission(0x58);
  Wire.write(reg);
  Wire.endTransmission(true);

  Wire.requestFrom(0x58, 1);
  while(Wire.available() < 1) {}

  //Read in the values
  int motoramp = Wire.read();

  //Return the values
  return motoramp;
}

//Find out how far away a sensor predicts the bad guys to be.
//Argument is whichever analog pin you want to read off of.
int proximity(int sensor) {

  if (sensor == 0) {
    return 12343.85 * pow(analogRead(A0),-1.15);
  }
  if (sensor == 1) {
    return 12343.85 * pow(analogRead(A1),-1.15);
  }
  if (sensor == 2) {
    return 12343.85 * pow(analogRead(A2),-1.15);
  }
  if (sensor == 3) {
    return 12343.85 * pow(analogRead(A3),-1.15);
  }
}

//Reset the values for both encoders
void reset_encoders() {

  Wire.beginTransmission(0x58);
  Wire.write(0x10);
  Wire.write(0x20);
  Wire.endTransmission(true);
}
