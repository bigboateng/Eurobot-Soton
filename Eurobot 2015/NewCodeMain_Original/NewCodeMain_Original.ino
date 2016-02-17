/* 
Southampton New Code
Jelmer van den Dries
23-4
*/

//Need these librarie
#include <SoftwareSerial.h>
#include <Wire.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

//MD25 Comms
#define CMD                 0x10                        
                                                              // This is a bug with arduino 1
#define MD25ADDRESS         0x58                              // Address of the MD25
#define SOFTWAREREG         0x0D                              // Byte to read the software version, Values of 0 eing sent using write have to be cast as a byte to stop them being misinterperted as NULL
#define SPEED1              (byte)0x00                        // Byte to send speed to first motor
#define SPEED2              0x01                              // Byte to send speed to second motor
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATE          0x0E                              // Byte to send Acceleration
#define RESETENCODERS       0x20

//Sensor Definitions                                           //
//#define ECHO_PIN_FL    5
#define ECHO_PIN_FR    3 
#define ECHO_PIN_REAR  7

#define TRIGGER_PIN_FR    2
//#define TRIGGER_PIN_FL    4
#define TRIGGER_PIN_REAR  6

#define MAX_DISTANCE   200                                      //Maximum Distance in cm
#define STOP_DISTANCE  7                                       // danger distance at the rear
#define TURN_DISTANCE  30                                      // danger distance at the front

//defining states
//shared states
#define DONE       99
#define START      0   
#define AVOID      1

//yellow states
#define AB         2
#define BC         3
#define CARPETY    4 
#define DB         5
#define BCUPONE    6
#define BSTANDS    7
#define CLPY       8
#define CLP3Y      9
#define YFANCY    10

//green states
#define ZY      80
#define YX      79
#define CARPETG 78
#define WY      77
#define GCUPONE 76
#define YSTANDS 75
#define CLPG    74
#define CLP4G   73
#define GFANCY  72

//side data
#define YELLOW  0
#define GREEN   1

//bumper data
#define NONE 0
#define RIGHT 1
#define LEFT 2
#define BOTH 3

//Robot calibration values. EDIT THESE VALUES FOR CALIBRATING.-----------------------------------------------------------------
//servo values:
const int bridge_up          = 7;
const int bridge_carpet      = 30;
const int bridge_down        = 130;
const int bridge_mid         = 65;
const int claw_open          = 55;
const int claw_shut          = 120;
const int claw_stand         = 120; 
const int claw_cup           = 117;
const int claw_2_stands      = 100;
const int arm_out            = 150;
const int arm_back           = 110;

//Shared values:
const int sensor_count       = 2;
const int sdt                = 300;                                         //standard delay time to allow the robot to come to rest
const int clear_wall         = 60;
const int Left90             = -151;                                         //90 degree turn on encoder1, left turn must be negative
const int Right90            = 151;
const int clear_stands       = 500;
const int wall_2stands       = 70;
const int dist_bridge        = 43;
const int turn_around_stands = 340;
const int dist_wall_clap1    = 50;
const int return_to_stands   = 280;
const int go_home            = 450;

//Yellow distances;
const int dist_start_to_b   = 120; 
const int dist_b_to_c       = 500;
const int dist_cpt_to_standy= 140;
const int dist_stand_to_d   = 165; 
const int dist_d_to_b       = 760;
const int dist_b_to_drop    = 70;
const int dist_drop_to_b    = 200;
const int turn_at_b         = 225;                                     // non 90 degree turn (scary :/ )
const int dist_b_to_cup     = 125;
const int dist_cup_to_b     = 150;
const int turn_at_dropy     = 225;
const int dist_b_to_drop2   = 200;
const int dist_drop_to_b2   = 50;
const int turn_at_b2        = 217;
const int dist_b_to_e       = 825; // stage 2
const int turn_at_e         = 265;
const int dist_clap3        = 120;
const int dist_clap3_hit    = 50;
const int dist_to_f         = 360;   //stage 3
const int dist_clap1        = 180;
const int turn_to_cup3      = 125;
const int dist_f_to_cup3    = 130;
const int turn_at_cup3      = 190;
const int dist_cup3_to_line = 220;
const int dist_line_to_wally= 240;
const int turn_to_wall_f    = 10;
const int dist_to_2standsy  = 113;
const int reverse_standsy   = 0;
const int turn_at_clap1     = 80;
const int hit_clap1         = 80;

//Green distances;
const int dist_start_to_y   = 120; 
const int dist_y_to_x       = 500;
const int dist_cpt_to_standg= 140;
const int dist_stand_to_w   = 160; 
const int dist_w_to_y       = 740;
const int dist_y_to_drop    = 70;
const int dist_drop_to_y    = 200;
const int turn_at_y         = 225;                                     // non 90 degree turn (scary :/ )
const int dist_y_to_cup     = 125;
const int dist_cup_to_y     = 150;
const int turn_at_dropg     = 225;
const int dist_y_to_drop2   = 200;
const int dist_drop_to_y2   = 50;
const int turn_at_y2        = 218;
const int dist_y_to_v       = 820;   //stage 2
const int turn_at_v         = 270;
const int dist_clap4        = 120;
const int dist_clap4_hit    = 170;
const int dist_to_u         = 355;   //stage 3
const int dist_clap5        = 185;
const int turn_to_cup4      = 125;
const int dist_u_to_cup4    = 120;
const int turn_at_cup4      = 190;
const int dist_cup4_to_line = 220;
const int dist_line_to_wallg= 240;
const int turn_to_wall_u    = 10;
const int dist_to_2standsg  = 152;
const int reverse_standsg   = 0;
const int turn_at_clap6     = 80;
const int hit_clap6         = 80;



//bumper pins
const int leftBumper = 14;
const int rightBumper = 15;

//Stepper Motor
//declare variables for the motor pins
int motorPin1 = 8;    // Blue   - 28BYJ48 pin 1
int motorPin2 = 11;    // Pink   - 28BYJ48 pin 2
int motorPin3 = 12;    // Yellow - 28BYJ48 pin 3
int motorPin4 = 13;    // Orange - 28BYJ48 pin 4
                        // Red    - 28BYJ48 pin 5 (VCC)

int motorSpeed = 1200;  //variable to set stepper speed
int count = 0;          // count of steps made
int countsperrev = 512; // number of steps per full revolution
int lookup[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};

// NewPing setup of pins and maximum distance.
//NewPing sonarFL(TRIGGER_PIN_FL, ECHO_PIN_FL, MAX_DISTANCE);
NewPing sonarFR(TRIGGER_PIN_FR, ECHO_PIN_FR, MAX_DISTANCE);
NewPing sonarRear(TRIGGER_PIN_REAR, ECHO_PIN_REAR, MAX_DISTANCE);

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo myservoclaw, myservobridge;

unsigned int uS;
unsigned int distFL;
unsigned int distFR;
unsigned int distRear;
unsigned int distFront;

int objective = START;
int prevstate = START;

int posclaw = 55;
int posbridge;

unsigned long start_time = 0;           //shutdowntimer
unsigned long elapsed_time = 0;

const int pullCord_pin = 17;  //A3
const int colour_pin = 16; //A2 HIGH is yellow
int colour;

#define FR 0
#define FL 1
#define REAR 2
#define FRONT 3

int sensor;
int k = 0;
int s = 0;

void setup(){
  Wire.begin();
  delay(200);
  byte softVer = getSoft();
  lcd.init();
  lcd.backlight();
  setAcceleration(4);
  encodeReset();
  checkcolour();
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(pullCord_pin, INPUT);
  pinMode(colour_pin, INPUT);
  pinMode(leftBumper, INPUT);
  pinMode(rightBumper, INPUT);
  lcd.setCursor(0,0);
  //lcd.print(digitalRead(pullCord_pin));
  //objective = CLP4G;
  if (digitalRead(pullCord_pin) == HIGH){
    myservoclaw.attach(9);
    myservobridge.attach(10);
    bridgeCarpet();
    clawOpen();
    objective = START;
  }
  else {
    lcd.print("False Start");
    objective = DONE;
  }
}

void loop(){
  //square();
  /*
  clawOpen();
  delay(500);
  claw2Stands();
  delay(5000);
  //bridgeDown();
  //bridgeUp();
  //check_bumpers();
  */
  
  check_time();
  delay(35);
  //lcdState();
  if (colour == YELLOW){
    yellowMachine();
  }
  else {
    greenMachine();
  }
  
}

void yellowMachine(){
  if (objective == START){
    wait_for_start();
  }
  if (objective == AVOID){
    avoid();
  }
  if (objective == DONE){
    lcd.setCursor(0,1);
    lcd.print("   DONE    ");
    stopMotor();
  }
  if (objective == AB){
    AtoB();
  }
  if (objective == BC){
    BtoC();
  }
  if (objective == CARPETY){
    CarpetY();
  }
  if (objective == DB){
    DtoB();
  }
  if (objective == BCUPONE){
    BCup1();
  }
  if (objective == BSTANDS){
    BtoStands();
  }
  if (objective == CLPY){
    ClapperY();
  }
  if (objective == CLP3Y){
    Clapper3Y();
  }
  if (objective == YFANCY){
    YFancy();
  }
}

void AtoB(){
  if (encoder1() > dist_start_to_b){                       // edit this value for distance
      stopMotor();
      delay(sdt);
      encodeReset();
      turnLeft90();
      delay(sdt);
      encodeReset();
      objective = BC;                
    }
    else if (getDistanceFR() < TURN_DISTANCE){
      s++;
      if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }      
    }
    else {
      s = 0;
      moveForward(40);
    }
}

void BtoC(){
  if (encoder1() > dist_b_to_c){
        stopMotor();
        delay(sdt);
        encodeReset();
        turnLeft90();
        delay(sdt);
        encodeReset();
        objective = CARPETY;
      }
  else if (getDistanceFR() < TURN_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(60);
   }
}

void CarpetY(){
  do{
    moveForward(-20);
  }while (check_bumpers() != BOTH);
  stopMotor();
  bridgeDown();
  delay(500);
  encodeReset();
  bridgeUp();
  do{
    moveForward(15);
  }while (encoder1() < clear_wall);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_cpt_to_standy);
  stopMotor();
  clawStand();
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(40);
  }while (encoder1() < dist_stand_to_d);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  objective = DB;
}

void DtoB(){
  if (encoder1() > dist_d_to_b){
        stopMotor();
        delay(sdt);
        encodeReset();
        turnRight90();
        delay(sdt);
        encodeReset();
        objective = BCUPONE;
      }
  else if (getDistanceFR() < TURN_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(40);
   }
}

void BCup1(){
  do{
    moveForward(30);
  }while (encoder1() < dist_b_to_drop);
  stopMotor();
  clawOpen();
  encodeReset();
  do{
    moveForward(-50);
  }while (encoder1() > -dist_drop_to_b);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_at_b);                  // weird turn
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_b_to_cup);
  stopMotor();
  clawCup();
  delay(sdt);
  encodeReset();
  do{
    moveForward(20);
  }while (encoder1() < 50);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (encoder1() > -dist_cup_to_b);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(turn_at_dropy);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_b_to_drop2);
  stopMotor();
  clawOpen();
  encodeReset();
  do{
    moveForward(-30);
  }while (encoder1() > -dist_drop_to_b2);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(turn_at_b2);
  stopMotor();
  delay(sdt);
  encodeReset();
  objective = BSTANDS;
}

void BtoStands(){
  if (encoder1() > dist_b_to_e){
        objective = CLPY;
      }
  else if (getDistanceFR() < TURN_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(60);
   }
}

void ClapperY(){
  do{
    moveForward(40);
  }while (encoder1() < 250);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-40);
  }while (encoder1() > -clear_stands);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(turn_at_e);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < clear_wall);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(154);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-40);
  }while (encoder1() > -dist_clap3);
  stopMotor();
  armRight(arm_out);
  do{
    moveForward(40);
  }while (encoder1() < dist_clap3_hit);
  stopMotor();
  armLeft(arm_back);
  delay(sdt);
  encodeReset();
  objective = CLP3Y;
}

void Clapper3Y(){
  if (encoder1() < -dist_to_f){
        objective = YFANCY;
      }
  else if (getDistanceRear() < STOP_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = REAR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(-40);
   }
}

void YFancy(){
  do{
    moveForward(-30);
  }while (encoder1() > -dist_clap1);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();    // start reallign
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < clear_wall);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(turn_to_cup3);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_f_to_cup3);
  stopMotor();
  clawCup();
  delay(sdt);
  encodeReset();
  turnRight(turn_at_cup3);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(40);
  }while (encoder1() < dist_cup3_to_line);
  stopMotor();
  clawOpen();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-40);
  }while (encoder1() > -dist_line_to_wally);
  stopMotor();
  delay(sdt);
  turnLeft(turn_to_wall_f);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < wall_2stands);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_to_2standsy);           /////////
  stopMotor();
  claw2Stands();
  delay(333);
  encodeReset();
  do{
    moveForward(-30);
  }while (encoder1() > -reverse_standsy);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_around_stands);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < 40);
  stopMotor();
  delay(sdt); 
  clawOpen();
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH && encoder1() > -320);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < dist_wall_clap1);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < dist_bridge);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_at_clap1);
  stopMotor();
  delay(sdt);
  encodeReset();
  bridgeMid();
  turnLeft(hit_clap1);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < 40);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  bridgeUp2();
  encodeReset();
  do{
    moveForward(50);
  }while (encoder1() < return_to_stands);
  stopMotor();
  claw2Stands();
  delay(sdt);
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(50);
  }while (encoder1() < go_home);
  stopMotor();
  objective = DONE;
}


void greenMachine(){
  if (objective == START){
    wait_for_start();
  }
  if (objective == AVOID){
    avoid();
  }
  if (objective == DONE){
    lcd.setCursor(0,1);
    lcd.print("   DONE    ");
    stopMotor();
  }
  if (objective == ZY){
    ZtoY();
  }
  if (objective == YX){
    YtoX();
  }
  if (objective == CARPETG){
    CarpetG();
  }
  if (objective == WY){
    WtoY();
  }
  if (objective == GCUPONE){
    GCup1();
  }
  if (objective == YSTANDS){
    YtoStands();
  }
  if (objective == CLPG){
    ClapperG();
  }
  if (objective == CLP4G){
    Clapper4G();
  }
  if (objective == GFANCY){
    GFancy();
  }
}

void ZtoY(){
  if (encoder1() > dist_start_to_y){                       // edit this value for distance
      stopMotor();
      delay(sdt);
      encodeReset();
      turnRight90();
      delay(sdt);
      encodeReset();
      objective = YX;                
    }
    else if (getDistanceFR() < TURN_DISTANCE){
      s++;
      if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
    }
    else {
      s = 0;
      moveForward(40);
    }
}

void YtoX(){
  if (encoder1() > dist_y_to_x){
        stopMotor();
        delay(sdt);
        encodeReset();
        turnRight90();
        delay(sdt);
        encodeReset();
        objective = CARPETG;
      }
  else if (getDistanceFR() < TURN_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(60);
   }
}

void CarpetG(){
  do{
    moveForward(-20);
  }while (check_bumpers() != BOTH);
  stopMotor();
  bridgeDown();
  delay(500);
  encodeReset();
  bridgeUp();
  do{
    moveForward(15);
  }while (encoder1() < clear_wall);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_cpt_to_standg);
  stopMotor();
  clawStand();
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(40);
  }while (encoder1() < dist_stand_to_w);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  encodeReset();
  objective = WY;
}

void WtoY(){
  if (encoder1() > dist_w_to_y){
        stopMotor();
        delay(sdt);
        encodeReset();
        turnLeft90();
        delay(sdt);
        encodeReset();
        objective = GCUPONE;
      }
  else if (getDistanceFR() < TURN_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(40);
   }
}

void GCup1(){
  do{
    moveForward(30);
  }while (encoder1() < dist_y_to_drop);
  stopMotor();
  clawOpen();
  encodeReset();
  do{
    moveForward(-50);
  }while (encoder1() > -dist_drop_to_y);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(turn_at_y);                  // weird turn
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_y_to_cup);
  stopMotor();
  clawCup();
  delay(sdt);
  encodeReset();
  do{
    moveForward(20);
  }while (encoder1() < 50);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (encoder1() > -dist_cup_to_y);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_at_dropg);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_y_to_drop2);
  stopMotor();
  clawOpen();
  encodeReset();
  do{
    moveForward(-30);
  }while (encoder1() > -dist_drop_to_y2);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_at_y2);
  stopMotor();
  delay(sdt);
  encodeReset();
  objective = YSTANDS;
}

void YtoStands(){
  if (encoder1() > dist_y_to_v){
        objective = CLPG;
      }
  else if (getDistanceFR() < TURN_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = FR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(60);
   }
}

void ClapperG(){
  do{
    moveForward(40);
  }while (encoder1() < 250);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-40);
  }while (encoder1() > -clear_stands);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_at_v);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < clear_wall);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(154);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-40);
  }while (encoder1() > -dist_clap4);
  stopMotor();
  delay(sdt);
  armLeft(arm_out);
  encodeReset();
  do{
    moveForward(40);
  }while (encoder1() < dist_clap4_hit);
  stopMotor();
  armRight(arm_back);
  delay(sdt);
  encodeReset();
  objective = CLP4G;
}

void Clapper4G(){
  if (encoder1() < -dist_to_u){
        objective = GFANCY;
      }
  else if (getDistanceRear() < STOP_DISTANCE){
    s++;
    if (s > sensor_count){
        sensor = REAR;
        prevstate = objective;
        objective = AVOID;
      }
  } 
  else {
    s = 0;
     moveForward(-40);
   }
}

void GFancy(){
  do{
    moveForward(-30);
  }while (encoder1() > -dist_clap5);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-15);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < clear_wall);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight(turn_to_cup4);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_u_to_cup4);
  stopMotor();
  clawCup();
  delay(sdt);
  encodeReset();
  turnLeft(turn_at_cup4);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(40);
  }while (encoder1() < dist_cup4_to_line);
  stopMotor();
  clawOpen();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-40);
  }while (encoder1() > -dist_line_to_wallg);
  stopMotor();
  delay(sdt);
  turnRight(turn_to_wall_u);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < wall_2stands);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(30);
  }while (encoder1() < dist_to_2standsg);
  stopMotor();
  claw2Stands();
  delay(333);
  encodeReset();
  turnLeft(turn_around_stands);
  stopMotor();
  delay(sdt);
  encodeReset();;
  do{
    moveForward(30);
  }while (encoder1() < 40);
  stopMotor();
  delay(sdt);
  clawOpen();
  encodeReset();
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH && encoder1() > -320);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < dist_wall_clap1);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnRight90();
  stopMotor();
  delay(sdt);
  do{
    moveForward(-30);
  }while (check_bumpers() != BOTH);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < dist_bridge);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft(turn_at_clap6);
  stopMotor();
  delay(sdt);
  encodeReset();
  bridgeMid();
  turnRight(hit_clap6);
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(15);
  }while (encoder1() < 40);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();
  stopMotor();
  delay(sdt);
  encodeReset();
  bridgeUp2();
  do{
    moveForward(50);
  }while (encoder1() < return_to_stands);
  stopMotor();
  delay(sdt);
  encodeReset();
  claw2Stands();
  turnRight90();
  stopMotor();
  delay(sdt);
  encodeReset();
  do{
    moveForward(50);
  }while (encoder1() < go_home);
  clawOpen();  
  objective = DONE;
}
  
void avoid(){
  setAcceleration(7);
  stopMotor();
  delay(500);
  if (sensor = FR){
    if (getDistanceFR() > TURN_DISTANCE){
      setAcceleration(3);
      s = 0;
      objective = prevstate;
    }
    else {
        lcd.setCursor(0,1);
        lcd.print(" FRONT RIGHT ");
        objective = AVOID;
    }
  }
  /*
  if (sensor = FL){
    if (getDistanceFL() > TURN_DISTANCE){
      setAcceleration(3);
      s = 0;
      objective = prevstate;
    }
    else {
       objective = AVOID;
    }
  }
  
  if (sensor = FRONT){
    lcd.setCursor(0,0);
    lcd.print("   Front    ");
    if (getDistanceFront() > TURN_DISTANCE){
      setAcceleration(3);
      s = 0;
      objective = prevstate;
    }
    else {
      objective = AVOID;
    }
  }
  */
  if (sensor = REAR){
    lcd.setCursor(0,0);
    lcd.print("     Rear    ");
    if (getDistanceRear() > TURN_DISTANCE){
      setAcceleration(3);
      s = 0;
      objective = prevstate;
    }
    else {
      objective = AVOID;
    }
  }
}

void clawShut(){
    for(posclaw = posclaw; posclaw <= claw_shut; posclaw ++){                                  // in steps of 1 degree from 50 to 90
    myservoclaw.write(posclaw);                                            // tell servo to go to position in variable 'pos' 
    delay(5);                                                     // waits 15ms for the servo to reach the position
    }
  //pos = a;
}

void clawCup(){
  for(posclaw = posclaw; posclaw <= claw_cup; posclaw ++){                                  // in steps of 1 degree from 50 to 90
  myservoclaw.write(posclaw);                                            // tell servo to go to position in variable 'pos' 
  delay(5);                                                     // waits 15ms for the servo to reach the position
  }
}

void clawStand(){
  for(posclaw = posclaw; posclaw <= claw_stand; posclaw ++){                                  // in steps of 1 degree from 50 to 90
  myservoclaw.write(posclaw);                                            // tell servo to go to position in variable 'pos' 
  delay(5);                                                     // waits 15ms for the servo to reach the position
  }
}

void claw2Stands(){
  for(posclaw = posclaw; posclaw <= claw_2_stands; posclaw++){
    myservoclaw.write(posclaw);
    delay(5);
  }
}

void clawOpen(){
    for(posclaw = posclaw; posclaw >= claw_open; posclaw--){     //                                
    myservoclaw.write(posclaw);              // tell servo to go to position in variable 'pos' 
    delay(5);                       // waits 15ms for the servo to reach the position 
    }
}

void bridgeDown(){
   for(posbridge = bridge_up; posbridge <= bridge_down; posbridge ++){                                  // in steps of 1 degree 
    myservobridge.write(posbridge);                                            // tell servo to go to position in variable 'pos' 
    delay(5);                                                     // waits 15ms for the servo to reach the position
   }
}

void bridgeUp(){
  for(posbridge = bridge_down; posbridge>= bridge_up; posbridge--){     //                                  
    myservobridge.write(posbridge);              // tell servo to go to position in variable 'pos' 
    delay(5);                       // waits 15ms for the servo to reach the position 
    }
}

void bridgeUp2(){
  for(posbridge = bridge_mid; posbridge>= bridge_up; posbridge--){     //                                  
    myservobridge.write(posbridge);              // tell servo to go to position in variable 'pos' 
    delay(5);                       // waits 15ms for the servo to reach the position 
    }
}

void bridgeCarpet(){
  for(posbridge = bridge_down; posbridge>= bridge_carpet; posbridge--){     //                                  
    myservobridge.write(posbridge);              // tell servo to go to position in variable 'pos' 
    delay(5);                       // waits 15ms for the servo to reach the position 
    }
}

void bridgeMid(){
   for(posbridge = bridge_up; posbridge <= bridge_mid; posbridge ++){                                  // in steps of 1 degree 
    myservobridge.write(posbridge);                                            // tell servo to go to position in variable 'pos' 
    delay(5);                                                     // waits 15ms for the servo to reach the position
   }
}  

void moveForward(int velocity){
  check_time();
  setAcceleration(3);
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED2);
    Wire.write(velocity + 128);                                           
    Wire.endTransmission();
  
    Wire.beginTransmission(MD25ADDRESS);  
    Wire.write(SPEED1);
    Wire.write(velocity + 128);
    Wire.endTransmission();
}

void setAcceleration(int acceleration){
  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to set acceleration
  Wire.write(ACCELERATE);
  Wire.write(acceleration);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to set acceleration
  Wire.write(ACCELERATE);
  Wire.write(acceleration);
  Wire.endTransmission();
}

void turnRight90(){
  check_time();
    encodeReset();
    setAcceleration(4);
    do{
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED2);
      Wire.write(108);                                           
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED1);
      Wire.write(148);
      Wire.endTransmission();
    }while(encoder1() < 151);
    setAcceleration(10);
    stopMotor();                      // wait for robot to stop
}

void turnLeft90(){
    check_time();  
    encodeReset();
    setAcceleration(4);
    do{
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED2);
      Wire.write(148);                                           
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED1);
      Wire.write(108);
      Wire.endTransmission();
    }while(encoder1() > -151);
    setAcceleration(10);
    stopMotor();                      // wait for robot to stop
}

void turnRight(int d){
  check_time();
    encodeReset();
    setAcceleration(4);
    do{
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED2);
      Wire.write(108);                                           
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED1);
      Wire.write(148);
      Wire.endTransmission();
    }while(encoder1() < d);
    setAcceleration(10);
    stopMotor();                      // wait for robot to stop
}

void turnLeft(int d){ 
    check_time();  
    encodeReset();
    setAcceleration(4);
    do{
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED2);
      Wire.write(148);                                           
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                    
      Wire.write(SPEED1);
      Wire.write(108);
      Wire.endTransmission();
    }while(encoder1() > -d);
    setAcceleration(10);
    stopMotor();                      // wait for robot to stop
}

void stopMotor(){                                           // Function to stop motors
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED2);
  Wire.write(128);                                           // Sends a value of 128 to motor 2 this value stops the motor
  Wire.endTransmission();
  
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();
}  

void encodeReset(){                                          // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                          // Putting the value 0x20 to reset encoders
  Wire.endTransmission();
  delay(100);
}

byte getSoft(){                                              // Function that gets the software version
  Wire.beginTransmission(MD25ADDRESS);                       // Send byte to read software version as a single byte
  Wire.write(SOFTWAREREG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 1);                          // request 1 byte form MD25
  while(Wire.available() < 1);                               // Wait for it to arrive
  byte software = Wire.read();                               // Read it in
  
  return(software);
}

long encoder1(){                                            // Function to read and display value of encoder 1 as a long
  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                     // Fourth byte for encoder 1, LL

  delay(50);                                                // Wait for everything to make sure everything is sent
  
  return(poss1);
}

long encoder2(){                                            // Function to read and display velue of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);           
  Wire.write(ENCODERTWO);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to become available
  long poss2 = Wire.read();
  poss2 <<= 8;
  poss2 += Wire.read();                
  poss2 <<= 8;
  poss2 += Wire.read();                
  poss2 <<= 8;
  poss2  +=Wire.read();               
  
  delay(50);                                                // Wait to make sure everything is sent
   
  return(poss2);
}

void anticlockwise()
{
  for(int i = 0; i < 8; i++)
  {
    setOutput(i);
    delayMicroseconds(motorSpeed);
  }
}

void clockwise()
{
  for(int i = 7; i >= 0; i--)
  {
    setOutput(i);
    delayMicroseconds(motorSpeed);
  }
}

void setOutput(int out){
  digitalWrite(motorPin1, bitRead(lookup[out], 0));
  digitalWrite(motorPin2, bitRead(lookup[out], 1));
  digitalWrite(motorPin3, bitRead(lookup[out], 2));
  digitalWrite(motorPin4, bitRead(lookup[out], 3));
}

int getDistanceFR(){
  distFR = sonarFR.ping()/US_ROUNDTRIP_CM;
  return(distFR);
}
/*
int getDistanceFL(){
  distFL =sonarFL.ping()/US_ROUNDTRIP_CM;
  return(distFL);
}
*/

/*
int getDistanceFront(){
  distFL = sonarFL.ping()/US_ROUNDTRIP_CM;
  delay(33);
  distFR = sonarFR.ping()/US_ROUNDTRIP_CM;
  if (distFL > distFR){
    distFront = distFR;
  }
  else {
    distFront = distFL;
  }
  return(distFront);
}
*/

int getDistanceRear(){
  distRear = sonarRear.ping()/US_ROUNDTRIP_CM;
  return(distRear);
}

void checkcolour(){
  if (digitalRead(colour_pin) == HIGH){
    colour = YELLOW;
    lcd.print("YELLOW"); //Make sure lcd intialisation is before checkside
    lcd.setCursor(0, 0);  
  }
  else if (digitalRead(colour_pin) == LOW){
    colour = GREEN;
    lcd.print("GREEN");
    lcd.setCursor(0, 0);
  }
  else {
    lcd.print("WTF");
    lcd.setCursor(0,0);
  }
}

void check_time(){
  //Calculates elapsed time
  elapsed_time = millis() - start_time;
  if (elapsed_time > 88000){
    do{
      stopMotor();
      lcd.setCursor(0,0);
      lcd.print("Out of Time     ");
    }while (elapsed_time >= 88000);
    objective = DONE;
  }
}

void wait_for_start(){
  delay(50);
  if(digitalRead(pullCord_pin) == LOW){     //HIGH is pressed
    k++;
    if (k > 5){
      start_time = millis();
      k = 0;
      if (colour == YELLOW){
        objective = AB;
      }
      else {
        objective = ZY;
      }
    }
  }
  else {
    k = 0;
    objective = START;
  }
}

void lcdState(){
  lcd.setCursor(0,0);
  lcd.print("State ");
  lcd.print(objective);
  lcd.print("  ");
}

int check_bumpers(){
  int left = digitalRead(14);
  int right = digitalRead(15);
  if ((left && right == HIGH)){
    lcd.setCursor(0,1);
    lcd.print("Both  ");
    return(BOTH);
  }
  else if (left == HIGH){
    lcd.setCursor(0,1);
    lcd.print("Left  ");
    return(LEFT);
  }
  else if (right == HIGH){
    lcd.setCursor(0,1);
    lcd.print("Right ");
    return(RIGHT);
  }
  else {
    lcd.setCursor(0,1);
    lcd.print("None  ");
    return(NONE);
  }
}
      

void hitWall(){
  for (int i = 0; i < 999; i++){
    int a = check_bumpers();
    if (a == BOTH){
      stopMotor();
      i = 999;
    }
    else {
      moveForward(-20);
      delay(20);
    }
  }
}

void armRight(int value){
  count = 0;
  do{
    clockwise();
    count++;
  }while (count < value);
}

void armLeft(int value){
  count = 0;
  do{
    anticlockwise();
    count++;
  }while (count < value);
}
  

void square(){
  do{
    moveForward(60);
  }while (encoder1() < 300);
  stopMotor();
  delay(sdt);
  encodeReset();
  turnLeft90();
  delay(sdt);
  encodeReset();
}
