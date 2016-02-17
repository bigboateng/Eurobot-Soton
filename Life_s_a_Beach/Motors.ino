//Fudgefactors
const int STOP_DIST = 0.0f; // stop distance (given in mm), increase if robot is overshooting on distance
const float SLIP_FACTOR = 1.0f; // decrease if there is a lot of slip

// Motor addressed used for Wire (I2C bus) library
const int M0_ADR = 0x0;
const int M1_ADR = 0x1;

// Speeds for each direction for each motor
const int M0_SPEED = 0;
const int M0_REV_SPEED = 0;
const int M1_SPEED = 0;
const int M1_REV_SPEED = 0;

// motor sync delay
const int MOTOR_SYNC_DELAY = 0;

// start one motor with a set speed
// possible values: motor = 0,1; motorspeed<###.
void start_motor(int motor, int motorspeed) {
  Wire.beginTransmission(WIRE_ADR);
  if (motor == 0) {
    Wire.write(M0_ADR); // 1ST Motor
  } else if (motor == 1) {
    Wire.write(M1_ADR); // 2ND Motor
  }
  Wire.write(motorspeed);
  Wire.endTransmission(true);
}

// start both motors with the same speed
void start_motors(int motor0_speed, int motor1_speed) {
  start_motor(0, motor0_speed);
  delay(MOTOR_SYNC_DELAY);
  start_motor(1, motor1_speed);
}

//Stop one motor
void stop_motor(int motor) {
  Wire.beginTransmission(WIRE_ADR);
  if (motor == 0) {
    Wire.write(M0_ADR); // Motor 1
  } else if (motor == 1) {
    Wire.write(M1_ADR); // Motor 2
  }
  Wire.write(0);
  Wire.endTransmission(true);
}

// Stop both motors
void stop_motors() {
  stop_motor(0);
  delay(MOTOR_SYNC_DELAY);
  stop_motor(1);
}

//Move forward the given distance in mm
//use avoid = True to allow obstacle avoidance
//use avoid = False for a dumb movement
//returns the number of turns
int move_forward(int mm, bool avoid) {
  int motor0_speed = M0_SPEED;
  int motor1_speed = M1_SPEED;

  if (mm < 0) {
    motor0_speed = M0_REV_SPEED;
    motor1_speed = M1_REV_SPEED;
  }

  //Add fudgefactors and change from mm to wheel rotation counts
  float new_mm = (abs(mm) * SLIP_FACTOR) - STOP_DIST;
  int required_counts = static_cast<int>(new_mm / COUNT_LENGTH);

  //Run motors until desired count is reached or obstacle is encountered
  reset_encoders();
  start_motors(motor0_speed, motor1_speed);
  bool obstacle = encoder_count(required_counts, 0);
  stop_motors();

  return calc_mm(required_counts);
}

// rotate about one side, moving one wheel and keeping the other still
int rotate_side(int side, int deg) {
  int turn_velocity = (deg > 0 ? (side == 0 ? M1_SPEED : M0_SPEED) : (side == 0 ? M1_REV_SPEED : M0_REV_SPEED));

  //Convert degrees to distance and then wheel counts
  int dist = (int)abs(ROBOT_LENGTH * 2 * 3.141593 * deg / 360);
  int required_counts = calc_counts(dist);

  //Start turning
  reset_encoders();
  start_motor(side, turn_velocity);
  bool obstacle = encoder_count(required_counts, side);
  stop_motors();

  return calc_mm(required_counts);
}

// Rotate on the spot
int rotate_spot(int deg) {
  int motor0_speed = (deg < 0 ? M0_REV_SPEED : M0_SPEED);
  int motor1_speed = (deg > 0 ? M1_REV_SPEED : M1_SPEED);

  //Convert degrees to distance and then wheel counts
  int dist = (int)abs(ROBOT_LENGTH * 3.141593 * deg / 360);
  int required_counts = calc_counts(dist);

  //Start turning
  reset_encoders();
  start_motors(motor0_speed, motor1_speed);
  bool obstacle = encoder_count(required_counts, 0);
  stop_motors();

  return calc_mm(required_counts);
}

