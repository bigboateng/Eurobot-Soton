const int ENCODER_DELEY = 10;
const float COUNT_LENGTH = 2;//TO CALC  // units = count/mm

//Encoder addresses on the Wire I2C bus
const int ENCODER0 = 0x6;
const int ENCODER1 = 0x2;

// returns true if the required_count is reached
// returns false if obstacle is reached
// required_counts is passed by reference, and will be the number of steps overshot or otherwise after
boolean encoder_count(int& required_counts, int side){
  boolean obstacle = is_obstacle_infront();
  int a = 0; // counter for obstacle checks();
  while(( abs(get_encoder_value(side)) < required_counts ) && (!obstacle)) {
    delay(ENCODER_DELEY);
    //check for obstacles on every 10th loop round, to save time
    a++;
    if (a%10 == 0) { obstacle = is_obstacle_infront(); }
  }
  return obstacle;
}

// get encoder values
// side values = 0,1
long get_encoder_value(int side){
  
  //Get position of encoder for counting
  Wire.beginTransmission(WIRE_ADR);
  if (side == 0) {
    Wire.write(ENCODER0);
  } else if (side == 1){
    Wire.write(ENCODER1);
  }
  Wire.endTransmission(true);
  Wire.requestFrom(WIRE_ADR, 4);
  
  while(Wire.available() < 4) {}

  is_the_end();
  
  //Read in the values
  // <<= means left shift and write
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
}

//Reset the values for both encoders
void reset_encoders() {
  Wire.beginTransmission(WIRE_ADR);
  Wire.write(0x10);
  Wire.write(0x20);
  Wire.endTransmission(true);
}

//calculate the required counts based on distance in mm
//turns mm into counts
int calc_counts(int mm){
  return static_cast<int>(mm*COUNT_LENGTH);
}

// does the reverse of calc_required_counts
// turns counts into length in mm.
int calc_mm(int counts){
  return (int)(counts/COUNT_LENGTH);
}

