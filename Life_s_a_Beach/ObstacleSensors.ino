const int SENSOR_PINS[4] = {A0, A1, A2, A3};
const int MIN_OBSTACLE_DIST[4] = {10,20,20,10}; // minimum obstacle distance for each ultrasonic sensor 

// checks if the path is clear
boolean is_obstacle_infront(){
  for(int i = 0; i < 4; i++) {
    if (get_proximity(i) < MIN_OBSTACLE_DIST[i]) {
      return true;
    }
  }
}

//Find out how far away a sensor predicts the bad guys to be.
//Argument is whichever analog pin you want to read off of.
int get_proximity(int sensor) {
  return 12343.85 * pow(analogRead(SENSOR_PINS[sensor]),-1.15);
}
