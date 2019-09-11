#include <Sparki.h>

#define CYCLE_TIME .100  // seconds
#define AXLE_LENGTH 0.0857  // Meters -- Distance between wheels

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_DISTANCE_MEASURE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
unsigned int time;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void measure_30cm_speed() {
  // TODO
  unsigned int time = millis();

  sparki.moveForward();

  while(((line_center > threshold) && (line_left > threshold) && (line_right > threshold)))
  {
    readSensors();
  }

  sparki.println(time);
}


void updateOdometry() {
  // TODO 
  // xI (k + 1) = xI (k) + ∆x(k)
}

void displayOdometry() {
  // TODO
  sparki.print("pose_x");
  sparki.println(pose_x);
  sparki.print("pose_y");
  sparki.println(pose_y);
  sparki.print("pose_theta");
  sparki.println(pose_theta);
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  time = millis();

  updateOdometry();

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // If all line detectos read high line readings, stop (you have reached the finish)
      if(line_center < threshold && line_left >= threshold && line_right >= threshold)
      {
        // If the center detecter is strongest, go strait
        sparki.moveForward();
      }
      else if(line_left < threshold && line_left < line_right)
      {
        // else if the left sensor is the strongest, turn left
        sparki.moveLeft();
      }
      else(line_right < threshold && line_right < line_left)
      {
        // else if the right sensor is strongest, turn right
        sparki.moveRight();
      }
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }


  delay(1000*CYCLE_TIME);
}
