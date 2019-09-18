#include <Sparki.h>
#include <math.h>

#define CYCLE_TIME .100  // seconds
#define AXLE_LENGTH 0.0857  // Meters -- Distance between wheels

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2

#define LEFT 1
#define RIGHT 2
#define FORWARD 3
#define ORIGIN 4

int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
int LAST_MOVEMENT = 0;

const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

double speed = 0.02739226;

// pose_x and pose_y in meters
float pose_x = 0., pose_y = 0., pose_theta = 0.;
unsigned long time;

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
  sparki.clearLCD();
  unsigned long start = millis();
  sparki.moveForward(30);

//  while(((line_center > threshold) && (line_left > threshold) && (line_right > threshold)))
//  {
//    readSensors();
//  }

  sparki.print("Time: ");
  sparki.println(millis()-start);
  sparki.updateLCD();
  sparki.moveStop();
  delay(20000);
}

void updateOdometry() {
  // TODO
  switch (LAST_MOVEMENT) {
    // case: was forward
    case FORWARD:
      // add x distance to pose_x
      // cos(theta) * speed m/s * 100 ms * (1 s / 1000 ms)
      pose_x += cos(pose_theta) * speed * 100 / (1000);

      // add y motion
      // sin(theta) * speed m/s * 100 ms * (1 s / 1000 ms)
      pose_y += sin(pose_theta) * speed * 100 / (1000);
      break;
    // case: was moveLeft
    case LEFT:
      pose_theta += 2*(speed*0.1)/AXLE_LENGTH;
      break;
    // case: was moveRight
    case RIGHT:
      pose_theta -= 2*(speed*0.1)/AXLE_LENGTH;
      break;
    case ORIGIN:
      pose_x = 0;
      pose_y = 0;
      pose_theta = 0;
    default:
      break;
  }
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("pose_x");
  sparki.println(pose_x);
  sparki.print("pose_y");
  sparki.println(pose_y);
  sparki.print("pose_theta");
  sparki.println(pose_theta);
  sparki.updateLCD();
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  //time = millis();

  updateOdometry();
  displayOdometry();
  readSensors();

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // If all line detectos read high line readings, stop (you have reached the finish)
      
      if(((line_center < threshold) && (line_left < threshold) && (line_right < threshold)))
      {
        sparki.moveForward();
        LAST_MOVEMENT = ORIGIN;
      }
      else if(line_center < threshold && line_left > threshold && line_right > threshold)
      {
        // If the center detecter is strongest, go strait
        time = millis();
        sparki.moveForward();
        LAST_MOVEMENT = FORWARD;
      }
      else if(line_left < threshold && line_left < line_right)
      {
        // else if the left sensor is the strongest, turn left
        time = millis();
        sparki.moveLeft();
        LAST_MOVEMENT = LEFT;
      }
      else if(line_right < threshold && line_right < line_left) // commented out to find bug where he only turnned right.
      {
        // else if the right sensor is strongest, turn right
        time = millis();
        sparki.moveRight();
        LAST_MOVEMENT = RIGHT;
      }
      break;

    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }

  //Should check here to see if this will be negative.
    delay(100 - (millis() - time));
  sparki.moveStop();

//  int threshold = 500;
// 
//  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
//  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
//  int lineRight  = sparki.lineRight();  // measure the right IR sensor
// 
//  if ( lineLeft < threshold ) // if line is below left line sensor
//  {  
//    sparki.moveLeft(); // turn left
//  }
// 
//  if ( lineRight < threshold ) // if line is below right line sensor
//  {  
//    sparki.moveRight(); // turn right
//  }
// 
//  // if the center line sensor is the only one reading a line
//  if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
//  {
//    sparki.moveForward(); // move forward
//  }  
// 
//  sparki.clearLCD(); // wipe the screen
// 
//  sparki.print("Line Left: "); // show left line sensor on screen
//  sparki.println(lineLeft);
// 
//  sparki.print("Line Center: "); // show center line sensor on screen
//  sparki.println(lineCenter);
// 
//  sparki.print("Line Right: "); // show right line sensor on screen
//  sparki.println(lineRight);
// 
//  sparki.updateLCD(); // display all of the information written to the screen
// 
//  delay(100); // wait 0.1 seconds
}
