#include <Sparki.h>

// State constants
#define ROTATE 1
#define DRIVE_OBJ 2
#define GRAB 3
#define TURN_AROUND 4
#define DRIVE_LINE 5
#define FOLLOW_LINE 6
#define STOP 7

// Set up some global variables with default values to be replaced during operation
int current_state = ROTATE;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int sonic_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

void rotate();
void drive_obj();
void grab();
void turn_around();
void drive_line();
void follow_line();
void stop_sparki();


// Setup the robot
void setup()
{
  // Called on powerup
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  sonic_distance = sparki.ping(); // Read the distance sensor
  line_left = sparki.lineLeft(); // Read the left IR sensor
  line_right = sparki.lineRight(); // Read the right IR sensor
  line_center = sparki.lineCenter(); // Read the center IR sensor
}



// Drive the robot based on state
void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);

  switch (current_state)
  {
    case ROTATE:
      rotate();
      break;

    case DRIVE_OBJ:
      drive_obj();
      break;

    case GRAB:
      grab();
      break;
    case TURN_AROUND:
      // turn 180 degrees
      turn_around();
      break;
    case DRIVE_LINE:
      // Drive until you detect a line
      drive_line();
      break;

    case FOLLOW_LINE:
      // follow the detected line
      follow_line();
      break;

    case STOP:
      // Finished, stop the robot
      stop_sparki();
      break;

    case default:
      //Finshed all tasks
      sparki.RGB(RGB_BLUE);
      delay(500);
      sparki.RGB(RGB_OFF);
  }
  // Your state machine code goes here

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}

void rotate()
{
  // Test transition "detectObj <= 30cm"
  // if object detected within 30 cm
  if (sonic_distance != -1 && sonic_distance <= 30)
  {
    // if true, set state to "object found"
    current_state = DRIVE_OBJ;
    sparki.moveLeft(1); //Rotate a little further to get full frame of obj
  }
  else
  {
    // rotate 5 degrees
    sparki.moveLeft(5);
  }
  return;
}

// Drive until you are less than 7 cm from the object
void drive_obj()
{
  // If the object is within 7 cm
    // change state to grab
  if(sonic_distance != -1 && sonic_distance <= 7)
    current_state = GRAB;

  else
    sparki.moveForward(2);
    // else, drive one more cm
  return;
}

void grab()
{
  // Move forward slightly
  sparki.moveForward(2);
  // Perform grab motion
  sparki.gripperClose();

  //NOTE: Not dealing with possibility of grab failure here

  current_state = turn_around();

  return;
}

// Turn 180 degrees
void turn_around()
{
  sparki.moveLeft(180);
  current_state = DRIVE_LINE;
}

// Drive until the line is detected
void drive_line()
{
  // Check if the line is detected
  if (line_left < threshold || line_right < threshold || line_center < threshold)
  {
    // Change state to FOLLOW_LINE
    current_state = FOLLOW_LINE;
  }
  else
  {
    // else, no line detected, drive forward
    sparki.moveForward(2);
  }
  return;
}

void follow_line()
{
  // If all line detectos read high line readings, stop (you have reached the finish)
  if((line_left < threshold) && (line_right < threshold) && (line_center < threshold))
  {
    current_state = STOP;
    return;
  }

    //else, stear the robot:
        // If the center detecter is strongest, go strait
        // else if the left sensor is the strongest, turn left
        // else if the right sensor is strongest, turn right
  else if(line_center < threshold && line_left >= threshold && line_right >= threshold)
  {
    sparki.moveForward(2);
  }
  else if(line_left < threshold)
  {
    sparki.moveRight(1);
  }
  else if(line_right < threshold)
  {
    sparki.moveLeft(1);
  }

  return;
}

void stop_sparki()
{
  sparki.moveStop();

  sparki.beep();

  sparki.gripperOpen();

  current_state = 100;

  return;
}
