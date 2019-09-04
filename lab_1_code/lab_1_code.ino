#include <Sparki.h>

// State constants
#define DETECT_OBJ 1
#define DRIVE_OBJ 2
#define GRAB 3
#define TURN_AROUND 4
#define DETECT_LINE 5
#define ORIENT_LINE 6
#define FOLLOW_LINE 7
#define STOP 8

// Set up some global variables with default values to be replaced during operation
int current_state = DETECT_OBJ;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int sonic_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
int edge_left = 1000;
int edge_right = 1000;

void detect_obj();
void drive_obj();
void grab();
void turn_around();
void detect_line();
void orient_line();
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
  edge_left = sparki.edgeLeft(); // Read the left edge IR sensor
  edge_right = sparki.edgeRight(); // Read the left right IR sensor
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
    case DETECT_OBJ:
      detect_obj();
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
    case DETECT_LINE:
      // Drive until you detect a line
      detect_line();
      break;
    case ORIENT_LINE:
      orient_line();
      break;
    case FOLLOW_LINE:
      // follow the detected line
      follow_line();
      break;
    case STOP:
      // Finished, stop the robot
      stop_sparki();
      break;
    default:
      //Finshed all tasks
      sparki.RGB(RGB_BLUE);
      delay(500);
      sparki.RGB(RGB_OFF);
      break;
  }
  // Your state machine code goes here

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}

void detect_obj()
{
  // Test transition "detectObj <= 30cm"
  // if object detected within 30 cm
  if (sonic_distance != -1 && sonic_distance <= 30)
  {
    // if true, set state to "object found"
    current_state = DRIVE_OBJ;
    sparki.moveLeft(10); //Rotate a little further to get full frame of obj
  }
  else
  {
    // rotate 5 degrees
    sparki.moveLeft(1);
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
  sparki.moveForward(3);
  // Perform grab motion
  sparki.gripperClose();
  
  // FIX THIS TO BE ASYNCHRONOUS
  delay(6000);
  sparki.gripperStop();

  //NOTE: Not dealing with possibility of grab failure here

  // Robot is turning arround while it grabs... We need to time wait while grabbing to fix this
  current_state = TURN_AROUND;

  return;
}

// Turn 180 degrees
void turn_around()
{
  sparki.moveLeft(180);
  current_state = DETECT_LINE;
}

// Drive until the line is detected
void detect_line()
{
  // Check if the line is detected
  if (line_left < threshold || line_right < threshold || line_center < threshold)
  {
    current_state = ORIENT_LINE;
  }
  else
  {
    // else, no line detected, drive forward
    sparki.moveForward(2);
  }
  return;
}

void orient_line()
{
  if(((line_center < threshold) && (line_left < threshold) && (line_right < threshold)))
  {
    sparki.moveForward(3);
    sparki.moveLeft(45);
  }
  else if(line_center < threshold && line_left >= threshold && line_right >= threshold)
  {
    // If the center detecter is strongest, go strait
    sparki.moveForward(1);
    current_state = FOLLOW_LINE;
  }
  else if(line_left < threshold && line_left < line_right)
  {
    // else if the left sensor is the strongest, turn left
    sparki.moveLeft(1);
  }
  else
  {
    // else if the right sensor is strongest, turn right
    sparki.moveRight(1);
  }
}

void follow_line()
{
  // If all line detectos read high line readings, stop (you have reached the finish)
  if(((line_center < threshold) && (line_left < threshold) && (line_right < threshold)))
  {
    current_state = STOP;
    return;
  }    
  else if(line_center < threshold && line_left >= threshold && line_right >= threshold)
  {
    // If the center detecter is strongest, go strait
    sparki.moveForward(1);
  }
  else if(line_left < threshold && line_left < line_right)
  {
    // else if the left sensor is the strongest, turn left
    sparki.moveLeft(1);
  }
  else
  {
    // else if the right sensor is strongest, turn right
    sparki.moveRight(1);
  }

  return;
}

void stop_sparki()
{
  sparki.moveStop();

  sparki.beep();

  sparki.gripperOpen();
  delay(6000);
  sparki.gripperStop();

  current_state = 100;

  return;
}
