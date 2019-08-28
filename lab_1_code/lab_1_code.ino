#include <Sparki.h>

// State constants
#define ROTATE 1
#define DRIVE_OBJ 2
#define GRAB 3
#define DRIVE_LINE 4
#define FOLLOW_LINE 5
#define STOP 6

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
  sonic_distance = sparki.ping(); // Replace with code to read the distance sensor
  line_left = sparki.lineLeft(); // Replace with code to read the left IR sensor
  line_right = sparki.lineRight(); // Replace with code to read the right IR sensor
  line_center = sparki.lineCenter(); // Replace with code to read the center IR sensor
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
    
    // else, drive one more cm
  return;
}

void grab()
{
  // Move forward slightly
  // Perform grab motion
  return;
}

void drive_line()
{
  return;
}

void follow_line()
{
  return;
}

void stop_sparki()
{
  return;
}
