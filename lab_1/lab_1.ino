#include <Sparki.h>

#define STATE_FIRST_ONE 0
#define STATE_SECOND_ONE 1

// Set up some global variables with default values to be replaced during operation
int current_state = STATE_SEARCH_OBJ;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int cm_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;


void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance = 0; // Replace with code to read the distance sensor
  line_left = 0; // Replace with code to read the left IR sensor
  line_right = 0; // Replace with code to read the right IR sensor
  line_center = 0; // Replace with code to read the center IR sensor
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);

  // Your state machine code goes here

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}
