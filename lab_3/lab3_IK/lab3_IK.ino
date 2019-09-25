#include <Sparki.h>
#include <math.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

// Limits to qualify success
#define SUCCESS_DISTANCE_ERROR 1
#define SUCCESS_HEADING_ERROR 1

// Limits to qualify fixing bearing error
#define FIX_BEARING_ERROR_DISTANCE 1
#define FIX_BEARING_ERROR_BEARING 1

// Limits to qualify fixing distance error
#define FIX_DISTANCE_ERROR_DISTANCE 1
// #define FIX_DISTANCE_ERROR_BEARING 1     // These are the same values as above?

// Limits to qualify fixing heading error
#define FIX_HEADING_ERROR_DISTANCE 1

#define FWD 1
#define NONE 0
#define BCK -1

#define LEFT 1
#define RIGHT 2
#define FORWARD 3
#define ORIGIN 4



// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
/*const*/ int current_state = CONTROLLER_GOTO_POSITION_PART2;
int LAST_MOVEMENT = 0;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  sparki.clearLCD();
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.0,0.5, to_radians(0));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = sqrt(sq((pose_x - dest_pose_x))+sq((pose_y - dest_pose_y))); // Updated by Cooper, was 0
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry() {

  //Save old theta
  float old_theta = pose_theta;

  //Update theta
  pose_theta += ((left_right_pct * ROBOT_SPEED * CYCLE_TIME / (1000)) -
    (left_speed_pct * ROBOT_SPEED * CYCLE_TIME / (1000))) / AXLE_DIAMETER;

  // Bound theta, not sure if this should be here
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;

  // add x distance to pose_x
  // cos(theta) * speed m/s * 100 ms * (1 s / 1000 ms)
  pose_x += cos(abs(pose_theta-old_theta)/2.0) * (.5) *
    ((left_right_pct * ROBOT_SPEED * CYCLE_TIME / (1000)) +
    (left_speed_pct * ROBOT_SPEED * CYCLE_TIME / (1000)));

  // add y motion
  // sin(theta) * speed m/s * 100 ms * (1 s / 1000 ms)
  pose_y += sin(abs(pose_theta-old_theta)/2.0) * (.5) *
    ((left_right_pct * ROBOT_SPEED * CYCLE_TIME / (1000)) +
    ((left_speed_pct * ROBOT_SPEED * CYCLE_TIME / (1000)));
}

void updateErrors()
{
  d_err = sqrt(sq((pose_x - dest_pose_x))+sq((pose_y - dest_pose_y))); //distance error
  b_err = atan2((dest_pose_y-pose_y),(dest_pose_x - pose_x)) - pose_theta; //bearing error
  h_err = dest_pose_theta - pose_theta; // heading error (rad)
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;


  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      d_err = sqrt(sq((pose_x - dest_pose_x))+sq((pose_y - dest_pose_y)));
      b_err = atan2((dest_pose_y-pose_y),(dest_pose_x - pose_x)) - pose_theta;

      displayOdometry();
      Serial.println(b_err);
      delay(100);

      if(b_err > 0)
        sparki.moveLeft(to_degrees(b_err));
      else if(b_err < 0)
        sparki.moveRight(to_degrees(abs(b_err)));
      pose_theta += b_err;
      b_err = 0;

      displayOdometry();

      sparki.moveForward(d_err * 100);
      pose_x = dest_pose_x;
      pose_y = dest_pose_y;
      d_err = 0;

      displayOdometry();

      h_err = dest_pose_theta - pose_theta;

      if(h_err < 0)
        sparki.moveRight(to_degrees(-h_err));
      else
        sparki.moveLeft(to_degrees(h_err));
      pose_theta = dest_pose_theta;
      h_err = 0;
      displayOdometry();

      sparki.moveStop();

      current_state = 0;


      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      updateOdometry();

      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

      // Calculate errors
      updateErrors();

      // If the heading error and distance error are within acceptable limits, then finish
      if (d_err <= SUCCESS_DISTANCE_ERROR && h_err <= SUCCESS_HEADING_ERROR)
      {
          // Start millis counter

          // Run motors at percentage towards destination
      }
      // If the distance error is greater than a certain limit and the bearing error is greater than a certain limit, care only about fixing bearing error
      else if (d_err >= FIX_BEARING_ERROR_DISTANCE && b_err >= FIX_BEARING_ERROR_BEARING)
      {
          // Start millis counter

          // Run motors at percentage towards destination
      }
      // If the bearing error is smaller than a certain limit, care about distance error
      else if (d_err >= FIX_DISTANCE_ERROR_DISTANCE && b_err < FIX_BEARING_ERROR_BEARING)
      {
          // Start millis counter

          // Run motors at percentage towards destination
      }
      // If the distance error is smaller than a certain limit, care about heading error only
      else if (d_err < FIX_HEADING_ERROR_DISTANCE)
      {
          // Start millis counter

          // Run motors at percentage towards destination
      }


      begin_time = millis();
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();

  // Stop sparki

  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
