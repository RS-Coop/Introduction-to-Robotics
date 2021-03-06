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
// 1 cm is precise for this robot, use 5 cm and 15 degrees
#define SUCCESS_DISTANCE_ERROR .07 // meters
#define SUCCESS_HEADING_ERROR 15  // degrees

// Limits to qualify fixing bearing error
#define DISTANCE_THREASHOLD .05 // meters

// Coefficients for thresholding
// Best: .1
#define P1_OVER 2
// Best: 0
#define P2_OVER 10
// Best: 20
#define P3_OVER .0

// Best:
#define P1_UNDER 2
// Best:
#define P2_UNDER 0
// Best:
#define P3_UNDER 10

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
/*const*/ int current_state = CONTROLLER_GOTO_POSITION_PART3;
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
float heading_gain = 0;
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
  set_pose_destination(0.0,0.2, to_radians(180));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
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

  //Update theta
  pose_theta += ((right_speed_pct * ROBOT_SPEED * CYCLE_TIME) -
    (left_speed_pct * ROBOT_SPEED * CYCLE_TIME)) / AXLE_DIAMETER;

  // add x distance to pose_x
  // cos(theta) * speed m/s * 100 ms * (1 s / 1000 ms)
  pose_x += cos(pose_theta) * (.5) *
    ((right_speed_pct * ROBOT_SPEED * CYCLE_TIME) +
    (left_speed_pct * ROBOT_SPEED * CYCLE_TIME));

  // add y motion
  // sin(theta) * speed m/s * 100 ms * (1 s / 1000 ms)
  pose_y += sin(pose_theta) * (.5) *
    ((right_speed_pct * ROBOT_SPEED * CYCLE_TIME) +
    (left_speed_pct * ROBOT_SPEED * CYCLE_TIME));

    // Bound theta, not sure if this should be here
  if (pose_theta > M_PI + M_PI/2.) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI-M_PI/2.) pose_theta += 2.*M_PI;

}

void updateErrors()
{
  d_err = sqrt(sq((pose_x - dest_pose_x))+sq((pose_y - dest_pose_y))); //distance error
  b_err = atan2((dest_pose_y-pose_y),(dest_pose_x - pose_x)) - pose_theta; //bearing error
  h_err = dest_pose_theta - pose_theta; // heading error (rad)

  if (b_err > M_PI + M_PI/2.) b_err -= 2.*M_PI;
  if (b_err < -M_PI - M_PI/2.) b_err += 2.*M_PI;
  //if (h_err > M_PI + M_PI/2.) h_err -= 2.*M_PI;
  //if (h_err < -M_PI - M_PI/2.) h_err += 2.*M_PI;
  if (h_err > 2*M_PI) h_err -= 2.*M_PI;
  if (h_err < 0) h_err += 2.*M_PI;

}

void displayOdometry() {
  sparki.clearLCD();
/*  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); */
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
//  sparki.print("l%"); sparki.print(left_speed_pct); sparki.print("r%"); sparki.println(right_speed_pct);
  sparki.updateLCD();
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
    {
      updateOdometry();
      displayOdometry();

      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

      // Calculate errors
      updateErrors();
      //I think we can just calculate the gains here.
      //heading_gain = distance_gain/d_err;


      // If the heading error and distance error are within acceptable limits, then finish
      if (d_err <= SUCCESS_DISTANCE_ERROR && abs(h_err) <= to_radians(SUCCESS_HEADING_ERROR))
      {
          current_state = 0;
          break;
      }
      // If the distance error is greater than a certain limit, emphasize fixing bearing error
      if (d_err >= DISTANCE_THREASHOLD)
      {
          //Calculate percentage rates to spin wheeles
          dX = P1_OVER * d_err;
          dTheta = P2_OVER * b_err + P3_OVER * h_err;
      }
      else
      {
          dX = P1_UNDER * d_err;
          dTheta = P2_UNDER * b_err + P3_UNDER * h_err;
      }

      //I think we need to bound Xr.
      if(dX >= 0.3)
        dX = 0.3;

      float phi_l = ((2*dX) - (dTheta*AXLE_DIAMETER))/(2*WHEEL_RADIUS);
      float phi_r = ((2*dX) + (dTheta*AXLE_DIAMETER))/(2*WHEEL_RADIUS);

      if(phi_l >= 0 && phi_r >= 0)
      {
        if (phi_l >= phi_r)
        {
          left_speed_pct = 1;
          right_speed_pct = phi_r / phi_l;
        }
        else
        {
          left_speed_pct = phi_l / phi_r;
          right_speed_pct = 1;
        }
      }
      else if(phi_l < 0)
      {
        right_speed_pct = 1;
        left_speed_pct = -1; 
      }
      else
      {
        right_speed_pct = -1;
        left_speed_pct = 1;
      }

      //Accounting for wheels spinning backwards.
      //I actually dont think the sign thing is neccesary.
      if(phi_l < 0)
      {
        left_dir = DIR_CW;
      }
      else
      {
        left_dir = DIR_CCW;
      }

      if(phi_r < 0)
      {
        right_dir = DIR_CCW;
      }
      else
      {
        right_dir = DIR_CW;
      }

      // Start millis counter
      begin_time = millis();

      // Run motors at percentage towards destination
      sparki.motorRotate(MOTOR_LEFT, left_dir, abs(int(left_speed_pct*100.)));
      sparki.motorRotate(MOTOR_RIGHT, right_dir, abs(int(right_speed_pct*100.)));

      break;
    }
   case 0:
      sparki.moveStop();
//      sparki.clearLCD();
//      sparki.print("DONE");
//      sparki.updateLCD();
    }

  end_time = millis();

  // Stop sparki
  //sparki.moveStop();

  delay_time = end_time - begin_time;
  Serial.println(end_time - begin_time);
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
  {
    sparki.moveStop();
    Serial.println(end_time - begin_time);
    delay(10);
  }
}
