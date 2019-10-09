import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
#TODO: Create data structure to hold map representation

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.1 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME

        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        #TODO: Implement loop closure here
        if False:
            rospy.loginfo("Loop Closure Triggered")

        #TODO: Implement CYCLE TIME
        rospy.sleep(0)



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    #TODO: Set up your publishers and subscribers
    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #TODO: Copy this data into your local odometry variable

def callback_update_state(data):
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables

def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


