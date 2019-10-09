import rospy
import json
import copy
import time
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#DONE: Track servo angle in radians
SRV_angle
#DONE: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
IR_center
IR_left
IR_right
PING_dist
#DONE: Create data structure to hold map representation
map_array = np.zeros([20, 10]);

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

    #DONE: Init your node to register it with the ROS core
    rospy.init_node('sparki', anonymous=True)
    init()

    while not rospy.is_shutdown():
        #DONE: Implement CYCLE TIME
        rate = rospy.Rate(1/CYCLE_TIME)

        #Ping the ultrasonic
        ping = Empty
        publisher_ping.pub(ping)
        rospy.loginfo(Ping_dist)

        #DONE: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        msg = Float32MultiArray()
        if IR_center > IR_THRESHOLD and IR_left < IR_THRESHOLD and IR_right < IR_THRESHOLD:
            #Publish to move forward
            msg.data = [1.0,1.0]

        elif IR_left > IR_THRESHOLD and IR_right < IR_THRESHOLD and IR_center < IR_THRESHOLD:
            #Publish to move left
            msg.data = [-1.0,1.0]

        elif IR_right > IR_THRESHOLD and IR_left < IR_THRESHOLD and IR_center < IR_THRESHOLD:
            #publish to move right
            msg.data = [1.0,-1.0]

        publisher_motor.pub(msg)

        #DONE: Implement loop closure here
        if IR_right > IR_THRESHOLD and IR_left > IR_THRESHOLD and IR_center > IR_THRESHOLD:
            rospy.loginfo("Loop Closure Triggered")
            reset = Pose2D
            reset.data = [0,0,0]
            publisher_odom.pub(reset)

        #DONE: Implement CYCLE TIME
        rate.sleep()



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    #DONE: Set up your publishers and subscribers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=1)
    publisher_ping = rospy.Publisher('/sparki/ping_command', String, queue_size=1)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=1)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=1)

    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)

    #DONE: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose_init = Pose2D
    pose_init.data = [0,0,0]
    publisher_odom.pub(pose_init)

    #DONE: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    publisher_servo.publish(45)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #DONE: Copy this data into your local odometry variable
    pose2d_sparki_odometry = data

def callback_update_state(data):
    global SRV_angle, IR_left, IR_center, IR_right, PING_dist
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #DONE: Load data into your program's local state variables
    SRV_angle = state_dict['servo']
    PING_dist = state_dict['ping']
    IR_FULL = state_dict['light_sensors']
    IR_left = IR_FULL[1]
    IR_center = IR_FULL[2]
    IR_right = IR_FULL[3]


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
