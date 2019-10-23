import rospy
import json
import copy
import time
import numpy as np
import math
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import matplotlib.pyplot as plt
from matplotlib import colors

# GLOBALS
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#DONE: Track servo angle in radians
SRV_angle = 0
#DONE: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
IR_center = 0
IR_left = 0
IR_right = 0
PING_dist = 0
#DONE: Create data structure to hold map representation
y_size = 10
x_size = 20
world_array = np.zeros([y_size, x_size]);

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS
IR_THRESHOLD = 700 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.05 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #DONE: Init your node to register it with the ROS core
    rospy.init_node('sparki', anonymous=True)
    init()

    # rospy.Timer(rospy.Duration(10), display_map)

    while not rospy.is_shutdown():
        #DONE: Implement CYCLE TIME
        rate = rospy.Rate(1.0/CYCLE_TIME)

        #Ping the ultrasonic
        publisher_ping.publish(Empty())
        #Are updating map from within callback
        # rospy.loginfo("Ping:%s",PING_dist)
        # try:
        #     convert_ultra_to_world(PING_dist)
        # except:
        #     pass

        #DONE: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        msg = Float32MultiArray()
        msg.data = [0.0,0.0]
        if IR_right < IR_THRESHOLD and IR_right < IR_left:
	        #publish to move right
            msg.data = [0.8,-1.0]
            #rospy.loginfo("Right Turn")

        elif IR_left < IR_THRESHOLD and IR_left < IR_right:
            #Publish to move left
            msg.data = [-1.0,0.8]
            #rospy.loginfo("Left Turn")

        elif IR_center < IR_THRESHOLD and IR_left > IR_THRESHOLD and IR_right > IR_THRESHOLD:
	        #Publish to move forward
            msg.data = [1.0,1.0]
            #rospy.loginfo("Forward")

        # else: #Only added this because it wasnt working
        #     msg.data = [0.0,0.0]
        #     #rospy.loginfo("Default")

        # msg.data = [1.0,1.0] #For debugging purposes.

        publisher_motor.publish(msg)

        #DONE: Implement loop closure here
        if IR_right < IR_THRESHOLD and IR_left < IR_THRESHOLD and IR_center < IR_THRESHOLD:
            #rospy.loginfo("Loop Closure Triggered")
            reset = Pose2D()
            reset.x = 0.0
            reset.y = 0.0
            reset.theta = 0.0
            publisher_odom.publish(reset)

        #DONE: Implement CYCLE TIME
        rate.sleep()



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    #DONE: Set up your publishers and subscribers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=1)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=1)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=1)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=1)

    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)

    rospy.sleep(0.1)
    #DONE: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    # pose_init = Pose2D()
    # pose_init.x = 0.0
    # pose_init.y = 0.0
    # pose_init.theta = 0.0
    # publisher_odom.publish(pose_init)

    #DONE: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    publisher_servo.publish(Int16(45))

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
    try:
        PING_dist = state_dict['ping']
        x, y = convert_ultra_to_world(PING_dist)
        populate_map_from_ping(x, y)
    except:
        PING_dist = None
    IR_FULL = state_dict['light_sensors']
    IR_left = IR_FULL[1]
    IR_center = IR_FULL[2]
    IR_right = IR_FULL[3]


def convert_ultrasonic_to_robot_coords(x_us):
    #DONE: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_us = x_us/100.0
    x_r, y_r = 0., 0.

    x_r = x_us * math.cos(SRV_angle)
    y_r = x_us * math.sin(SRV_angle)

    # rospy.loginfo("Robot coordinates:%s,%s",x_r,y_r)

    return (x_r, y_r)

def convert_robot_coords_to_world(pos_vec):
    #DONE: Using odometry, convert robot-centric coordinates into world coordinates
    x_r, y_r = pos_vec
    x_w, y_w = 0., 0.

    _theta = pose2d_sparki_odometry.theta
    _x = pose2d_sparki_odometry.x
    _y = pose2d_sparki_odometry.y


    x_w = x_r*math.cos(_theta) - y_r*math.sin(_theta) + _x
    y_w = x_r*math.sin(_theta) + y_r*math.cos(_theta) + _y

    # rospy.loginfo("Robot position:%s,%s",_x,_y)

    # rospy.loginfo("World coordinates:%s,%s",x_w,y_w)

    return x_w, y_w

def convert_ultra_to_world(ultra_dist):
    return convert_robot_coords_to_world(convert_ultrasonic_to_robot_coords(ultra_dist))

def world_to_map(x,y):
    return math.floor(y), math.floor(x)

def map_to_world(i,j):
    return j+0.5, i+0.5

def populate_map_from_ping(x_ping, y_ping):
    #DONE: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    i, j = world_to_map(x_ping, y_ping)
    map_array[i, j] = 1
    rospy.loginfo("Object at %d,%d",i,j)

def display_map(x):
    plt.close('all')
    cmap = colors.ListedColormap(['blue', 'red'])
    bounds=[0,0.5,1]

    norm = colors.BoundaryNorm(bounds, cmap.N)
    plt.imshow(world_array, interpolation='nearest', origin='lower', cmap=cmap, norm=norm)

    plt.show()

def ij_to_cell_index(i,j):
    #DONE: Convert from i,j coordinates to a single integer that identifies a grid cell
    return j + i * x_size

def cell_index_to_ij(cell_index):
    #DONE: Convert from cell_index to (i,j) coordinates
    i = '%.0f'%(cell_index / x_size)
    j = cell_index % x_size
    return i, j


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    # This should be an admissible heuristic. Since this doesn't account for obsticles
    # the manhattan distance should always be optimistic.
    start_i, start_j = cell_index_to_ij(cell_index_from)
    dest_i, dest_j = cell_index_to_ij(cell_index_to)
    return abs(start_i - dest_i) + abs(start_j - dest_j)

if __name__ == "__main__":
    main()
