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
y_size = 120
x_size = 180
world_array = np.zeros([y_size, x_size])

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

    rospy.Timer(rospy.Duration(10), display_map)
    # rospy.Timer(rospy.Duration(10), printArr)

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
            msg.data = [0.1,-1.0]
            #rospy.loginfo("Right Turn")

        elif IR_left < IR_THRESHOLD and IR_left < IR_right:
            #Publish to move left
            msg.data = [-1.0,0.1]
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
            rospy.loginfo("Loop Closure Triggered")
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

    rospy.sleep(0.5)
    #DONE: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    # pose_init = Pose2D()
    # pose_init.x = 0.0
    # pose_init.y = 0.0
    # pose_init.theta = 0.0
    # publisher_odom.publish(pose_init)

    #DONE: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    publisher_servo.publish(Int16(80))


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #DONE: Copy this data into your local odometry variable
    pose2d_sparki_odometry = data


def callback_update_state(data):
    global SRV_angle, IR_left, IR_center, IR_right, PING_dist, world_array
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #DONE: Load data into your program's local state variables
    SRV_angle = math.radians(state_dict['servo'])
    IR_FULL = state_dict['light_sensors']
    IR_left = IR_FULL[1]
    IR_center = IR_FULL[2]
    IR_right = IR_FULL[3]
    try:
        PING_dist = state_dict['ping']
        if PING_dist >= 0:
            rospy.loginfo('Object: %f',PING_dist)
            x, y = convert_ultra_to_world(PING_dist)
            # rospy.loginfo('Object Loc: %f,%f',x,y)
            populate_map_from_ping(x, y)
    except:
        rospy.loginfo('No object')
        PING_dist = None


def convert_ultrasonic_to_robot_coords(x_us):
    global SRV_angle
    #DONE: Using US sensor reading and servo angle, return value in robot-centric coordinates
    # x_us_m = x_us/100.0 #Dont need this becuase simulator is doing meters.
    x_r, y_r = 0., 0.

    x_r = x_us * math.cos(SRV_angle)
    y_r = x_us * math.sin(SRV_angle)
    rospy.loginfo("Servo angle: %s", SRV_angle)
    rospy.loginfo("Robot coordinates:%s,%s",x_r,y_r)

    return (x_r, y_r)


def convert_robot_coords_to_world(pos_vec):
    global pose2d_sparki_odometry
    #DONE: Using odometry, convert robot-centric coordinates into world coordinates
    x_r, y_r = pos_vec
    x_w, y_w = 0., 0.

    _theta = pose2d_sparki_odometry.theta
    _x = pose2d_sparki_odometry.x
    _y = pose2d_sparki_odometry.y


    x_w = x_r*math.cos(_theta) - y_r*math.sin(_theta) + _x
    y_w = x_r*math.sin(_theta) + y_r*math.cos(_theta) + _y

    rospy.loginfo("Robot position:%s,%s",_x,_y)

    rospy.loginfo("World coordinates:%s,%s",x_w,y_w)

    return x_w, y_w


def convert_ultra_to_world(ultra_dist):
    return convert_robot_coords_to_world(convert_ultrasonic_to_robot_coords(ultra_dist))


def world_to_map(x,y):
    x_cm = 100.0*x #Convert to cm for map
    y_cm = 100.0*y
    return int(math.floor(y_cm)), int(math.floor(x_cm))


def map_to_world(i,j):
    return (j+0.5)/100.0, (i+0.5)/100.0 #Put in center of map region and convert to meters


def populate_map_from_ping(x_ping, y_ping):
    global world_array
    #DONE: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    # rospy.loginfo('converting to map')
    i, j = world_to_map(x_ping, y_ping)
    rospy.loginfo("Object at %d,%d",i,j)

    try:
        world_array[i, j] = 1
    except BaseException as e:
        rospy.loginfo("ERROR: %s", e)

    rospy.loginfo("World Check: %d", world_array[i, j])


def display_map(x):
    global world_array
    cmap = colors.ListedColormap(['blue', 'red'])
    bounds=[0,0.5,1]

    norm = colors.BoundaryNorm(bounds, cmap.N)
    plt.imshow(world_array, interpolation='nearest', origin='lower', cmap=cmap, norm=norm)

    plt.draw()
    plt.pause(0.0001)
    plt.clf()


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
    start_i, start_j = cell_index_to_ij(cell_index_from)
    dest_i, dest_j = cell_index_to_ij(cell_index_to)
    manDist = abs(start_i - dest_i) + abs(start_j - dest_j)
    if manDist == 1 and world_array[start_i, start_j] != 1 and world_array[dest_i, dest_j] != 1:
        return manDist
    else:
        return 99


def printArr(x):
    global world_array
    print(world_array)


if __name__ == "__main__":
    main()
