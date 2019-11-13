import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
#This class deals with interactions between a single drone
#Currently will only support kinematic related pubs/subs
class DroneController:

    def __init__(self, name):
        #Initiate pubs and subs for single drone
        #NOTE: Need to determine if we need specific
        #publisher for land and takeoff.
        self.pilot_pub = rospy.Publisher(name + '/cmd_vel', Twist, queue_size=1)
        self.camera_pub = rospy.Publisher(name + '/camera_control', JointState, queue_size=1)

        self.odom_sub = rospy.Subscriber(name + '/odom', Odometry, odom_callback)
        self.camera_sub = rospy.Subscriber(name + '/joint_states', JointState, camera_callback)

        sleep(1.0)


    def odom_callback():
        pass

    def camera_callback():
        pass

    def takeoff():
        #NOTE: Need to preform flat trim here to calibrate
        pass

    def land():
        pass

    def failsafe():
        #Stop moving drone and land
        pass

class SwarmController:
    drones = [] #List of DroneController objects

    def __init__(self):
        pass
