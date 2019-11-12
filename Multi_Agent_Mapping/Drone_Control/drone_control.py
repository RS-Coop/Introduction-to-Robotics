import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
#This class deals with interactions between a single drone
#Currently will only support kinematic related pubs/subs
class DroneController:

    def __init__(self):
        #Initiate pubs and subs for single drone
        #NOTE: Need to determine if we need specific
        #publisher for land and takeoff.
        self.pilot_pub = rospy.Publisher('', Twist, queue_size=1)
        self.camera_pub = rospy.Publisher('', JointState, queue_size=1)

        self.odom_sub = rospy.Subscriber('', Odometry, odom_callback)
        self.camera_sub = rospy.Subscriber('', JointState, camera_callback)

    def odom_callback():
        pass

    def camera_callback():
        pass

class CentralNode:
    drones = [] #List of DroneController objects

    def __init__(self):
        pass
