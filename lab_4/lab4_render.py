import rospy
from std_msgs.msg import Empty

def img_render():
    pub = rospy.Publisher('/sparki/render_sim', Empty, queue_size=1)
    rospy.init_node('renderer', anonymous=True)
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        pub.publish(Empty())
        rate.sleep()

if __name__ == '__main__':
    try:
        img_render()
    except:
        pass
