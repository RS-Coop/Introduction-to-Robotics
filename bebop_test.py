# Program to cause the Bebop2 to take off, track a face, and land after a fixed interval

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import cv2 # To install: pip3 install opencv-contrib-python
import cv_bridge # To install: sudo apt-get install ros-melodic-cv-bridge
import dlib # To install: pip3 install dlib
import time

FLIGHT_TIME = 20 # seconds
FACE_REC_INTERVAL = .2 # seconds
FRAME_WIDTH = 428
FRAME_HEIGHT = 240

bridge = None
face_detector = dlib.get_frontal_face_detector()
win = dlib.image_window()
last_image = None

drone_pub = None

last_img_call = 0

def img_callback(img_msg):
  global bridge, last_image, last_img_call, FACE_REC_INTERVAL, FRAME_WIDTH, FRAME_HEIGHT

  # Only call this function once every FACE_REC_INTERVAL seconds
  if time.time() - last_img_call < FACE_REC_INTERVAL: return
  last_img_call = time.time()
  unscaled_cv_image = bridge.imgmsg_to_cv2(img_msg, "mono8")
  cv_image = cv2.resize(unscaled_cv_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_LINEAR)
  last_image = cv_image
  FRAME_WIDTH = last_image.shape[1]
  FRAME_HEIGHT = last_image.shape[0]



def find_faces(cv_image):
  global face_detector, win, last_call
  
  face_position = [None, None]

  faces = face_detector(cv_image, 1)
  if len(faces) > 0:
    print("Detections: {}".format(len(faces)))
    for i, d in enumerate(faces):
      print("Face {}: Left: {}, Top: {}, Right: {}, Bottom: {}".format(i, d.left(), d.top(), d.right(), d.bottom()))
      if i == 0: face_position = (.5 * (d.right() + d.left()) / FRAME_WIDTH, .5 * (d.bottom() + d.top()) / FRAME_HEIGHT)

  # Draw faces
  rects = dlib.rectangles()
  rects.extend([d for d in faces])
  win.clear_overlay()
  win.set_image(cv_image) 
  win.add_overlay(rects)

  return face_position

def adjust_drone_pos(face_pos):
  global drone_pub

  if face_pos[0] is None: return

  pos_update = Twist()

  if face_pos[0] < 0.4:
    # Turn CCW
    print("CCW")
    pos_update.angular.z = 0.2 # Turn Counterclockwise
  elif face_pos[0] > 0.6:
    # Turn CW
    print("CW")
    pos_update.angular.z = -0.2 # Turn Clockwise

  if face_pos[1] < 0.4:
    # Increase altitude
    print("Ascend")
    pos_update.linear.z = 0.1

  elif face_pos[1] > 0.6:
    # Reduce altitude
    print("Descend")
    pos_update.linear.z = -0.1

  drone_pub.publish(pos_update)




def main():
  global last_image, drone_pub
  rospy.init_node("FaceTracker")
  camera_sub = rospy.Subscriber("/bebop/image_raw", Image, img_callback,queue_size=1)
  drone_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
  takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
  landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)

  # Wait until the camera starts giving us frames
  while last_image is None:
    time.sleep(0.5)

  takeoff_pub.publish(Empty())
  time.sleep(2.) # Give the drone time to take off!

  start_time = time.time()
  while not rospy.is_shutdown() and time.time() - start_time < FLIGHT_TIME:
    if last_image is not None:
      face_position = find_faces(last_image)    
      last_image = None

      adjust_drone_pos(face_position)    

  landing_pub.publish(Empty())
  print("Shutdown.")

  cv2.destroyAllWindows()


if __name__ == '__main__':
  bridge = cv_bridge.CvBridge()
  main()
