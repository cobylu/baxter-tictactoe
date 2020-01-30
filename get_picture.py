from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image

bridge = CvBridge()

def image_callback(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        cv2.imwrite('view_1.jpg', cv2_img)

rospy.init_node('get_images', anonymous=True)
rospy.Subscriber('cameras/left_hand_camera/image', Image, image_callback)