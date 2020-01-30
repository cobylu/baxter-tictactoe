from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image


class BaxterCV():
    def __init__(self):
        bridge = CvBridge()
        rospy.init_node('get_images', anonymous=True)
        rospy.Subscriber('cameras/left_hand_camera/image', Image, image_callback)
        
    def image_callback(self, msg):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            cv2.imwrite('baxter_img.jpg', cv2_img)