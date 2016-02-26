#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
image_pub = rospy.Publisher("output",Image,queue_size=1)

def callback(original_image):
    H = original_image.height;    
    W = original_image.width;    
    image_in = bridge.imgmsg_to_cv2(original_image,"bgr8")
    new_image = bridge.imgmsg_to_cv2(original_image,"bgr8")
    for x in range(H):
	for y in range(W):
		new_image[x][y] = image_in[x][W-y-1]
    image_pub.publish(bridge.cv2_to_imgmsg(new_image,"bgr8"))

def listener():
    rospy.init_node('virtual_mirror_node', anonymous=False)

    rospy.Subscriber("input", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
