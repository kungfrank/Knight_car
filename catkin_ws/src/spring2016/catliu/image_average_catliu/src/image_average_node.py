#!/usr/bin/env python
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

class ImageAverageNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher("~topic_out",Image,queue_size=1)
        # Create subscriber
        self.subscriber = rospy.Subscriber("~topic_in", CompressedImage, self.callback)
        self.avg = None
        self.numFrames = 0.0

    # Define Timer callback
    def callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        cv_image = cv_image.astype('float32')
        if self.numFrames == 0:
            self.avg = cv_image
        else:
            cv2.addWeighted(self.avg, self.numFrames/(self.numFrames+1),cv_image, 1/(self.numFrames+1), 0.0, self.avg)
        self.numFrames+=1

        img_msg = self.bridge.cv2_to_imgmsg(self.avg.astype('uint8'), "bgr8")
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id

        self.publisher.publish(img_msg)

if __name__ == '__main__':
    rospy.init_node('image_average_node')
    node = ImageAverageNode()
    # spin to keep the script for exiting
    rospy.spin()
