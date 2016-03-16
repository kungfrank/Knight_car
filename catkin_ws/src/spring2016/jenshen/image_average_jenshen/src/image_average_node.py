#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

class ImageAverager:
    def __init__(self):
        self.publisher = rospy.Publisher("~topic_out", Image, queue_size=1)
        self.subscriber = rospy.Subscriber("~topic_in", CompressedImage, self.callback)
        self.image_avg = None
        self.num_frames = 0
    
    def callback(self, msg):
        # Load image
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        
        # Convert image to float
        image_np_float = image_np.astype('float')
        
        # Start accumulating image averages
        if self.image_avg is None:
            self.image_avg = image_np_float
        # Average new image with previous average
        else:
            prev_weight = float(self.num_frames)/float(self.num_frames+1)
            new_weight = 1.0 - prev_weight
            self.image_avg = cv2.addWeighted(self.image_avg, prev_weight, image_np_float, new_weight, 0.0)
            
        self.num_frames = self.num_frames+1

        msg = bridge.cv2_to_imgmsg(self.image_avg.astype('uint8'), "bgr8")
        self.publisher.publish(msg)

def init_image_averager_node():
    rospy.init_node('image_average_node', anonymous=True)
    averager = ImageAverager()
    
    rospy.spin() #Keeps the script for exiting

if __name__ == '__main__':
    init_image_averager_node() 
