#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

# bridge = CvBridge()
# publisher = None

class ImageAverage(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        self.pub_img = rospy.Publisher("~image/average",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()
        self.sub_img = rospy.Subscriber("~img_in",CompressedImage, self.avgImage)

        self.prior_img = None
        self.n = 0.0

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def avgImage(self,msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.n = self.n+1.0
        # time_1 = time.time()
        if self.prior_img == None:
            self.prior_img = cv_image
        else:
            firstDiv = 1.0/self.n
           # rospy.loginfo(str(1.0-(1.0/self.n))+ "is first " + str(1.0/self.n) + " is second" )
            self.prior_img = self.prior_img.astype('float32') * (1-firstDiv) + cv_image.astype('float32') * (firstDiv)
        # time_2 = time.time()
        toreturn = self.bridge.cv2_to_imgmsg(self.prior_img.astype('uint8'), "bgr8") 
        self.pub_img.publish(toreturn)


        # time_3 = time.time()
        # rospy.loginfo("[%s] Took %f sec to decompress."%(self.node_name,time_1 - time_start))
        # rospy.loginfo("[%s] Took %f sec to conver to Image."%(self.node_name,time_2 - time_1))
        # rospy.loginfo("[%s] Took %f sec to publish."%(self.node_name,time_3 - time_2))

if __name__ == '__main__':
    rospy.init_node('image_average_lapentab',anonymous=False)
    node = ImageAverage()
    rospy.spin()

