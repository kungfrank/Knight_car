#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

# bridge = CvBridge()
# publisher = None

class DecoderNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        self.average_image_rgb = []
        self.average_count = 0;
        #self.publish_freq = self.setupParam("~publish_freq",1.0)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~topic", Image,queue_size=1)
        #self.last_stamp = rospy.Time.now()        
        self.sub_compressed_img = rospy.Subscriber("~topic_in",CompressedImage,self.cbImg,queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImg(self,msg):
        # time_start = time.time()
        

        np_arr = np.fromstring(msg.data, np.uint8)
         
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # time_1 = time.time()
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        rgb_in = np.fromstring(img_msg.data, np.uint8)
        #average_rgb_in = np.fromstring(self.average_image.data, np.uint8)
        #print rgb_in
        #print average_rgb_in
        W = img_msg.width
        H = img_msg.height
        if (self.average_count !=0):
            count = float(self.average_count)
            self.average_image_rgb = (rgb_in + self.average_image_rgb*self.average_count)/(count+1.0)
            rgb_out = np.round(self.average_image_rgb)
        else:
            rgb_out = rgb_in.copy()
            self.average_image_rgb = rgb_out
        self.average_count+=1
        #print self.average_count

       	img_msg.data = rgb_out.astype(np.uint8).tostring()

        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(img_msg)

        # time_3 = time.time()
        # rospy.loginfo("[%s] Took %f sec to decompress."%(self.node_name,time_1 - time_start))
        # rospy.loginfo("[%s] Took %f sec to conver to Image."%(self.node_name,time_2 - time_1))
        # rospy.loginfo("[%s] Took %f sec to publish."%(self.node_name,time_3 - time_2))

if __name__ == '__main__': 
    rospy.init_node('decoder_low_freq',anonymous=False)
    node = DecoderNode()
    rospy.spin()

