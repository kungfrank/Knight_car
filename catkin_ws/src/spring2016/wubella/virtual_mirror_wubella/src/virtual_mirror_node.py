#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
from duckietown_msg_wubella.msg import FlipDirection

# bridge = CvBridge()
# publisher = None

class VirtualMirror(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        #self.publish_freq = self.setupParam("~publish_freq",1.0)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~topic", Image,queue_size=1)
        #self.last_stamp = rospy.Time.now()        
        self.sub_compressed_img = rospy.Subscriber("~topic_in",CompressedImage,self.cbImg,queue_size=1)
        
        self.flip_direction = FlipDirection()
        self.flip_direction.direction = self.flip_direction.horz#default horizontal mirror
        self.param_pub = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)
        self.param_pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbPubTimer)
        self.param_sub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbSubTimer)

    def cbPubTimer(self,event):
        self.param_pub.publish(self.flip_direction)

    def cbSubTimer(self,event):
        direction_string = rospy.get_param("~flip_direction", 1.0)
        if direction_string == self.flip_direction.vert or direction_string == self.flip_direction.horz:
            self.flip_direction.direction = direction_string
        rospy.loginfo(self.flip_direction.direction)

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
        W = img_msg.width
        H = img_msg.height
        
        #rgb_in = np.reshape(rgb_in, (H, W, 3))
        #print rgb_in.shape
        rgb_out = rgb_in.copy()
        if self.flip_direction.direction == self.flip_direction.horz:
            for i in range (0, H-1):
                for j in range (0, W-1):
                    rgb_out[3*(i*W+j)] = rgb_in[3*(i*W+W-j-1)]
                    rgb_out[3*(i*W+j)+1] = rgb_in[3*(i*W+W-j-1)+1]
                    rgb_out[3*(i*W+j)+2] = rgb_in[3*(i*W+W-j-1)+2]
        if self.flip_direction.direction == self.flip_direction.vert:
            for i in range (0, H-1):
                for j in range (0, W-1):
                    rgb_out[3*(i*W+j)] = rgb_in[3*((H-i-1)*W+j)]
                    rgb_out[3*(i*W+j)+1] = rgb_in[3*((H-i-1)*W+j)+1]
                    rgb_out[3*(i*W+j)+2] = rgb_in[3*((H-i-1)*W+j)+2]
        

       	img_msg.data = rgb_out.tostring()

        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(img_msg)

        # time_3 = time.time()
        # rospy.loginfo("[%s] Took %f sec to decompress."%(self.node_name,time_1 - time_start))
        # rospy.loginfo("[%s] Took %f sec to conver to Image."%(self.node_name,time_2 - time_1))
        # rospy.loginfo("[%s] Took %f sec to publish."%(self.node_name,time_3 - time_2))

if __name__ == '__main__': 
    rospy.init_node('virtual_mirror_wubella',anonymous=False)
    node = VirtualMirror()
    rospy.spin()

