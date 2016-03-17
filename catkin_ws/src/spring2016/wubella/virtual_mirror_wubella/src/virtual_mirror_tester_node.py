#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
from std_msgs.msg import String

# bridge = CvBridge()
# publisher = None

class VirtualMirrorTester(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        self.testImageOrig = cv2.imread('/home/wyatt/duckietown/catkin_ws/src/spring2016/wubella/virtual_mirror_wubella/src/01_orig.png')
        self.testImageHorz = cv2.imread('/home/wyatt/duckietown/catkin_ws/src/spring2016/wubella/virtual_mirror_wubella/src/01_horz.png')
        self.testImageVert = cv2.imread('/home/wyatt/duckietown/catkin_ws/src/spring2016/wubella/virtual_mirror_wubella/src/01_vert.png')
        self.testImageMsgOrig = self.bridge.cv2_to_imgmsg(self.testImageOrig, "bgr8")
        self.testImageMsgHorz = self.bridge.cv2_to_imgmsg(self.testImageHorz, "bgr8")
        self.testImageMsgVert = self.bridge.cv2_to_imgmsg(self.testImageVert, "bgr8")
        #self.publish_freq = self.setupParam("~publish_freq",1.0)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~original/compressed", CompressedImage,queue_size=1)

        #self.last_stamp = rospy.Time.now()        
        self.sub_raw = rospy.Subscriber("~image_in", CompressedImage,self.cbImg,queue_size=1)
        
        self.result_pub = rospy.Publisher("~result", String, queue_size=1)
        self.param_pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.pubOriginal)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    def pubOriginal(self, msg):
        compressed_img_msg = CompressedImage()
        compressed_img_msg.format = "png"
        compressed_img_msg.data = np.array(cv2.imencode('.png', self.testImageOrig)[1]).tostring()

        #compressed_img_msg.header.stamp = msg.header.stamp
        #compressed_img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(compressed_img_msg)


    def cbImg(self,msg):
        direction_string = rospy.get_param('/charles/virtual_mirror_node/flip_direction')
        #rospy.loginfo(direction_string)
        if direction_string == 'vert':
            if np.array(cv2.imencode('.png', self.testImageVert)[1]).tostring() == msg.data:
                self.result_pub.publish("Pass")
                rospy.loginfo("Pass")
            else:
                self.result_pub.publish("Fail")
                rospy.loginfo("Fail")
            rospy.set_param('/charles/virtual_mirror_node/flip_direction', 'horz')
        
        elif direction_string == 'horz':
            if np.array(cv2.imencode('.png', self.testImageHorz)[1]).tostring() == msg.data:
                self.result_pub.publish("Pass")
                rospy.loginfo("Pass")
            else:
                self.result_pub.publish("Fail")
                rospy.loginfo("Fail")
            rospy.set_param('/charles/virtual_mirror_node/flip_direction', 'vert')

if __name__ == '__main__': 
    rospy.init_node('virtual_mirror_tester_node',anonymous=False)
    node = VirtualMirrorTester()
    rospy.spin()

