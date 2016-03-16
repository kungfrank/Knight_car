#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from duckietown_msg_jenshen.msg import Flip
import time

class VirtualMirror(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.virt_mirror = None
        self.publisher = rospy.Publisher("~topic_out", Image, queue_size=1)
        self.subscriber = rospy.Subscriber("~topic_in", CompressedImage, self.callback)
        self.config_pub = rospy.Publisher("~flip_direction", Flip, queue_size=1)

        # Setup Parameters
        self.flip = Flip()
        self.flip_direction = self.setupParam("~flip_direction", "horz")
        self.update_flip_direction(self.flip_direction)
        
        # Timer
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.cbParamTimer)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.cbPubTimer)

    def cbParamTimer(self, event):
        self.flip_string = rospy.get_param("~flip_direction", 1.0)
        self.update_flip_style(self.flip_string)

    def update_flip_direction(self, flip_string):
        self.flip.direction = Flip.HORZ # default = horizontal
        self.flip_code = 1

        if flip_string == 'vert':
            self.flip_code = 0
            rospy.loginfo('flip = vertical')
            self.flip.direction = Flip.VERT
        else:
            rospy.loginfo('flip = horizontal')

    def cbPubTimer(self, event):
        self.config_pub.publish(self.flip)
        
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.flip(image_np, self.flip_code)

        msg = bridge.cv2_to_imgmsg(image_np, encoding="passthrough")
        self.publisher.publish(msg)    

if __name__ == "__main__":
    rospy.init_node("virtual_mirror_jenshen_node",anonymous=True)
    virtual_mirror = VirtualMirror()
    rospy.spin()
