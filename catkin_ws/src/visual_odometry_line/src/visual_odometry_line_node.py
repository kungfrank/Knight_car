#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose
from scipy.stats import multivariate_normal, entropy
from math import floor, atan2, pi, cos, sin
import time

# Visual Odometry Line
# Author: Wyatt Ubellacker
# Inputs: SegmentList from line detector
# Outputs: Calculated Linear and Angular Velocity and uncertanity


class VisualOdometryLineNode(object):
    def __init__(self):
        self.node_name = "Visual Odometry Line"
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments)
        self.old_segment_list = SegmentList()

        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
        self.pub_segments = rospy.Publisher("~segment_list_repeater", SegmentList, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def processSegments(self,segment_list_msg):
        t_start = rospy.get_time()


        self.pub_segments.publish(segment_list_msg)
        self.old_segment_list = segment_list_msg
        # print "time to process segments:"
        # print rospy.get_time() - t_start
    
    def onShutdown(self):
        rospy.loginfo("[VisualOdometryLineNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('visual_odometry_line',anonymous=False)
    visual_odometry_line_node = VisualOdometryLineNode()
    rospy.on_shutdown(visual_odometry_line_node.onShutdown)
    rospy.spin()

