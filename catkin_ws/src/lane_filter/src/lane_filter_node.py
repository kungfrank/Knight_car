#!/usr/bin/env python
import rospy
import numpy as np
from LaneFilter import *

class LaneFilterNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber("~segments", SegmentList, self.filterLane)

    def filterLane(self,image_msg):
	pass

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()

