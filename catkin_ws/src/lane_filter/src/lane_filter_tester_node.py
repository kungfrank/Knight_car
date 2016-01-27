#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment

class LaneFilterTesterNode(object):
    def __init__(self):
        node_name = "Lane Filter Tester"
        pub_fake_segment_list = rospy.Publisher("~segment_list", SegmentList)
        seg = Segment()
        seg.points[0].x = rospy.get_param("~x1",0.1)
        seg.points[0].y = rospy.get_param("~y1",0.1)
        seg.points[1].x = rospy.get_param("~x2",0.3)
        seg.points[1].y = rospy.get_param("~y2",0.1)
        color = rospy.get_param("color","white")


        if color=="white":
            seg.color=seg.WHITE
        elif color=="yellow":
            seg.color=seg.YELLOW
        elif color=="red":
            seg.color=seg.RED

        seg_list = SegmentList()
        seg_list.segments[0] = seg
        pub_fake_segment_list.publish(seg_list)

    def onShutdown(self):
        rospy.loginfo("[LaneFilterTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('lane_filter_tester',anonymous=False)
    lane_filter_tester_node = LaneFilterTesterNode()
    rospy.on_shutdown(lane_filter.onShutdown)
    rospy.spin()
