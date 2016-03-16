#!/usr/bin/env python
import rospy

class StaticObjectDetectorNode:
	def __init__(self):
		self.name = 'static_object_detector_node'
		rospy.loginfo('[%s] started', self.name)

if __name__=="__main__":
	rospy.init_node('static_object_detector_node')
	node = StaticObjectDetectorNode()
	rospy.spin()