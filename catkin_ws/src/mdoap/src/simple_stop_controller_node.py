#!/usr/bin/env python
import rospy

class SimpleStopControllerNode:
	def __init__(self):
		self.name = 'simple_stop_controller_node'
		rospy.loginfo('[%s] started', self.name)

if __name__=="__main__":
	rospy.init_node('simple_stop_controller_node')
	node = SimpleStopControllerNode()
	rospy.spin()