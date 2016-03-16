#!/usr/bin/env python
import rospy

class ObstacleSafetyNode:
	def __init__(self):
		self.name = 'obstacle_safety_node'
		rospy.loginfo('[%s] started', self.name)

if __name__=="__main__":
	rospy.init_node('obstacle_safety_node')
	node = ObstacleSafetyNode()
	rospy.spin()