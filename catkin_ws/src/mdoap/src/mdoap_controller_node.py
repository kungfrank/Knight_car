#!/usr/bin/env python
import rospy

class MDOAPControllerNode:
	def __init__(self):
		self.name = 'mdoap_controller_node'
		rospy.loginfo('[%s] started', self.name)

if __name__=="__main__":
	rospy.init_node('mdoap_controller_node')
	node = MDOAPControllerNode()
	rospy.spin()