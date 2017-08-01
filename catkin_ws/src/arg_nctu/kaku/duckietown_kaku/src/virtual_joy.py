#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamic_reconfigure.server import Server
from dynamic_pid.cfg import virtual_gainConfig

class virtual_joy(object):
	def __init__(self):
		self.node_name = rospy.get_name() 

		srv = Server(virtual_gainConfig, self.callback)

		self.pub_gazebo = rospy.Publisher('/duckiebot_with_gripper/cmd_vel', Twist, queue_size=1)
		# Subscription
		self.sub_joy = rospy.Subscriber("/joy", Joy, self.cbJoy, queue_size=1)
		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):

		rospy.loginfo("[%s] Shutting down..." %self.node_name)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbJoy(self, joy_msg):

		model_state_msg = Twist()
		model_state_msg.linear.x = joy_msg.axes[1] * self.v_gain
		model_state_msg.linear.y = 0
		model_state_msg.linear.z = 0

		model_state_msg.angular.x = 0
		model_state_msg.angular.y = 0
		model_state_msg.angular.z = -joy_msg.axes[3] * self.w_gain
		
		self.pub_gazebo.publish(model_state_msg)

	def callback(self, config, level):
   		rospy.loginfo("Reconfigure Request: V_gain = {V_gain}, \tW_gain = {W_gain}".format(**config))
  		self.v_gain= config.V_gain
   		self.w_gain = config.W_gain

   		return config

if __name__ == "__main__":
	rospy.init_node("virtual_joy",anonymous=False)
	virtual_joy = virtual_joy()
	rospy.spin()
