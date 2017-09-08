#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamic_reconfigure.server import Server
from dynamic_pid.cfg import virtual_gainConfig


class virtual_joy(object):
	def __init__(self):
		self.node_name = rospy.get_name() 

		srv = Server(virtual_gainConfig, self.callback)

		self.pub_gazebo = rospy.Publisher('/duckiebot_with_gripper/cmd_vel', Twist, queue_size=1)
		self.pub_gazebo_gripper = rospy.Publisher('/duckiebot_gripper/cmd_vel', Twist, queue_size=1)

		self.pub_encoder_reset = rospy.Publisher('/reset_encoder', Bool, queue_size=1)
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

		model_state_msg.linear.x = 0
		model_state_msg.angular.z = 0
		if joy_msg.buttons[0] == 1:
			model_state_msg.angular.z = -1
		elif joy_msg.buttons[1] == 1:
			model_state_msg.angular.z = 1

		self.pub_gazebo_gripper.publish(model_state_msg)
		
		if (joy_msg.buttons[7] == 1):
			reset_msg = Bool()
			reset_msg.data = True
			self.pub_encoder_reset.publish(reset_msg)

	def callback(self, config, level):
		rospy.loginfo("Reconfigure Request: V_gain = {V_gain}, \tW_gain = {W_gain}".format(**config))
		self.v_gain= config.V_gain
		self.w_gain = config.W_gain

		return config

if __name__ == "__main__":
	rospy.init_node("virtual_joy",anonymous=False)
	virtual_joy = virtual_joy()
	rospy.spin()
