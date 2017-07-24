#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from duckietown_msgs.msg import Twist2DStamped
import time
from dynamic_reconfigure.server import Server
from dynamic_pid.cfg import PIDConfig

class gazebo_sub_jointstate(object):
	def __init__(self):
		self.node_name = rospy.get_name() 

		# self.d_PerCount = ((3.14159265 * 0.13) / 5940)*0.3/0.635
		# self.Length =  0.18
		self.R = 0
		self.L = 0

		self.pretime = 0.0
		self.sub_joint_state_car = rospy.Subscriber('/duckiebot_with_gripper/joint_states', JointState, self.cbJoinstate, queue_size=1)
		# Subscription
		self.sub_encoder = rospy.Subscriber("/encoder", Point, self.cbEncoder, queue_size=1)
		self.pub_car_cmd = rospy.Publisher("/gazebo_sub_jointstate/control_value",Point,queue_size=1)
		self.pub_threshold = rospy.Publisher("/gazebo_sub_jointstate/threshold_value",Int64,queue_size=1)
		# self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)
		srv = Server(PIDConfig, self.callback)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	# def updateParams(self,event):
	# 	print "P = ",self.kp,"\tI = ",self.ki,"\tD = ",self.kd

	def custom_shutdown(self):

		rospy.loginfo("[%s] Shutting down..." %self.node_name)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbJoinstate(self, jointstate_msg):

		time_now = time.time()
		time_inteval = time_now - self.pretime

		R_current = jointstate_msg.position[0] * ( 2970 / (2 * math.pi))
		L_current = jointstate_msg.position[1] * ( 2970 / (2 * math.pi))

		err_R = R_current - self.R
		err_L = L_current - self.L

		u_R = int(self.kp * err_R + self.kd * err_R/time_inteval)
		u_L = int(self.kp * err_L + self.kd * err_L/time_inteval)
		print "R_current = ",R_current,"\tL_current = ", L_current,"\tR_real = ",self.R,"\tL_real = ", self.L,"\tu_R = ",u_R,"\tu_L = ",u_L

		control_value_msg = Point()
		control_value_msg.x = u_R
		control_value_msg.y = u_L
		control_value_msg.z = 0

		self.pub_car_cmd.publish(control_value_msg)

		self.pretime = time_now


	def cbEncoder(self, encoder_msg):
		self.R = encoder_msg.y
		self.L = encoder_msg.x
		
	def callback(self, config, level):
   		rospy.loginfo("Reconfigure Request: P = {P_param}, \tI = {I_param},\tD = {D_param},\tT = {Threshold_param}".format(**config))
  		self.kp = config.P_param
   		self.ki = config.I_param
   		self.kd = config.D_param
   		
   		threshold_msg = Int64()
   		threshold_msg.data = config.Threshold_param
   		self.pub_threshold.publish(threshold_msg)
   		
   		return config

if __name__ == "__main__":
	rospy.init_node("gazebo_sub_jointstate",anonymous=True)
	gazebo_sub_jointstate = gazebo_sub_jointstate()
	rospy.spin()
