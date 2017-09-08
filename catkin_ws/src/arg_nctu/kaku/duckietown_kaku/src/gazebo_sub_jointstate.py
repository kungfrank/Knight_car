#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from duckietown_msgs.msg import Twist2DStamped
import time
from dynamic_reconfigure.server import Server
from dynamic_pid.cfg import PIDConfig
from std_srvs.srv import Empty

class gazebo_sub_jointstate(object):
	def __init__(self):
		self.node_name = rospy.get_name() 

		# self.d_PerCount = ((3.14159265 * 0.13) / 5940)*0.3/0.635
		# self.Length =  0.18
		self.R = 0
		self.L = 0
		self.err_sumR = 0
		self.err_sumL = 0
		self.err_lastR = 0
		self.err_lastL = 0

		self.swith = True
		self.pretime = 0.0
		self.sub_joint_state_car = rospy.Subscriber('/duckiebot_with_gripper/joint_states', JointState, self.cbJoinstate, queue_size=1)
		# Subscription

		self.pub_car_cmd = rospy.Publisher("/gazebo_sub_jointstate/control_value",Point,queue_size=1)
		# self.pub_threshold = rospy.Publisher("/gazebo_sub_jointstate/threshold_value",UInt32,queue_size=1)
		# self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)
		srv = Server(PIDConfig, self.callback)

		self.sub_encoder = rospy.Subscriber("/encoder", Point, self.cbEncoder, queue_size=1)
		self.sub_reset_encoder = rospy.Subscriber("/reset_encoder", Bool, self.cbReset, queue_size=1)
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
		if not self.swith:
			return
		# time_now = time.time()
		# self.time_inteval = time_now - self.pretime

		R_current = jointstate_msg.position[0] * ( 2970 / (1 * math.pi))
		L_current = jointstate_msg.position[1] * ( 2970 / (1 * math.pi))
		print "R_current = ",R_current,"\tL_current = ",L_current
		print "R_real = ",self.R,"\tL_real = ",self.L,"\n"
		
		err_R = R_current - self.R
		err_L = L_current - self.L

		if (err_R < self.Th and err_R >= 0) or (err_R > -self.Th and err_R <= 0):
			err_R = 0
		if (err_L < self.Th and err_L >= 0) or (err_L > -self.Th and err_L <= 0):
			err_L = 0

		self.err_sumR += err_R # * self.time_inteval
		self.err_sumL += err_L # * self.time_inteval

		# if self.err_sumR > 255:
		# 	self.err_sumR = 255
		# if self.err_sumR < -255:
		# 	self.err_sumR = -255

		# if self.err_sumL > 255:
		# 	self.err_sumL = 255
		# if self.err_sumL < -255:
		# 	self.err_sumL = -255

		err_kd_R = err_R - self.err_lastR
		err_kd_L = err_L - self.err_lastL

		# print "err_sumR = ",self.err_sumR,"\terr_sumL = ",self.err_sumL
		# print "err_kd_R = ",err_kd_R,"\terr_kd_L = ",err_kd_L

		u_R = int(self.kp * err_R + self.ki * self.err_sumR + self.kd * err_kd_R) #/self.time_inteval)
		u_L = int(self.kp * err_L + self.ki * self.err_sumL + self.kd * err_kd_L) #/self.time_inteval)
		# print "u_R = ",u_R,"\tu_L = ",u_L

		self.pub_control_valuse(u_R,u_L)

		# self.pretime = time_now
		self.err_lastR = err_R
		self.err_lastL = err_L

	def pub_control_valuse(self,u_R,u_L):
		control_value_msg = Point()
		control_value_msg.x = u_R
		control_value_msg.y = u_L
		control_value_msg.z = 0

		self.pub_car_cmd.publish(control_value_msg)

	def cbEncoder(self, encoder_msg):
		self.R = encoder_msg.y
		self.L = encoder_msg.x

	def cbReset(self, reset_msg):
		if reset_msg.data == True:
			self.swith = False
			print "====================== ENCODER RESET ======================"
			self.err_sumR = 0
			self.err_sumL = 0
			self.err_lastR = 0
			self.err_lastL = 0

			self.pub_control_valuse(0,0)
			self.reset_gazebo()
			# time.sleep(1)

			self.swith = True
	
	def reset_gazebo(self):
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
			reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
			reset()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def callback(self, config, level):
   		rospy.loginfo("Reconfigure Request: P = {P_param}, \tI = {I_param},\tD = {D_param},\tT = {Threshold_param}".format(**config))
  		self.kp = config.P_param
   		self.ki = config.I_param
   		self.kd = config.D_param
   		self.Th = config.Threshold_param
   		
   		# threshold_msg = Int64()
   		# threshold_msg.data = config.Threshold_param
   		# self.pub_threshold.publish(threshold_msg)
   		
   		return config

if __name__ == "__main__":
	rospy.init_node("gazebo_sub_jointstate",anonymous=True)
	gazebo_sub_jointstate = gazebo_sub_jointstate()
	rospy.spin()
