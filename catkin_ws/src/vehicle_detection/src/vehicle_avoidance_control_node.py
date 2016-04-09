#!/usr/bin/env python
from duckietown_msgs.msg import WheelsCmdStamped, VehiclePose
from std_msgs.msg import Bool

import os
import rospkg
import rospy
import yaml

class VehicleAvoidanceControlNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		self.config	= self.setupParam("~config", "baseline")
		self.cali_file_name = self.setupParam("~cali_file_name", "default")
		rospack = rospkg.RosPack()
		self.cali_file = rospack.get_path('duckietown') + \
				"/config/" + self.config + \
				"/vehicle_detection/vehicle_avoidance_control_node/" +  \
				self.cali_file_name + ".yaml"
		if not os.path.isfile(self.cali_file):
			rospy.logwarn("[%s] Can't find calibration file: %s.\n" 
					% (self.node_name, self.cali_file))
		self.loadConfig(self.cali_file)
		self.wheels_cmd_pub = rospy.Publisher("~wheels_cmd",
				WheelsCmdStamped, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~flag",
				Bool, queue_size=1)
		self.subscriber = rospy.Subscriber("~vehicle_pose",
				VehiclePose, self.callback,  queue_size = 1)

	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def loadConfig(self, filename):
		stream = file(filename, 'r')
		data = yaml.load(stream)
		rospy.loginfo('[%s] data: %s' % (self.node_name, data,))
		stream.close()
		self.distance_threshold = data['distance_threshold']
		rospy.loginfo('[%s] distance_threshold: %.4f', self.node_name, 
				self.distance_threshold)

	def callback(self, data):
		if not data.detection.data:
			vehicle_too_close = False
		else:
			distance = data.rho.data
			min_distance = self.distance_threshold
			vehicle_too_close = False
			if distance < min_distance:
				vehicle_too_close = True
		self.publishCmd()
		self.vehicle_detected_pub.publish(vehicle_too_close)

	def publishCmd(self): 
		wheels_cmd_msg = WheelsCmdStamped()
		wheels_cmd_msg.vel_left = 0.0
		wheels_cmd_msg.vel_right = 0.0
		self.wheels_cmd_pub.publish(wheels_cmd_msg)
   
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()

