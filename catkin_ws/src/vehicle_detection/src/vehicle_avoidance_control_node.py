#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, VehiclePose, BoolStamped

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
		self.car_cmd_pub = rospy.Publisher("~car_cmd",
				Twist2DStamped, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
				BoolStamped, queue_size=1)
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
                vehicle_too_close = BoolStamped()
                vehicle_too_close.header.stamp = data.header.stamp
		if not data.detection.data:
			vehicle_too_close.data = False
		else:
			distance = data.rho.data
			min_distance = self.distance_threshold
			vehicle_too_close.data = False
			if distance < min_distance:
				vehicle_too_close.data = True
		self.publishCmd(data.header.stamp)
		self.vehicle_detected_pub.publish(vehicle_too_close)

	def publishCmd(self,stamp): 
		cmd_msg = Twisted2DStamped()
                cmd_msg.header.stamp = stamp
		cmd_msg.v = 0.0
		cmd_msg.omega = 0.0
		self.car_cmd_pub.publish(car_cmd_msg)
   
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()

