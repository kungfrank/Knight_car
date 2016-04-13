#!/usr/bin/env python
import rospy
import time
from led_detection.LEDDetector import LEDDetector
from std_msgs.msg import Byte
from duckietown_msgs.msg import Vector2D, AprilTags, LEDDetection, LEDDetectionArray, LEDDetectionDebugInfo, LEDInterpreter
from sensor_msgs.msg import CompressedImage
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np

#this is a stup for traffic light testing

class LEDInterpreterNode(object):
	def __init__(self):
		self.trafficLightIntersection = False
		self.node_name = rospy.get_name()
		self.pub_interpret = rospy.Publisher("~signals_detection", signalings/SignalsDetection, queue_size = 1)
		self.sub_tags = rospy.Subscriber("apriltags_postprocessing_fast_node/apriltags", AprilTags, self.CheckTags)
		self.sub_LEDs = rospy.Subscriber("~raw_led_detection", LEDDetectionArray, self.Interpreter, queue_size = 1)


		self.protocol = self.setParam("~LED_Protocol") #should be a list of tuples
		self.label = self.setParam("~location_config") # should be a list
		#self._traffic = True
		# self._light = None
		self._freq = None

		self.lightGo = self.protocol['traffic_light_go']['frequency_idx']
		self.lightStop = self.protocol['traffic_light_stop']['frequency_idx']

		rospy.loginfo("Initialized.")


	def Interpreter(self, msg):
		if self._traffic is not False:
			for item in msg:
				#only recognize if it is traffic light
				if item.pixels_normalized.x > self.label['left'] and item.pixels_normalized.x < self.label['right'] and item.pixels_normalized.y > self.label['top']:

					self._freq = item.frequency
					if self._freq = self.lightGo:
						msg = "go"

					elif self._freq = self.lightStop:
						msg = "stop"



	def CheckTags(self, msg):
	#task of this is to check on what type of intersection we are
		self.trafficLightIntersection = False
		for info in msg.infos:
			if info.traffic_sign_type == info.TRAFFIC_LIGHT_AHEAD: #TODO put correct constant
				self.trafficLightIntersection = True
			



	def setupParam(self,param_name,default_value):
			value = rospy.get_param(param_name,default_value)
			rospy.set_param(param_name,value) #Write to parameter server for transparancy
			# rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
			return value


	def onShutdown(self):
		    rospy.loginfo("[LED Interpreter] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_qlai_tester',anonymous=False)
    interpreternode = LEDInterpreterNode()
    rospy.on_shutdown(interpreternode.onShutdown)
    rospy.spin()

