#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import BoolStamped

class LogicGateNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.gate_type = rospy.get_param("~gate_type")
		# validate gate_type
		if not self.validateGateType():
			rospy.signal_shutdown("Invalid gate_type.")
			return

		self.inputs_dict = rospy.get_param("~inputs")
		if not self.validateInputs():
			rospy.signal_shutdown("Invalid input definition.")
			return

		self.sub_list = list()
		self.input_msg_dict = dict()
		self.last_published_msg = None
		self.pub = rospy.Publisher(rospy.get_param("~output_topic"),BoolStamped,queue_size=1)
		for input_name, input_dict in self.inputs_dict.items():
			topic_name = input_dict["topic"]
			# Initialze local copy as None
			self.input_msg_dict[input_name] = None
			self.sub_list.append(rospy.Subscriber(topic_name, BoolStamped, self.cbBoolStamped, callback_args=input_name))

	def validateInputs(self):
		valid_flag = True
		for input_name, input_dict in self.inputs_dict.items():
			if "topic" not in input_dict:
				rospy.logfatal("[%s] topic not defined for input %s" %(self.node_name,input_name))
				valid_flag = False
		return valid_flag

	def validateGateType(self):
		valid_gate_types = ["AND","OR"]
		if self.gate_type not in valid_gate_types:
			rospy.logfatal("[%s] gate_type %s is not valid." %(self.node_name,self.gate_type))
			return False
		return True

	def publish(self,msg):
		if msg is None:
			return
		
		if self.last_published_msg is not None:
			if msg.data == self.last_published_msg.data:
				# Only publish when data changes
				return

		self.pub.publish(msg)
		self.last_published_msg = msg

	def getOutputMsg(self):
		bool_list = list()
		latest_time_stamp = rospy.Time(0)

		for input_name, input_msg in self.input_msg_dict.items():
			if input_msg is None:
				return None
			bool_list.append(input_msg.data)
			# Keeps track of latest timestamp
			if input_msg.header.stamp >= latest_time_stamp:
				latest_time_stamp = input_msg.header.stamp
		
		# Perform logic operation
		msg = BoolStamped()
		msg.header.stamp = latest_time_stamp

		if self.gate_type == "AND":
			msg.data = all(bool_list)
		elif self.gate_type == "OR":
			msg.data = any(bool_list)
		return msg

	def cbBoolStamped(self, msg, input_name):
		self.input_msg_dict[input_name] = msg
		self.publish(self.getOutputMsg())

	def on_shutdown(self):
	    rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('logic_gate_node', anonymous=False)
    # Create the NodeName object
    node = LogicGateNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()