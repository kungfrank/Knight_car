#!/usr/bin/env python
import sys
import argparse
import numpy as np
import math
import time
from __builtin__ import True

import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, Pose2DStamped

import RobotRaconteur as RR

duckie_servicedef="""
#Service to provide simple interface to the Duckiebot
service Duckiebot_Interface

option version 0.4

object Duckiebot
property double v
property double omega
property double x
property double y
property double theta

function void sendCmd(double v, double omega)
function void sendStop()

end object
"""

class DuckiebotHost(object):
	def __init__(self):
		rospy.init_node("duckie_rr_bridge",anonymous=False)
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing "%(self.node_name))

		self.last_pub_msg = None
		self.last_pub_time = rospy.Time.now()

		# Setup Properties
		self._v = 0
		self._omega = 0
		self._x = 0
		self._y = 0
		self._theta = 0

		# Publications
		self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions 
       	self.sub_velocity = rospy.Subscriber("~velocity", Twist2DStamped, self.cb_velocity)
       	self.sub_pose = rospy.Subscriber("~pose",Pose2DStamped, self.cb_pose)
        
    @property
    def v(self):
    	return self._v
    
    @property
    def omega(self):
    	return self._omega
    
    @property
    def x(self):
    	return self._x
    
    @property
    def y(self):
    	return self._y
    
    @property
    def theta(self):
    	return self._theta
    
    def sendCmd(self, v, omega):
    	car_cmd_msg = Twist2DStamped()
    	car_cmd_msg.header.stamp = rospy.Time.now()
    	car_cmd_msg.v = v
    	car_cmd_msg.omega = omega

    	self.pub_car_cmd.publish(car_cmd_msg)

    def sendStop(self):
    	car_cmd_msg = Twist2DStamped()
    	car_cmd_msg.header.stamp = rospy.Time.now()
    	car_cmd_msg.v = 0
    	car_cmd_msg.omega = 0

    	self.pub_car_cmd.publish(car_cmd_msg)

    # ---- Private Functions -----
 	def cb_velocity(self,vel_msg):
 		self._v = vel_msg.v
 		self._omega = vel_msg.omega

 	def cb_pose(self,pose_msg):
 		self._x = pose_msg.x
 		self._y = pose_msg.y
 		self._theta = pose_msg.theta

 	def close(self)
 		self.sendStop()

def cb_shutdown():
	duck.close()
	# This must be here to prevent segfault
	RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
	print "DUCKIEBOT RR BRIDGE"
	print "==================="
	# Parse command line arguments
	parser = argparse.ArgumentParser(
		description='Initialize the Duckiebot')
	parser.add_argument('--port',type=int,default=0,
		help='TCP port to host service on' +\
		'(will auto-generate if not specified)')
	parser.add_argument('veh', 
		help='The name of the duckiebot being launched')

	args = parser.parse_args(sys.argv[1:])

	veh = args.veh

	# Enable numpy
	RR.RobotRaconteurNode.s.UseNumPy = True

	# Set the node name
	RR.RobotRaconteurNode.s.NodeName = "DuckiebotServer.%s"%veh

	# Initialize the object
	duck = DuckiebotHost()

	# Create transport, register it, and start the server
	print "Registering Transport..."
	t = RR.TCPTransport()
	t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
		RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
		RR.IPNodeDiscoveryFlags_SITE_LOCAL)

	RR.RobotRaconteurNode.s.RegisterTransport(t)

	port = args.port
	t.StartServer(port)
	if (port == 0)
		port = t.GetListenPort()

	# Register the service type and the service
	print "Starting Service..."
	RR.RobotRaconteurNode.s.RegisterServiceType(duckie_servicedef)
	RR.RobotRaconteurNode.s.RegisterService("Duckiebot","Duckiebot_Interface.Duckiebot")

	print "Service Started, connect via \n" +\
		"tcp://localhost:%s/DuckiebotServer.%s/Duckiebot"%(port,veh)

	rospy.on_shutdown(cb_shutdown)
	rospy.spin()