#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import BoolStamped, FSMState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import time
import math

class Timer(object):
    def __init__(self):
        self.node_name = "timer"
        self.active = True
	self.i = 0

        
	## publishers and subscribers
        #self.sub_mode      = rospy.Subscriber("/kaku/fsm_node/mode",FSMState, self.processStateChange)
        self.sub_mode      = rospy.Subscriber("~fsm_node/mode",FSMState, self.processStateChange)
	#self.sub_in_lane      = rospy.Subscriber("/kaku/lane_filter_node/in_lane", BoolStamped, self.processTimer)
	self.sub_in_lane      = rospy.Subscriber("~lane_filter_node/in_lane", BoolStamped, self.processTimer)

	#self.sub_switch = rospy.Subscriber("/kaku/timer/switch", BoolStamped, self.cbSwitch, queue_size=1)
	self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.pub_time_is_up = rospy.Publisher("~time_is_up", BoolStamped, queue_size=1, latch=True)

    def processStateChange(self, msg):

	self.state=msg.state


    def processTimer(self, msgg):
        if self.active == True:
    	    print "##################1. fsm state is " + `self.state`

	    if self.i ==0:
	        if self.state == "LANE_FOLLOWING_TURN_RIGHT" or "LANE_FOLLOWING_TURN_LEFT":
	            self.timer_start = time.time()
		    self.i = 1
	            print "####################Timer start######################"
    	    self.timer_end = time.time()

	    if (self.timer_end - self.timer_start) > 2: 
	        print "####################Timer End##########################"
		msg = BoolStamped()
		msg.data = True
		self.pub_time_is_up.publish(msg)

	if not self.active:
	    self.timer_start = time.time()
	    
	    msg = BoolStamped()
            msg.data = False
            self.pub_time_is_up.publish(msg)

	    self.i = 0
	    print "#####################Timer not active######################"

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data
 
    def onShutdown(self):
        rospy.loginfo("[Timer] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('Timer',anonymous=False)
    lane_filter_node = Timer()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()

