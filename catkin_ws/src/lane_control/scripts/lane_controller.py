#!/usr/bin/env python
import rospy
import numpy as np
#from duckietown_msgs.msg import CarLanePose
from duckietown_msgs.msg import CarControl
from duckietown_msgs.msg import CarLanePose

class lane_controller(object):
    def __init__(self):
        self.carPose = CarLanePose()

        # Publications
        self.pub_control = rospy.Publisher("lane_control",CarControl,queue_size=1)

        # Subscriptions
        self.sub_car_vicon_ = rospy.Subscriber("~car_vicon", CarLanePose, self.cbPose, queue_size=1)
        
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

        # control gains
        self.cross_track_gain = 1;
        self.heading_err_gain = 0;
	self.pub_counter = 0;

    def setupParam(self,para_name,default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)

    def cbPose(self,msg):
        self.carPose = msg 
        car_control_msg = CarControl()
        car_control_msg.need_steering = False
        car_control_msg.speed = 0.1 #*self.speed_gain #Left stick V-axis. Up is positive
        car_control_msg.steering =  -(self.cross_track_gain * msg.cross_track_err + \
                                    self.heading_err_gain * msg.heading_err) #*self.steer_gain #Right stick H-axis. Right is negative
        self.pub_control.publish(car_control_msg)
	self.pub_counter += 1
	if self.pub_counter % 50 == 0:
		self.pub_counter = 1
		print "lane_controller publish"
		print car_control_msg



if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
