#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import CarControl
from sensor_msgs.msg import Joy

class JoyMapper(object):
    def __init__(self):
        self.joy = Joy()
        self.joy.axes = [0.0,0.0,1.0,0.0] #Joy initialized with empty .axes.

        # Setup Parameters
        self.pub_timestep = self.setupParam("~pub_timestep",0.02)
        self.speed_gain = self.setupParam("~speed_gain",1.0)
        self.steer_gain = self.setupParam("~steer_gain",np.pi*(15.0/180.0))

        # Publications
        self.pub_control = rospy.Publisher("~joy_control",CarControl,queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy,queue_size=1)
        
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParam(self,para_name,default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)

    def cbJoy(self,joy_msg):
        self.joy = joy_msg

    def cbLaneControl(self,lane_control_msg):
        self.lane_control = lane_control_msg

    def publishControl(self,event):
        car_control_msg = CarControl()
        car_control_msg.need_steering = False
        car_control_msg.speed = self.joy.axes[1]*self.speed_gain #Left stick V-axis. Up is positive
        car_control_msg.steering = self.joy.axes[3]*self.steer_gain #Right stick H-axis. Right is negative
        self.pub_control.publish(car_control_msg)


if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
