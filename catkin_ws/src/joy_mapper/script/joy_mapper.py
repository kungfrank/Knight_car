#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import CarControl, WheelsCmd
from sensor_msgs.msg import Joy
import time

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Setup Parameters
        self.speed_gain = self.setupParam("~speed_gain", 1.0)
        self.steer_gain = self.setupParam("~steer_gain", 1.0)
        self.left_right_ratio = self.setupParam("~left_right_ratio", 1.0)

        # Publications
        self.pub_wheels = rospy.Publisher("~wheels_cmd", WheelsCmd, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.has_complained = False

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()

    def publishControl(self):
        speed = self.joy.axes[1] * self.speed_gain #Left stick V-axis. Up is positive
        steering = self.joy.axes[3] * self.steer_gain
        wheels_cmd_msg = WheelsCmd()

        # Car Steering Mode
        ratio = self.left_right_ratio
        vel_left = (speed - steering)*ratio
        vel_right = (speed + steering)*(1.0/ratio)
        wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)
        rospy.loginfo("[%s] left %f, right %f" % (self.node_name,wheels_cmd_msg.vel_left,wheels_cmd_msg.vel_right))
        self.pub_wheels.publish(wheels_cmd_msg)

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
