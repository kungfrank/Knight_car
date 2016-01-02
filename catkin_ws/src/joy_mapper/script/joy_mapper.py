#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import CarControl
from sensor_msgs.msg import Joy
import time


class JoyMapper(object):
    def __init__(self):
        self.joy = Joy()
        self.joy.axes = [0.0,0.0,1.0,0.0] #Joy initialized with empty .axes.

        # Setup Parameters
        self.pub_timestep = self.setupParam("~pub_timestep", 0.02)
        self.speed_gain = self.setupParam("~speed_gain", 1.0)
        self.steer_gain = self.setupParam("~steer_gain", 1.0)

        print('pub_timestep = %s' % self.pub_timestep)
        print('speed_gain   = %s' % self.speed_gain)
        print('steer_gain   = %s' % self.steer_gain)

        # Publications
        self.pub_control = rospy.Publisher("~joy_control", CarControl, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

        self.last_msg = None
        self.last_msg_time = None 

    def setupParam(self, para_name, default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)

    def cbJoy(self, joy_msg):
        self.joy = joy_msg

    def cbLaneControl(self, lane_control_msg):
        self.lane_control = lane_control_msg

    def publishControl(self, event):
        speed = self.joy.axes[1] * self.speed_gain #Left stick V-axis. Up is positive
        steering = self.joy.axes[3] * self.steer_gain

        # do not publish if values did not change
        if self.last_msg is not None:
            same = (self.last_msg.speed == speed) and (self.last_msg.steering == steering)
            # if it's same, publish only every 2 seconds
            if same:
                delta = time.time() - self.last_msg_time 
                if delta < 2.0:
                    return

        car_control_msg = CarControl()
        car_control_msg.need_steering = False
        car_control_msg.speed = speed 
        car_control_msg.steering = steering #*self.steer_gain #Right stick H-axis. Right is negative
        print("controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering))
        self.pub_control.publish(car_control_msg)
        self.last_msg = car_control_msg
        self.last_msg_time = time.time()


if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
