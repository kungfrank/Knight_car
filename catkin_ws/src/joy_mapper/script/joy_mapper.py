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
        self.pub_timestep = self.setupParam("~pub_timestep", 0.02)
        self.speed_gain = self.setupParam("~speed_gain", 1.0)
        self.steer_gain = self.setupParam("~steer_gain", 1.0)

        # Publications
        self.pub_control = rospy.Publisher("~joy_control", CarControl, queue_size=1)
        self.pub_wheels = rospy.Publisher("~wheels_cmd", WheelsCmd, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)

        self.has_complained = False


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg

    def publishControl(self, event):
        # Don't publish if havn't received any Joy msgs
        if self.joy is None:
            if not self.has_complained:
                rospy.loginfo("[%s] Has not received Joy msg yet. Not publishing." %(self.node_name))
                self.has_complained = True
            return

        speed = self.joy.axes[1] * self.speed_gain #Left stick V-axis. Up is positive
        steering = self.joy.axes[3] * self.steer_gain

        # # do not publish if values did not change
        # if self.last_pub_msg is not None:
        #     same = (self.last_pub_msg.speed == speed) and (self.last_pub_msg.steering == steering)
        #     # if it's same, publish only every 2 seconds
        #     if same:
        #         delta = event.current_real - self.last_pub_time 
        #         if delta.to_sec() < 2.0:
        #             return

        # car_control_msg = CarControl()
        # car_control_msg.need_steering = False
        # car_control_msg.speed = speed 
        # car_control_msg.steering = steering #*self.steer_gain #Right stick H-axis. Right is negative
        # car_control_msg.header.stamp = self.joy.header.stamp
        # rospy.loginfo("[%s] controls: speed %f, steering %f" % (self.node_name,car_control_msg.speed, car_control_msg.steering))
        # self.pub_control.publish(car_control_msg)


        wheels_cmd_msg = WheelsCmd()

        # Tank Steering Mode
        # wheels_cmd_msg.vel_left = self.joy.axes[1]
        # wheels_cmd_msg.vel_right = self.joy.axes[4]

        # Car Steering Mode
        ratio = 1.0

        gain = 1.0
        vel_left = gain*(speed - steering)*ratio
        vel_right = gain*(speed + steering)*(1.0/ratio)
        

        wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)
        rospy.loginfo("[%s] left %f, right %f" % (self.node_name,self.joy.axes[1],self.joy.axes[4]))
        self.pub_wheels.publish(wheels_cmd_msg)

        # self.last_pub_time = event.current_real
        # self.last_pub_msg = car_control_msg

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
