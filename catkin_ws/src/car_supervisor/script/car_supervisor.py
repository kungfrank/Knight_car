#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import CarControl
import time

class CarSupervisor(object):
    def __init__(self):

        self.joy_control = CarControl()
        self.lane_control = CarControl()
        self.lane_control.need_steering = False
        
        # Setup Parameters
        self.pub_timestep = self.setupParam("~pub_timestep",0.02)
        self.joystick_mode = self.setupParam("~joystick_mode",True)
        
        # Publications
        self.pub_car_control = rospy.Publisher("~car_control",CarControl,queue_size=1)

        # Subscriptions
        self.sub_joy = rospy.Subscriber("~joy_control",CarControl,self.cbJoyControl,queue_size=1)
        self.sub_lane_control = rospy.Subscriber("~lane_control",CarControl,self.cbLaneControl,queue_size=1)

        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbPubTimer)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
        self.last = None
        self.last_time = None

    def setupParam(self,para_name,default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)

    def cbJoyControl(self,joy_control_msg):
        self.joy_control = joy_control_msg

    def cbLaneControl(self,lane_control_msg):
        self.lane_control = lane_control_msg

    def cbPubTimer(self,event):
        self.publishControl()

    def publishControl(self):
        if self.joystick_mode:
            # Always publishes joy_control when joystic_mode is true
            what = self.joy_control
        else:
            # When not in joy stick mode, publish lane_control when need_settering is true.
            if self.lane_control.need_steering:
                what = self.lane_control
            else:
                what = self.joy_control

        values = (what.speed, what.steering, what.need_steering)

        if self.last == values:
            delta = time.time() - self.last_time
            if delta < 2.0:
                return

        self.pub_car_control.publish(what)
        self.last = what
        self.last_time = time.time()


if __name__ == "__main__":
    rospy.init_node("car_supervisor",anonymous=False)
    car_supervisor = CarSupervisor()
    rospy.spin()
