#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import CarControl

class CarSupervisor(object):
    def __init__(self):

        self.joy_control = CarControl()
        self.lane_control = CarControl()
        self.lane_control.need_steering = False
        
        # Setup Parameters
        self.pub_timestep = self.setupParam("~pub_timestep",0.02)

        # Publications
        self.pub_car_control = rospy.Publisher("~car_control",CarControl,queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("~joy_control", CarControl, self.cbJoyControl,queue_size=1)
        self.sub_lane_control = rospy.Subscriber("~lane_control",CarControl,self.cbLaneControl,queue_size=1)

        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbPubTimer)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParam(self,para_name,default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)

    def cbJoyControl(self,joy_control_msg):
        self.joy_control_ = joy_control_msg

    def cbLaneControl(self,lane_control_msg):
        self.lane_control = lane_control_msg

    def cbPubTimer(self,event):
        self.publishControl()

    def publishControl(self):
        # TODO: Implement more intelligent mixing logic
        if self.lane_control.need_steering:
            self.pub_car_control.publish(self.lane_control)
        else:
            self.pub_car_control.publish(self.joy_control)

if __name__ == "__main__":
    rospy.init_node("car_supervisor",anonymous=False)
    car_supervisor = CarSupervisor()
    rospy.spin()
