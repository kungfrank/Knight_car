#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import CarControl

class CarSupervisor(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.joy_control = None
        self.lane_control = None
        self.last_pub_msg = None
        
        rospy.loginfo("[%s] Initializing " %(self.node_name))

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

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for easier debugging with rosparam
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value        

    def cbJoyControl(self,joy_control_msg):
        self.joy_control = joy_control_msg

    def cbLaneControl(self,lane_control_msg):
        self.lane_control = lane_control_msg

    def cbPubTimer(self,event):
        if self.joystick_mode:
            # Always publishes joy_control when joystic_mode is true
            if self.joy_control is None:
                return

            what = self.joy_control
        else:
            # When not in joy stick mode, publish lane_control when need_settering is true.
            if self.lane_control is not None:
                if self.lane_control.need_steering:
                    what = self.lane_control
                else:
                    if self.joy_control is None:
                        return
                
                    what = self.joy_control
            else:
                if self.joy_control is None:
                    return
                
                what = self.joy_control

        values = (what.speed, what.steering, what.need_steering)

        if self.last_pub_msg is not None:
            if (self.last_pub_msg.speed, self.last_pub_msg.steering, self.last_pub_msg.need_steering) == values:
                delta = event.current_real - self.last_pub_msg.header.stamp 
                if delta.to_sec() < 2.0:
                    return

        self.pub_car_control.publish(what)
        self.last_pub_msg = what

if __name__ == "__main__":
    rospy.init_node("car_supervisor",anonymous=False)
    car_supervisor = CarSupervisor()
    rospy.spin()
