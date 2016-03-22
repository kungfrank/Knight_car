#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import CarControl, WheelsCmdStamped, LanePose, StopLineReading
from sensor_msgs.msg import Joy

class lane_supervisor(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = LanePose()
        self.lane_control = WheelsCmdStamped()
        self.joy_control  = WheelsCmdStamped()
        self.safe = True
        self.parallel_autonomy_mode = False
        self.in_lane = True
        self.at_stop_line = False
        self.stop = False

        # Params:
        self.max_cross_track_error=self.setupParameter("~max_cross_track_error",0.1)
        self.max_heading_error=self.setupParameter("~max_heading_error",math.pi/4)
        self.max_speed=self.setupParameter("~max_speed",1.0)
        self.max_steer=self.setupParameter("~max_steer",0.2)

        # Publicaiton
        self.pub_wheels_cmd = rospy.Publisher("~wheels_control",WheelsCmdStamped,queue_size=1)
        self.pub_safe       = rospy.Publisher("~safe",Bool,queue_size=1)

        # Subscriptions
        self.sub_lane_pose    = rospy.Subscriber("~lane_pose", LanePose, self.cbLanePose, queue_size=1)
        self.sub_lane_control = rospy.Subscriber("~wheels_control_lane",WheelsCmdStamped,self.cbLaneControl, queue_size=1)
        self.sub_joy_control  = rospy.Subscriber("~wheels_control_joy",WheelsCmdStamped,self.cbJoyControl, queue_size=1)
        self.sub_joy          = rospy.Subscriber("~joy",Joy,self.cbJoy,queue_size=1)
        self.sub_at_stop_line = rospy.Subscriber("~stop_line_reading",StopLineReading, self.cbStopLine, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updateParams(self,event):
        self.max_cross_track_error = rospy.get_param("~max_cross_track_error")
        self.max_heading_error     = rospy.get_param("~max_heading_error")
        self.max_speed             = rospy.get_param("~max_speed")
        self.max_steer             = rospy.get_param("~max_steer")

    def cbStopLine(self,stop_line_msg):
        if not stop_line_msg.at_stop_line:
            self.at_stop_line=False
            self.stop = False
        else:
            if not self.at_stop_line:
                self.at_stop_line=True
                self.stop = True
                rospy.sleep(2)
                self.stop=False

    def cbJoy(self,joy_msg):
        print joy_msg.buttons
        if (joy_msg.buttons[5] == 1):
            print "Toggle parallel autonomy mode"
            self.parallel_autonomy_mode = not self.parallel_autonomy_mode

    def cbLanePose(self,lane_pose_msg):
        self.in_lane=lane_pose_msg.in_lane
        self.lane_reading = lane_pose_msg 
        cross_track_err = math.fabs(lane_pose_msg.d) 
        heading_err = math.fabs(lane_pose_msg.phi)
        if cross_track_err > self.max_cross_track_error or heading_err > self.max_heading_error:
            self.safe = False
        else:
            self.safe = True
        self.pub_safe.publish(self.safe)

    def cbLaneControl(self,lane_control_msg):
        self.lane_control = lane_control_msg

    def cbJoyControl(self,joy_control_msg):
        self.joy_control = joy_control_msg
        wheels_cmd_msg = self.mergeJoyAndLaneControl()
        self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def mergeJoyAndLaneControl(self):
        wheels_cmd_msg = WheelsCmdStamped()
        if not self.parallel_autonomy_mode:
            wheels_cmd_msg = self.joy_control
        elif self.stop:
            wheels_cmd_msg.vel_right=0
            wheels_cmd_msg.vel_left=0
        elif self.safe or not self.in_lane:
            car_control_joy = self.wheelsCmdToCarControl(self.joy_control)
            car_control_joy.speed = min(car_control_joy.speed,self.max_speed)
            car_control_joy.steering = np.clip(car_control_joy.steering, -self.max_steer, self.max_steer)
            wheels_cmd_msg = self.carControlToWheelsCmd(car_control_joy)
        else:
            car_control_joy = self.wheelsCmdToCarControl(self.joy_control)
            car_control_lane = self.wheelsCmdToCarControl(self.lane_control)
            car_control_merged = CarControl()
            car_control_merged.speed = min(car_control_joy.speed,self.max_speed) # take the speed from the joystick 
            car_control_merged.steering = car_control_lane.steering # take the heading from the lane controller
            wheels_cmd_msg = self.carControlToWheelsCmd(car_control_merged)
        car_control = self.wheelsCmdToCarControl(wheels_cmd_msg)
        if car_control.speed >= 0:
            new_theta = -car_control.speed - 0.3
            if type(new_theta) is np.float64:
                new_theta = new_theta.item()
            rospy.set_param("lane_controller_node/k_theta",new_theta)
        return wheels_cmd_msg

    def wheelsCmdToCarControl(self,wheels_cmd):
        car_control = CarControl()
        car_control.speed = 0.5*(wheels_cmd.vel_right + wheels_cmd.vel_left)
        car_control.steering = 0.5*(wheels_cmd.vel_right - wheels_cmd.vel_left)
        return car_control

    def carControlToWheelsCmd(self,car_control):
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.vel_right = car_control.speed + car_control.steering
        wheels_cmd.vel_left  = car_control.speed - car_control.steering
        return wheels_cmd

if __name__ == "__main__":
    rospy.init_node("lane_supervisor",anonymous=False)
    lane_supervisor_node = lane_supervisor()
    rospy.spin()
