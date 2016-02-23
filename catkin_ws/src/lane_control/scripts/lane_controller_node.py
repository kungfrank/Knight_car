#!/usr/bin/env python
import rospy
import numpy as np
import math
#from duckietown_msgs.msg import CarLanePose
from duckietown_msgs.msg import CarControl, WheelsCmdStamped
# from duckietown_msgs.msg import CarLanePose
from duckietown_msgs.msg import LanePose

class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Setup parameters
        self.setGains()

        # Publicaiton
        # self.pub_lane_cmd = rospy.Publisher("~lane_control",CarControl,queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_control",WheelsCmdStamped,queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        self.k_d = self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",d_thres) # Maximum desire theta

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres

    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        
        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = CarControl()
        car_control_msg.need_steering = True
        car_control_msg.speed = 0.0
        car_control_msg.steering = 0.0
        # self.pub_lane_cmd.publish(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self,stamp,speed,steering):
        # lane_cmd_msg = CarControl()
        # lane_cmd_msg.header.stamp = stamp
        # lane_cmd_msg.speed = speed
        # lane_cmd_msg.steering = steering
        # lane_cmd_msg.need_steering = True

        wheels_cmd_msg = WheelsCmdStamped()
        wheels_cmd_msg.header.stamp = stamp
        speed_gain = 1.0
        steer_gain = 0.5
        vel_left = (speed_gain*speed - steer_gain*steering)
        vel_right = (speed_gain*speed + steer_gain*steering)
        wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        # self.pub_lane_cmd.publish(lane_cmd_msg)
        self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self,lane_pose_msg):
        self.lane_reading = lane_pose_msg 

        cross_track_err = lane_pose_msg.d
        heading_err = lane_pose_msg.phi

        car_control_msg = CarControl()
        car_control_msg.need_steering = True
        car_control_msg.speed = self.v_bar #*self.speed_gain #Left stick V-axis. Up is positive
        
        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        car_control_msg.steering =  self.k_d * cross_track_err + self.k_theta * heading_err #*self.steer_gain #Right stick H-axis. Right is negative
        
        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        self.publishCmd(lane_pose_msg.header.stamp,self.v_bar,car_control_msg.steering)

        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
