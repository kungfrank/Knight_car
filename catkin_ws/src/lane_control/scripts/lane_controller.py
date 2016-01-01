#!/usr/bin/env python
import rospy
import numpy as np
import math
#from duckietown_msgs.msg import CarLanePose
from duckietown_msgs.msg import CarControl
from duckietown_msgs.msg import CarLanePose

class lane_controller(object):
    def __init__(self):
        self.carPose = CarLanePose()
	self.pub_counter = 0
	self.is_shutdown = False

	# control gains
	self.v_bar = 0.0
	self.k_d = 0.0
	self.k_theta = 0.0
	self.d_thres = 0.0
	self.theta_thres = 0.0
	# self.setGains()
	self.getGains()

        # Publications
        self.pub_control = rospy.Publisher("~lane_control",CarControl,queue_size=1)

        # Subscriptions
        self.sub_car_vicon_ = rospy.Subscriber("~car_vicon", CarLanePose, self.cbPose, queue_size=1)
        
	# safe shutdown
	rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(2.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setGains(self):
        self.v_bar = 0.5 # nominal speed, 0.5m/s
        self.k_theta = -2.0
        self.k_d = - (self.k_theta ** 2) / ( 4.0 * self.v_bar)
        self.theta_thres = math.pi / 6
        self.d_thres = math.fabs(self.k_theta / self.k_d) * self.theta_thres

	rospy.set_param("~v_bar",self.v_bar)
        rospy.set_param("~k_d", self.k_d)
        rospy.set_param("~k_theta", self.k_theta)
        rospy.set_param("~theta_thres", self.theta_thres)
        rospy.set_param("~d_thres", self.d_thres)
        print "set gains, v_bar %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f" \
               % (self.v_bar, self.k_d, self.k_theta, self.theta_thres, self.d_thres)

    def getGains_event(self, event):
	self.getGains()

    def getGains(self):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        theta_thres = rospy.get_param("~theta_thres")
        d_thres = rospy.get_param("~d_thres")

        if (v_bar!=self.v_bar or k_d!=self.k_d or k_theta!=self.k_theta \
                or theta_thres!=self.theta_thres or d_thres!=self.d_thres):
            print "gains changed!"
            print "old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f" \
               % (self.v_bar, self.k_d, self.k_theta, self.theta_thres, self.d_thres)
            print "new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f" \
               % (v_bar, k_d, k_theta, theta_thres, d_thres)
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.theta_thres = theta_thres
            self.d_thres = d_thres

    def setupParam(self,para_name,default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)
    
    def custom_shutdown(self):
	print "~~~shutting down~~~"
	self.is_shutdown = True
	car_control_msg = CarControl()
	car_control_msg.need_steering = True
	car_control_msg.speed = 0.0
	car_control_msg.steering = 0.0
	self.pub_control.publish(car_control_msg)

    def cbPose(self,msg):
	if self.is_shutdown == True:
	    return
        self.carPose = msg 
        car_control_msg = CarControl()
        car_control_msg.need_steering = True
        car_control_msg.speed = self.v_bar #*self.speed_gain #Left stick V-axis. Up is positive
        if math.fabs(msg.cross_track_err) > self.d_thres:
            msg.cross_track_err = msg.cross_track_err / math.fabs(msg.cross_track_err) * self.d_thres
        car_control_msg.steering =  self.k_d * msg.cross_track_err + \
                                    self.k_theta * msg.heading_err #*self.steer_gain #Right stick H-axis. Right is negative
        # controller mapping issue
	# car_control_msg.steering = -car_control_msg.steering
	# print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        self.pub_control.publish(car_control_msg)

        # debuging
        self.pub_counter += 1
        if self.pub_counter % 50 == 0:
            self.pub_counter = 1
            print "lane_controller publish"
            print car_control_msg





if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
