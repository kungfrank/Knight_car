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

        # Publications
        self.pub_control = rospy.Publisher("lane_control",CarControl,queue_size=1)

        # Subscriptions
        self.sub_car_vicon_ = rospy.Subscriber("~car_vicon", CarLanePose, self.cbPose, queue_size=1)
        
        # control gains
        self.setGains()

        # timer
        self.pub_counter = 0;
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(2.0), self.getGains)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setGains():
        self.v_bar = self.setupParam("~v_bar",0.2) # nominal speed, 0.2m/s
        self.k_theta = -2.0
        self.k_d = - (k_theta ** 2) / ( 4.0 * self.v_bar)
        self.theta_thres = math.pi / 6
        self.d_thres = math.fabs(k_theta / k_d) * theta_thres

        rospy.set_param("~k_d", self.k_d)
        rospy.set_param("~k_theta", self.k_theta)
        rospy.set_param("~theta_thres", self.theta_thres)
        rospy.set_param("~d_thres", self.d_thres)
        print "set gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f" \
               % (self.v_bar, self.k_d, self.k_theta, self.theta_thres, self.d_thres)


    def getGains():
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

    def cbPose(self,msg):
        self.carPose = msg 
        car_control_msg = CarControl()
        car_control_msg.need_steering = False
        car_control_msg.speed = self.v_bar #*self.speed_gain #Left stick V-axis. Up is positive
        if fabs(msg.cross_track_err) > self.d_thres:
            msg.cross_track_err = msg.cross_track_err / fabs(msg.cross_track_err) * self.d_thres
        car_control_msg.steering =  self.k_d * msg.cross_track_err + \
                                    self.k_theta * msg.heading_err #*self.steer_gain #Right stick H-axis. Right is negative
        print "controls: speed %f, steeting %f" % (car_control_msg.speed, car_control_msg.steering)
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
