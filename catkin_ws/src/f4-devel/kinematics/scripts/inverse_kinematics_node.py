#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from kinematics import Inverse_kinematics
from numpy import *

# Inverse Kinematics Node
# Author: Jason Pazis

class InverseKinematicsNode(object):
    def __init__(self):
        self.node_name = 'inverse_kinematics_node'

        # Read parameters
        #self.veh_name = self.setupParameter("~veh_name","megaman")
        fi_theta_dot_function = self.setupParameter('~fi_theta_dot_function', 'Duty_fi_theta_dot_naive')
        fi_v_function = self.setupParameter('~fi_v_function', 'Duty_fi_v_naive')
        theta_dot_weights = self.setupParameter('~theta_dot_weights', [-1.0])
        v_weights = self.setupParameter('~v_weights', [1.0])

        # Setup the inverse kinematics model
        self.ik = Inverse_kinematics.Inverse_kinematics(fi_theta_dot_function, fi_v_function, matrix(theta_dot_weights), matrix(v_weights))

        #Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.carCmdCallback)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

        rospy.loginfo("[%s] has started", self.node_name)

    def carCmdCallback(self, msg_car_cmd):
        [d_L, d_R] = self.ik.evaluate(msg_car_cmd.omega, msg_car_cmd.v)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header = msg_car_cmd.header

        # cmd_msg.header.stamp = ros::Time::now();  # Record the time the command was given to the wheels_driver
        msg_wheels_cmd.vel_left = d_L
        msg_wheels_cmd.vel_right = d_R
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
