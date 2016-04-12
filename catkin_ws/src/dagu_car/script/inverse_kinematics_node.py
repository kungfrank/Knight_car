#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from numpy import *


# Inverse Kinematics Node
# Author: Robert Katzschmann

class InverseKinematicsNode(object):
    def __init__(self):
        self.node_name = 'inverse_kinematics_node'

        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.car_cmd_callback)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Read in parameters
        self.k_l = self.setup_parameter('k_l', 25.0)
        self.k_r = self.setup_parameter('k_r', 25.0)
        self.radius_l = self.setup_parameter('radius_l', 0.02)
        self.radius_r = self.setup_parameter('radius_r', 0.02)
        self.baseline = self.setup_parameter('baseline', 0.1)

        rospy.loginfo("[%s] has started", self.node_name)

    def car_cmd_callback(self, msg_car_cmd):
        # conversion from linear and angular velocities to motor rotation rate
        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.baseline) / self.radius_r
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.baseline) / self.radius_l

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r / self.k_r
        u_l = omega_l / self.k_l

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header = msg_car_cmd.header
        msg_wheels_cmd.vel_right = u_r
        msg_wheels_cmd.vel_left = u_l
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
