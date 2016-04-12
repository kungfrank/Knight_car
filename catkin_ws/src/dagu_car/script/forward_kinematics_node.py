#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, Pose2DStamped
from numpy import *


# Forward Kinematics Node
# Authors: Robert Katzschmann
# Inputs: wheels cmd
# Outputs: velocity

class ForwardKinematicsNode(object):
    def __init__(self):
        self.node_name = 'forward_kinematics_node'

        # Read in parameters
        self.k_l = self.setup_parameter('k_l', 25.0)
        self.k_r = self.setup_parameter('k_r', 25.0)
        self.radius_l = self.setup_parameter('radius_l', 0.02)
        self.radius_r = self.setup_parameter('radius_r', 0.02)
        self.baseline = self.setup_parameter('baseline', 0.1)

        # Setup the publisher and subscribers
        self.pub_velocity = rospy.Publisher("~velocity", Twist2DStamped, queue_size=1)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheels_cmd_callback)

        rospy.loginfo("[%s] has started", self.node_name)

    def wheels_cmd_callback(self, msg_wheels_cmd):
        # Conversion from motor duty to motor rotation rate
        omega_r = self.k_r * msg_wheels_cmd.vel_right
        omega_l = self.k_l * msg_wheels_cmd.vel_left

        # Compute linear and angular velocity of the platform
        v = (self.radius_r * omega_r + self.radius_l * omega_l) / 2.0
        omega = (self.radius_r * omega_r - self.radius_l * omega_l) / self.baseline

        # Stuff the v and omega into a message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        msg_velocity.v = v
        msg_velocity.omega = omega
        self.pub_velocity.publish(msg_velocity)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node', anonymous=False)
    forward_kinematics_node = ForwardKinematicsNode()
    rospy.spin()
