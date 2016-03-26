#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from kinematics import Inverse_kinematics
from numpy import *

# Inverse Kinematics Node
# Authors: Jason Pazis
# Inputs: 
# Outputs: 

class InverseKinematicsNode(object):
    def __init__(self):
        self.node_name = 'inverse_kinematics_node'

        # Read parameters
        #self.veh_name = self.setupParameter("~veh_name","megaman")

        # Setup the inverse kinematics model
        self.ik = Inverse_kinematics.Inverse_kinematics('Duty_fi_theta_dot_naive', 'Duty_fi_v_naive', matrix([-1.0]), matrix([1.0]))

        #Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.carCmdCallback)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

        rospy.loginfo("[%s] has started", self.node_name)

    def carCmdCallback(self, msg_car_cmd):
        [d_L, d_R] = self.ik.evaluate(msg_car_cmd.omega, msg_car_cmd.v)

        # Stuff the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header = msg_car_cmd.header

        # cmd_msg.header.stamp = ros::Time::now();  # Record the time the command was given to the wheels_driver
        msg_wheels_cmd.vel_left = d_L
        msg_wheels_cmd.vel_right = d_R
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
