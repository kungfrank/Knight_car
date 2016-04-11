#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import ThetaDotSample, Vsample, WheelsCmdStamped, Pose2DStamped
from tf.transformations import *
import numpy


# Virtual Learning Node
# Author: Robert Katzschmann, Jason Pazis
# Inputs: pose, wheels_cmd
# Outputs: theta_dot_sample, v_sample

class VirtualLearningNode(object):
    def __init__(self):
        self.node_name = 'virtual_learning_node'

        # Setup the publishers and subscribers
        self.sub_pose = rospy.Subscriber("~pose", Pose2DStamped, self.pose2DCallback)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheelsCmdCallback)
        self.pub_theta_dot = rospy.Publisher("~theta_dot_sample", ThetaDotSample, queue_size=1)
        self.pub_v = rospy.Publisher("~v_sample", Vsample, queue_size=1)

        self.pose_prev = None
        self.wheels_cmd = None
        self.msg_theta_dot = ThetaDotSample()
        self.msg_v = Vsample()

        rospy.loginfo("[%s] has started", self.node_name)

    def pose2DCallback(self, msg_pose):
        if (self.pose_prev is not None) and (self.wheels_cmd is not None):
            # Calculate the change in pose
            dx = msg_pose.x - self.pose_prev.x
            dy = msg_pose.y - self.pose_prev.y
            dtheta = msg_pose.theta - self.pose_prev.theta
            dt = (msg_pose.header.stamp - self.pose_prev.header.stamp).to_sec()

            # Stuff the measurements into messages and publish
            self.msg_theta_dot.d_L = self.wheels_cmd.vel_left
            self.msg_theta_dot.d_R = self.wheels_cmd.vel_right
            self.msg_theta_dot.dt = dt
            self.msg_theta_dot.theta_angle_pose_delta = dtheta
            self.pub_theta_dot.publish(self.msg_theta_dot)

            self.msg_v.d_L = self.wheels_cmd.vel_left
            self.msg_v.d_R = self.wheels_cmd.vel_right
            self.msg_v.dt = dt
            self.msg_v.x_axis_pose_delta = dx
            self.msg_v.y_axis_pose_delta = dy
            self.msg_v.theta_angle_pose_delta = dtheta
            self.pub_v.publish(self.msg_v)

        self.pose_prev = msg_pose

    def wheelsCmdCallback(self, msg_wheels_cmd):
        if (self.wheels_cmd is not None):
            if (self.wheels_cmd.vel_left != msg_wheels_cmd.vel_left) or (self.wheels_cmd.vel_right != msg_wheels_cmd.vel_right):
                self.pose_prev = None
        self.wheels_cmd = msg_wheels_cmd
        

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparency
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('virtual_learning_node', anonymous=False)
    virtual_learning_node = VirtualLearningNode()
    rospy.spin()
