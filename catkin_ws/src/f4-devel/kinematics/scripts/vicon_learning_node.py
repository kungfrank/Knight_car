#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import ThetaDotSample, Vsample, WheelsCmdStamped
from geometry_msgs.msg import  PoseStamped
from tf.transformations import *
import numpy

# Vicon Learning Node
# Author: Teddy Ort
# Inputs: pose, wheels_cmd
# Outputs: theta_dot_sample, v_sample

class ViconLearningNode(object):
    def __init__(self):
        self.node_name = 'vicon_learning_node'

        # Setup the publishers and subscribers
        self.sub_pose = rospy.Subscriber("~pose", PoseStamped, self.poseCallback)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheelsCmdCallback)
        self.pub_theta_dot = rospy.Publisher("~theta_dot_sample", ThetaDotSample, queue_size=1)
        self.pub_v = rospy.Publisher("~v_sample", Vsample, queue_size=1)

        # Initialize the pose to 0, and wheels_cmd to None
        self.pose_prev = PoseStamped()
        self.pose_cur = PoseStamped()
        self.wheels_cmd_prev = None

        rospy.loginfo("[%s] has started", self.node_name)

    def poseCallback(self, msg_pose):
        self.pose_cur = msg_pose

    def wheelsCmdCallback(self, msg_wheels_cmd):
        if self.wheels_cmd_prev is not None:

            # Calculate the change in pose
            trans_cur = self.poseToTransform(self.pose_cur.pose)
            trans_prev = self.poseToTransform(self.pose_prev.pose)
            trans_diff = numpy.dot(inverse_matrix(trans_prev), trans_cur)
            dx, dy, dz = translation_from_matrix(trans_diff)
            dw = euler_from_matrix(trans_diff)[2] #Only the z axis
            dt = (msg_wheels_cmd.header.stamp - self.wheels_cmd_prev.header.stamp).to_sec()

            # Stuff the measurements into messages and publish
            msg_theta_dot = ThetaDotSample()
            msg_theta_dot.d_L = self.wheels_cmd_prev.vel_left
            msg_theta_dot.d_R = self.wheels_cmd_prev.vel_right
            msg_theta_dot.dt = dt
            msg_theta_dot.theta_angle_pose_delta = dw
            self.pub_theta_dot.publish(msg_theta_dot)


            msg_v = Vsample()
            msg_v.d_L = self.wheels_cmd_prev.vel_left
            msg_v.d_R = self.wheels_cmd_prev.vel_right
            msg_v.dt = dt
            msg_v.x_axis_pose_delta = -dy   # The vicon frame is setup with -y facing forward on the duckiecar
            msg_v.y_axis_pose_delta = dx
            msg_v.theta_angle_pose_delta = dw
            self.pub_v.publish(msg_v)

        # Update
        self.wheels_cmd_prev = msg_wheels_cmd
        self.pose_prev = self.pose_cur

    def poseToTransform(self, pose):
        # Convert the pose to a 4x4 homogeneous transform
        pos = (pose.position.x, pose.position.y, pose.position.z)
        rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        trans = concatenate_matrices(translation_matrix(pos), quaternion_matrix(rot))
        return trans

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('vicon_learning_node', anonymous=False)
    vicon_learning_node = ViconLearningNode()
    rospy.spin()