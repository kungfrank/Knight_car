#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import ThetaDotSample, Vsample, WheelsCmdStamped
from geometry_msgs.msg import  PoseStamped
from tf.transformations import *
#import numpy
from numpy import *

# Vicon Learning Node
# Author: Teddy Ort, Jason Pazis
# Inputs: pose, wheels_cmd
# Outputs: theta_dot_sample, v_sample

class ViconLearningNode(object):
    def __init__(self):
        self.node_name = 'vicon_learning_node'

        # Setup the publishers and subscribers
        self.sub_pose = rospy.Subscriber("~pose", PoseStamped, self.poseCallback)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheelsCmdCallback)
        self.pub_theta_dot = rospy.Publisher("~theta_dot_sample", ThetaDotSample, queue_size=10)
        self.pub_v = rospy.Publisher("~v_sample", Vsample, queue_size=10)

        # Initialize the pose to 0, and wheels_cmd to None
        # self.pose_prev = PoseStamped()
        # self.pose_cur = PoseStamped()
        # self.wheels_cmd_prev = None

        self.pose_list = []
        self.wheels_cmd_list = []

        self.timeOffset = 0.0

        rospy.loginfo("[%s] has started", self.node_name)

    def poseCallback(self, msg_pose):
        self.pose_list.append(msg_pose)
        n = 0
        # make sure we have at least two wheels commands and two poses
        while len(self.wheels_cmd_list) > 1 and len(self.pose_list) > 1+n:
            # discard any pose messages received before the first wheel command plus timeOffset
            while len(self.pose_list) > 1+n and (self.pose_list[0].header.stamp.to_sec() + self.timeOffset < self.wheels_cmd_list[0].header.stamp.to_sec()):
                del self.pose_list[0]
            # need at least two poses to calculate delta
            if len(self.pose_list) > 1+n:
                # make sure that the second pose is not beyond the duration of the current wheel command
                if self.pose_list[1+n].header.stamp.to_sec() < (self.wheels_cmd_list[1].header.stamp.to_sec() + self.timeOffset):
                    # Calculate the change in pose
                    #print 'pose', self.pose_list[1]
                    dx = self.pose_list[0].pose.position.x - self.pose_list[1+n].pose.position.x
                    dy = self.pose_list[0].pose.position.y - self.pose_list[1+n].pose.position.y
                    q = [self.pose_list[1+n].pose.orientation.x, self.pose_list[1+n].pose.orientation.y, self.pose_list[1+n].pose.orientation.z, self.pose_list[1+n].pose.orientation.w]
                    q = q/linalg.norm(q)
                    cur_phi = arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]))
                    cur_theta = arcsin(2*(q[0]*q[2]-q[3]*q[1]))
                    cur_psi = arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2.0*(q[2]*q[2]+q[3]*q[3]))
                    q = [self.pose_list[0].pose.orientation.x, self.pose_list[0].pose.orientation.y, self.pose_list[0].pose.orientation.z, self.pose_list[0].pose.orientation.w]
                    q = q/linalg.norm(q)
                    delta_phi = cur_phi - arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]))
                    while delta_phi > pi:
                        delta_phi = delta_phi - 2.0*pi
                    while delta_phi <= -pi:
                        delta_phi = delta_phi + 2.0*pi
                    # delta_theta = cur_theta - arcsin(2*(q[0]*q[2]-q[3]*q[1]))
                    # delta_psi = cur_psi - arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2.0*(q[2]*q[2]+q[3]*q[3]))
                    # print 'delta_phi', delta_phi, 'delta_theta', delta_theta, 'delta_psi', delta_psi
                    #dw = euler_from_matrix(trans_diff)[2] #Only the z axis
                    dw = -delta_phi
                    dt = self.pose_list[1+n].header.stamp.to_sec() - self.pose_list[0].header.stamp.to_sec()
                    #print 'dt', dt

                    # Stuff the measurements into messages and publish
                    msg_theta_dot = ThetaDotSample()
                    msg_theta_dot.d_L = self.wheels_cmd_list[0].vel_left
                    msg_theta_dot.d_R = self.wheels_cmd_list[0].vel_right
                    msg_theta_dot.dt = dt
                    msg_theta_dot.theta_angle_pose_delta = dw
                    self.pub_theta_dot.publish(msg_theta_dot)


                    msg_v = Vsample()
                    msg_v.d_L = self.wheels_cmd_list[0].vel_left
                    msg_v.d_R = self.wheels_cmd_list[0].vel_right
                    msg_v.dt = dt
                    msg_v.x_axis_pose_delta = -dy   # The vicon frame is setup with -y facing forward on the duckiecar
                    msg_v.y_axis_pose_delta = dx
                    msg_v.theta_angle_pose_delta = dw
                    self.pub_v.publish(msg_v)
                else:
                    del self.wheels_cmd_list[0]

                del self.pose_list[0]
        # print 'pose', msg_pose
        # self.pose_cur = msg_pose

    def wheelsCmdCallback(self, msg_wheels_cmd):
        self.wheels_cmd_list.append(msg_wheels_cmd)
        # if self.wheels_cmd_prev is not None:

        #     # Calculate the change in pose
        #     #trans_cur = self.poseToTransform(self.pose_cur.pose)
        #     #trans_prev = self.poseToTransform(self.pose_prev.pose)
        #     #trans_diff = numpy.dot(inverse_matrix(trans_prev), trans_cur)
        #     #dx, dy, dz = translation_from_matrix(trans_diff)
        #     dx = self.pose_cur.pose.position.x - self.pose_prev.pose.position.x
        #     dy = self.pose_cur.pose.position.y - self.pose_prev.pose.position.y
        #     q = [self.pose_cur.pose.orientation.x, self.pose_cur.pose.orientation.y, self.pose_cur.pose.orientation.z, self.pose_cur.pose.orientation.w]
        #     q = q/linalg.norm(q)
        #     cur_phi = arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]))
        #     cur_theta = arcsin(2*(q[0]*q[2]-q[3]*q[1]))
        #     cur_psi = arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2.0*(q[2]*q[2]+q[3]*q[3]))
        #     q = [self.pose_prev.pose.orientation.x, self.pose_prev.pose.orientation.y, self.pose_prev.pose.orientation.z, self.pose_prev.pose.orientation.w]
        #     q = q/linalg.norm(q)
        #     delta_phi = cur_phi - arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]))
        #     while delta_phi > pi:
        #         delta_phi = delta_phi - 2.0*pi
        #     while delta_phi <= -pi:
        #         delta_phi = delta_phi + 2.0*pi
        #     # delta_theta = cur_theta - arcsin(2*(q[0]*q[2]-q[3]*q[1]))
        #     # delta_psi = cur_psi - arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2.0*(q[2]*q[2]+q[3]*q[3]))
        #     # print 'delta_phi', delta_phi, 'delta_theta', delta_theta, 'delta_psi', delta_psi
        #     #dw = euler_from_matrix(trans_diff)[2] #Only the z axis
        #     dw = delta_phi
        #     dt = (msg_wheels_cmd.header.stamp - self.wheels_cmd_prev.header.stamp).to_sec()

        #     # Stuff the measurements into messages and publish
        #     msg_theta_dot = ThetaDotSample()
        #     msg_theta_dot.d_L = self.wheels_cmd_prev.vel_left
        #     msg_theta_dot.d_R = self.wheels_cmd_prev.vel_right
        #     msg_theta_dot.dt = dt
        #     msg_theta_dot.theta_angle_pose_delta = dw
        #     self.pub_theta_dot.publish(msg_theta_dot)


        #     msg_v = Vsample()
        #     msg_v.d_L = self.wheels_cmd_prev.vel_left
        #     msg_v.d_R = self.wheels_cmd_prev.vel_right
        #     msg_v.dt = dt
        #     msg_v.x_axis_pose_delta = -dy   # The vicon frame is setup with -y facing forward on the duckiecar
        #     msg_v.y_axis_pose_delta = dx
        #     msg_v.theta_angle_pose_delta = dw
        #     self.pub_v.publish(msg_v)

        # # Update
        # self.wheels_cmd_prev = msg_wheels_cmd
        # self.pose_prev = self.pose_cur

    # def poseToTransform(self, pose):
    #     # Convert the pose to a 4x4 homogeneous transform
    #     pos = (pose.position.x, pose.position.y, pose.position.z)
    #     rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    #     trans = concatenate_matrices(translation_matrix(pos), quaternion_matrix(rot))
    #     # print 'pos', pos
    #     # print 'rot', rot
    #     # print 'transform', trans
    #     return trans

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('vicon_learning_node', anonymous=False)
    vicon_learning_node = ViconLearningNode()
    rospy.spin()