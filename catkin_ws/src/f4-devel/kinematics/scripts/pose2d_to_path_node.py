#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Pose2DStamped, KinematicsWeights
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr
import numpy as np

# Pose2d to Path Node
# Author: Teddy Ort
# Inputs: 
# Outputs: 

class Pose2dToPathNode(object):
    def __init__(self):
        self.node_name = 'pose2d_to_path_node'

        # Store the frames used to link the robot and vicon paths
        self.Tv_w = tr.identity_matrix()
        self.Tw_r1 = tr.identity_matrix()
        self.adjVicon = True
        self.adjRobot = True

        # setup publishers and subscribers
        self.sub_vicon = rospy.Subscriber("~vicon_pose", PoseStamped, self.viconPoseCallback)
        self.sub_pose = rospy.Subscriber("~pose", Pose2DStamped, self.poseCallback)
        self.sub_weights = rospy.Subscriber("~weights", KinematicsWeights, self.weightsCallback)
        self.pub_path = rospy.Publisher("~path", Path, queue_size=1)

        # Setup the path message
        self.msg_path = Path()

        rospy.loginfo("[%s] has started!", self.node_name)

    def viconPoseCallback(self, msg_vicon):
        if self.adjVicon:
            self.Tv_w = np.dot(self.poseToTransform(msg_vicon.pose), tr.rotation_matrix(-np.pi/2,(0,0,1)))
            #print "Found vicon initial frame: {vicon}".format(vicon=self.Tv_w)
            self.adjVicon = False

    def poseCallback(self, msg_pose):
        self.msg_path.header = msg_pose.header
        p = PoseStamped()
        p.pose.position.x = msg_pose.x
        p.pose.position.y = msg_pose.y
        q = tr.quaternion_from_euler(0,0,msg_pose.theta)
        for i,attr in enumerate(['x', 'y', 'z', 'w']):
            p.pose.orientation.__setattr__(attr, q[i])

        if self.adjRobot:
            self.Tw_r1 = np.linalg.inv(self.poseToTransform(p.pose))
            self.msg_path.poses = []
            #print "Found robot initial frame: {robot}".format(robot=self.Tw_r1)
            self.adjVicon = True
            while self.adjVicon and not rospy.is_shutdown():
                rospy.sleep(0.01)
            self.adjRobot = False

        p.pose = self.adjustRobotPose(p.pose)
        self.msg_path.poses.append(p)
        self.pub_path.publish(self.msg_path)

    def weightsCallback(self, msg_weights):
        self.adjRobot = True

    def adjustRobotPose(self, pose):
        Tr2_w = self.poseToTransform(pose)
        Tadj = np.dot(np.dot(self.Tv_w, self.Tw_r1),Tr2_w)
        pose.position.x, pose.position.y, pose.position.z = Tadj[0:3,3]
        q = tr.quaternion_from_matrix(Tadj)
        for i,attr in enumerate(['x', 'y', 'z', 'w']):
            pose.orientation.__setattr__(attr, q[i])
        return pose

    def poseToTransform(self, pose):
        # Convert the pose to a 4x4 homogeneous transform
        pos = (pose.position.x, pose.position.y, pose.position.z)
        rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        trans = tr.concatenate_matrices(tr.translation_matrix(pos), tr.quaternion_matrix(rot))
        return trans

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('pose2d_to_path_node', anonymous=False)
    pose2d_to_path_node = Pose2dToPathNode()
    rospy.spin()