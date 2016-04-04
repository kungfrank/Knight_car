#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Pose2DStamped, KinematicsWeights
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr

# Pose2d to Path Node
# Author: Teddy Ort
# Inputs: 
# Outputs: 

class Pose2dToPathNode(object):
    def __init__(self):
        self.node_name = 'pose2d_to_path_node'

        # setup publishers and subscribers
        self.sub_vicon = rospy.Subscriber("~vicon_pose", PoseStamped, self.viconPoseCallback)
        self.sub_pose = rospy.Subscriber("~pose", Pose2DStamped, self.poseCallback)
        self.sub_weights = rospy.Subscriber("~weights", KinematicsWeights, self.weightsCallback)
        self.pub_path = rospy.Publisher("~path", Path, queue_size=1)

        # Setup the path message
        self.msg_path = Path()

        rospy.loginfo("[%s] has started", self.node_name)

    def viconPoseCallback(self, msg_vicon):
        pass

    def poseCallback(self, msg_pose):
        self.msg_path.header = msg_pose.header
        p = PoseStamped()
        p.pose.position.x = msg_pose.x
        p.pose.position.y = msg_pose.y
        q = tr.quaternion_from_euler(0,0,msg_pose.theta)
        for i,attr in enumerate(['x', 'y', 'z', 'w']):
            p.pose.orientation.__setattr__(attr, q[i])
        self.msg_path.poses.append(p)
        self.pub_path.publish(self.msg_path)

    def weightsCallback(self, msg_weights):
        pass

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('pose2d_to_path_node', anonymous=False)
    pose2d_to_path_node = Pose2dToPathNode()
    rospy.spin()