#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import AprilTags, TagDetection
import tf2_ros
from tf2_msgs.msg import TFMessage
import tf.transformations as tr
from geometry_msgs.msg import Transform, TransformStamped
import numpy as np
from localization import PoseAverage

# Localization Node
# Author: Teddy Ort
# Inputs: apriltags/duckietown_msgs/AprilTags - A list of april tags in a camera frame
# Outputs: pose2d/duckietown_msgs/Pose2dStamped - The estimated pose of the robot in the world frame in 2D coordinates
#          pose3d/geometry_msgs/PoseStamped - The estimated pose of the robot in the world frame in 3D coordinates

class LocalizationNode(object):
    def __init__(self):
        self.node_name = 'localization_node'

        # Constants
        self.world_frame = "world"
        self.duckiebot_frame = "duckiebot"

        # Setup the publishers and subscribers
        self.sub_april = rospy.Subscriber("~apriltags", AprilTags, self.tag_callback)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1, latch=True)

        # Setup the transform listener
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        rospy.loginfo("[%s] has started", self.node_name)

    def tag_callback(self, msg_tag):
        # Listen for the transform of the tag in the world
        avg = PoseAverage.PoseAverage()
        for tag in msg_tag.detections:
            try:
                Tt_w = self.tfbuf.lookup_transform("world", "tag_{id}".format(id=tag.id), rospy.Time(), rospy.Duration(1))
                Mtbase_w=self.transform_to_matrix(Tt_w.transform)
                Mt_tbase = tr.concatenate_matrices(tr.translation_matrix((0,0,0.17)), tr.euler_matrix(0,0,np.pi))
                Mt_w = tr.concatenate_matrices(Mtbase_w,Mt_tbase)
                Mt_r=self.transform_to_matrix(tag.transform)
                if abs(tr.euler_from_matrix(Mt_r)[2]) < 0.001:
                    Mr_t=np.linalg.inv(Mt_r)
                    Mr_w=np.dot(Mt_w,Mr_t)
                    Tr_w = self.matrix_to_transform(Mr_w)
                    avg.add_pose(Tr_w)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logwarn("Error looking up transform for tag_%s", tag.id)
                rospy.logwarn(ex.message)

        Tr_w =  avg.get_average() # Average of the opinions
        # Broadcast the robot transform
        if Tr_w is not None:
            T = TransformStamped()
            T.transform = Tr_w
            T.header.frame_id = self.world_frame
            T.header.stamp = rospy.Time.now()
            T.child_frame_id = self.duckiebot_frame
            self.pub_tf.publish(TFMessage([T]))

    def transform_to_matrix(self, T):
        # Return the 4x4 homogeneous matrix for a TransformStamped.msg T from the geometry_msgs
        trans = (T.translation.x, T.translation.y, T.translation.z)
        rot = (T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w)
        return np.dot(tr.translation_matrix(trans), tr.quaternion_matrix(rot))

    def matrix_to_transform(self, M):
        # Return a TransformStamped.msg T from the geometry_msgs from a 4x4 homogeneous matrix
        T=Transform()
        (T.translation.x, T.translation.y, T.translation.z) = tr.translation_from_matrix(M)
        (T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w) = tr.quaternion_from_matrix(M)
        return T

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('localization_node', anonymous=False)
    localization_node = LocalizationNode()
    rospy.spin()