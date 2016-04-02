#!/usr/bin/env python
import rospy
import unittest
import rostest
import time
from duckietown_msgs.msg import AprilTags, TagDetection, WheelsCmdStamped
from geometry_msgs.msg import Transform
import tf.transformations as tr
import numpy as np
from numpy.testing import *

class VisualOdometryAprilTagsTesterNode(unittest.TestCase):
    def test_april_tags_subscribers(self):
        #Setup the publisher and subscribers
        self.pub_april_tags = rospy.Publisher("visual_odometry_april_tags_node/april_tags", AprilTags, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("visual_odometry_april_tags_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Wait for the forward_kinematics_node to finish starting up
        timeout = time.time()+5.0
        while (self.pub_wheels_cmd.get_num_connections() < 1 or self.pub_april_tags.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)
        self.assertEqual(self.pub_wheels_cmd.get_num_connections(), 1)
        self.assertEqual(self.pub_april_tags.get_num_connections(), 1)

    def test_april_tags_single_interval(self):
        #Setup the publisher and subscribers
        self.pub_april_tags = rospy.Publisher("visual_odometry_april_tags_node/april_tags", AprilTags, queue_size=1, latch=True)
        self.pub_wheels_cmd = rospy.Publisher("visual_odometry_april_tags_node/wheels_cmd", WheelsCmdStamped, queue_size=1, latch=True)

        # Wait for the forward_kinematics_node to finish starting up
        timeout = time.time()+5.0
        while (self.pub_wheels_cmd.get_num_connections() < 1 or self.pub_april_tags.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

        # Publish a single wheels cmd, and two simple april tag messages
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.vel_left = 1
        msg_wheels_cmd.vel_right = 1
        self.pub_wheels_cmd.publish(msg_wheels_cmd)
        rospy.sleep(0.2)    #Wait so the tags come in the right order
        T1 = Transform()
        T2 = Transform()
        T1.translation.y = 2
        T2.translation.y = 2
        T2.rotation.x, T2.rotation.y, T2.rotation.z, T2.rotation.w = tr.quaternion_about_axis(-np.pi/2, (0,0,1))

        msg_tag1 = AprilTags()
        tag = TagDetection()
        tag.transform = T1
        msg_tag1.detections.append(tag)
        msg_tag1.header.stamp = rospy.Duration(0)
        self.pub_april_tags.publish(msg_tag1)
        rospy.sleep(0.2)    #Wait so the tags come in the right order
        msg_tag2 = AprilTags()
        msg_tag1.header.stamp = rospy.Duration(1)
        tag.transform = T2
        msg_tag1.detections.append(tag)
        self.pub_april_tags.publish(msg_tag1)

        # Wait 1 second for the file to be output
        rospy.sleep(3)
        res = np.genfromtxt(rospy.get_param("visual_odometry_april_tags_node/filename", ''))
        assert_almost_equal(res, np.array([1,1,1,np.pi/2, 2, 2]))

if __name__ == '__main__':
    rospy.init_node('test_forward_kinematics_node', anonymous=False)
    rostest.rosrun('visual_odometry', 'visual_odometry_april_tags_tester_node', VisualOdometryAprilTagsTesterNode)
