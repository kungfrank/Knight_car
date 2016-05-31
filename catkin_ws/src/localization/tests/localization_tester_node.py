#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import AprilTags, TagDetection, Pose2DStamped
import tf2_ros
import tf.transformations as tr
import numpy as np
import numpy.testing


class LocalizationTesterNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('localization_tester_node', anonymous=True)

        # Setup the publisher and subscriber
        self.pub_tag = rospy.Publisher("~apriltags", AprilTags, queue_size = 1, latch = True)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while self.pub_tag.get_num_connections() < 1 and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Test timed out while waiting for node to come up.")

    def test_publishers_and_subscribers(self):
        self.setup()
        self.assertGreaterEqual(self.pub_tag.get_num_connections(),1)

    def test_single_tag_localization(self):
        self.setup()
        # Publish two matching apriltag observations
        msg_tag = AprilTags()
        tag = TagDetection()
        tag.id = 100
        trans = tag.transform.translation
        rot = tag.transform.rotation
        (trans.x, trans.y, trans.z) = (1,-1,0)
        (rot.x, rot.y, rot.z, rot.w) = (0,0,1,0)
        msg_tag.detections.append(tag)
        self.pub_tag.publish(msg_tag)

        # Wait for the node to publish a robot transform
        tfbuf = tf2_ros.Buffer()
        tfl = tf2_ros.TransformListener(tfbuf)
        try:
            Tr_w = tfbuf.lookup_transform("world", "duckiebot", rospy.Time(), rospy.Duration(5))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.assertFalse(True, "Test timed out waiting for the transform to be broadcast.")

        trans = (Tr_w.transform.translation.x, Tr_w.transform.translation.y, Tr_w.transform.translation.z)
        rot = (Tr_w.transform.rotation.x, Tr_w.transform.rotation.y, Tr_w.transform.rotation.z, Tr_w.transform.rotation.w)

        numpy.testing.assert_array_almost_equal(trans, (2, 0, 0))
        numpy.testing.assert_array_almost_equal(rot, (0,0,1,0))

    def test_avg_tag_localization(self):
        self.setup()

        # Publish a matching apriltag observation (from the static publisher in the test file)
        msg_tag = AprilTags()
        tag_100 = TagDetection()
        tag_100.id = 100
        trans = tag_100.transform.translation
        rot = tag_100.transform.rotation
        (trans.x, trans.y, trans.z) = (1,-1,0)
        (rot.x, rot.y, rot.z, rot.w) = (0,0,1,0)
        msg_tag.detections.append(tag_100)
        tag_101 = TagDetection()
        tag_101.id = 101
        trans = tag_101.transform.translation
        rot = tag_101.transform.rotation
        (trans.x, trans.y, trans.z) = (1,0,0)
        (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_euler(0,0,np.pi/2)
        msg_tag.detections.append(tag_101)
        self.pub_tag.publish(msg_tag)

        # Wait for the node to publish a robot transform
        tfbuf = tf2_ros.Buffer()
        tfl = tf2_ros.TransformListener(tfbuf)
        try:
            Tr_w = tfbuf.lookup_transform("world", "duckiebot", rospy.Time(), rospy.Duration(5))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.assertFalse(True, "Test timed out waiting for the transform to be broadcast.")

        trans = (Tr_w.transform.translation.x, Tr_w.transform.translation.y, Tr_w.transform.translation.z)
        rot = (Tr_w.transform.rotation.x, Tr_w.transform.rotation.y, Tr_w.transform.rotation.z, Tr_w.transform.rotation.w)

        numpy.testing.assert_array_almost_equal(trans, (2, 0.5, 0))
        numpy.testing.assert_array_almost_equal(rot, tr.quaternion_from_euler(0,0,3*np.pi/4))


if __name__ == '__main__':
    rostest.rosrun('localization', 'localization_tester_node', LocalizationTesterNode)
