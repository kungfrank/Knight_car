#!/usr/bin/env python
import rospy
import unittest
import rostest
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo
import math

class ApriltagsPostprocessingTesterNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('apriltags_postprocessing_tester_node', anonymous=False)
        self.msg_received = False
        tags_msg = AprilTagsWithInfos()

        # Setup the publisher and subscriber
        self.pub = rospy.Publisher("~apriltags_in", AprilTagDetectionArray, queue_size = 1, latch = True)
        self.sub = rospy.Subscriber("~apriltags_out", AprilTagsWithInfos, self.tags_callback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while (self.pub.get_num_connections() < 1 or self.sub.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tags_callback(self, tags_msg):
        self.tags_msg = tags_msg
        self.msg_received = True

    def test_publishers_and_subscribers(self):
        self.setup()  # Setup the node
        self.assertGreaterEqual(self.pub.get_num_connections(), 1, "No connections found on apriltags_in topic")
        self.assertGreaterEqual(self.sub.get_num_connections(), 1, "No connections found on apriltags_out topic")


    def test_transform_to_duckiebot_frame(self):
        self.setup()  # Setup the node

        # Publish a point that should lie on the floor in front of duckiebot
        scale_z = rospy.get_param("apriltags_postprocessing_node/scale_z")
        camera_theta = rospy.get_param("apriltags_postprocessing_node/camera_theta")
        camera_z = rospy.get_param("apriltags_postprocessing_node/camera_z")
        z = camera_z/(math.sin(camera_theta*math.pi/180.0)*scale_z) # Find the distance to floor in camera frame

        tags_msg = AprilTagDetectionArray()
        tag = AprilTagDetection()
        tag.pose.pose.position.z = z
        tag.pose.pose.orientation.w = 1
        tags_msg.detections.append(tag)
        tag.id = 1
        self.pub.publish(tags_msg)

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")

        # Check that the result has the same id
        tag_in = self.tags_msg.detections[0]
        self.assertEqual(tag_in.id, 1)

        # Check that the result is on the floor (z=0)
        self.assertAlmostEqual(tag_in.pose.pose.position.z, 0)

    def test_adding_duckietown_tag_info(self):
        self.setup()  # Setup the node

        # Publish a tag with id=1
        tags_msg = AprilTagDetectionArray()
        tag = AprilTagDetection()
        tag.pose.pose.orientation.w = 1
        tags_msg.detections.append(tag)
        tag.id = 1
        self.pub.publish(tags_msg)

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")

        # Check that the result has the same id
        tag_in = self.tags_msg.detections[0]
        self.assertEqual(tag_in.id, 1)

        # Check that the result is a stop sign
        tag_info = self.tags_msg.infos[0]
        self.assertEqual(tag_info.tag_type, TagInfo.SIGN)
        self.assertEqual(tag_info.traffic_sign_type, TagInfo.STOP)

if __name__ == '__main__':
    rostest.rosrun('apriltags_ros', 'apriltags_postprocessing_tester_node', ApriltagsPostprocessingTesterNode)
