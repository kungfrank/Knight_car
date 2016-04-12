#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import AprilTags

class ApriltagsIntegrationTester(unittest.TestCase):
    def __init__(self, *args):
        super(ApriltagsIntegrationTester, self).__init__(*args)
        self.msg_tags = AprilTags()
        self.msg_received = False

    def setup(self):
        # Setup the node
        rospy.init_node('apriltags_integration_tester', anonymous=False)
        self.msg_received = False

        # Setup the publisher and subscriber
        self.sub_tag = rospy.Subscriber("~apriltags", AprilTags, self.tagCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.sub_tag.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tagCallback(self, msg_tag):
        self.msg_tags = msg_tag
        self.msg_received = True

    def test_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.sub_tag.get_num_connections(), 1, "No connections found on apriltags topic")

    def test_apriltag_detection(self):
        self.setup()

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(10) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")

        # Check for the apriltag with id 60
        found = False
        for tag in self.msg_tags.detections:
            found = True if tag.id == 60 else found
        self.assertTrue(found, "Expected apriltag with id=60 not found.")

if __name__ == '__main__':
    rospy.init_node('apriltags_integration_tester', anonymous=False)
    rostest.rosrun('apriltags', 'apriltags_integration_tester', ApriltagsIntegrationTester)
