#!/usr/bin/env python
import rospy
import unittest
import rostest
from sensor_msgs.msg import Image
from duckietown_msgs.msg import BoolStamped

class ApriltagsPreprocessingTesterNode(unittest.TestCase):
    def __init__(self, *args):
        super(ApriltagsPreprocessingTesterNode, self).__init__(*args)
        self.msg_raw = Image()
        self.msg_received = False

    def setup(self):
        # Setup the node
        rospy.init_node('apriltags_preprocessing_tester_node', anonymous=False)

        # Setup the publisher and subscriber
        self.pub_switch_global  = rospy.Publisher("~global_switch", BoolStamped, queue_size=1, latch=True)
        self.sub_raw_global = rospy.Subscriber( "~global_image_raw", Image, self.rawCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.pub_switch_global.get_num_connections() < 1 or self.sub_raw_global.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def rawCallback(self, msg_raw):
        self.msg_raw = msg_raw
        self.msg_received = True


    def test_publisher_and_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_switch_global.get_num_connections(), 1, "No connections found on switch topic")
        self.assertGreaterEqual(self.sub_raw_global.get_num_connections(), 1, "No connections found on raw topic")


if __name__ == '__main__':
    rospy.init_node('apriltags_preprocessing_tester_node', anonymous=False)
    rostest.rosrun('apriltags', 'apriltags_preprocessing_tester_node', ApriltagsPreprocessingTesterNode)
