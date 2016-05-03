#!/usr/bin/env python
import rospy
import unittest, rostest
from rostest_example.Quacker import *
from std_msgs.msg import String, Int32

class DuckiecallTesterNode(unittest.TestCase):
    def __init__(self, *args):
        super(DuckiecallTesterNode, self).__init__(*args)
        self.msg_duckiecall = String()
        self.msg_received = False

    def setup(self):
        # Setup the node
        rospy.init_node('duckiecall_tester_node', anonymous=False)

        # Setup the publisher and subscriber
        self.pub_num_of_quacks = rospy.Publisher("~number_of_quacks", Int32, queue_size=1, latch=True)
        self.sub_duckiecall = rospy.Subscriber("~duckiecall", String, self.duckiecallCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.pub_num_of_quacks.get_num_connections() < 1 or self.sub_duckiecall.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def duckiecallCallback(self, msg_duckiecall):
        self.msg_duckiecall = msg_duckiecall
        self.msg_received = True

    def test_publisher_and_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_num_of_quacks.get_num_connections(), 1, "No connections found on num_of_quacks topic")
        self.assertGreaterEqual(self.sub_duckiecall.get_num_connections(), 1, "No connections found on duckiecall topic")

    def test_duckiecall_output(self):
        self.setup()    # Setup the node

        # Send the message to the number_of_quacks topic
        msg_num_of_quacks = Int32()
        msg_num_of_quacks.data = 3
        self.pub_num_of_quacks.publish(msg_num_of_quacks)


        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        # Send an error if the timeout was hit
        self.assertLess(rospy.Time.now(), timeout, "The test timed out with no response from the duckiecall_node")

        # Test the response
        response = self.msg_duckiecall.data
        self.assertEqual(response, "Quack! Quack! Quack!")   # Three Quacks! expected

if __name__ == '__main__':
    rospy.init_node('duckiecall_tester_node', anonymous=False)
    rostest.rosrun('rostest_example', 'duckiecall_tester_node', DuckiecallTesterNode)
