#!/usr/bin/env python
import rospy
import unittest, rostest
from std_msgs.msg import Float32MultiArray, Int32


class AverageQuacksTesterNode(unittest.TestCase):
    def __init__(self, *args):
        super(AverageQuacksTesterNode, self).__init__(*args)
        self.msg_quacks = Int32()
        self.msg_received = False

    def setup(self):
        # Setup the node
        rospy.init_node('average_quacks_tester_node', anonymous=False)

        # Setup the publisher and subscriber
        self.pub_list = rospy.Publisher("~list", Float32MultiArray, queue_size=1, latch=True)
        self.sub_quacks = rospy.Subscriber("~number_of_quacks", Int32, self.quacksCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.pub_list.get_num_connections() < 1 or self.sub_quacks.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def quacksCallback(self, msg_quacks):
        self.msg_quacks = msg_quacks
        self.msg_received = True

    def test_publisher_and_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_list.get_num_connections(), 1, "No connections found on list topic")
        self.assertGreaterEqual(self.sub_quacks.get_num_connections(), 1, "No connections found on number_of_quacks topic")

    def test_average_quacks_output(self):
        self.setup()    # Setup the node

        # Send a message to the list topic
        msg_list = Float32MultiArray()
        msg_list.data = [1,2,3]
        self.pub_list.publish(msg_list)

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        # Send an error if the timeout was hit
        self.assertLess(rospy.Time.now(), timeout, "The test timed out with no response from the average_quacks_node")

        # Test the response
        response = self.msg_quacks.data
        self.assertEqual(response, 2)   # The average of 1,2, and 3 is 2

if __name__ == '__main__':
    rostest.rosrun('rostest_example', 'average_quacks_tester_node', AverageQuacksTesterNode)
