#!/usr/bin/env python
import rospy
import unittest
import rostest
import time
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

class TestInverseKinematicsNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestInverseKinematicsNode, self).__init__(*args)
        self.vel_left = None
        self.vel_right = None
        self.msg_received = False

    def setup_publisher_and_subscriber(self):
        #Setup the publisher and subscribers
        self.pub_car_cmd = rospy.Publisher("inverse_kinematics_node/car_cmd", Twist2DStamped, queue_size=1, latch=True)
        self.sub_wheels_cmd = rospy.Subscriber("inverse_kinematics_node/wheels_cmd", WheelsCmdStamped, self.carCmdCallback)

        # Wait for the inverse_kinematics_node to finish starting up
        timeout = time.time()+5.0
        while (self.pub_car_cmd.get_num_connections() < 1 or self.sub_wheels_cmd.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

    def test_publishers_and_subscribers(self):
        self.setup_publisher_and_subscriber()
        self.assertGreaterEqual(self.pub_car_cmd.get_num_connections(), 1)
        self.assertGreaterEqual(self.sub_wheels_cmd.get_num_connections(), 1)

    def test_inverse_kinematics_calculation_straight(self):
        self.setup_publisher_and_subscriber()
        self.msg_received = False
        msg_car_cmd = Twist2DStamped()
        msg_car_cmd.v = 2
        msg_car_cmd.omega = 0
        self.pub_car_cmd.publish(msg_car_cmd)

        # Wait for the message to be received
        timeout = time.time()+5.0
        while not self.msg_received and not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

        self.assertAlmostEqual(self.vel_left, 1)
        self.assertAlmostEqual(self.vel_right, 1)


    def test_inverse_kinematics_calculation_curved(self):
        self.setup_publisher_and_subscriber()
        self.msg_received = False
        msg_car_cmd = Twist2DStamped()
        msg_car_cmd.v = 1
        msg_car_cmd.omega = 1
        self.pub_car_cmd.publish(msg_car_cmd)

        # Wait for the message to be received
        timeout = time.time()+5.0
        while not self.msg_received and not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)
        self.assertTrue(self.msg_received, "Test timed out while waiting for msg on output topic.")

        self.assertAlmostEqual(self.vel_left, 0)
        self.assertAlmostEqual(self.vel_right, 1)

    def carCmdCallback(self, msg):
        self.vel_left = msg.vel_left
        self.vel_right = msg.vel_right
        self.msg_received = True

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_tester_node', anonymous=False)
    rostest.rosrun('kinematics', 'inverse_kinematics_tester_node', TestInverseKinematicsNode)
