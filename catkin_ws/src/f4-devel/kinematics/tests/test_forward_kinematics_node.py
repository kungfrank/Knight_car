#!/usr/bin/env python
import rospy
import unittest, rostest
import sys, time
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, Pose2DStamped, KinematicsWeights

class TestForwardKinematicsNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestForwardKinematicsNode, self).__init__(*args)
        self.v = None
        self.omega = None
        self.msg_received = False

    def test_forward_kinematics_msg_throughput(self):

        #Setup the publisher and subscribers
        self.pub_wheels_cmd = rospy.Publisher("forward_kinematics_node/wheels_cmd", WheelsCmdStamped, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("forward_kinematics_node/velocity", Twist2DStamped, self.velocityCallback)

        # Wait for the forward_kinematics_node to finish starting up
        timeout = time.time()+10.0
        while (self.sub.get_num_connections() < 1 or self.pub_wheels_cmd.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

        # Publish a wheels_cmd
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.vel_left = 1
        msg_wheels_cmd.vel_right = 1
        self.pub_wheels_cmd.publish(msg_wheels_cmd)


        # Wait for the message to be received
        while not self.msg_received and not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

        # Check the result
        self.assertAlmostEqual(self.v, 2)
        self.assertAlmostEqual(self.omega, 0)

    def velocityCallback(self, msg_velocity):
        self.v = msg_velocity.v
        self.omega = msg_velocity.omega
        self.msg_received = True


if __name__ == '__main__':
    rospy.init_node('test_forward_kinematics_node', anonymous=False)
    rostest.rosrun('kinematics', 'test_forward_kinematics_node', TestForwardKinematicsNode)
