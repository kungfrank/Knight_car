#!/usr/bin/env python
import rospy
import unittest
import rostest
import time
import math
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped

class TestPositionFilterNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestPositionFilterNode, self).__init__(*args)
        self.pose = Pose2DStamped()
        self.pose_prev = Pose2DStamped()
        self.msg_received = False


    def setup_publisher_and_subscriber(self):
        #Setup the publisher and subscribers
        self.pub_velocity = rospy.Publisher("position_filter_node/velocity", Twist2DStamped, queue_size=1, latch=True)
        self.sub_pose = rospy.Subscriber("position_filter_node/pose", Pose2DStamped, self.poseCallback)

        # Wait for the inverse_kinematics_node to finish starting up
        timeout = time.time()+5.0
        while (self.pub_velocity.get_num_connections() < 1 or self.sub_pose.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

    def test_position_filter_publisher_and_subscriber(self):
        self.msg_received = False
        self.setup_publisher_and_subscriber()
        self.assertGreaterEqual(self.sub_pose.get_num_connections(), 1)
        self.assertGreaterEqual(self.pub_velocity.get_num_connections(), 1)

    def test_position_filter_calculation_curved(self):
        self.msg_received = False
        self.setup_publisher_and_subscriber()

        msg_velocity = Twist2DStamped()
        msg_velocity.v = 1
        msg_velocity.omega = 1
        msg_velocity.header.stamp = rospy.Time.from_sec(1)
        self.pub_velocity.publish(msg_velocity)
        rospy.sleep(1)
        msg_velocity.header.stamp = rospy.Time.from_sec(2)
        self.pub_velocity.publish(msg_velocity)

        # Wait for the message to be received
        timeout = time.time()+5.0
        while not self.msg_received and not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

        dp = self.subtractPoseMsg(self.pose, self.pose_prev)
        self.assertAlmostEqual(dp.x, math.sin(1))
        self.assertAlmostEqual(dp.y, 1-math.cos(1))
        self.assertAlmostEqual(dp.theta, 1)

    def poseCallback(self, msg):
        self.pose_prev = self.pose
        self.pose = msg
        self.msg_received = True

    def subtractPoseMsg(self, p1, p0):
        # Returns the delta pose p1-p0
        p1.x -= p0.x
        p1.y -= p0.y
        p1.theta -= p0.theta
        return p1

if __name__ == '__main__':
    rospy.init_node('position_filter_tester_node', anonymous=False)
    rostest.rosrun('kinematics', 'position_filter_tester_node', TestPositionFilterNode)
