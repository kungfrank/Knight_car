#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import ThetaDotSample, Vsample, WheelsCmdStamped
from geometry_msgs.msg import  PoseStamped
import time
import tf.transformations as tr
import numpy as np

class TestViconLearningNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestViconLearningNode, self).__init__(*args)
        self.msg_v_received = False
        self.msg_theta_dot_received = False
        self.msg_theta_dot_sample = ThetaDotSample()
        self.msg_v_sample = Vsample()

    def setup_publisher_and_subscriber(self):
        #Setup the publishers and subscribers
        self.pub_pose = rospy.Publisher("vicon_learning_node/pose", PoseStamped,queue_size=1, latch=True)
        self.pub_wheels_cmd = rospy.Publisher("vicon_learning_node/wheels_cmd", WheelsCmdStamped,queue_size=1, latch=True)
        self.sub_theta_dot = rospy.Subscriber("vicon_learning_node/theta_dot_sample", ThetaDotSample, self.thetaSampleCallback)
        self.sub_v = rospy.Subscriber("vicon_learning_node/v_sample", Vsample, self.vSampleCallback)

        # Wait for the vicon_learning_node to finish starting up
        timeout = time.time()+5.0
        while (self.pub_pose.get_num_connections() < 1 or self.pub_wheels_cmd.get_num_connections() < 1 or \
                self.sub_theta_dot.get_num_connections() < 1 or self.sub_v.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

    def test_vicon_learning_publisher_and_subscriber(self):
        self.setup_publisher_and_subscriber()
        self.assertGreaterEqual(self.pub_pose.get_num_connections(), 1)
        self.assertGreaterEqual(self.pub_wheels_cmd.get_num_connections(), 1)
        self.assertGreaterEqual(self.sub_theta_dot.get_num_connections(), 1)
        self.assertGreaterEqual(self.sub_v.get_num_connections(), 1)

    # Test removed because it is no longer compatible with the current version of vicon_learning_node
    def donot_test_vicon_learning_sample_calculation(self):
        self.setup_publisher_and_subscriber()

        # publish the first wheels_cmd
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.vel_left = 1
        msg_wheels_cmd.vel_right = 1
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

        # publish a vicon pose
        msg_pose = PoseStamped()
        msg_pose.pose.position.x = 1
        msg_pose.pose.position.y = 1
        q = tr.quaternion_about_axis(np.pi/2,(0,0,1))
        for i, attr in enumerate(['x', 'y', 'z', 'w']):
            msg_pose.pose.orientation.__setattr__(attr, q[i])
        self.pub_pose.publish(msg_pose)

        # publish second wheels_cmd pose
        msg_wheels_cmd.header.stamp = rospy.Time(1)
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

        # Wait for the samples to come back
        timeout = time.time()+10.0
        while not (self.msg_v_received and self.msg_theta_dot_received) and not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

        # Notify if the timeout struck
        self.assertLess(time.time(),timeout,msg="WARNING: Timeout reached, no samples received from vicon_learning_node.")

        self.assertAlmostEqual(self.msg_v_sample.d_R, 1)
        self.assertAlmostEqual(self.msg_v_sample.d_L, 1)
        self.assertAlmostEqual(self.msg_v_sample.dt, 1)
        self.assertAlmostEqual(self.msg_v_sample.x_axis_pose_delta, -1)
        self.assertAlmostEqual(self.msg_v_sample.y_axis_pose_delta, 1)
        self.assertAlmostEqual(self.msg_v_sample.theta_angle_pose_delta, np.pi/2)
        self.assertAlmostEqual(self.msg_theta_dot_sample.d_R, 1)
        self.assertAlmostEqual(self.msg_theta_dot_sample.d_L, 1)
        self.assertAlmostEqual(self.msg_theta_dot_sample.dt, 1)
        self.assertAlmostEqual(self.msg_theta_dot_sample.theta_angle_pose_delta, np.pi/2)

    def thetaSampleCallback(self, msg):
        self.msg_theta_dot_sample = msg
        self.msg_theta_dot_received = True

    def vSampleCallback(self, msg):
        self.msg_v_sample = msg
        self.msg_v_received = True


if __name__ == '__main__':
    rospy.init_node('vicon_learning_tester_node', anonymous=False)
    rostest.rosrun('kinematics', 'vicon_learning_tester_node', TestViconLearningNode)
