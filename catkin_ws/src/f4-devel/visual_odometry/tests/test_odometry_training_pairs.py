#!/usr/bin/env python
PKG = 'visual_odometry'
import rostest
import rospy
import unittest
import math
import time
from geometry_msgs.msg import Point
from duckietown_msgs.msg import StopLineReading, LanePose, WheelsCmdStamped, Vsample, ThetaDotSample

class TestOdometryTrainingPairsCommon(unittest.TestCase):

    def setUp(self):

        ## Publishers
        self.pub_stop_line_reading = rospy.Publisher("odometry_training_pairs_node/stop_line_reading", StopLineReading, queue_size=1)
        self.pub_wheels_cmd_executed = rospy.Publisher("odometry_training_pairs_node/wheels_cmd_executed", WheelsCmdStamped, queue_size=1)
        self.pub_lane_pose = rospy.Publisher("odometry_training_pairs_node/lane_pose", LanePose, queue_size=1)

        ## Subscribers
        self.sub_theta_dot_sample = rospy.Subscriber("odometry_training_pairs_node/theta_dot_sample", ThetaDotSample, self.theta_dot_sample_CB)
        self.sub_v_sample = rospy.Subscriber("odometry_training_pairs_node/v_sample", Vsample, self.v_sample_CB)

        ## State variables
        self.theta_dot_received = ThetaDotSample()
        self.v_received = Vsample()

    def theta_dot_sample_CB(self, msg):
        rospy.loginfo("theta_dot_sample received")
        self.theta_dot_received = msg

    def v_sample_CB(self, msg):
        rospy.loginfo("v_sample received")
        self.v_received = msg

class TestOdometryTrainingPairsValid(TestOdometryTrainingPairsCommon):

    def test_theta_dot_simple(self):
        # publish wheel_cmd_executed
        wheels_cmd = WheelsCmdStamped()
        rate = rospy.Rate(10)
        for i in range(10):
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = 0.5
            wheels_cmd.vel_right = 0.5
            self.pub_wheels_cmd_executed.publish(wheels_cmd)
            rate.sleep()

        # publish lane_pose
        lane_pose = LanePose()
        lane_pose.d = 0
        lane_pose.sigma_d = 0
        lane_pose.phi = 0
        lane_pose.sigma_phi = 0
        lane_pose.status = 0
        lane_pose.in_lane = True
        for i in [1, 2]:
            lane_pose.header.stamp = rospy.Time.now()
            self.pub_lane_pose.publish(lane_pose)
            rate.sleep()

        # Wait for the odometry_training_pairs_node to finish starting up
        timeout = time.time()+2.0
        while not self.sub_theta_dot_sample.get_num_connections() and \
                not rospy.is_shutdown() and not time.time() > timeout:
            rospy.sleep(0.1)

        msg = self.theta_dot_received
        # self.assertEqual(hasattr(msg,'d_L'),True)
        self.assertEqual(msg.d_L, 0.5, 'd_L = %s' % msg.d_L)
        self.assertEqual(msg.d_R, 0.5, 'd_R = %s' % msg.d_R)
        self.assertAlmostEqual(msg.dt, 0.1, 2,'dt = %s' % msg.dt)
        self.assertEqual(msg.theta_angle_pose_delta, 0, 'theta_angle_pose_delta = %s' % msg.theta_angle_pose_delta)

    def test_v_dot_simple(self):
        # publish wheel_cmd_executed
        wheels_cmd = WheelsCmdStamped()
        rate = rospy.Rate(10)
        for i in range(10):
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = 0.5
            wheels_cmd.vel_right = 0.5
            self.pub_wheels_cmd_executed.publish(wheels_cmd)
            rate.sleep()

        # publish LanePose
        lane_pose = LanePose()
        lane_pose.d = 0
        lane_pose.sigma_d = 0
        lane_pose.phi = 0
        lane_pose.sigma_phi = 0
        lane_pose.status = 0
        lane_pose.in_lane = True
        for i in [1, 2]:
            lane_pose.header.stamp = rospy.Time.now()
            self.pub_lane_pose.publish(lane_pose)
            rate.sleep()

        # publish StopLineReading
        stop_line_reading = StopLineReading()
        stop_line_reading.stop_line_detected = True
        stop_line_reading.at_stop_line = False
        stop_line_reading.stop_line_point = Point()
        for x in [0.51, 0.5]:
            stop_line_reading.header.stamp = rospy.Time.now()
            stop_line_reading.stop_line_point.x = x
            self.pub_stop_line_reading.publish(stop_line_reading)
            rate.sleep()

        # Wait for the odometry_training_pairs_node to finish starting up
        timeout = time.time()+2.0
        while not self.sub_v_sample.get_num_connections() and \
                not rospy.is_shutdown() and not time.time() > timeout:
            rospy.sleep(0.1)

        msg = self.v_received
        # self.assertEqual(hasattr(msg,'d_L'),True)
        self.assertEqual(msg.d_L, 0.5, 'd_L = %s' % msg.d_L)
        self.assertEqual(msg.d_R, 0.5, 'd_R = %s' % msg.d_R)
        self.assertAlmostEqual(msg.dt, 0.1, 2,'dt = %s' % msg.dt)
        self.assertEqual(msg.theta_angle_pose_delta, 0, 'theta_angle_pose_delta = %s' % msg.theta_angle_pose_delta)
        self.assertAlmostEqual(msg.x_axis_pose_delta, 0.01, 5, 'x = %s' % msg.x_axis_pose_delta)
        self.assertEqual(msg.y_axis_pose_delta, 0, 'y = %s' % msg.y_axis_pose_delta)

if __name__ == '__main__':
    rospy.init_node('test_odometry_training_pairs',anonymous=False)
    rostest.rosrun(PKG, 'test_odometry_training_pairs', TestOdometryTrainingPairsValid)

    # rosunit.unitrun('visual_odometry', 'test_odometry_training_pairs', TestOdometryTrainingPairs)
    # unittest.main()
