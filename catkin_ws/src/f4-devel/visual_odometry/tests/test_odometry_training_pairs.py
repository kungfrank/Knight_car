#!/usr/bin/env python
PKG = 'visual_odometry'
import rospy
import unittest
import math
from duckietown_msgs.msg import StopLineReading, LanePose, WheelsCmdStamped, Vsample, ThetaDotSample

class TestOdometryTrainingPairsCommon(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_odometry_training_pairs',anonymous=False)

        ## Publishers
        self.pub_stop_lin_reading = rospy.Publisher("~stop_line_reading", StopLineReading, queue_size=1)
        self.pub_wheels_cmd_executed = rospy.Publisher("~wheels_cmd_executed", WheelsCmdStamped, queue_size=1)
        self.pub_lane_pose = rospy.Publisher("~lane_pose", LanePose, queue_size=1)

        ## Subscribers
        self.sub_theta_dot_sample = rospy.Subscriber("~theta_dot_sample", ThetaDotSample, self.theta_dot_sample_CB)
        self.v_sample = rospy.Subscriber("~v_sample", Vsample, self.v_sample_CB)

        ## State variables
        self.theta_dot_received = ThetaDotSample()

    def sendMsgs(self):
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
        lane_pose.header.stamp = rospy.Time.now()
        lane_pose.d = 0
        lane_pose.sigma_d = 0
        lane_pose.phi = 0
        lane_pose.sigma_phi = 0
        lane_pose.status = 'NORMAL'
        lane_pose.in_lane = True
        for i in [1, 2]:
            self.pub_lane_pose.publish(lane_pose)
            rate.sleep()
        # rospy.sleep(1)

    # assert that ~theta_dot_sample is correct
    def theta_dot_sample_CB(self, msg):
        rospy.loginfo("theta_dot_sample received")
        self.theta_dot_received = msg

    def v_sample_CB(self, msg):
        rospy.loginfo("v_sample received")
        self.v_received = msg

class TestOdometryTrainingPairsValid(TestOdometryTrainingPairsCommon):

    def test_obvious(self):
        self.assertEqual(True,True,'True = True')

    def test_theta_dot_sample(self):
        msg = self.theta_dot_received
        self.assertEqual(msg.d_L,0.5,'d_L = 0.5')
        self.assertEqual(msg.d_R,0.5,'d_R = 0.5')
        self.assertEqual(msg.dt,0.1,'d_L = 0.1')
        self.assertEqual(msg.theta_angle_pose_delta,0,'theta_angle_pose_delta = 0')

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_odometry_training_pairs', TestOdometryTrainingPairsValid)

    # rosunit.unitrun('visual_odometry', 'test_odometry_training_pairs', TestOdometryTrainingPairs)
    # unittest.main()
