#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import ThetaDotSample, Vsample, KinematicsWeights
import time
import numpy as np

class TestKinematicsLearningNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestKinematicsLearningNode, self).__init__(*args)

        self.msg_theta_dot_weights = KinematicsWeights()
        self.msg_v_weights = KinematicsWeights()
        self.msg_theta_dot_received = False
        self.msg_v_received = False


    def setup_publishers_and_subscribers(self):
        rospy.init_node('kinematics_learning_tester_node', anonymous=False)
        #Setup the publisher and subscribers
        self.sub_theta_dot_kinematics_weights = rospy.Subscriber("kinematics_learning_node/theta_dot_kinematics_weights", KinematicsWeights, self.thetaDotWeightsCallback)
        self.sub_v_kinematics_weights = rospy.Subscriber("kinematics_learning_node/v_kinematics_weights", KinematicsWeights, self.vWeightsCallback)
        self.pub_theta_dot_sample = rospy.Publisher("kinematics_learning_node/theta_dot_sample", ThetaDotSample, queue_size=1, latch=True)
        self.pub_v_sample = rospy.Publisher("kinematics_learning_node/v_sample", Vsample, queue_size=1, latch=True)

        # Wait for the inverse_kinematics_node to finish starting up
        timeout = time.time()+5.0
        while (self.sub_theta_dot_kinematics_weights.get_num_connections() < 1 or self.sub_v_kinematics_weights.get_num_connections() < 1 or \
                self.pub_theta_dot_sample.get_num_connections() < 1 or self.pub_v_sample.get_num_connections() < 1) and \
                not rospy.is_shutdown() and not time.time()>timeout:
            rospy.sleep(0.1)

    def test_publishers_and_subscribers(self):
        self.setup_publishers_and_subscribers()
        self.assertEqual(self.pub_theta_dot_sample.get_num_connections(),1)
        self.assertEqual(self.pub_v_sample.get_num_connections(),1)
        self.assertEqual(self.sub_theta_dot_kinematics_weights.get_num_connections(),1)
        self.assertEqual(self.sub_v_kinematics_weights.get_num_connections(),1)

    def test_learning_from_generated_samples(self):
        self.setup_publishers_and_subscribers()

        # Generate random samples and publish
        msg_v_sample = Vsample()
        msg_theta_dot_sample = ThetaDotSample()
        theta_dot_weights = np.array([-1, 1])
        v_weights = np.array([1,1])
        rate = rospy.Rate(30)
        timeout = time.time()+10.0
        while (not rospy.is_shutdown()) and (not (self.msg_theta_dot_received and self.msg_v_received)) and time.time()<timeout:
            sample = self.getRandomSamplePair(theta_dot_weights, v_weights)
            msg_v_sample.d_L = sample[0]
            msg_v_sample.d_R = sample[1]
            msg_v_sample.dt = sample[2]
            msg_v_sample.theta_angle_pose_delta = sample[3]
            msg_v_sample.x_axis_pose_delta = sample[4]
            msg_v_sample.y_axis_pose_delta = sample[5]
            msg_theta_dot_sample.d_L = sample[0]
            msg_theta_dot_sample.d_R = sample[1]
            msg_theta_dot_sample.dt = sample[2]
            msg_theta_dot_sample.theta_angle_pose_delta = sample[3]

            self.pub_v_sample.publish(msg_v_sample)
            self.pub_theta_dot_sample.publish(msg_theta_dot_sample)
            rate.sleep()

        # Notify if the timeout struck
        self.assertLess(time.time(),timeout,msg="WARNING: There was not enough time to receive the weights. Either increase the timeout or decrease the number of points needed by the learner.")

        # Check that the new weights are close to the generating ones
        self.assertAlmostEqual(self.msg_v_weights.weights[0],1, 0)
        self.assertAlmostEqual(self.msg_v_weights.weights[1],1, 0)
        self.assertAlmostEqual(self.msg_theta_dot_weights.weights[0],-1, 0)
        self.assertAlmostEqual(self.msg_theta_dot_weights.weights[1],1, 0)

    def getRandomSamplePair(self, theta_dot_weights, v_weights):
        cmd = np.random.rand(2,1)
        dt = np.random.rand()
        w = np.hstack((np.vstack((v_weights,theta_dot_weights)),np.random.randn(2,1)/100))
        vel = np.dot(w,np.vstack((cmd,1)))
        if vel[1] < 1e-6:
            dp = np.vstack((np.flipud(vel)*dt,0))
        else:
            dp = np.array([vel[1]*dt, vel[0]/vel[1] * np.sin(vel[1]*dt), vel[0]/vel[1] * (1- np.cos(vel[1]*dt))])
        return np.transpose(cmd).flatten().tolist()+[dt]+np.transpose(dp).flatten().tolist()

    def thetaDotWeightsCallback(self, msg):
        self.msg_theta_dot_weights = msg
        self.msg_theta_dot_received = True

    def vWeightsCallback(self, msg):
        self.msg_v_weights = msg
        self.msg_v_received = True

if __name__ == '__main__':
    rostest.rosrun('kinematics', 'kinematics_learning_tester_node', TestKinematicsLearningNode)
