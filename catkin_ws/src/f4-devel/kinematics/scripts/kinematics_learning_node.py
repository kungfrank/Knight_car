#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import ThetaDotSample, Vsample, KinematicsWeights
from kinematics import Linear_learner
from numpy import *

# Inverse Kinematics Node
# Author: Jason Pazis

class KinematicsLearningNode(object):
    def __init__(self):
        self.node_name = 'kinematics_learning_node'

        # Read parameters
        fi_theta_dot_function = self.setupParameter('~fi_theta_dot_function', 'Duty_fi_theta_dot_naive')
        fi_v_function = self.setupParameter('~fi_v_function', 'Duty_fi_v_naive')
        theta_dot_weights = self.setupParameter('~theta_dot_weights', [-1.0])
        v_weights = self.setupParameter('~v_weights', [1.0])
        self.noZeros = self.setupParameter('~learner_number_of_zero_entries', 10)
        self.noSamples = self.setupParameter('~learner_number_of_samples', 50)
        self.duty_threshold = self.setupParameter('~learner_duty_threshold', 0.4)

        # Setup the kinematics learning model
        self.kl = Linear_learner.Linear_learner(fi_theta_dot_function, fi_v_function)

        # Setup publishers and subscribers
        self.pub_theta_dot_kinematics_weights = rospy.Publisher("~theta_dot_kinematics_weights", KinematicsWeights, queue_size=1)
        self.pub_v_kinematics_weights = rospy.Publisher("~v_kinematics_weights", KinematicsWeights, queue_size=1)
        self.sub_theta_dot_sample = rospy.Subscriber("~theta_dot_sample", ThetaDotSample, self.thetaDotSampleCallback)
        self.sub_v_sample = rospy.Subscriber("~v_sample", Vsample, self.vSampleCallback)

        self.theta_dot_index = self.noZeros
        self.v_index = self.noZeros

        self.theta_dot_d_L = zeros((self.noZeros + self.noSamples,1))
        self.theta_dot_d_R = zeros((self.noZeros + self.noSamples,1))
        self.theta_dot_dt = zeros((self.noZeros + self.noSamples,1))
        self.theta_dot_dt[:self.noZeros,:] = 1
        self.theta_dot_theta_angle_pose_delta = zeros((self.noZeros + self.noSamples,1))

        self.v_d_L = zeros((self.noZeros + self.noSamples,1))
        self.v_d_R = zeros((self.noZeros + self.noSamples,1))
        self.v_dt = zeros((self.noZeros + self.noSamples,1))
        self.theta_dot_dt[:self.noZeros,:] = 1
        self.v_theta_angle_pose_delta = zeros((self.noZeros + self.noSamples,1))
        self.v_x_axis_pose_delta = zeros((self.noZeros + self.noSamples,1))
        self.v_y_axis_pose_delta = zeros((self.noZeros + self.noSamples,1))
        rospy.loginfo("[%s] has started", self.node_name)

    def thetaDotSampleCallback(self, theta_dot_sample):
        # Only use this sample if at least one of the motors is above the threshold
        if (abs(theta_dot_sample.d_L) > self.duty_threshold) or (abs(theta_dot_sample.d_R) > self.duty_threshold):
            self.theta_dot_d_L[self.theta_dot_index] = theta_dot_sample.d_L
            self.theta_dot_d_R[self.theta_dot_index] = theta_dot_sample.d_R
            self.theta_dot_dt[self.theta_dot_index] = theta_dot_sample.dt
            self.theta_dot_theta_angle_pose_delta[self.theta_dot_index] = theta_dot_sample.theta_angle_pose_delta
            self.theta_dot_index += 1

            # Only fit when a sufficient number of samples has been gathered
            if self.theta_dot_index >= (self.noZeros+self.noSamples):
                # Only fit if the set contains a sufficient number of samples with large absolute d_L and d_R values
                if (average(abs(self.theta_dot_d_L)) > self.duty_threshold) and (average(abs(self.theta_dot_d_R)) > self.duty_threshold):
                    weights = self.kl.fit_theta_dot(self.theta_dot_d_L, self.theta_dot_d_R, self.theta_dot_dt, self.theta_dot_theta_angle_pose_delta).flatten()
                    # Put the weights in a message and publish
                    msg_kinematics_weights = KinematicsWeights()
                    self.pub_theta_dot_kinematics_weights.publish(msg_kinematics_weights)
                # reset index
                self.theta_dot_index = self.noZeros


    def vSampleCallback(self, v_sample):
        # Only use this sample if at least one of the motors is above the threshold
        if (abs(v_sample.d_L) > self.duty_threshold) or (abs(v_sample.d_R) > self.duty_threshold):
            self.v_d_L[self.v_index] = v_sample.d_L
            self.v_d_R[self.v_index] = v_sample.d_R
            self.v_dt[self.v_index] = v_sample.dt
            self.v_theta_angle_pose_delta[self.v_index] = v_sample.theta_angle_pose_delta
            self.v_x_axis_pose_delta[self.v_index] = v_sample.x_axis_pose_delta
            self.v_y_axis_pose_delta[self.v_index] = v_sample.y_axis_pose_delta
            self.v_index += 1

            # Only fit when a sufficient number of samples has been gathered
            if self.v_index >= (self.noZeros+self.noSamples):
                # Only fit if the set contains a sufficient number of samples with large absolute d_L and d_R values
                if (average(abs(self.v_d_L)) > self.duty_threshold) and (average(abs(self.v_d_R)) > self.duty_threshold):
                    weights = self.kl.fit_v(self.v_d_L, self.v_d_R, self.v_dt, self.v_theta_angle_pose_delta, self.v_x_axis_pose_delta, self.v_y_axis_pose_delta).flatten()
                    # Put the weights in a message and publish
                    msg_kinematics_weights = KinematicsWeights()
                    self.pub_v_kinematics_weights.publish(msg_kinematics_weights)
                # reset index
                self.v_index = self.noZeros


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('kinematics_learning_node', anonymous=False)
    kinematics_learning_node = KinematicsLearningNode()
    rospy.spin()
