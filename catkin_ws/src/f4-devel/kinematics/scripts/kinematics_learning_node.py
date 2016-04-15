#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import ThetaDotSample, Vsample, KinematicsWeights
from kinematics import Linear_learner
from numpy import *
from threading import Thread, Lock

# Inverse Kinematics Node
# Author: Jason Pazis

class KinematicsLearningNode(object):
    def __init__(self):
        self.node_name = 'kinematics_learning_node'

        self.mutex = Lock()

        # Read parameters
        fi_theta_dot_function = self.setupParameter('~fi_theta_dot_function_param', 'Duty_fi_theta_dot_naive')
        fi_v_function = self.setupParameter('~fi_v_function_param', 'Duty_fi_v_naive')
        self.theta_dot_noZeros = self.setupParameter('~learner_number_of_zero_entries', 0)
        self.v_noZeros = self.setupParameter('~learner_number_of_zero_entries', 0)
        self.noThetaDotSamples = 2*self.setupParameter('~learner_number_of_theta_dot_samples_per_direction', 1000)
        self.noVsamples = self.setupParameter('~learner_number_of_v_samples', 1000)
        self.theta_dot_threshold = self.setupParameter('~learner_theta_dot_threshold', 0.1)
        self.v_disp_threshold = pow(self.setupParameter('~learner_v_disp_threshold', 0.01), 2)
        theta_dot_regularizer = self.setupParameter('~learner_theta_dot_regularizer', 0.1)
        v_regularizer = self.setupParameter('~learner_v_regularizer', 0.1)
        
        # Setup the kinematics learning model
        self.kl = Linear_learner.Linear_learner(fi_theta_dot_function, fi_v_function, theta_dot_regularizer, v_regularizer)

        # Setup publishers and subscribers
        self.pub_theta_dot_kinematics_weights = rospy.Publisher("~theta_dot_kinematics_weights", KinematicsWeights, queue_size=1)
        self.pub_v_kinematics_weights = rospy.Publisher("~v_kinematics_weights", KinematicsWeights, queue_size=1)
        self.sub_theta_dot_sample = rospy.Subscriber("~theta_dot_sample", ThetaDotSample, self.thetaDotSampleCallback)
        self.sub_v_sample = rospy.Subscriber("~v_sample", Vsample, self.vSampleCallback)

        self.theta_dot_negative_index = self.theta_dot_noZeros
        self.theta_dot_positive_index = self.theta_dot_negative_index + self.noThetaDotSamples/2
        self.v_index = self.v_noZeros
        
        self.theta_dot_d_L = zeros((self.theta_dot_noZeros + self.noThetaDotSamples,1))
        self.theta_dot_d_R = zeros((self.theta_dot_noZeros + self.noThetaDotSamples,1))
        self.theta_dot_dt = zeros((self.theta_dot_noZeros + self.noThetaDotSamples,1))
        self.theta_dot_dt[:self.theta_dot_noZeros,:] = 1 # initialize dt=1 for the dummy zero motion samples
        self.theta_dot_theta_angle_pose_delta = zeros((self.theta_dot_noZeros + self.noThetaDotSamples,1))

        self.v_d_L = zeros((self.v_noZeros + self.noVsamples,1))
        self.v_d_R = zeros((self.v_noZeros + self.noVsamples,1))
        self.v_dt = zeros((self.v_noZeros + self.noVsamples,1))
        self.v_dt[:self.v_noZeros,:] = 1 # initialize dt=1 for the dummy zero motion samples
        self.v_theta_angle_pose_delta = zeros((self.v_noZeros + self.noVsamples,1))
        self.v_x_axis_pose_delta = zeros((self.v_noZeros + self.noVsamples,1))
        self.v_y_axis_pose_delta = zeros((self.v_noZeros + self.noVsamples,1))
        rospy.loginfo("[%s] has started", self.node_name)

    def thetaDotSampleCallback(self, theta_dot_sample):
        #print 'theta_dot_sample', theta_dot_sample
        self.mutex.acquire()
        # Only use this sample if it is informative
        if (self.theta_dot_negative_index < self.theta_dot_noZeros+self.noThetaDotSamples/2) and (theta_dot_sample.theta_angle_pose_delta/theta_dot_sample.dt <= -self.theta_dot_threshold):
            #print 'adding negative'
            self.theta_dot_d_L[self.theta_dot_negative_index] = theta_dot_sample.d_L
            self.theta_dot_d_R[self.theta_dot_negative_index] = theta_dot_sample.d_R
            self.theta_dot_dt[self.theta_dot_negative_index] = theta_dot_sample.dt
            self.theta_dot_theta_angle_pose_delta[self.theta_dot_negative_index] = theta_dot_sample.theta_angle_pose_delta
            self.theta_dot_negative_index += 1
        elif (self.theta_dot_positive_index < self.theta_dot_noZeros+self.noThetaDotSamples) and (theta_dot_sample.theta_angle_pose_delta/theta_dot_sample.dt >= self.theta_dot_threshold):
            #print 'adding positive'
            self.theta_dot_d_L[self.theta_dot_positive_index] = theta_dot_sample.d_L
            self.theta_dot_d_R[self.theta_dot_positive_index] = theta_dot_sample.d_R
            self.theta_dot_dt[self.theta_dot_positive_index] = theta_dot_sample.dt
            self.theta_dot_theta_angle_pose_delta[self.theta_dot_positive_index] = theta_dot_sample.theta_angle_pose_delta
            self.theta_dot_positive_index += 1

        # print 'theta_dot_negative_index', self.theta_dot_negative_index, 'theta_dot_positive_index', self.theta_dot_positive_index
        # Only fit when a sufficient number of informative samples has been gathered
        if (self.theta_dot_negative_index >= (self.theta_dot_noZeros+self.noThetaDotSamples/2)) and (self.theta_dot_positive_index >= self.theta_dot_noZeros+self.noThetaDotSamples):
            weights = self.kl.fit_theta_dot(self.theta_dot_d_L, self.theta_dot_d_R, self.theta_dot_dt, self.theta_dot_theta_angle_pose_delta)
            weights = asarray(weights).flatten().tolist()
            print 'theta_dot weights', weights
            # Put the weights in a message and publish
            msg_kinematics_weights = KinematicsWeights()
            msg_kinematics_weights.weights = weights
            self.pub_theta_dot_kinematics_weights.publish(msg_kinematics_weights)
            # reset indices
            self.theta_dot_negative_index = self.theta_dot_noZeros
            self.theta_dot_positive_index = self.theta_dot_negative_index + self.noThetaDotSamples/2
        self.mutex.release()


    def vSampleCallback(self, v_sample):
        #print 'v_sample', v_sample
        # Only use this sample if it is informative
        #res = (v_sample.x_axis_pose_delta*v_sample.x_axis_pose_delta + v_sample.y_axis_pose_delta*v_sample.y_axis_pose_delta)/v_sample.dt
        #print 'v', res
        if (v_sample.x_axis_pose_delta*v_sample.x_axis_pose_delta + v_sample.y_axis_pose_delta*v_sample.y_axis_pose_delta)/v_sample.dt >= self.v_disp_threshold:
            self.mutex.acquire()
            self.v_d_L[self.v_index] = v_sample.d_L
            self.v_d_R[self.v_index] = v_sample.d_R
            self.v_dt[self.v_index] = v_sample.dt
            self.v_theta_angle_pose_delta[self.v_index] = v_sample.theta_angle_pose_delta
            self.v_x_axis_pose_delta[self.v_index] = v_sample.x_axis_pose_delta
            self.v_y_axis_pose_delta[self.v_index] = v_sample.y_axis_pose_delta
            self.v_index += 1

            # Only fit when a sufficient number of samples has been gathered
            if self.v_index >= (self.v_noZeros+self.noVsamples):
                weights = self.kl.fit_v(self.v_d_L, self.v_d_R, self.v_dt, self.v_theta_angle_pose_delta, self.v_x_axis_pose_delta, self.v_y_axis_pose_delta)
                weights = asarray(weights).flatten().tolist()
                print 'v weights', weights
                # Put the weights in a message and publish
                msg_kinematics_weights = KinematicsWeights()
                msg_kinematics_weights.weights = weights
                self.pub_v_kinematics_weights.publish(msg_kinematics_weights)
                # reset index
                self.v_index = self.v_noZeros
            self.mutex.release()


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('kinematics_learning_node', anonymous=False)
    kinematics_learning_node = KinematicsLearningNode()
    rospy.spin()
