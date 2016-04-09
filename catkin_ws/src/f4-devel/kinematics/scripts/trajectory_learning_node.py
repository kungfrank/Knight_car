#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import KinematicsWeights
from numpy import *
from threading import Thread, Lock
import os.path

# Trajectory Learning Node
# Author: Jason Pazis

class TrajectoryLearningNode(object):
    def __init__(self):
        self.node_name = 'trajectory_learning_node'

        self.mutex = Lock()

        # Read parameters
        self.veh_name = self.setupParameter("~veh_name","megaman")
        self.theta_dot_filename_1 = self.setupParameter("~theta_dot_filename_1", "theta_dot_filename_1")
        self.theta_dot_filename_2 = self.setupParameter("~theta_dot_filename_2", "theta_dot_filename_2")
        self.v_filename_1 = self.setupParameter("~v_filename_1", "v_filename_1")
        self.theta_delta_1 = self.setupParameter("~theta_delta_1", 20.0*pi)
        self.theta_delta_2 = self.setupParameter("~theta_delta_2", -20.0*pi)
        self.distance_1 = self.setupParameter("~distance_1", 1.0)
        self.fi_theta_dot_function = self.setupParameter('~fi_theta_dot_function_param', 'Duty_fi_theta_dot_compound_linear')
        self.fi_v_function = self.setupParameter('~fi_v_function_param', 'Duty_fi_v_compound_linear')

        self.theta_dot_regularizer = self.setupParameter('~learner_theta_dot_regularizer', 0.01)
        self.v_regularizer = self.setupParameter('~learner_v_regularizer', 0.01)
        
        # Setup publishers
        self.pub_theta_dot_kinematics_weights = rospy.Publisher("~theta_dot_kinematics_weights", KinematicsWeights, latch=True, queue_size=1)
        self.pub_v_kinematics_weights = rospy.Publisher("~v_kinematics_weights", KinematicsWeights, latch=True, queue_size=1)

        if os.path.isfile(self.theta_dot_filename_1) and os.path.isfile(self.theta_dot_filename_2):
            theta_dot_FI_1 = genfromtxt(self.theta_dot_filename_1)
            theta_dot_FI_2 = genfromtxt(self.theta_dot_filename_2)
            theta_dot_FI = theta_dot_FI_1
            theta_dot_FI[1,:] = theta_dot_FI_2[0,:]
            b = zeros((2,1))
            b[0,0] = self.theta_delta_1
            b[1,0] = self.theta_delta_2

            # Find the weights for theta_dot
            # Least squares fit
            # l2 regularization. Penalize all but the first weight
            regularizer = zeros((2,2))
            regularizer[1,1] = self.theta_dot_regularizer
            weights = matrix(linalg.lstsq(theta_dot_FI, b)[0].flatten())
            weights = asarray(weights).flatten().tolist()
            print 'theta_dot weights', weights
            # Put the weights in a message and publish
            msg_kinematics_weights = KinematicsWeights()
            msg_kinematics_weights.weights = weights
            self.pub_theta_dot_kinematics_weights.publish(msg_kinematics_weights)
        else:
            print 'could not compute theta_dot weights'

        if os.path.isfile(self.v_filename_1):
            v_FI_1 = genfromtxt(self.v_filename_1)
            v_FI = zeros_like(v_FI_1)
            v_FI[0,:] = v_FI_1[1,:]
            b = zeros((2,1))
            b[0,0] = self.distance_1
            b[1,0] = 0

            # Find the weights for v
            # Least squares fit
            # l2 regularization. Penalize all but the first weight
            regularizer = zeros((2,2))
            regularizer[1,1] = self.v_regularizer
            weights = matrix(linalg.lstsq(v_FI, b)[0].flatten())
            weights = asarray(weights).flatten().tolist()
            print 'v weights', weights
            # Put the weights in a message and publish
            msg_kinematics_weights = KinematicsWeights()
            msg_kinematics_weights.weights = weights
            self.pub_v_kinematics_weights.publish(msg_kinematics_weights)
        else:
            print 'could not compute v weights'
            

        rospy.loginfo("[%s] has started", self.node_name)


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('trajectory_learning_node', anonymous=False)
    trajectory_learning_node = TrajectoryLearningNode()
    rospy.spin()
