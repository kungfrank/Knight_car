#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Duty_fi_function import *
from scipy.linalg import solve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Linear_learner(object):
    def __init__(self, theta_dot_fi_function_name, v_fi_function_name, theta_dot_regularizer, v_regularizer):
        self.fi_theta_dot_function = globals()[theta_dot_fi_function_name]()
        self.fi_v_function = globals()[v_fi_function_name]()
        self.theta_dot_regularizer = theta_dot_regularizer
        self.v_regularizer = v_regularizer

    # fit theta_dot from a training set. file filename must contain a matrix whose
    # first four coloumns are the following:
    # duty_Left, duty_Right, dt, theta_angle_pose_delta
    def fit_theta_dot_from_file(self, filename):
        training_set = genfromtxt(filename)
        # extract motor duties, dt, and pose delta from the training set
        d_L = training_set[:, [0]]
        d_R = training_set[:, [1]]
        dt = training_set[:, [2]]
        theta_angle_pose_delta = training_set[:, [3]]
        return self.fit_theta_dot(d_L, d_R, dt, theta_angle_pose_delta)

    # fit v from a training set. file filename must contain a matrix whose
    # first six coloumns are the following:
    # duty_Left, duty_Right, dt, theta_angle_pose_delta, x_axis_pose_delta, y_axis_pose_delta
    def fit_v_from_file(self, filename):
        training_set = genfromtxt(filename) 
        # extract motor duties, dt, and pose delta from the training set
        d_L = training_set[:, [0]]
        d_R = training_set[:, [1]]
        dt = training_set[:, [2]]
        theta_angle_pose_delta = training_set[:, [3]]
        x_axis_pose_delta = training_set[:, [4]]
        y_axis_pose_delta = training_set[:, [5]]
        return self.fit_v(d_L, d_R, dt, theta_angle_pose_delta, x_axis_pose_delta, y_axis_pose_delta)


    def fit_theta_dot(self, d_L, d_R, dt, theta_angle_pose_delta):
        # we assume that theta_angle_pose_delta is expressed in radians
        # and that |theta_angle_pose_delta| is less than pi
        
        # compute theta_dot
        theta_dot = theta_angle_pose_delta/dt

        # x = d_L + d_R
        # y = theta_dot
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        # ax.set_xlabel('Duty cycle', fontsize=16, fontweight='bold')
        # ax.set_ylabel('theta_dot', fontsize=16, fontweight='bold')
        # plt.scatter(x, y)
        # plt.show(block=False)

        
        # compute features for theta_dot
        Fi_theta_dot = self.fi_theta_dot_function.computeFi(d_L, d_R)

        # Find the weights for theta_dot
        # Least squares fit
        # l2 regularization. Penalize all but the first weight
        regularizer = self.theta_dot_regularizer*len(Fi_theta_dot.T)*eye(len(Fi_theta_dot.T))
        regularizer[0,0] = 0.0
        # Weighted least squares. Each sample is weighted by its dt
        Fi_theta_dot_T_dt = Fi_theta_dot.T*dt.T
        gramian = dot(Fi_theta_dot_T_dt,Fi_theta_dot) + regularizer
        y = dot(Fi_theta_dot_T_dt,theta_dot)
        theta_dot_weights = matrix(linalg.lstsq(gramian, y)[0].flatten())

        return theta_dot_weights


    def fit_v(self, d_L, d_R, dt, theta_angle_pose_delta, x_axis_pose_delta, y_axis_pose_delta):
        # we assume that theta_angle_pose_delta is expressed in radians
        # and that |theta_angle_pose_delta| is less than pi

        # a 0 angle (or one that is too close to 0) would cause a division
        # by 0 for r and cause problems in calculating s
        abs_theta_angle_pose_delta = maximum(abs(theta_angle_pose_delta),0.00001)
        # the sign is used to estimate if we're going forward on backwards
        c = sign(d_L+d_R)*(sqrt(pow(x_axis_pose_delta, 2) + pow(y_axis_pose_delta, 2)))

        r = c/(2*sin(abs_theta_angle_pose_delta/2))
        s = abs_theta_angle_pose_delta*r
        v = s/dt

        # x = d_L + d_R
        # y = v
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # #fig = plt.figure()
        # #ax = fig.add_subplot(111)
        # ax.set_xlabel('d_L', fontsize=16, fontweight='bold')
        # ax.set_ylabel('d_R', fontsize=16, fontweight='bold')
        # ax.set_zlabel('v', fontsize=16, fontweight='bold')
        # plt.scatter(d_L, d_R, y)
        # plt.show(block=False)

        # compute features for v
        Fi_v = self.fi_v_function.computeFi(d_L, d_R)

        # Find the weights for v
        # Least squares fit
        # l2 regularization. Penalize all but the first weight
        regularizer = self.v_regularizer*len(Fi_v.T)*eye(len(Fi_v.T))
        regularizer[0,0] = 0.0
        # Weighted least squares. Each sample is weighted by its dt
        Fi_v_T_dt = Fi_v.T*dt.T
        gramian = dot(Fi_v_T_dt,Fi_v) + regularizer
        y = dot(Fi_v_T_dt,v)
        v_weights = matrix(linalg.lstsq(gramian, y)[0].flatten())

        return v_weights
