#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Linear_learner import Linear_learner
from Forward_kinematics import Forward_kinematics


learner = Linear_learner("./theta_dot_fi_function", "./v_fi_function", "./theta_dot_weights_file", "./v_weights_file")
learner.fit_theta_dot_from_file("./training_set")
learner.fit_v_from_file("./training_set")
kinematics = Forward_kinematics("./theta_dot_fi_function", "./v_fi_function", "./theta_dot_weights_file", "./v_weights_file")
theta_dot, v = kinematics.evaluate(matrix([0]), matrix([0]))
print 'd_L: 0, d_R: 0', 'theta_dot:', theta_dot, 'v:', v
theta_dot, v = kinematics.evaluate(matrix([0]), matrix([1]))
print 'd_L: 0, d_R: 1', 'theta_dot:', theta_dot, 'v:', v
theta_dot, v = kinematics.evaluate(matrix([1]), matrix([0]))
print 'd_L: 1, d_R: 0', 'theta_dot:', theta_dot, 'v:', v
theta_dot, v = kinematics.evaluate(matrix([1]), matrix([1]))
print 'd_L: 1, d_R: 1', 'theta_dot:', theta_dot, 'v:', v

		#print 'd_L:', d_L
		#print 'd_R:', d_R
		#print 'dt:', dt
		#print 'x_axis_pose_delta:', x_axis_pose_delta
		#print 'y_axis_pose_delta:', y_axis_pose_delta
		#print 'theta_angle_pose_delta:', theta_angle_pose_delta
		#print 'theta_dot:', theta_dot
		#print 'abs_theta_angle_pose_delta:', abs_theta_angle_pose_delta
		#print 'c:', c
		#print 'r:', r
		#print 's:', s
		#print 'v:', v
		#print 'Fi_v:', Fi_v
		#print 'Fi_theta_dot:', Fi_theta_dot
		#print 'v_weights:',v_weights
		#print 'theta_dot_weights:',theta_dot_weights
