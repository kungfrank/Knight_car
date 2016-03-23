#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Duty_fi_function import *


class Linear_learner(object):
	def __init__(self, theta_dot_fi_function_file, v_fi_function_file, theta_dot_weights_file, v_weights_file):
		# select fi functions dynamically from configuration file
		with open(theta_dot_fi_function_file, 'r') as fi_functions_file:
			fi_functions=fi_functions_file.readlines()
		fi_theta_dot_function = fi_functions[0].replace('\n', '')
		with open(v_fi_function_file, 'r') as fi_functions_file:
			fi_functions=fi_functions_file.readlines()
		fi_v_function = fi_functions[0].replace('\n', '')
		self.fi_theta_dot_function = globals()[fi_theta_dot_function]()
		self.fi_v_function = globals()[fi_v_function]()

		# save the name of the weights files
		self.theta_dot_weights_file = theta_dot_weights_file
		self.v_weights_file = v_weights_file

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
		self.fit_theta_dot(d_L, d_R, dt, theta_angle_pose_delta)

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
		self.fit_v(d_L, d_R, dt, theta_angle_pose_delta, x_axis_pose_delta, y_axis_pose_delta)


	def fit_theta_dot(self, d_L, d_R, dt, theta_angle_pose_delta):
		# we assume that theta_angle_pose_delta is expressed in radians
		# and that |theta_angle_pose_delta| is less than pi
		
		# compute theta_dot
		theta_dot = theta_angle_pose_delta/dt
		
		# compute features for theta_dot
		Fi_theta_dot = self.fi_theta_dot_function.computeFi(d_L, d_R)

		# Find the weights for theta_dot
		# We use a simple least squares fit
		theta_dot_weights = linalg.lstsq(Fi_theta_dot, theta_dot)[0]

		# save weights for theta_dot
		savetxt(self.theta_dot_weights_file, theta_dot_weights)

	def fit_v(self, d_L, d_R, dt, theta_angle_pose_delta, x_axis_pose_delta, y_axis_pose_delta):
		# we assume that theta_angle_pose_delta is expressed in radians
		# and that |theta_angle_pose_delta| is less than pi

		# a 0 angle (or one that is too close to 0) would cause a division
		# by 0 for r and cause problems in calculating s
		abs_theta_angle_pose_delta = maximum(abs(theta_angle_pose_delta),0.00001)
		c = sqrt(pow(x_axis_pose_delta, 2) + pow(y_axis_pose_delta, 2))
		r = c/(2*sin(abs_theta_angle_pose_delta/2))
		s = abs_theta_angle_pose_delta*r
		v = s/dt

		# compute features for v
		Fi_v = self.fi_v_function.computeFi(d_L, d_R)

		# Find the weights for v
		# We use a simple least squares fit
		v_weights = linalg.lstsq(Fi_v, v)[0]

		# save weights for v and theta_dot
		savetxt(self.v_weights_file, v_weights)
