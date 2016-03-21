#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Duty_fi_function import *


class Linear_learner(object):
	def __init__(self, fi_functions_file, v_weights_file, theta_dot_weights_file):
		# select fi functions dynamically from configuration file
		with open(fi_functions_file, 'r') as fi_functions_file:
			fi_functions=fi_functions_file.readlines()
		fi_v_function = fi_functions[0].replace('\n', '')
		fi_theta_dot_function = fi_functions[1].replace('\n', '')
		self.fi_v_function = globals()[fi_v_function]()
		self.fi_theta_dot_function = globals()[fi_theta_dot_function]()

		# save the name of the weights file
		self.v_weights_file = v_weights_file
		self.theta_dot_weights_file = theta_dot_weights_file


	def fitFromFile(self, filename):
		training_set = genfromtxt(filename)
		self.fit(training_set)


	def fit(self, training_set):
		# extract motor duties, dt, and pose delta from the training set
		d_L = training_set[:, [0]]
		d_R = training_set[:, [1]]
		dt = training_set[:, [2]]
		x_axis_pose_delta = training_set[:, [3]]
		y_axis_pose_delta = training_set[:, [4]]
		# we assume that theta_angle_pose_delta is expressed in radians
		# and that |theta_angle_pose_delta| is less than pi
		theta_angle_pose_delta = training_set[:, [5]]

		theta_dot = theta_angle_pose_delta/dt
		# a 0 angle (or one that is too close to 0) would cause a division
		# by 0 for r and cause problems in calculating s
		abs_theta_angle_pose_delta = maximum(abs(theta_angle_pose_delta),0.00001)
		c = sqrt(pow(x_axis_pose_delta, 2) + pow(y_axis_pose_delta, 2))
		r = c/(2*sin(abs_theta_angle_pose_delta/2))
		s = abs_theta_angle_pose_delta*r
		v = s/dt

		# compute features for v and theta_dot
		Fi_v = self.fi_v_function.computeFi(d_L, d_R)
		Fi_theta_dot = self.fi_theta_dot_function.computeFi(d_L, d_R)

		# Find the weights for v and theta_dot.
		# We use a simple least squares fit.
		v_weights = linalg.lstsq(Fi_v, v)[0]
		theta_dot_weights = linalg.lstsq(Fi_theta_dot, theta_dot)[0]

		# save weights for v and theta_dot
		savetxt(self.v_weights_file, v_weights)
		savetxt(self.theta_dot_weights_file, theta_dot_weights)

learner = Linear_learner("./fi_functions", "./v_weights_file", "./theta_dot_weights_file")
learner.fitFromFile("./training_set")

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
