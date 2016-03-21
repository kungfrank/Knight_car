#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Duty_fi_function import *

class Forward_kinematics(object):
	def __init__(self, fi_functions_file, v_weights_file, theta_dot_weights_file):
		
		# select fi functions dynamically from configuration file
		with open(fi_functions_file, 'r') as fi_functions_file:
			fi_functions=fi_functions_file.readlines()
		fi_v_function = fi_functions[0].replace('\n', '')
		fi_theta_dot_function = fi_functions[1].replace('\n', '')
		self.fi_v_function = globals()[fi_v_function]()
		self.fi_theta_dot_function = globals()[fi_theta_dot_function]()

		# input weights from file
		self.v_weights = genfromtxt(v_weights_file)
		self.theta_dot_weights = genfromtxt(theta_dot_weights_file)

		print 'fi_v_function:', fi_v_function, 'v_weights:', self.v_weights
		print 'fi_theta_dot_function:', fi_theta_dot_function, 'theta_dot_weights:', self.theta_dot_weights

	# compute forward kinematics. (v and theta_dot from d_L and d_R)
	def evaluate(self, d_L, d_R):
		fi_v = self.fi_v_function.computeFi(d_L, d_R)
		fi_theta_dot = self.fi_theta_dot_function.computeFi(d_L, d_R)
		return [inner(fi_v, self.v_weights), inner(fi_theta_dot, self.theta_dot_weights)]

kinematics = Forward_kinematics("./fi_functions", "./v_weights_file", "./theta_dot_weights_file")
print kinematics.evaluate(matrix([1]), matrix([1]))