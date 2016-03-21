#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Duty_fi_function import *

class Forward_kinematics(object):
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

		# input weights from file
		self.theta_dot_weights = genfromtxt(theta_dot_weights_file)
		self.v_weights = genfromtxt(v_weights_file)

	# compute forward kinematics. (theta_dot and v from d_L and d_R)
	def evaluate(self, d_L, d_R):
		fi_theta_dot = self.fi_theta_dot_function.computeFi(d_L, d_R)
		fi_v = self.fi_v_function.computeFi(d_L, d_R)
		return [inner(fi_theta_dot, self.theta_dot_weights).flatten()[0], inner(fi_v, self.v_weights).flatten()[0]]
