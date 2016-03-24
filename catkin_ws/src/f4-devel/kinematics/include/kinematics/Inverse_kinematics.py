#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
from scipy.linalg import solve
import csv
from Duty_fi_function import *

class Inverse_kinematics(object):
	def __init__(self, theta_dot_fi_function_file, v_fi_function_file, theta_dot_weights_file, v_weights_file):
		
		# select fi functions dynamically from configuration file
		with open(theta_dot_fi_function_file, 'r') as fi_functions_file:
			fi_functions=fi_functions_file.readlines()
		fi_theta_dot_function = fi_functions[0].replace('\n', '')
		with open(v_fi_function_file, 'r') as fi_functions_file:
			fi_functions=fi_functions_file.readlines()
		fi_v_function = fi_functions[0].replace('\n', '')
		fi_theta_dot_function = globals()[fi_theta_dot_function]()
		fi_v_function = globals()[fi_v_function]()

		# input weights from file
		theta_dot_weights = matrix(genfromtxt(theta_dot_weights_file))
		v_weights = matrix(genfromtxt(v_weights_file))

		# set up the liner system to be solved
		# currently this only works for up to three features per fi function
		self.A = zeros((3,3))
		self.A[0,0] = 1
		self.A[1,:] = fi_theta_dot_function.factorWeights(theta_dot_weights)
		self.A[2,:] = fi_v_function.factorWeights(v_weights)
		self.b = ones((3,1))

	# compute inverse kinematics. (d_L and d_R from theta_dot and v)
	def evaluate(self, theta_dot, v):
		self.b[1] = theta_dot
		self.b[2] = v
		x = solve(self.A, self.b)
		d_L = clip(x[1,0], -1.0, 1.0)
		d_R = clip(x[2,0], -1.0, 1.0)
		return [d_L, d_R]
