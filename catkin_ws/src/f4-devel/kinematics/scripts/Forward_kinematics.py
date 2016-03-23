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

	# compute forward kinematics and integrate for a dt long step
	# apart from theta dot and v this function returns relative
	# bearing (theta_delta) and distance (chord) from the starting point
	def evaluate_and_integrate(self, d_L, d_R, dt):
		theta_dot, v = self.evaluate(d_L, d_R)
		theta_delta = theta_dot*dt
		if (theta_delta%(2*pi)) < 0.000001:
			# straight line
			chord = v*dt
		else:
			# find chord length
			s = v*dt
			r = s/theta_delta
			chord = 2*r*sin(theta_delta/2.0)

		return [theta_dot, v, theta_delta, chord]

	# propagate forward kinematics. Find the new pose given current
	# pose, as well as relative bearing (theta_delta) and distance (chord)
	# x axis points forward. y axis points to the left
	def propagate(self, theta, x, y, theta_delta, chord):
		theta_res = theta + theta_delta
		x_res = x + cos(theta_res)*chord
		y_res = y + sin(theta_res)*chord
		return[theta_res, x_res, y_res]
