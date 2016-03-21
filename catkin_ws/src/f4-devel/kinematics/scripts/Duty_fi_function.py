#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *

class Duty_fi_function(object):
	def __init__(self):
		pass

	def computeFi(self, d_L, d_R):
		return ones_like(d_L)

# the simplest single feature fi function for v: sum of d_L and d_R
class Duty_fi_v_naive(Duty_fi_function):
	def computeFi(self, d_L, d_R):
		return d_L+d_R

# the simplest single feature fi function for v: difference between d_L and d_R
class Duty_fi_theta_dot_naive(Duty_fi_function):
	def computeFi(self, d_L, d_R):
		return d_L-d_R

# constant feature and sum of d_L and d_R
class Duty_fi_v_simple_linear(Duty_fi_function):
	def computeFi(self, d_L, d_R):
		return concatenate((ones_like(d_L), d_L+d_R), axis=1)

# constant feature and difference between d_L and d_R
class Duty_fi_theta_dot_simple_linear(Duty_fi_function):
	def computeFi(self, d_L, d_R):
		return concatenate((ones_like(d_L), d_L-d_R), axis=1)

# constant feature, d_L, and d_R
class Duty_fi_linear(Duty_fi_function):
	def computeFi(self, d_L, d_R):
		return concatenate((ones_like(d_L), concatenate((d_L, d_R), axis=1)), axis=1)
