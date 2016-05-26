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

    def factorWeights(self, weights):
        return [0.0, weights[0,0], weights[0,0]]

# the simplest single feature fi function for v: difference between d_L and d_R
class Duty_fi_theta_dot_naive(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        return d_L-d_R

    def factorWeights(self, weights):
        return [0.0, weights[0,0], -weights[0,0]]

# constant feature and sum of d_L and d_R
class Duty_fi_v_simple_linear(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        return concatenate((ones_like(d_L), d_L+d_R), axis=1)

    def factorWeights(self, weights):
        return [weights[0,0], weights[0,1], weights[0,1]]

# constant feature and difference between d_L and d_R
class Duty_fi_theta_dot_simple_linear(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        return concatenate((ones_like(d_L), d_L-d_R), axis=1)

    def factorWeights(self, weights):
        return [weights[0,0], weights[0,1], -weights[0,1]]

# constant feature, d_L, and d_R
class Duty_fi_linear(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        return concatenate((ones_like(d_L), concatenate((d_L, d_R), axis=1)), axis=1)

    def factorWeights(self, weights):
        return [weights[0,0], weights[0,1], weights[0,2]]

# d_L, and d_R
class Duty_fi_linear_no_constant(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        #print 'd_L', d_L, 'd_R', d_R
        return concatenate((d_L, d_R), axis=1)

    def factorWeights(self, weights):
        return [0.0, weights[0,0], weights[0,1]]

# constant feature and sum of d_L and d_R
class Duty_fi_v_compound_linear(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        return concatenate((d_L+d_R, d_L), axis=1)

    def factorWeights(self, weights):
        return [0.0, weights[0,0], weights[0,0]+weights[0,1]]

# constant feature and difference between d_L and d_R
class Duty_fi_theta_dot_compound_linear(Duty_fi_function):
    def computeFi(self, d_L, d_R):
        #print 'd_L', d_L, 'd_R', d_R
        return concatenate((d_L-d_R, d_L), axis=1)

    def factorWeights(self, weights):
        return [0.0, weights[0,0], -weights[0,0]+weights[0,1]]
