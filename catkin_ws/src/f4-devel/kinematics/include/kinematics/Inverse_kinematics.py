#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
from scipy.linalg import solve
import csv
from Duty_fi_function import *

class Inverse_kinematics(object):
    # Inverse_kinematics takes as input the name of the two feature functions to use as well as
    # the weights for theta and v.
    def __init__(self, fi_theta_dot_function_name, fi_v_function_name, theta_dot_weights, v_weights):
        # load fi_functions (notice that they are used only once, so we don't need to save them)
        fi_theta_dot_function = globals()[fi_theta_dot_function_name]()
        fi_v_function = globals()[fi_v_function_name]()

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
        #x = solve(self.A, self.b)
        #print x
        #d_L = x[1,0]
        #d_R = x[2,0]
        x = linalg.lstsq(self.A, self.b)[0].flatten()
        d_L = x[1]
        d_R = x[2]
        # TODO: add clipping again to limit 
        #d_L = clip(x[1,0], -1.0, 1.0)
        #d_R = clip(x[2,0], -1.0, 1.0)
        return [d_L, d_R]
