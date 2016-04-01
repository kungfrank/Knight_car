#!/usr/bin/env python

__author__ = 'jpazis'

from numpy import *
import csv
from Duty_fi_function import *

class Forward_kinematics(object):
    # Forward_kinematics takes as input the name of the two feature functions to use as well as
    # the weights for theta and v.
    def __init__(self, fi_theta_dot_function_name, fi_v_function_name, theta_dot_weights, v_weights):
        # save fi_functions
        self.fi_theta_dot_function = globals()[fi_theta_dot_function_name]()
        self.fi_v_function = globals()[fi_v_function_name]()

        # save weights
        self.theta_dot_weights = theta_dot_weights
        self.v_weights = v_weights

    # compute forward kinematics. (theta_dot and v from d_L and d_R)
    def evaluate(self, d_L, d_R):
        fi_theta_dot = self.fi_theta_dot_function.computeFi(d_L, d_R)
        fi_v = self.fi_v_function.computeFi(d_L, d_R)
        return [inner(fi_theta_dot, self.theta_dot_weights).flatten()[0], inner(fi_v, self.v_weights).flatten()[0]]

    def integrate(self, theta, x, y, theta_dot, v, dt):
        theta_delta = theta_dot*dt
        theta_res = theta + theta_delta
        if (theta_dot < 0.000001):
            # straight line
            x_res = x+ cos(theta) * v * dt
            y_res = y+ sin(theta) * v * dt
        else:
            # arc of circle, see "Probabilitic robotics"
            v_w_ratio = v / theta_dot
            x_res = x + v_w_ratio * sin(theta_res) - v_w_ratio * sin(theta)
            y_res = y + v_w_ratio * cos(theta) - v_w_ratio * cos(theta_res)
        return[theta_res, x_res, y_res]