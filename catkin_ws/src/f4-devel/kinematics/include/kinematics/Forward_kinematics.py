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

    # integrate for a dt long step. This function returns relative
    # bearing (theta_delta) and distance (chord) from the starting point
    def integrate(self, theta_dot, v, dt):
        theta_delta = theta_dot*dt
        if (theta_delta%(2*pi)) < 0.000001:
            # straight line
            chord = v*dt
        else:
            # find chord length
            s = v*dt
            r = s/theta_delta
            chord = 2*r*sin(theta_delta/2.0)

        return [theta_delta, chord]

    # propagate forward kinematics. Find the new pose given current
    # pose, as well as relative bearing (theta_delta) and distance (chord)
    # x axis points forward. y axis points to the left for theta==0
    def propagate(self, theta, x, y, theta_delta, chord):
        theta_res = theta + theta_delta
        x_res = x + cos(theta_res)*chord
        y_res = y + sin(theta_res)*chord
        return[theta_res, x_res, y_res]
