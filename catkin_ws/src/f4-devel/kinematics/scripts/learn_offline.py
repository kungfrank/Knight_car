#!/usr/bin/env python
from kinematics.Linear_learner import *
import sys, getopt
import os.path

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"")
    except getopt.GetoptError:
        print 'Usage:\tlearn_offline.py <inputfile>'
        sys.exit(2)

    filename = args[0] if len(args) > 0 and not args[0] == "" else "./training_data.txt"
    if os.path.isfile(filename):
        print "Parsing file: {filename}".format(filename=filename)

        # Calculate the weights
        theta_dot_fi_function = 'Duty_fi_linear_no_constant'
        v_fi_function = 'Duty_fi_linear_no_constant'
        learner = Linear_learner(theta_dot_fi_function, v_fi_function)
        theta_dot_weights = learner.fit_theta_dot_from_file(filename)
        v_weights = learner.fit_v_from_file(filename)
        print "theta_dot_weights:\t[{weight1}, {weight2}]".format(weight1=theta_dot_weights[0,0], weight2=theta_dot_weights[0,1])
        print "v_weights:\t\t[{weight1}, {weight2}]".format(weight1=v_weights[0,0], weight2=v_weights[0,1])
    else:
        print "ERROR: {filename} not found.".format(filename=filename)
        sys.exit(2)

if __name__ == "__main__":
   main(sys.argv[1:])