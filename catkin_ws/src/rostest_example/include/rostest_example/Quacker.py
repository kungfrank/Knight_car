#!/usr/bin/env python
import numpy as np

class Quacker(object):
    def __init__(self, quack="Quack!"):
        self.quack = quack

    def rounded_mean(self, x):
        # Returns the mean of x, rounded to the nearest integer
        return np.round(np.mean(np.array(x)))

    def get_quack_string(self, n):
        # Returns a string of n quacks based on the value in Quacker.quack
        return ' '.join([self.quack]*n)