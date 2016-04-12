#!/usr/bin/env python
import unittest, rosunit
from rostest_example.Quacker import *

class QuackerTester(unittest.TestCase):
    def test_quacker_default_quack(self):
        quacker = Quacker()
        self.assertEqual(quacker.quack, "Quack!")

    def test_get_quack_string(self):
        quacker = Quacker("Quack!")
        msg = quacker.get_quack_string(3)
        self.assertEqual(msg, "Quack! Quack! Quack!")

    def test_rounded_average(self):
        quacker = Quacker()
        x = quacker.rounded_mean([1, 1, 2, 3])  # Average is 7/4 ~ 2
        self.assertEqual(x,2)

if __name__ == '__main__':
    rosunit.unitrun('rostest_example', 'quacker_tester', QuackerTester)
