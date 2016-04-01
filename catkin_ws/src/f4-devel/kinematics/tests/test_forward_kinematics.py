#!/usr/bin/env python
import unittest
from kinematics.Forward_kinematics import *
import numpy as np

class TestForwardKinematics(unittest.TestCase):
    def test_with_linear_fi(self):
        fk = Forward_kinematics('Duty_fi_linear_no_constant', 'Duty_fi_linear_no_constant', np.matrix([-1, 1]), np.matrix([1,1]))

        theta_dot, v = fk.evaluate(np.matrix([0]), np.matrix([0]))
        self.assertAlmostEqual(theta_dot, 0)
        self.assertAlmostEqual(v, 0)

        theta_dot, v = fk.evaluate(np.matrix([1]), np.matrix([1]))
        self.assertAlmostEqual(theta_dot, 0)
        self.assertAlmostEqual(v, 2)

        theta_dot, v = fk.evaluate(np.matrix([1]), np.matrix([-1]))
        self.assertAlmostEqual(theta_dot, -2)
        self.assertAlmostEqual(v, 0)

        theta_dot, v = fk.evaluate(np.matrix([-1]), np.matrix([1]))
        self.assertAlmostEqual(theta_dot, 2)
        self.assertAlmostEqual(v, 0)

    def test_with_naive_fi(self):
        fk = Forward_kinematics('Duty_fi_theta_dot_naive', 'Duty_fi_v_naive', np.matrix([-1]), np.matrix([1]))

        theta_dot, v = fk.evaluate(np.matrix([0]), np.matrix([0]))
        self.assertAlmostEqual(theta_dot, 0)
        self.assertAlmostEqual(v, 0)

        theta_dot, v = fk.evaluate(np.matrix([1]), np.matrix([1]))
        self.assertAlmostEqual(theta_dot, 0)
        self.assertAlmostEqual(v, 2)

        theta_dot, v = fk.evaluate(np.matrix([1]), np.matrix([-1]))
        self.assertAlmostEqual(theta_dot, -2)
        self.assertAlmostEqual(v, 0)

        theta_dot, v = fk.evaluate(np.matrix([-1]), np.matrix([1]))
        self.assertAlmostEqual(theta_dot, 2)
        self.assertAlmostEqual(v, 0)


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun('kinematics', 'test_forward_kinematics', TestForwardKinematics)
    unittest.main()
