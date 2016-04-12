#!/usr/bin/env python
import unittest
import numpy as np
from kinematics.Inverse_kinematics import *

class TestInverseKinematics(unittest.TestCase):
    def test_with_linear_fi(self):
        ik = Inverse_kinematics('Duty_fi_linear_no_constant', 'Duty_fi_linear_no_constant', np.matrix([-1, 1]), np.matrix([1,1]))

        dL, dR = ik.evaluate(np.matrix([0]), np.matrix([0]))
        self.assertAlmostEqual(dL, 0)
        self.assertAlmostEqual(dR, 0)


        dL, dR = ik.evaluate(np.matrix([0]), np.matrix([2]))
        self.assertAlmostEqual(dL, 1)
        self.assertAlmostEqual(dR, 1)

        dL, dR = ik.evaluate(np.matrix([-2]), np.matrix([0]))
        self.assertAlmostEqual(dL, 1)
        self.assertAlmostEqual(dR, -1)

        dL, dR = ik.evaluate(np.matrix([2]), np.matrix([0]))
        self.assertAlmostEqual(dL, -1)
        self.assertAlmostEqual(dR, 1)

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun('kinematics', 'test_inverse_kinematics', TestInverseKinematics)
    unittest.main()
