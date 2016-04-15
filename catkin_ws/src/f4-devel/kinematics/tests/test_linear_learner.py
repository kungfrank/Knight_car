#!/usr/bin/env python
import unittest
from rospkg import RosPack
import numpy as np
import numpy.testing
from kinematics.Linear_learner import *

class TestLinearLearner(unittest.TestCase):
    # Test removed because it is no longer compatible with the current version of linear_learner
    def donot_test_linear_no_constant(self):
        theta_dot_fi_function = 'Duty_fi_linear_no_constant'
        v_fi_function = 'Duty_fi_linear_no_constant'
        learner = Linear_learner(theta_dot_fi_function, v_fi_function)
        rp = RosPack()
        filepath = rp.get_path('kinematics') + "/tests/test_training_set.txt"
        theta_dot_weights = learner.fit_theta_dot_from_file(filepath)
        v_weights = learner.fit_v_from_file(filepath)

        #np.testing.assert_almost_equal(v_weights, np.matrix([1.0, 0.0]))
        #np.testing.assert_almost_equal(theta_dot_weights, np.matrix([-1, 0.0]))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('kinematics', 'test_linear_learner', TestLinearLearner)
    unittest.main()
