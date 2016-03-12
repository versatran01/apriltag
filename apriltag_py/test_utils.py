import unittest2 as unittest
import numpy as np
from apriltag_py.utils import mod2pi
import numpy.testing as nt


class TestUtils(unittest.TestCase):
    def setUp(self):
        self.pi2 = np.pi * 2
        self.pi4 = np.pi * 4
        self.pi = np.pi
        self.pi_2 = np.pi / 2
        self.pi_3 = np.pi / 3
        self.pi_4 = np.pi / 4
        self.pi_6 = np.pi / 6

    def test_mod2pi(self):
        nt.assert_almost_equal(mod2pi(0, 0), 0)
        nt.assert_almost_equal(mod2pi(self.pi2, 0), 0)
        nt.assert_almost_equal(mod2pi(0, self.pi2), 0)
        nt.assert_almost_equal(mod2pi(self.pi4, self.pi2), 0)
        nt.assert_almost_equal(mod2pi(self.pi2, self.pi4), 0)
        nt.assert_almost_equal(mod2pi(self.pi4, 0), 0)
        nt.assert_almost_equal(mod2pi(0, self.pi4), 0)

        nt.assert_almost_equal(mod2pi(-self.pi_3), -self.pi_3)
        nt.assert_almost_equal(mod2pi(self.pi_3, self.pi2 - self.pi_3),
                               self.pi_3 * 2)

        nt.assert_almost_equal(mod2pi(self.pi_3, self.pi_6), self.pi_6)
        nt.assert_almost_equal(mod2pi(self.pi_6, self.pi_3), -self.pi_6)
