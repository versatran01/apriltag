import unittest2 as unittest
import numpy as np
from apriltag_py.utils import mod2pi, angle_dist, angle_union
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
        nt.assert_almost_equal(mod2pi(self.pi, -self.pi), 0)

        nt.assert_almost_equal(mod2pi(-self.pi_3), -self.pi_3)
        nt.assert_almost_equal(mod2pi(self.pi_3, self.pi2 - self.pi_3),
                               self.pi_3 * 2)

        nt.assert_almost_equal(mod2pi(self.pi_3, self.pi_6), self.pi_6)
        nt.assert_almost_equal(mod2pi(self.pi_6, self.pi_3), -self.pi_6)

    def test_angle_dist(self):
        nt.assert_almost_equal(angle_dist(0, 0), 0)
        nt.assert_almost_equal(angle_dist(self.pi2, 0), 0)
        nt.assert_almost_equal(angle_dist(0, self.pi2), 0)

        nt.assert_almost_equal(angle_dist(-self.pi_3), self.pi_3)
        nt.assert_almost_equal(angle_dist(self.pi_3, self.pi2 - self.pi_3),
                               self.pi_3 * 2)

        nt.assert_almost_equal(angle_dist(self.pi_3, self.pi_6), self.pi_6)
        nt.assert_almost_equal(angle_dist(self.pi_6, self.pi_3), self.pi_6)
    
    def test_angle_union(self):
        r1 = (np.pi / 6, np.pi * 11 / 6)
        r2 = (np.pi * 2 / 6, np.pi * 10 / 6)
        nt.assert_almost_equal(angle_union(r1, r2)[0], np.pi * 2 / 3)
        
        r3 = (np.pi / 6, np.pi * 4 / 6)
        r4 = (np.pi * 2/ 6, np.pi * 5 / 6)
        nt.assert_almost_equal(angle_union(r3, r4)[0], np.pi * 2 / 3)
        
        r5 = (np.pi / 4, np.pi / 2)
        r6 = (np.pi / 2, np.pi * 3 / 4)
        nt.assert_almost_equal(angle_union(r5, r6)[0], np.pi / 2)