"""Regression checks for physical events, not copies of controller formulas."""
import unittest
import numpy as np
from corrections import OneShotObstacle, contact_ttc


class PhysicalEventTests(unittest.TestCase):
    def test_expired_obstacle_never_returns(self):
        o = OneShotObstacle([60,18],[-.8,-.45],life=60,t_spawn=2)
        for t in np.arange(0,202,.05):
            o.update(float(t))
            if t < 2:
                self.assertFalse(o.is_alive())
            elif t < 62:
                self.assertTrue(o.is_alive())
            else:
                self.assertFalse(o.is_alive())
        self.assertAlmostEqual(o.t0_active,2)

    def test_inside_body_is_current_contact_for_every_velocity(self):
        for v in [[0,0],[-1,0],[1,0],[0,2]]:
            self.assertEqual(contact_ttc(np.array([.5,0]),np.array(v),1),0)

    def test_external_separation_is_not_contact(self):
        self.assertTrue(np.isinf(contact_ttc(np.array([2.,0]),np.array([1.,0]),1)))

    def test_head_on_contact_time(self):
        self.assertAlmostEqual(contact_ttc(np.array([5.,0]),np.array([-2.,0]),1),2)

    def test_triggered_obstacle_is_not_released_by_clock(self):
        o = OneShotObstacle([0,0],[1,0],life=2,trigger=(5,6))
        o.update(10)
        self.assertFalse(o.is_alive())
        o.check_trigger(5.5,10)
        o.update(11)
        self.assertTrue(o.is_alive())
        o.update(12)
        o.check_trigger(5.5,20)
        o.update(20)
        self.assertFalse(o.is_alive())


if __name__ == '__main__':
    unittest.main()
