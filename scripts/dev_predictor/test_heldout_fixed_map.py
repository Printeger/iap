import unittest
import numpy as np
from heldout_fixed_map import grouped, position_metric, solve


class HeldoutMeasurementGeometry(unittest.TestCase):
    def test_unobserved_attitude_does_not_hide_observable_position(self):
        A = np.pad(np.eye(3), ((0, 0), (0, 3)))
        self.assertTrue(position_metric(A, 5.)['valid'])
        result = solve(A, np.array([.1, -.2, .3]), np.zeros(6), np.array([1., 0., 0.]))
        self.assertTrue(result['valid'])
        self.assertEqual(result['unobservable_nuisance_directions'], 3)
        np.testing.assert_allclose(result['increment'][:3], [.1, -.2, .3])

    def test_shared_pose_nuisance_cannot_be_conditioned_away(self):
        c = np.array([1., 2., 3.])
        n = np.eye(3)
        A = np.column_stack((n, np.cross(np.tile(c, (3, 1)), n)))
        self.assertFalse(position_metric(A, 5.)['valid'])
        self.assertFalse(solve(A, np.ones(3), np.zeros(6), np.array([1., 0., 0.]))['valid'])

    def test_repeated_support_representation_does_not_add_information(self):
        A = np.eye(6); b = np.arange(6, dtype=float); w = np.arange(1, 7, dtype=float)
        keys = np.column_stack((np.arange(6), np.zeros((6, 3), dtype=int)))
        a, y = grouped(A, b, w, keys)
        d, z = grouped(np.tile(A, (3, 1)), np.tile(b, 3), np.tile(w, 3), np.tile(keys, (3, 1)))
        np.testing.assert_allclose(d.T@d, a.T@a, atol=1e-12)
        np.testing.assert_allclose(d.T@z, a.T@y, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
