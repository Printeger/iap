import unittest
import numpy as np
from fusion_scientific_audit import position_reference, gnss_reference, skew


class IndependentReferenceTest(unittest.TestCase):
    def test_shared_attitude_must_be_eliminated_after_sources_join(self):
        # Each source confounds a different position with the same attitude.
        # Summing individual marginals loses their complementary constraint.
        a = np.array([[1., 0., 0., 1.]])
        b = np.array([[0., 1., 0., 1.]])
        separately = position_reference(a) + position_reference(b)
        together = position_reference(np.vstack((a, b)))
        np.testing.assert_allclose(separately, 0., atol=1e-14)
        np.testing.assert_allclose(together[:2, :2], [[.5, -.5], [-.5, .5]])

    def test_unobserved_nuisance_does_not_remove_observed_position(self):
        np.testing.assert_allclose(position_reference(np.c_[np.eye(3), np.zeros((3, 3))]), np.eye(3))

    def test_antenna_jacobian_matches_rotation_finite_difference(self):
        lever = np.array([.4, -.2, .1])
        for axis in np.eye(3):
            h = 1e-7
            k = skew(axis)
            r = np.eye(3) + np.sin(h)*k + (1-np.cos(h))*(k@k)
            np.testing.assert_allclose((r@lever-lever)/h, -skew(lever)@axis, atol=3e-8)

    def test_constellation_clocks_do_not_create_position_information(self):
        rows = [{'constellation': c, 'los_map': [1., 0., 0.], 'final_sigma_m': 2.}
                for c in ('G', 'C')]
        np.testing.assert_allclose(position_reference(gnss_reference(rows, np.zeros(3))), 0., atol=1e-14)


if __name__ == '__main__':
    unittest.main()
