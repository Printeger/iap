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

    def test_unknown_cross_correlation_cauchy_bound(self):
        rng = np.random.default_rng(41021)
        for _ in range(50):
            a, b = rng.normal(size=(5, 5)), rng.normal(size=(7, 7))
            u, _, vt = np.linalg.svd(rng.normal(size=(5, 7)), full_matrices=False)
            correlation = u @ np.diag(rng.uniform(-1., 1., 5)) @ vt
            ga, lb = a@a.T, b@b.T
            cross = a @ correlation @ b.T
            covariance = np.block([[ga, cross], [cross.T, lb]])
            upper = 2*np.block([[ga, np.zeros((5, 7))], [np.zeros((7, 5)), lb]])
            self.assertGreater(np.linalg.eigvalsh(upper-covariance).min(), -1e-10)

    def test_unbounded_map_alignment_cannot_be_absolute_lidar_precision(self):
        # A perfectly known relative pose does not measure absolute map drift.
        rows = np.c_[np.eye(3), -np.eye(3)]
        np.testing.assert_allclose(position_reference(rows), 0., atol=1e-14)
        covariances = []
        for sigma in (.01, .1, 1., 10.):
            prior = np.c_[np.zeros((3, 3)), np.eye(3)/sigma]
            info = position_reference(np.vstack((rows, prior)))
            covariances.append(np.linalg.inv(info))
        for first, second in zip(covariances, covariances[1:]):
            self.assertGreaterEqual(np.linalg.eigvalsh(second-first).min(), 0.)

    def test_free_attitude_covariance_exceeds_conditional_covariance(self):
        rng = np.random.default_rng(21)
        rows = rng.normal(size=(100, 6))
        conditional = np.linalg.inv(rows[:, :3].T@rows[:, :3])
        marginal = np.linalg.inv(position_reference(rows))
        np.testing.assert_allclose(marginal, np.linalg.inv(rows.T@rows)[:3, :3], rtol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(marginal-conditional).min(), 0.)


if __name__ == '__main__':
    unittest.main()
