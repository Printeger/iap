"""Synthetic physical laws only; no solver covariance is treated as truth."""
import unittest
import numpy as np
from local_increment_validation import transport_retained_error_law


class BoundedResponse(unittest.TestCase):
    def test_shared_datum_and_local_bias_survive_correctly(self):
        # Start/end share datum d; increment has physical local bias .02.
        mean, cov = transport_retained_error_law([3., 3., 0.],
            [[4., 4., 0.], [4., 4., 0.], [0., 0., .01]], 1, [[1.]], [[1.]], [.02])
        D = np.array([-1., 1.])
        self.assertAlmostEqual(D @ mean, .02)
        self.assertAlmostEqual(D @ cov @ D, .01)

    def test_reused_observation_cross_relation_is_required(self):
        # State x=e; same measurement innovation e is reused with gain -1.
        _, correct = transport_retained_error_law([0., 0.], [[1., 1.], [1., 1.]],
                                                    0, [[1.]], [[-1.]], [0.])
        _, wrong = transport_retained_error_law([0., 0.], np.eye(2),
                                                  0, [[1.]], [[-1.]], [0.])
        self.assertAlmostEqual(correct[0, 0], 0.)
        self.assertAlmostEqual(wrong[0, 0], 2.)

    def test_bounded_recursion_matches_full_latent_history(self):
        # Three independent physical acquisition errors; no archive in state.
        B = np.array([[1., 0., 0.], [1., 1., 0.]])
        mu = np.array([.3, -.1, .04]);Q = np.diag([2., .5, .2])
        old_mean, old_cov = B @ mu, B @ Q @ B.T
        joint_mean = np.r_[old_mean, mu[2]]
        joint_cov = np.zeros((3, 3));joint_cov[:2, :2] = old_cov;joint_cov[2, 2] = Q[2, 2]
        mean, cov = transport_retained_error_law(joint_mean, joint_cov, 1, [[.7]], [[.2]], [.01])
        full_response = np.array([[1., 0., 0.], [.7, .7, .2]])
        np.testing.assert_allclose(mean, full_response @ mu + [0., .01])
        np.testing.assert_allclose(cov, full_response @ Q @ full_response.T)
        self.assertEqual(cov.shape, (2, 2))

    def test_same_marginals_different_temporal_relation_change_increment(self):
        D = np.array([-1., 1.])
        _, shared = transport_retained_error_law([0., 0.], [[1., 1.], [1., 1.]], 1, [[1.]], np.zeros((1, 0)), [0.])
        _, independent = transport_retained_error_law([0., 0.], np.eye(2), 1, [[1.]], np.zeros((1, 0)), [0.])
        self.assertAlmostEqual(D @ shared @ D, 0.)
        self.assertAlmostEqual(D @ independent @ D, 2.)

    def test_missing_law_invalid_blocks_and_budget_fail_without_fallback(self):
        with self.assertRaisesRegex(ValueError, 'UNAVAILABLE'):
            transport_retained_error_law(None, None, 1, [[1.]], [[1.]], [0.])
        with self.assertRaisesRegex(ValueError, 'BLOCKS_MISSING'):
            transport_retained_error_law([0., 0., 0.], np.eye(2), 1, [[1.]], [[1.]], [0.])
        with self.assertRaisesRegex(ValueError, 'NOT_PSD'):
            transport_retained_error_law([0., 0.], [[1., 2.], [2., 1.]], 1, [[1.]], np.zeros((1, 0)), [0.])
        with self.assertRaisesRegex(ValueError, 'BUDGET_EXCEEDED'):
            transport_retained_error_law([0., 0.], np.eye(2), 1, [[1.]], np.zeros((1, 0)), [0.], state_dimension_budget=1)


if __name__ == '__main__':
    unittest.main()
