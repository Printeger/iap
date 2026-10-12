import unittest
import numpy as np
from local_increment_validation import condense_physical_score_law


class NativeScoreTransport(unittest.TestCase):
    def test_shared_alias_cross_and_bias_follow_native_elimination(self):
        H = np.array([[4., 2.], [2., 3.]])
        # One controlled physical perturbation affects the frozen alias and
        # both score blocks. It is a fixture law, not real source calibration.
        forcing = np.array([1., 2., 3.])
        mean, covariance, condensed, response = condense_physical_score_law(
            H, 1, 1, forcing*.4, np.outer(forcing, forcing)*.09)
        np.testing.assert_allclose(mean, [.4, .8])
        np.testing.assert_allclose(covariance, [[.09, .18], [.18, .36]])
        np.testing.assert_allclose(condensed, [[2.]])
        for shift in (-.001, .001):
            native_full = np.linalg.solve(H, forcing[1:]*shift)[1:]
            retained = np.linalg.solve(condensed, (response @ (forcing*shift))[1:])
            np.testing.assert_allclose(retained, native_full, atol=1e-15)

    def test_same_loss_does_not_supply_missing_physical_law(self):
        H = np.array([[4., 2.], [2., 3.]])
        with self.assertRaisesRegex(ValueError, 'PHYSICAL_JOINT_SCORE_LAW_UNAVAILABLE'):
            condense_physical_score_law(H, 1, 1, None, None)
        _, coupled, _, _ = condense_physical_score_law(H, 1, 1, np.zeros(3), np.ones((3,3)))
        _, independent, _, _ = condense_physical_score_law(H, 1, 1, np.zeros(3), np.eye(3))
        self.assertNotEqual(coupled[0,1], independent[0,1])
        self.assertNotEqual(coupled[1,1], independent[1,1])

    def test_repeated_condensation_matches_joint_physical_control(self):
        rng = np.random.default_rng(491)
        A = rng.normal(size=(7,7)); H = A.T @ A + np.eye(7)
        law = rng.normal(size=(9,3)); bias = law @ [.3, -.1, .2]
        m, C, reduced, first = condense_physical_score_law(H, 2, 2, bias, law @ law.T)
        m2, C2, reduced2, second = condense_physical_score_law(reduced, 2, 2, m, C)
        direct = second @ first
        np.testing.assert_allclose(m2, direct @ bias, atol=1e-13)
        np.testing.assert_allclose(C2, direct @ law @ law.T @ direct.T, atol=1e-13)
        for stimulus in law.T:
            full = np.linalg.solve(H, stimulus[2:])[4:]
            compressed = np.linalg.solve(reduced2, (direct @ stimulus)[2:])
            np.testing.assert_allclose(compressed, full, atol=1e-13)

    def test_singular_unqualified_and_out_of_budget_fail(self):
        with self.assertRaisesRegex(ValueError, 'SINGULAR'):
            condense_physical_score_law([[0.,0.],[0.,1.]],1,0,[0.,0.],np.eye(2))
        with self.assertRaisesRegex(ValueError, 'NOT_PSD'):
            condense_physical_score_law(np.eye(2),1,0,[0.,0.],[[1.,2.],[2.,1.]])
        with self.assertRaisesRegex(ValueError, 'BUDGET'):
            condense_physical_score_law(np.eye(2),1,1,[0.,0.,0.],np.eye(3),score_budget=1)


if __name__ == '__main__':
    unittest.main()
