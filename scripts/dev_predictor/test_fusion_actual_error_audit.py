import unittest
import numpy as np
from fusion_actual_error_audit import posterior_at_state


class PosteriorProjectionTest(unittest.TestCase):
    def test_local_pose_translation_and_velocity_cross_terms(self):
        # A rotated local translation and correlated world velocity must keep
        # the signed p-v term. Omitting it produces a different covariance.
        angle = .6
        r = np.array([[np.cos(angle), -np.sin(angle), 0.],
                      [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        pose = np.eye(4);pose[:3, :3] = r
        rng = np.random.default_rng(61)
        a = rng.normal(size=(25, 25));cov = a@a.T
        transform = np.eye(4)
        meta = {'reference_time_s': 11.,
                'coordinates': {'T_map_world': transform.ravel().tolist()},
                'postopt_evidence': {'tangent_dimensions': [6, 3, 6, 3, 3, 2, 2],
                    'state_stamp': 10., 'joint_covariance_row_major': cov.ravel().tolist(),
                    'linearization_means': pose.ravel().tolist(),
                    'optimized_means': pose.ravel().tolist()+[1., 2., 3.]}}
        position, diagnostic, shift = posterior_at_state(meta)
        expected = r@cov[3:6, 3:6]@r.T
        cross = r@cov[3:6, 6:9]
        np.testing.assert_allclose(position, expected)
        np.testing.assert_allclose(diagnostic, expected+cov[6:9, 6:9]+cross+cross.T)
        self.assertGreater(np.linalg.norm(diagnostic-expected-cov[6:9, 6:9]), 1.)
        np.testing.assert_allclose(shift, [1., 2., 3.])


if __name__ == '__main__':
    unittest.main()
