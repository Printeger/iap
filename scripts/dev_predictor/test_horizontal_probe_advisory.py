import unittest
import numpy as np

from horizontal_probe_advisory import (PROBES, probe_response, project_information,
                                       svd_probe_matrix, clock_projected_rows,
                                       translation_gauge_information)


class HorizontalProbe(unittest.TestCase):
    def test_left_rotation_finite_difference(self):
        from local_increment_validation import corrected_pose
        for d in PROBES:
            numerical = np.zeros((3, 6))
            for j in range(6):
                step = np.eye(6)[j]*1e-6
                def point(delta):
                    pose = corrected_pose(np.eye(4), delta)
                    return pose[:3, 3]+pose[:3, :3]@d
                numerical[:, j] = (point(step)-point(-step))/2e-6
            np.testing.assert_allclose(numerical, probe_response(d), atol=1e-10)

    def test_symmetric_probes_cancel_cross_not_rotation(self):
        a = np.eye(6)+np.arange(36).reshape(6, 6)/40
        h = a.T@a
        out = project_information(h, 5., 5.)
        c = np.linalg.inv(h)
        expected = c[:3, :3]+sum(probe_response(d)[:, 3:]@c[3:, 3:]@
                                probe_response(d)[:, 3:].T for d in PROBES)/4
        np.testing.assert_allclose(out['projected_matrix'], expected, atol=1e-12)
        m, observed = svd_probe_matrix(a)
        np.testing.assert_allclose(out['projected_matrix'], m, atol=1e-12)
        self.assertTrue(observed.all())

    def test_zero_probe_observable_position_unobserved_rotation(self):
        h = np.diag([2., 3., 4., 0., 0., 0.])
        zero = project_information(h, 5., 5., zero_probe=True)
        self.assertTrue(zero['valid'])
        np.testing.assert_allclose(zero['projected_matrix'], np.diag([.5, 1/3, .25]))
        actual = project_information(h, 5., 5.)
        self.assertFalse(actual['valid'])
        self.assertIsNone(actual['S'])

    def test_component_nullspace_not_zero_risk(self):
        h = np.diag([2., 3., 4., 1., 1., 0.])
        out = project_information(h, 5., 5.)
        self.assertIsNone(out['Q_H'])
        self.assertIsNotNone(out['Q_V'])
        self.assertIsNone(out['S'])

    def test_clock_elimination_source_envelope_and_monotonicity(self):
        g = np.c_[np.eye(6), np.ones(6)]
        l = np.diag([2., 3., 4., 1., 2., 3.])
        a = np.vstack([g, np.c_[l, np.zeros(6)]])/np.sqrt(2)
        pose = clock_projected_rows(a)
        baseline = project_information(pose.T@pose, 5., 5.)
        m, observed = svd_probe_matrix(a)
        np.testing.assert_allclose(m, baseline['projected_matrix'], atol=1e-12)
        self.assertTrue(observed.all())
        improved = clock_projected_rows(np.vstack([g*2, np.c_[l, np.zeros(6)]])/np.sqrt(2))
        better = project_information(improved.T@improved, 5., 5.)
        self.assertLessEqual(better['S'], baseline['S'])
        doubled = project_information(pose.T@pose/4, 5., 5.)
        self.assertAlmostEqual(doubled['S'], 2*baseline['S'])

    def test_original_invalid_and_non_psd_stay_invalid(self):
        for h, status in [(np.eye(6), 'STALE'), (np.zeros((6, 6)), 'VALID'),
                          (-np.eye(6), 'VALID'), (np.full((6, 6), np.nan), 'VALID')]:
            out = project_information(h, 5., 5., input_status=status)
            self.assertFalse(out['valid'])
            self.assertIsNone(out['S'])

    def test_map_translation_gauge_eliminated_without_prior(self):
        a = np.eye(6)+np.arange(36).reshape(6, 6)/40
        h = a.T@a
        gauge = translation_gauge_information(h)
        nuisance = a[:, :3]
        u, _, _ = np.linalg.svd(nuisance, full_matrices=False)
        independent = a-u@(u.T@a)
        np.testing.assert_allclose(gauge, independent.T@independent, atol=1e-12)
        # Finite known-map probe becomes unavailable without GNSS anchoring.
        self.assertTrue(project_information(h/2, 5., 5.)['valid'])
        self.assertFalse(project_information(gauge/2, 5., 5.)['valid'])
        for translation in ([1., 0., 0.], [0., 2., -3.]):
            np.testing.assert_allclose(gauge@np.r_[translation, np.zeros(3)], 0.)
        # A reliable GNSS anchor restores pose/probe observability.
        anchored = gauge+np.diag([1., 1., 1., 0., 0., 0.])
        self.assertTrue(project_information(anchored/2, 5., 5.)['valid'])

    def test_four_source_conditions_are_only_numerical_mechanisms(self):
        # Fixed synthetic source rows, never native model qualification.
        good_g = np.diag([1., 1., 1., 0., 0., 0.])
        bad_g = np.diag([1., 0., 0., 0., 0., 0.])
        good_l = np.diag([100., 100., 100., 10., 10., 10.])
        bad_l = np.diag([0., 0., 100., 10., 10., 0.])
        for name, g, l in [('both_good', good_g, good_l),
                           ('GNSS_bad', bad_g, good_l),
                           ('LiDAR_bad', good_g, bad_l),
                           ('both_bad', bad_g, bad_l)]:
            with self.subTest(name=name):
                result = project_information((g+translation_gauge_information(l))/2, 5., 5.)
                self.assertEqual(result['valid'], name == 'both_good')
        # GNSS changes anchor precision; LiDAR changes rotational probe response.
        original = project_information((good_g+translation_gauge_information(good_l))/2, 5., 5.)
        gnss_improved = project_information((good_g*4+translation_gauge_information(good_l))/2, 5., 5.)
        lidar_improved = project_information((good_g+translation_gauge_information(good_l*4))/2, 5., 5.)
        self.assertLess(gnss_improved['S'], original['S'])
        self.assertLess(lidar_improved['S'], original['S'])


if __name__ == '__main__':
    unittest.main()
