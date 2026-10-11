import unittest

import numpy as np

from local_increment_validation import PoseSeries, quaternion_rotation, relative_error


def pose(x=0., yaw=0.):
    T = np.eye(4)
    T[:3, :3] = quaternion_rotation([0., 0., np.sin(yaw/2), np.cos(yaw/2)])
    T[:3, 3] = [x, .2*x, -.1*x]
    return T


def rows(stamps, source='glio'):
    return [dict(source=source, frame='map', body='imu' if source == 'glio' else 'drone_0',
                 stamp=t, x=t, y=0., z=0., qx=0., qy=0., qz=0., qw=1.) for t in stamps]


class LocalIncrementContract(unittest.TestCase):
    def test_common_datum_including_rotation_cancels_without_alignment(self):
        start, end = pose(1., .2), pose(1.5, .4)
        datum = pose(19., 1.4)
        r = relative_error(datum @ start, datum @ end, start, end)
        self.assertLess(r['translation_error_m'], 1e-12)
        self.assertLess(r['rotation_error_rad'], 1e-7)
        # The previous absolute position object would retain a large error.
        self.assertGreater(np.linalg.norm((datum @ end)[:3, 3]-end[:3, 3]), 10.)

    def test_local_deformation_is_retained(self):
        start, end = pose(1., .2), pose(1.5, .4)
        warped = end.copy()
        warped[:3, 3] += [.03, -.04, .02]
        r = relative_error(start, warped, start, end)
        self.assertAlmostEqual(r['horizontal_error_m'], .05, places=12)
        self.assertGreater(r['translation_error_m'], .05)
        # A global change of coordinates preserves that unfavorable result.
        datum = pose(19., 1.4)
        d = relative_error(datum @ start, datum @ warped, start, end)
        np.testing.assert_allclose(d['translation_error_start_body_m'], r['translation_error_start_body_m'], atol=1e-12)

    def test_initial_attitude_error_affects_increment_not_only_translation(self):
        r = relative_error(pose(0., .1), pose(.5, .1), pose(0.), pose(.5))
        self.assertGreater(r['horizontal_error_m'], .04)

    def test_stationary_window_remains_defined(self):
        r = relative_error(pose(0.), pose(0.), pose(0.), pose(0.))
        self.assertEqual(r['horizontal_error_m'], 0.)
        self.assertEqual(r['truth_motion_distance_m'], 0.)

    def test_missing_two_frame_estimate_bracket_is_unavailable(self):
        series = PoseSeries(rows([0., .1, .3, .4]), 'glio')
        with self.assertRaisesRegex(ValueError, '150MS'):
            series.at(.2)
        with self.assertRaisesRegex(ValueError, 'NOT_BRACKETED'):
            series.at(.5)

    def test_truth_gate_is_not_widened(self):
        series = PoseSeries(rows([0., .2], 'truth'), 'truth')
        with self.assertRaisesRegex(ValueError, '50MS'):
            series.at(.1)

    def test_interpolation_is_shortest_arc_and_preserves_original_samples(self):
        data = rows([0., .1])
        data[1]['qw'] = -1.
        series = PoseSeries(data, 'glio')
        T, evidence = series.at(.05)
        np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)
        self.assertEqual(T[0, 3], .05)
        self.assertFalse(evidence['exact_sample'])
        self.assertTrue(series.at(0.)[1]['exact_sample'])

    def test_conflicting_stamp_and_unknown_extrinsic_fail(self):
        data = rows([0., 0., .1])
        data[1]['x'] = 1.
        series = PoseSeries(data, 'glio')
        with self.assertRaisesRegex(ValueError, 'CONFLICTING_DUPLICATE'):
            series.at(0.)
        with self.assertRaisesRegex(ValueError, 'CONFLICTING_DUPLICATE'):
            series.at(.05)
        # An unrelated exact pose does not use either ambiguous value.
        self.assertEqual(series.at(.1)[0][0, 3], .1)
        data = rows([0., .1])
        data[0]['body'] = 'camera'
        with self.assertRaisesRegex(ValueError, 'EXTRINSIC_UNQUALIFIED'):
            PoseSeries(data, 'glio')


if __name__ == '__main__':
    unittest.main()
