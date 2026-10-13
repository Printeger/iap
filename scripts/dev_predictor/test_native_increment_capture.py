import unittest
import copy

import numpy as np

from advisory_validation import NativeIncrementCapture
from local_increment_validation import native_pose_at, native_endpoint


def evidence():
    means = np.r_[np.eye(4).ravel(), [1., .2, 0.], np.zeros(6), np.eye(3).ravel(), np.zeros(3), np.zeros(2)]
    samples = np.array([[100.+i*.005, 0., 0., 9.81, 0., 0., 1.] for i in range(-1, 12)])
    return dict(model='postopt_X_V_B_R_E_active_clocks_joint_v1', optimized_valid=True,
                update_sequence=1, frame_id=10, state_stamp=100., gnss_stamp=100.05,
                optimized_means=means.tolist(), mean_dimensions=[16, 3, 6, 9, 3, 2],
                keys=[(ord(s) << 56) | i for s, i in [('x', 10), ('v', 10), ('b', 10), ('r', 0), ('e', 0), ('c', 10)]], epoch_source_identity=44,
                imu_measurements=samples.ravel().tolist())


def event(e):
    return {'report': dict(advisory_postopt=e, estimation_frame_id=e['frame_id'],
                advisory_coordinates_valid=True, advisory_coordinates_frame_id=e['frame_id'],
                advisory_coordinates_epoch_source_identity=e['epoch_source_identity'],
                advisory_t_map_world=np.eye(4).ravel().tolist(), current_motion_quality=1)}


class NativeIncrementSelection(unittest.TestCase):
    def test_report_eigen_matrix_layout_matches_recording_codec(self):
        T = np.eye(4)
        T[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
        T[:3, 3] = [-18., 2., 1.5]
        record = event(evidence())
        record['report']['advisory_t_map_world'] = T.ravel(order='F').tolist()
        pose, _ = native_endpoint([record], 100.025, T)
        np.testing.assert_allclose(pose, native_pose_at(evidence(), 100.025, T))
        record['report']['advisory_t_map_world'] = T.ravel().tolist()
        with self.assertRaisesRegex(ValueError, 'COORDINATE_OWNER_MISMATCH'):
            native_endpoint([record], 100.025, T)

    def test_capture_retains_past_and_future_owner_without_full_stream(self):
        written = []
        capture = NativeIncrementCapture(written.append)
        capture.observe({'owner': 'old'}, 8.)
        capture.observe({'owner': 'start'}, 9.8)
        capture.begin('request-A', 10., 2.3)
        capture.observe({'owner': 'end'}, 11.)
        capture.observe({'owner': 'outside'}, 13.)
        self.assertEqual([r['report']['owner'] for r in written], ['start', 'end'])
        self.assertTrue(all(r['request_id'] == 'request-A' for r in written))
        self.assertLessEqual(len(capture.recent), 20)

    def test_invalid_report_is_explicit_null_not_zero_or_dropped(self):
        written = []
        capture = NativeIncrementCapture(written.append)
        capture.begin('request', 0., 2.)
        capture.observe({'optimized_valid': False, 'mean': float('nan'), 'fixed_array': np.array([1., float('nan')])}, 1.)
        self.assertEqual(len(written), 1)
        self.assertIsNone(written[0]['report']['mean'])
        self.assertFalse(written[0]['report']['optimized_valid'])
        self.assertEqual(written[0]['report']['fixed_array'], [1., None])

    def test_two_second_cadence_preserves_both_full_post_windows(self):
        written = []
        capture = NativeIncrementCapture(written.append)
        capture.begin('A', 10., 2.3)
        capture.begin('B', 12., 2.3)
        capture.observe({'owner': 'overlap'}, 12.2)
        capture.observe({'owner': 'B-only'}, 12.4)
        self.assertEqual([(r['request_id'], r['report']['owner']) for r in written],
                         [('A', 'overlap'), ('B', 'overlap'), ('B', 'B-only')])
        self.assertEqual(capture.until, 14.3)

    def test_native_exact_epoch_uses_sensor_transport_without_posterior_covariance(self):
        T = native_pose_at(evidence(), 100.05, np.eye(4))
        np.testing.assert_allclose(T[:3, 3], [.05, .01, 0.], atol=1e-12)
        self.assertAlmostEqual(np.arctan2(T[1, 0], T[0, 0]), .05, places=12)

    def test_missing_sensor_interval_or_wrong_state_owner_fails(self):
        e = evidence()
        e['imu_measurements'] = []
        with self.assertRaisesRegex(ValueError, 'IMU_missing'):
            native_pose_at(e, 100.05, np.eye(4))
        with self.assertRaisesRegex(ValueError, 'OUTSIDE_NATIVE'):
            native_pose_at(evidence(), 101., np.eye(4))
        e = evidence()
        e['keys'] = [(ord('x') << 56) | 11]
        with self.assertRaisesRegex(ValueError, 'OWNER_MISMATCH'):
            native_pose_at(e, 100.05, np.eye(4))

    def test_native_endpoint_selection_uses_epoch_not_error(self):
        early = evidence()
        later = copy.deepcopy(early)
        later.update(frame_id=11, update_sequence=2, gnss_stamp=100.1,
                     keys=[(ord(s) << 56) | i for s, i in [('x', 11), ('v', 11), ('b', 11), ('r', 0), ('e', 0), ('c', 11)]])
        later['optimized_means'][3] = 100.
        T, owner = native_endpoint([event(later), event(early)], 100.025, np.eye(4))
        self.assertEqual(owner['frame_id'], 10)
        self.assertFalse(owner['covariance_used'])
        self.assertAlmostEqual(T[0, 3], .025, places=12)

    def test_conflicting_native_identity_or_quality_is_not_selected_away(self):
        first = event(evidence())
        different = copy.deepcopy(first)
        different['report']['advisory_postopt']['optimized_means'][3] = .001
        with self.assertRaisesRegex(ValueError, 'CONFLICTING_NATIVE_STATE_IDENTITY'):
            native_endpoint([first, different], 100.025, np.eye(4))
        different = copy.deepcopy(first)
        different['report']['current_motion_quality'] = 0
        with self.assertRaisesRegex(ValueError, 'CONFLICTING_NATIVE_STATE_QUALIFICATION'):
            native_endpoint([first, different], 100.025, np.eye(4))


if __name__ == '__main__':
    unittest.main()
