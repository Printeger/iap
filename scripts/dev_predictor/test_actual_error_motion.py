import unittest
from fusion_actual_error_audit import truth_motion_evidence


class TruthMotionTest(unittest.TestCase):
    def samples(self, velocity):
        return [dict(stamp=10+i*.01, x=velocity*i*.01, y=0, z=1,
                     qx=0, qy=0, qz=0, qw=1) for i in range(11)]

    def test_static_truth_does_not_become_motion_from_estimator_drift(self):
        self.assertFalse(truth_motion_evidence(self.samples(0), 10.05)['moving'])

    def test_motion_uses_independent_truth_displacement(self):
        evidence=truth_motion_evidence(self.samples(.4), 10.05)
        self.assertTrue(evidence['moving'])
        self.assertAlmostEqual(evidence['speed_mps'], .4)

    def test_no_extrapolation_or_time_gate_relaxation(self):
        with self.assertRaisesRegex(ValueError, 'truth_time_unmatched'):
            truth_motion_evidence(self.samples(.4), 10)
        with self.assertRaisesRegex(ValueError, 'truth_time_gap'):
            truth_motion_evidence([self.samples(.4)[0], self.samples(.4)[-1]], 10.07)


if __name__ == '__main__':
    unittest.main()
