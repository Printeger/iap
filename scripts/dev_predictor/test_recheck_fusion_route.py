import unittest
import numpy as np
from recheck_fusion_route import sample_bspline


class CapturedSplineTest(unittest.TestCase):
    def test_uniform_cubic_matches_exact_affine_curve(self):
        knots = np.arange(-3., 8.)
        controls = np.c_[np.arange(7.), 2*np.arange(7.)+1., -np.arange(7.)]
        times = np.linspace(0., 4., 101)
        expected = np.c_[times+1., 2*(times+1.)+1., -(times+1.)]
        np.testing.assert_allclose(sample_bspline(knots, controls, 3, times), expected, atol=1e-14)

    def test_uniform_cubic_start_and_end_preserve_original_blending(self):
        controls = np.array([[1., 2., 3.], [2., 3., 4.], [4., 9., 8.],
                             [5., 4., 3.], [6., 1., 2.]])
        value = sample_bspline(np.arange(-3., 6.), controls, 3, [0., 2.])
        np.testing.assert_allclose(value[0], (controls[0]+4*controls[1]+controls[2])/6)
        np.testing.assert_allclose(value[1], (controls[-3]+4*controls[-2]+controls[-1])/6)


if __name__ == '__main__':
    unittest.main()
