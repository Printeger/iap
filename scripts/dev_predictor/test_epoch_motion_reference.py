import unittest
import numpy as np
from epoch_motion_reference import intervals,propagate,exp,reference_transition

class IndependentMotionReferenceTest(unittest.TestCase):
    def setup_input(self,moving=True):
        m=np.r_[np.eye(4).ravel(),[20.,0.,0.] if moving else [0.,0.,0.],np.zeros(6),np.eye(3).ravel(),[0.,0.,0.],[3.,.2]]
        samples=[[10.+i*.002,2. if moving else 0.,0.,9.81,0.,0.,0.] for i in range(42)]
        return {'state_stamp':10.,'gnss_stamp':10.08,'imu_measurements':np.array(samples).ravel().tolist(),
                'linearization_means':m.tolist(),'tangent_dimensions':[6,3,6,3,3,2]}
    def test_moving_and_stationary_have_distinct_actual_mean_propagation(self):
        for moving in (False,True):
            e=self.setup_input(moving);rows,p,f=reference_transition(e)
            self.assertAlmostEqual(p[3],1.6064 if moving else 0.,places=10)
            self.assertAlmostEqual(p[16],20.16 if moving else 0.,places=10)
            self.assertAlmostEqual(p[37],3.016,places=10)
            self.assertAlmostEqual(f[3,6],.08,places=7)
            self.assertAlmostEqual(f[3,9],-.5*.08**2,places=7)
            self.assertAlmostEqual(f[-2,-1],.08,places=7)
    def test_missing_backward_stale_and_gapped_imu_are_rejected(self):
        e=self.setup_input();data=e['imu_measurements']
        for raw,start,end in (([],10,10.08),(data,10.08,10),(data,10,10.3),(data[7:],10,10.08),(data[:7]+data[84:],10,10.08)):
            with self.assertRaises(ValueError):intervals(raw,start,end)
    def test_rotated_state_and_velocity_cross_covariance(self):
        e=self.setup_input();m=np.array(e['linearization_means']);t=m[:16].reshape(4,4)
        t[:3,:3]=exp(np.array([0.,0.,.6]));e['linearization_means']=m.tolist()
        _,_,f=reference_transition(e);a=np.random.default_rng(61).normal(size=(23,23));p=a@a.T
        after=f@p@f.T
        self.assertGreater(np.linalg.norm(after[:6,:6]-p[:6,:6]),.1)
        # A frame rotation must rotate the p-v cross term along with covariance.
        r=t[:3,:3];j=np.zeros((3,23));j[:,3:6]=r
        np.testing.assert_allclose(j@after@j.T,r@after[3:6,3:6]@r.T)

if __name__=='__main__':unittest.main()
