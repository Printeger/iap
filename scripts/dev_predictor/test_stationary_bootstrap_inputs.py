import unittest
import numpy as np
from local_increment_validation import (
    stationary_simulator_bootstrap_inputs, quaternion_rotation)


class StationaryBootstrapInputs(unittest.TestCase):
    def setUp(self):
        self.recipe = {'held_before_first_estimator_feedback': True,
                       'input_phase': 'initialization_before_fixed_lag_update',
                       'imu_generation': 'ideal_specific_force_and_angular_velocity',
                       'initial_fix_generation': 'noise_free_receiver_lla',
                       'initial_body_enu_calibration_known': True,
                       'calibration_uncertainty': 'none_in_declared_simulator_recipe'}
        self.pose = np.eye(4)
        self.velocity = np.array([.01, -.02, .03])
        self.bias = np.array([.001, -.002, .003, -.004, .005, -.006])
        self.fix = np.array([100., 200., 300.])

    def inputs(self, **changes):
        values = dict(initial_pose=self.pose, initial_velocity=self.velocity,
                      initial_bias=self.bias, initial_anchor_ecef=self.fix,
                      initial_rotation_ecef_map=np.eye(3), first_fix_ecef=self.fix,
                      body_to_enu_calibration=np.eye(3), enu_to_ecef_calibration=np.eye(3),
                      rest_specific_force_body=[0., 0., 9.81],
                      model_gravity_map=[0., 0., -9.81], physical_recipe=self.recipe)
        values.update(changes)
        return stationary_simulator_bootstrap_inputs(**values)

    def test_nonzero_initialized_velocity_and_bias_are_not_erased(self):
        law = self.inputs()
        np.testing.assert_array_equal(law['mean'][6:9], self.velocity)
        np.testing.assert_array_equal(law['mean'][9:15], self.bias)
        np.testing.assert_array_equal(law['covariance'], np.zeros((24, 24)))
        self.assertFalse(law['complete_retained_or_innovation_law'])
        self.assertFalse(law['production_predictor_connected'])

    def test_one_global_datum_retains_gravity_and_alignment_discrepancy(self):
        pose = np.eye(4); pose[:3, 3] = [.02, -.01, .03]
        # Nominal initializer tilt is a world datum for pose(0), but the original
        # solver gravity/alignment cannot silently receive that datum correction.
        pose[:3, :3] = quaternion_rotation([np.sin(.02), 0., 0., np.cos(.02)])
        enu_ecef = quaternion_rotation([0., 0., np.sin(.1), np.cos(.1)])
        law = self.inputs(initial_pose=pose, initial_rotation_ecef_map=enu_ecef,
                          enu_to_ecef_calibration=enu_ecef)
        np.testing.assert_allclose(law['physical_anchor_ecef'] +
                                   law['physical_rotation_ecef_map'] @ pose[:3, 3], self.fix)
        np.testing.assert_allclose(law['physical_rotation_ecef_map'] @ pose[:3, :3], enu_ecef, atol=1e-15)
        np.testing.assert_allclose(pose[:3, :3].T @ law['physical_gravity_map'], [0., 0., -9.81], atol=1e-14)
        np.testing.assert_allclose(law['mean'][18:21], [.04, 0., 0.], atol=1e-15)
        self.assertGreater(np.linalg.norm(law['mean'][21:24]), .3)
        np.testing.assert_array_equal(law['mean'][:6], np.zeros(6))

    def test_real_gravity_calibration_is_independent_of_solver_gravity(self):
        body = quaternion_rotation([0., np.sin(.03), 0., np.cos(.03)])
        law = self.inputs(body_to_enu_calibration=body,
                          rest_specific_force_body=body.T @ [0., 0., 9.81],
                          model_gravity_map=[0., 0., -9.7])
        np.testing.assert_allclose(law['physical_rotation_ecef_map'], body)
        np.testing.assert_allclose(law['physical_gravity_map'], body.T @ [0., 0., -9.81])
        self.assertGreater(np.linalg.norm(law['mean'][21:24]), .5)
        with self.assertRaisesRegex(ValueError, 'REST_FORCE_INCONSISTENT'):
            self.inputs(rest_specific_force_body=[0., 0., 9.7], model_gravity_map=[0., 0., -9.7])

    def test_absent_or_nonideal_physics_never_becomes_a_zero_seed(self):
        for recipe in (None, {}, {**self.recipe, 'held_before_first_estimator_feedback': False},
                       {**self.recipe, 'input_phase': 'current_postopt_report'},
                       {**self.recipe, 'calibration_uncertainty': 'nominal_only'},
                       {**self.recipe, 'imu_generation': 'solver_noise'},
                       {**self.recipe, 'initial_body_enu_calibration_known': False}):
            with self.assertRaisesRegex(ValueError, 'PHYSICAL_RECIPE_UNAVAILABLE'):
                self.inputs(physical_recipe=recipe)
        with self.assertRaisesRegex(ValueError, 'MISSING_OR_NONFINITE'):
            self.inputs(initial_velocity=[np.nan, 0., 0.])


if __name__ == '__main__':
    unittest.main()
