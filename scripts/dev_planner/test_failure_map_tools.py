#!/usr/bin/env python3
"""Saved-map fixtures drive the same GridMap and A* used online."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analyze_failure_map import inspect
from analyze_curve_observation import inspect as inspect_curve


class FailureMapToolsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name) / "timeout"
        self.directory.mkdir()
        self.flags = np.full((20, 20, 5), 4, dtype=np.uint8)
        self.meta = {
            "schema_version": "iap_gridmap_failure_v2",
            "kind": "timeout", "generation": 3,
            "frame_id": "map", "origin_m": [-1.0, -1.0, 0.0],
            "max_boundary_m": [1.0, 1.0, 0.5],
            "dimensions": [20, 20, 5], "resolution_m": 0.1,
            "cell_flags_file": "cells.bin", "planning_time_s": 10.1,
            "cloud_stamp_s": 10.0, "environment_max_age_s": 0.5,
            "motion_quality": 1, "motion_allow_bridged": False,
            "motion_stamp_s": 10.0, "motion_error_proxy_m": 0.0,
            "motion_body_radius_m": 0.05,
            "motion_tracking_reserve_m": 0.0,
            "motion_budget_m": 0.55, "motion_max_age_s": 0.5,
            "search_pool_dimensions": [20, 20, 10],
            "search_step_size_m": 0.1,
            "search_pool_center_m": [0.0, 0.0, 0.25],
            "search_requested_start_m": [-0.5, 0.0, 0.25],
            "search_requested_end_m": [0.5, 0.0, 0.25],
            "control_points_m": [[-0.8, 0.0, 0.25],
                                 [-0.5, 0.0, 0.25],
                                 [0.0, 0.0, 0.25],
                                 [0.5, 0.0, 0.25],
                                 [0.8, 0.0, 0.25]],
            "segment_start_index": 1, "segment_end_index": 3,
            "search_failure": "TIME_BUDGET",
        }

    def tearDown(self):
        self.tmp.cleanup()

    def write(self):
        (self.directory / "snapshot.json").write_text(json.dumps(self.meta))
        self.flags.tofile(self.directory / "cells.bin")

    def test_route_exists_but_online_timed_out(self):
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "ONLINE_SEARCH_TIMEOUT")

    def test_advisory_failure_does_not_claim_physical_no_route(self):
        self.meta["kind"] = "exhausted"
        self.meta["search_failure"] = "ADVISORY_NO_PATH"
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "ADVISORY_PREFERENCE_ONLY")

    def test_bad_endpoint_has_executable_alternative(self):
        self.meta["kind"] = "endpoint"
        self.meta["search_failure"] = "END_BLOCKED"
        self.flags[15, 10, 2] |= 3
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "ENDPOINT_SELECTION")

    def test_complete_wall_blocks_observed_pool(self):
        self.meta["kind"] = "exhausted"
        self.meta["search_failure"] = "NO_PATH"
        self.flags[10, :, :] |= 3
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "NO_ROUTE_IN_OBSERVED_SEARCH_POOL")

    def test_unknown_wall_is_not_treated_as_free(self):
        self.meta["kind"] = "exhausted"
        self.meta["search_failure"] = "NO_PATH_WITH_UNOBSERVED"
        self.flags[10, :, :] = 0
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "NO_ROUTE_IN_OBSERVED_SEARCH_POOL")

    def test_raw_clearance_blocks_without_inflation_flag(self):
        self.meta["kind"] = "exhausted"
        self.meta["search_failure"] = "NO_PATH"
        self.flags[10, :, :] |= 1
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "NO_ROUTE_IN_OBSERVED_SEARCH_POOL")

    def test_rounded_endpoint_needs_its_connector(self):
        self.meta["kind"] = "endpoint"
        self.meta["search_failure"] = "END_BLOCKED"
        self.meta["motion_body_radius_m"] = 0.0
        self.meta["search_requested_end_m"] = [0.54, 0.0, 0.25]
        self.meta["control_points_m"][3] = [0.54, 0.0, 0.25]
        self.meta["search_pool_center_m"] = [0.02, 0.0, 0.25]
        self.flags[14, 10, 2] |= 1
        self.write()
        report = inspect(self.directory, 10)
        self.assertEqual(report["original_replay_failure"], "END_BLOCKED")
        self.assertEqual(report["classification"], "ENDPOINT_SELECTION")

    def test_stale_map_is_inconclusive(self):
        self.meta["planning_time_s"] = 11.0
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "INCONCLUSIVE_STALE_OR_INVALID_EVIDENCE")

    def test_missing_motion_evidence_is_inconclusive(self):
        self.meta["motion_error_proxy_m"] = None
        self.write()
        self.assertEqual(inspect(self.directory, 10)["classification"],
                         "INCONCLUSIVE_STALE_OR_INVALID_EVIDENCE")

    def test_offline_budget_exhaustion_is_inconclusive(self):
        self.flags[10, :, :] |= 3
        self.write()
        self.assertEqual(inspect(self.directory, 0.000001)["classification"],
                         "INCONCLUSIVE_OFFLINE_BUDGET")

    def test_old_snapshot_cannot_claim_same_rule(self):
        self.meta["schema_version"] = "iap_gridmap_failure_v1"
        self.write()
        with self.assertRaisesRegex(ValueError, "v2 failure snapshot"):
            inspect(self.directory, 10)

    def test_attribution_distinguishes_unknown_barrier_without_mutating_map(self):
        self.meta.update(real_start_p_m=self.meta["search_requested_start_m"],
                         planning_goals_m=[self.meta["search_requested_end_m"]],
                         guide_fitting_reserve_m=0., guide_reserve_taper_distance_m=.5)
        self.flags[10,:,:]=0
        self.write()
        before=(self.directory/"cells.bin").read_bytes()
        report=inspect(self.directory,10,attribution=True)
        self.assertTrue(report["observed"]["exhausted"])
        self.assertFalse(report["observed"]["goals_reachable"])
        self.assertTrue(report["geometry_only"]["goals_reachable"])
        self.assertFalse(report["geometry_only"]["execution_authorized"])
        self.assertGreater(report["observed"]["component_nodes"],0)
        self.assertEqual(before,(self.directory/"cells.bin").read_bytes())
        self.assertTrue(report["production_authorization_unchanged"])
        self.assertTrue(report["cut_complete"])
        self.assertGreater(report["cut_edges"], 0)

    def attribution(self, budget_s=10):
        self.meta.update(real_start_p_m=self.meta["search_requested_start_m"],
                         guide_fitting_reserve_m=0., guide_reserve_taper_distance_m=.5)
        self.meta.setdefault("planning_goals_m", [self.meta["search_requested_end_m"]])
        self.write()
        return inspect(self.directory, budget_s, attribution=True)

    def test_attribution_recorded_obstacles_cut_both_graphs(self):
        self.flags[10,:,:] |= 1
        report = self.attribution()
        self.assertTrue(report["geometry_only"]["exhausted"])
        self.assertFalse(report["geometry_only"]["goals_reachable"])
        self.assertIn("RECORDED_GEOMETRY_SUFFICIENT_TO_CUT_ORIGINAL_POOL", report["causes"])
        self.assertGreater(report["boundary_reasons"]["INSUFFICIENT_CLEARANCE"], 0)

    def test_attribution_unknown_never_erases_raw_obstacle(self):
        self.flags[10,:,:] = 1
        report = self.attribution()
        self.assertFalse(report["geometry_only"]["goals_reachable"])
        self.assertTrue(report["geometry_only"]["exhausted"])

    def test_attribution_invalid_targets_still_measure_real_start_component(self):
        self.flags[15,10,2] |= 3
        report = self.attribution()
        self.assertGreater(report["observed"]["component_nodes"], 0)
        self.assertFalse(report["observed"]["goals"][0]["eligible"])
        self.assertEqual(report["goal_components"][0], -2)

    def test_attribution_pool_truncation_keeps_original_lattice(self):
        self.meta["search_pool_dimensions"] = [20,10,10]
        self.flags[10,5:15,:] |= 1
        report = self.attribution()
        self.assertFalse(report["observed"]["goals_reachable"])
        self.assertTrue(report["observed_expanded"]["goals_reachable"])
        self.assertIn("ORIGINAL_POOL_TRUNCATES_OBSERVED_PATH", report["causes"])
        delta = (np.array(report["observed_expanded"]["pool_center_m"])-
                 np.array(self.meta["search_pool_center_m"]))/self.meta["search_step_size_m"]
        np.testing.assert_allclose(delta, np.round(delta), atol=1e-12)

    def test_attribution_targets_register_separate_components(self):
        self.flags[10,:,:] |= 1
        self.meta["planning_goals_m"] = [[.5,0,.25],[-.2,0,.25]]
        report = self.attribution()
        self.assertTrue(report["observed"]["goals"][1]["reached"])
        self.assertFalse(report["observed"]["goals"][0]["reached"])
        self.assertEqual(report["goal_components"][1], 0)
        self.assertGreater(report["goal_components"][0], 0)

    def test_attribution_timeout_never_claims_no_route(self):
        self.flags[10,:,:] |= 1
        report = self.attribution(.000001)
        self.assertFalse(report["observed"]["exhausted"])
        self.assertEqual(report["observed"]["failure"], "TIME_BUDGET")
        self.assertEqual(report["causes"], [])

    def test_attribution_processing_loss_requires_current_support(self):
        self.flags[10,:,:] = 0
        self.meta.update(observation_evidence_available=True,
            observation_sources_file="observation_sources.bin",
            current_frame={"frame_id": 9, "stamp_s": 10., "sensor_position_m": [-.5,.05,.25],
                "beam_evidence_complete": True, "max_range_m": 1.,
                "hits_file": "current_frame_hits.csv", "beams_file": "current_frame_beams.csv"})
        (self.directory/"current_frame_beams.csv").write_text(
            "lidar_dx,lidar_dy,lidar_dz,outcome,range_m,map_dx,map_dy,map_dz\n1,0,0,1,.9,1,0,0\n")
        (self.directory/"current_frame_hits.csv").write_text(
            "lidar_x,lidar_y,lidar_z,map_x,map_y,map_z\n.9,0,0,.4,.05,.25\n")
        sources = np.zeros_like(self.flags)
        sources[10,10,2] = 128|16
        sources.tofile(self.directory/"observation_sources.bin")
        report = self.attribution()
        self.assertGreater(report["unknown_boundary_classifications"]["PROCESSING_SUPPORT_LOSS"], 0)
        witness = report["key_ray_evidence"][0]
        self.assertEqual(witness["loss_producer"], "current_replace")
        self.assertEqual(witness["beam_witnesses"][0]["relation"], "PREFIX_INTERSECTION")
        sources[10,10,2] = 16
        sources.tofile(self.directory/"observation_sources.bin")
        report = self.attribution()
        self.assertNotIn("PROCESSING_SUPPORT_LOSS", report["unknown_boundary_classifications"])

    def test_old_curve_snapshot_cannot_explain_missing_ray_provenance(self):
        self.write()
        self.assertEqual(inspect_curve(self.directory)["classification"],
                         "INCONCLUSIVE_MISSING_CURVE_AND_FRAME")

    def test_curve_observation_distinguishes_ray_dedup_and_window_removal(self):
        self.meta.update(
            schema_version="iap_gridmap_failure_v3", kind="curve_unobserved",
            first_unobserved_position_m=[0.05, 0.05, 0.25],
            first_unobserved_voxel_index=[10, 10, 2], first_unobserved_time_s=0.0,
            curve_checked_from_time_s=0.0, curve_checked_to_time_s=0.1,
            curve_sample_step_s=0.02,
            actual_curve={"degree": 3, "interval_s": 0.1,
                          "control_points_m": [[0.05, 0.05, 0.25]] * 4,
                          "knots_s": [-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4]},
            observation_evidence_available=True,
            observation_sources_file="observation_sources.bin",
            current_frame={"frame_id": 9, "stamp_s": 10.0,
                           "beam_evidence_complete": True,
                           "hits_file": "current_frame_hits.csv",
                           "beams_file": "current_frame_beams.csv"})
        self.flags[10, 10, 2] = 0
        self.write()
        (self.directory / "current_frame_hits.csv").write_text("lidar_x,lidar_y,lidar_z\n")
        (self.directory / "current_frame_beams.csv").write_text("outcome,range_m\n")
        for flag, expected in (
                (0, "CURRENT_RAY_COVERAGE_GAP"),
                (128, "ENDPOINT_DEDUPLICATION_GAP"),
                (16, "OBSERVATION_REMOVED_CURRENT_REPLACE"),
                (32, "OBSERVATION_REMOVED_ACTIVE_DELTA"),
                (48, "OBSERVATION_REMOVED_ACTIVE_REPLACE"),
                (2, "OBSERVATION_MASK_INCONSISTENT")):
            sources = np.zeros_like(self.flags)
            sources[10, 10, 2] = flag
            sources.tofile(self.directory / "observation_sources.bin")
            report = inspect_curve(self.directory)
            self.assertEqual(report["classification"], expected)
            self.assertTrue(report["curve_replay_matches"])
            json.dumps(report)
        self.meta["curve_evaluation_time_s"] = 10.8
        self.write()
        self.assertEqual(inspect_curve(self.directory)["classification"],
                         "INCONCLUSIVE_STALE_OBSERVATION_EVIDENCE")
        self.meta["curve_evaluation_time_s"] = 10.1
        self.meta["first_unobserved_voxel_index"] = [11, 10, 2]
        self.write()
        with self.assertRaisesRegex(ValueError, "disagree"):
            inspect_curve(self.directory)


if __name__ == "__main__":
    unittest.main()
