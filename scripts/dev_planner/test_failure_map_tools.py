#!/usr/bin/env python3
"""Saved-map fixtures drive the same GridMap and A* used online."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analyze_failure_map import inspect


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


if __name__ == "__main__":
    unittest.main()
