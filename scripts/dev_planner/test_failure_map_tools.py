#!/usr/bin/env python3
"""Small saved-map fixtures for the offline failure diagnosis."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analyze_failure_map import inspect


class FailureMapToolsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name) / "search"
        self.directory.mkdir()
        self.flags = np.full((20, 20, 3), 4, dtype=np.uint8)
        self.meta = {
            "schema_version": "iap_gridmap_failure_v1",
            "kind": "search", "generation": 3,
            "frame_id": "map", "origin_m": [-1.0, -1.0, 0.0],
            "dimensions": [20, 20, 3], "resolution_m": 0.1,
            "cell_flags_file": "cells.bin", "required_clearance_m": 0.15,
            "other_endpoint_m": [-0.5, 0.0, 0.15],
            "failure_position_m": [0.5, 0.0, 0.15],
            "search_failure": "TIME_BUDGET",
        }

    def tearDown(self):
        self.tmp.cleanup()

    def write(self):
        (self.directory / "snapshot.json").write_text(json.dumps(self.meta))
        self.flags.tofile(self.directory / "cells.bin")

    def test_route_exists_but_online_timed_out(self):
        self.write()
        self.assertEqual(inspect(self.directory, 20)["classification"],
                         "ROUTE_EXISTS_ON_SNAPSHOT_ONLINE_TIMEOUT")

    def test_unobserved_target_is_not_physical_no_route(self):
        self.flags[15, 10, 1] = 0
        self.write()
        self.assertEqual(inspect(self.directory, 20)["classification"],
                         "TARGET_UNOBSERVED")

    def test_complete_raw_wall_blocks_observed_pool(self):
        self.flags[10, :, :] |= 3
        self.write()
        self.assertEqual(inspect(self.directory, 20)["classification"],
                         "NO_ROUTE_IN_OBSERVED_SEARCH_POOL")


if __name__ == "__main__":
    unittest.main()
