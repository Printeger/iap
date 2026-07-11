#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "analyze_safety_planner_run.py"
)
spec = importlib.util.spec_from_file_location("analyze_safety_planner_run", SCRIPT_PATH)
analyzer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(analyzer)


def manifest():
    return {
        "p0.skip_occupied_voxels": True,
        "p0_6.fixture.enabled": True,
        "p0_6.fixture.name": "occupied_overlap_box_v1",
        "p0_6.fixture.x_min": -1.5,
        "p0_6.fixture.x_max": 1.5,
        "p0_6.fixture.y_min": -0.75,
        "p0_6.fixture.y_max": 0.75,
        "p0_6.fixture.z_min": 1.0,
        "p0_6.fixture.z_max": 2.0,
        "p0_6.fixture.raw_hpl_m": 1.0,
        "p0_6.fixture.raw_vpl_m": 1.2,
        "p0_6.fixture.raw_c_pi": 1.2,
        "p0_6.fixture.low_raw_cost_threshold": 2.0,
    }


def health_summary():
    return {
        "occupied_skip_count_max": 2,
        "reason_counts": {"ok": 3},
        "dominant_unknown_reason_counts": {"occupied_skip": 3},
        "dominant_unknown_reason_latest": "occupied_skip",
    }


class P0_6AnalyzerTest(unittest.TestCase):
    def test_fixture_overlay_passes_when_occupied_low_raw_cells_are_skipped(self):
        rows, fixture = analyzer.p0_6_overlap_rows(
            manifest(),
            [
                {
                    "stamp": 1.0,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.5,
                    "pl": "nan",
                    "hpl": "nan",
                    "vpl": "nan",
                    "c_pi": 10.0,
                    "valid": 0,
                    "unknown": 1,
                    "stale": 0,
                    "source_flags": analyzer.P0_OCCUPIED_SKIP_SOURCE_FLAG,
                },
                {
                    "stamp": 1.0,
                    "x": 3.0,
                    "y": 0.0,
                    "z": 1.5,
                    "pl": 1.2,
                    "hpl": 1.0,
                    "vpl": 1.2,
                    "c_pi": 1.2,
                    "valid": 1,
                    "unknown": 0,
                    "stale": 0,
                    "source_flags": 0,
                },
            ],
            [],
        )

        summary = analyzer.summarize_p0_6_overlap(rows, health_summary())
        failures = []
        inconclusive = []
        analyzer.validate_p0_6_formal_semantics(
            manifest(), fixture, summary, failures, inconclusive
        )

        self.assertEqual(failures, [])
        self.assertEqual(inconclusive, [])
        self.assertEqual(summary["occupied_overlap_count"], 1)
        self.assertEqual(summary["occupied_low_raw_cost_count"], 1)
        self.assertEqual(summary["occupied_skip_count"], 1)
        self.assertEqual(summary["occupied_valid_low_risk_count"], 0)

    def test_fixture_overlay_fails_when_occupied_low_raw_cell_is_final_valid_low_risk(self):
        rows, fixture = analyzer.p0_6_overlap_rows(
            manifest(),
            [
                {
                    "stamp": 1.0,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.5,
                    "pl": 1.2,
                    "hpl": 1.0,
                    "vpl": 1.2,
                    "c_pi": 1.2,
                    "valid": 1,
                    "unknown": 0,
                    "stale": 0,
                    "source_flags": 0,
                },
            ],
            [],
        )

        summary = analyzer.summarize_p0_6_overlap(rows, health_summary())
        failures = []
        inconclusive = []
        analyzer.validate_p0_6_formal_semantics(
            manifest(), fixture, summary, failures, inconclusive
        )

        self.assertEqual(summary["occupied_valid_low_risk_count"], 1)
        self.assertTrue(
            any("final valid low-risk" in failure for failure in failures),
            failures,
        )


if __name__ == "__main__":
    unittest.main()
