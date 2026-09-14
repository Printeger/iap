#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = (Path(__file__).resolve().parents[1] /
          "scripts/dev_planner/analyze_p4_gnss_sensitivity.py")
SPEC = importlib.util.spec_from_file_location("gnss_sensitivity", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def detail_rows(sample_index=1, x=0.0, satellite_count=8):
    elevations = [.35, .45, .55, .65, .75, .85, .95, 1.05]
    azimuths = [0.0, .8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6]
    sigmas = [5.0, 5.5, 6.0, 5.2, 5.8, 6.2, 5.4, 5.7]
    rows = []
    for index in range(satellite_count):
        rows.append({
            "schema_version": "p4_forward_route_decision_v7",
            "decision_event_id": "5", "planning_attempt_id": "7",
            "candidate_id": "2", "sample_role": "FIRST_FAILED",
            "sample_index": str(sample_index), "sat_id": str(index + 1),
            "x": str(x), "y": "0", "z": "1",
            "arc_length_m": str(x), "query_time_s": "20.0",
            "gnss_epoch_identity": "101", "occupancy_generation": "11",
            "risk_generation": "21", "satellite_set_hash": "8123",
            "used": "1", "elevation_rad": str(elevations[index]),
            "azimuth_rad": str(azimuths[index]), "kappa": "0.2",
            "sigma_eff_m": str(sigmas[index]),
            "candidate_raw_hpl": "52.380657245698934",
            "candidate_raw_vpl": "194.60916250452635",
            "receiver_raw_hpl": "50.0", "receiver_raw_vpl": "180.0",
            "anchor_hpl": "14.0", "anchor_vpl": "30.0",
            "spatial_delta_h": "2.380657245698934",
            "spatial_delta_v": "14.60916250452635",
            "temporal_growth_h": "0", "temporal_growth_v": "0",
            "final_hpl": "16.380657245698934",
            "final_vpl": "44.60916250452635",
            "hal": "20", "val": "40", "failure_reason": "ok",
        })
    return rows


class GnssSensitivityTest(unittest.TestCase):
    def test_replays_production_pl_and_reports_counterfactuals(self):
        report = MODULE.analyze_detail_rows(detail_rows())
        self.assertTrue(report["attribution_valid"])
        sample = report["samples"][0]
        self.assertAlmostEqual(sample["replayed_raw_hpl_m"],
                               52.380657245698934, places=10)
        self.assertAlmostEqual(sample["replayed_raw_vpl_m"],
                               194.60916250452635, places=10)
        self.assertAlmostEqual(sample["weighted_geometry_condition"],
                               182.9626835095647, places=8)
        self.assertLess(sample["unit_sigma_hpl_m"],
                        sample["replayed_raw_hpl_m"])
        self.assertIn("worst_excluded_sat_h", sample)

    def test_deduplicates_stage_rewrites_and_measured_boundary_is_continuous(self):
        first = detail_rows(sample_index=1, x=.44)
        duplicate = [dict(row) for row in first]
        second = detail_rows(sample_index=2, x=.46)
        report = MODULE.analyze_detail_rows(first + duplicate + second)
        self.assertEqual(report["sample_count"], 2)
        self.assertAlmostEqual(
            report["spatial_discrimination"][
                "within_route_raw_hpl_jump_max_m"],
            0.0, places=12)

    def test_refuses_attribution_when_required_geometry_is_missing(self):
        rows = detail_rows()
        rows[0].pop("sigma_eff_m")
        report = MODULE.analyze_detail_rows(rows)
        self.assertFalse(report["attribution_valid"])
        self.assertIn("required_satellite_fields_missing", report["failures"])

    def test_reports_satellite_set_change_separately(self):
        first = detail_rows(sample_index=1, x=0.0)
        second = detail_rows(sample_index=2, x=.2, satellite_count=7)
        for row in second:
            row["satellite_set_hash"] = "7123"
        report = MODULE.analyze_detail_rows(first + second)
        self.assertEqual(
            report["spatial_discrimination"]["satellite_set_change_count"], 1)


if __name__ == "__main__":
    unittest.main()
