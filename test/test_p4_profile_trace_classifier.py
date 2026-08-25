import importlib.util
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PATH = REPO / "scripts/dev_planner/classify_p4_profile_trace.py"
SPEC = importlib.util.spec_from_file_location("p4_trace_classifier", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def rows_with_invalid(invalid):
    rows = []
    for arm in MODULE.ARMS:
        for index in range(200):
            row = {
                "schema_version": "p4_equal_arc_profile_trace_v1",
                "planning_attempt_id": "7", "collision_segment_id": "2",
                "request_hash": "abc", "arm": arm,
                "sample_index": str(index), "sample_valid": "1",
                "sample_stale": "0", "top_reason": "ok",
                "corner_reason": "none", "spatial_weight": "0.125",
                "temporal_weight": "1", "occupancy_class": "FREE",
                "point_x": "1", "point_y": "2", "point_z": "3",
                "query_time_s": "10", "query_tau_s": "0.5",
                "sample_cost": "1.25", "risk_generation_id": "9",
                "frame_id": "map", "corner_id": "0",
                "temporal_layer": "0", "horizon_id": "0",
                "horizon_s": "0", "voxel_x": "1", "voxel_y": "2",
                "voxel_z": "3", "voxel_position_x": "1",
                "voxel_position_y": "2", "voxel_position_z": "3",
                "source_flags": "7", "corner_cost": "1.25",
                "corner_valid": "1", "corner_stale": "0",
                "corner_unknown": "0", "occupancy_source": "free",
            }
            if index in invalid:
                row.update(invalid[index])
            rows.append(row)
    return rows


class P4ProfileTraceClassifierTest(unittest.TestCase):
    def test_mutually_exclusive_categories_and_exact_200_per_arm(self):
        invalid = {
            0: {"sample_valid": "0", "top_reason": "unknown_voxel",
                "corner_reason": "occupied_skip", "spatial_weight": "0",
                "occupancy_class": "RAW_OCCUPIED"},
            1: {"sample_valid": "0", "top_reason": "unknown_voxel",
                "corner_reason": "occupied", "source_flags": "2147483648",
                "occupancy_class": "RAW_OCCUPIED",
                "occupancy_source": "raw_cloud"},
            2: {"sample_valid": "0", "top_reason": "out_of_map"},
            3: {"sample_valid": "0", "top_reason": "time_after_horizon"},
            4: {"sample_valid": "0", "sample_stale": "1",
                "top_reason": "stale_voxel"},
            5: {"sample_valid": "0", "top_reason": "unknown_voxel",
                "corner_reason": "provider_invalid"},
            6: {"sample_valid": "0", "top_reason": "mystery",
                "corner_reason": "none"},
        }
        result = MODULE.classify_trace_rows(rows_with_invalid(invalid))
        self.assertEqual(result["identity_count"], 1)
        self.assertEqual(result["rows"][0]["sample_counts"], {
            "original": 200, "risk": 200,
        })
        self.assertEqual(set(result["totals"]), set(MODULE.CATEGORIES))
        self.assertTrue(all(value == 2 for value in result["totals"].values()))
        indices = result["rows"][0]["invalid_sample_indices"]
        for arm in MODULE.ARMS:
            self.assertEqual(indices[arm]["TIME_SUPPORT"], [3])
            self.assertEqual(indices[arm]["OTHER"], [6])

    def test_missing_sample_fails_closed(self):
        with self.assertRaisesRegex(
                MODULE.TraceClassificationError, "sample_coverage_mismatch"):
            MODULE.classify_trace_rows(rows_with_invalid({})[:-1])

    def test_truncated_invalid_sample_fails_closed(self):
        rows = rows_with_invalid({
            0: {"sample_valid": "0", "top_reason": "time_after_horizon"},
        })
        del rows[0]["query_tau_s"]
        with self.assertRaisesRegex(
                MODULE.TraceClassificationError, "trace_header_mismatch"):
            MODULE.classify_trace_rows(rows)

    def test_noncanonical_typed_invalid_evidence_fails_closed(self):
        rows = rows_with_invalid({
            0: {"sample_valid": "0", "top_reason": "time_after_horizon",
                "source_flags": "7.0"},
        })
        with self.assertRaisesRegex(
                MODULE.TraceClassificationError, "source_flags"):
            MODULE.classify_trace_rows(rows)

    def test_false_occupied_strings_and_invalid_weights_do_not_authorize(self):
        rows = rows_with_invalid({
            0: {"sample_valid": "0", "top_reason": "unknown_voxel",
                "corner_reason": "provider_not_occupied",
                "occupancy_class": "NOT_OCCUPIED"},
        })
        result = MODULE.classify_trace_rows(rows)
        self.assertEqual(result["totals"]["PROVIDER_INVALID"], 2)
        self.assertEqual(result["totals"]["POSITIVE_WEIGHT_OCCUPIED_SKIP"], 0)
        rows = rows_with_invalid({
            0: {"sample_valid": "0", "top_reason": "unknown_voxel",
                "spatial_weight": "-0.5", "temporal_weight": "-0.5"},
        })
        with self.assertRaisesRegex(
                MODULE.TraceClassificationError, "out_of_range"):
            MODULE.classify_trace_rows(rows)

    def test_conflicting_sample_level_corner_rows_fail_closed(self):
        rows = rows_with_invalid({
            0: {"sample_valid": "0", "top_reason": "occupied",
                "corner_reason": "occupied_skip",
                "occupancy_class": "RAW_OCCUPIED"},
        })
        duplicate = dict(rows[0])
        duplicate["corner_id"] = "1"
        duplicate["top_reason"] = "time_after_horizon"
        rows.insert(1, duplicate)
        with self.assertRaisesRegex(
                MODULE.TraceClassificationError, "sample_field_conflict"):
            MODULE.classify_trace_rows(rows)

    def test_duplicate_corner_evidence_fails_closed(self):
        rows = rows_with_invalid({})
        rows.insert(1, dict(rows[0]))
        with self.assertRaisesRegex(
                MODULE.TraceClassificationError, "duplicate_corner_evidence"):
            MODULE.classify_trace_rows(rows)


if __name__ == "__main__":
    unittest.main()
