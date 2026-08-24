import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts/dev_planner/analyze_p4_g0c_calibration.py"
SPEC = importlib.util.spec_from_file_location("analyze_p4_g0c_calibration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


CSV_FIELDS = [
    "schema_version", "stamp", "planning_attempt_id", "collision_segment_id",
    "request_hash", "snapshot_generation_id", "snapshot_stamp_s",
    "snapshot_frame", "query_base_time_s", "occupancy_epoch", "status",
    "reason", "selection_applied", "original_hash", "risk_hash",
    "selected_hash", "original_sample_count", "original_valid_count",
    "original_unknown_count", "original_stale_count", "original_non_finite_count",
    "original_mean", "original_max", "risk_sample_count", "risk_valid_count",
    "risk_unknown_count", "risk_stale_count", "risk_non_finite_count",
    "risk_mean", "risk_max", "original_path_length", "risk_path_length",
    "path_length_ratio", "original_search_latency_ms", "risk_search_latency_ms",
    "total_search_latency_ms",
]


class P4G0CAnalyzerTest(unittest.TestCase):
    def setUp(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v1.json",
            REPO / "config/icra27/p4_threshold_registry_v1.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )

    def _row(self, index):
        return {
            "schema_version": "p4_collision_guide_decision_v1",
            "stamp": str(index),
            "planning_attempt_id": str(index + 1),
            "collision_segment_id": "1",
            "request_hash": f"request-{index}",
            "snapshot_generation_id": "7",
            "snapshot_stamp_s": "10.0",
            "snapshot_frame": "map",
            "query_base_time_s": "10.0",
            "occupancy_epoch": "19",
            "status": "ORIGINAL_SELECTED",
            "reason": "METRICS_ONLY",
            "selection_applied": "0",
            "original_hash": f"original-{index}",
            "risk_hash": f"risk-{index}",
            "selected_hash": f"original-{index}",
            "original_sample_count": "200",
            "original_valid_count": "200",
            "original_unknown_count": "0",
            "original_stale_count": "0",
            "original_non_finite_count": "0",
            "original_mean": "2.0",
            "original_max": "3.0",
            "risk_sample_count": "200",
            "risk_valid_count": "200",
            "risk_unknown_count": "0",
            "risk_stale_count": "0",
            "risk_non_finite_count": "0",
            "risk_mean": "1.0",
            "risk_max": "1.0",
            "original_path_length": "10.0",
            "risk_path_length": "11.0",
            "path_length_ratio": "1.1",
            "original_search_latency_ms": "40.0",
            "risk_search_latency_ms": "50.0",
            "total_search_latency_ms": "90.0",
        }

    def _make_bundle(self, root, row_counts=None, mutate=None):
        plan = MODULE.expand_run_plan(self.bundle.protocol, root)
        row_counts = row_counts or [7] * 15
        global_index = 0
        for run_index, (record, count) in enumerate(zip(plan, row_counts)):
            run_dir = Path(record["run_dir"])
            run_dir.mkdir(parents=True)
            csv_path = run_dir / "p4_decisions.csv"
            rows = []
            for _ in range(count):
                rows.append(self._row(global_index))
                global_index += 1
            manifest = {
                "schema_version": "p4_g0c_run_manifest_v1",
                "gate": "G0C",
                "run_id": record["run_id"],
                "seed": record["seed"],
                "repetition": record["repetition"],
                "protocol_sha256": self.bundle.protocol_sha256,
                "registry_sha256": self.bundle.registry_sha256,
                "fixture_sha256": self.bundle.fixture_sha256,
                "effective_config_sha256": MODULE.effective_config_sha256(
                    self.bundle.protocol["effective_values"]
                ),
                "effective_values": dict(
                    self.bundle.protocol["effective_values"]
                ),
                "csv_path": str(csv_path.resolve()),
                "required_process_set": ["iap_rosnode", "ego_planner_node"],
                "required_processes_ok": True,
                "runner_state": "COMPLETE",
                "launch_exit_code": 0,
                "retry_count": 0,
                "record_bag": False,
                "start_rviz": False,
                "selection_applied": False,
                "immutable_run_id": True,
                "overwrite_allowed": False,
            }
            if mutate is not None:
                mutate(run_index, manifest, rows)
            with csv_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            (run_dir / "p4_g0c_run_manifest.json").write_text(
                json.dumps(manifest, sort_keys=True) + "\n"
            )

    def test_type7_quantile_has_stable_tie_sources_and_unit_conversion(self):
        q = MODULE.quantile_type7([(3.0, 7), (1.0, 4), (1.0, 2), (5.0, 8)], 0.5)
        self.assertEqual(q["value"], 2.0)
        self.assertEqual(q["lower_source_row_index"], 4)
        self.assertEqual(q["upper_source_row_index"], 7)
        self.assertEqual(q["fraction"], 0.5)
        self.assertEqual(MODULE.milliseconds_to_seconds(90.0), 0.09)
        self.assertEqual(
            MODULE.quantile_type7([(3.0, 2), (1.0, 1)], 0.0)["value"], 1.0
        )
        self.assertEqual(
            MODULE.quantile_type7([(3.0, 2), (1.0, 1)], 1.0)["value"], 3.0
        )
        with self.assertRaisesRegex(MODULE.AnalysisError, "finite"):
            MODULE.quantile_type7([(math.nan, 0)], 0.95)

    def test_exact_100_boundary_is_eligible_and_draft_uses_frozen_formulas(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root, [7] * 10 + [6] * 5)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "DRAFT_ELIGIBLE")
        self.assertEqual(result["complete_decision_count"], 100)
        self.assertEqual(result["denominator_decision_count"], 100)
        self.assertEqual(result["failures"], [])
        draft = result["threshold_draft"]
        self.assertEqual(draft["state"], "DRAFT_UNCALIBRATED")
        self.assertEqual(draft["gates"]["mean_improvement_min"]["value"], 1.0)
        self.assertEqual(draft["gates"]["max_improvement_min"]["value"], 2.0)
        self.assertAlmostEqual(draft["gates"]["path_ratio_max"]["value"], 1.12)
        self.assertAlmostEqual(
            draft["gates"]["total_search_timeout_s"]["value"], 0.108
        )
        self.assertEqual(len(draft["calibration_bundle_sha256"]), 64)
        self.assertNotIn("PASS", json.dumps(draft))
        self.assertNotIn("FROZEN", json.dumps(draft))

    def test_invalid_rows_and_manifest_truth_fail_closed_without_filtering(self):
        cases = {
            "metrics_only_false": lambda _, manifest, rows: (
                manifest["effective_values"].update({"p4.metrics_only": False})
            ),
            "selection_applied": lambda _, manifest, rows: (
                rows[0].update({"selection_applied": "1"})
            ),
            "coverage": lambda _, manifest, rows: (
                rows[0].update({"risk_valid_count": "199"})
            ),
            "timeout": lambda _, manifest, rows: (
                rows[0].update({"risk_search_latency_ms": "200.0"})
            ),
            "path_ratio": lambda _, manifest, rows: (
                rows[0].update({"path_length_ratio": "1.3001"})
            ),
            "noise_floor": lambda _, manifest, rows: (
                rows[0].update({"original_mean": "1.0", "risk_mean": "1.0"})
            ),
            "hash_mismatch": lambda _, manifest, rows: (
                manifest.update({"registry_sha256": "0" * 64})
            ),
            "config_mismatch": lambda _, manifest, rows: (
                manifest["effective_values"].update({
                    "p1.debug_csv_enable": True
                })
            ),
        }
        for expected, mutate_one in cases.items():
            def mutate(index, manifest, rows):
                if index == 0:
                    mutate_one(index, manifest, rows)

            with self.subTest(expected=expected), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root, mutate=mutate)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertEqual(result["denominator_decision_count"], 105)
                self.assertTrue(any(expected in item for item in result["failures"]))
                self.assertNotIn("threshold_draft", result)

    def test_missing_duplicate_and_below_100_matrix_cannot_be_excluded(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root, [6] * 15)
            missing = root / "p4-g0c-seed271-rep03"
            for child in missing.iterdir():
                child.unlink()
            missing.rmdir()
            duplicate_manifest = (
                root / "p4-g0c-seed271-rep02" / "p4_g0c_run_manifest.json"
            )
            payload = json.loads(duplicate_manifest.read_text())
            payload["run_id"] = "p4-g0c-seed271-rep01"
            duplicate_manifest.write_text(json.dumps(payload, sort_keys=True) + "\n")
            result = MODULE.analyze(self.bundle, root)

        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any("missing_run" in item for item in result["failures"]))
        self.assertTrue(any("duplicate_run" in item for item in result["failures"]))
        self.assertTrue(any(
            "minimum_complete_decisions" in item for item in result["failures"]
        ))
        self.assertEqual(result["denominator_decision_count"], 84)


if __name__ == "__main__":
    unittest.main()
