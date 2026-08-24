import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts" / "dev_planner" / "p4_g0c_protocol.py"
SPEC = importlib.util.spec_from_file_location("p4_g0c_protocol", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class P4G0CProtocolTest(unittest.TestCase):
    def test_registered_bundle_has_exact_seed_major_matrix_and_hash_binding(self):
        bundle = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v1.json",
            REPO / "config/icra27/p4_threshold_registry_v1.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        plan = MODULE.expand_run_plan(bundle.protocol, Path("/runs"))

        self.assertEqual(bundle.protocol["seeds"], [211, 223, 237, 253, 271])
        self.assertEqual(bundle.protocol["repetitions"], [1, 2, 3])
        self.assertEqual(len(plan), 15)
        self.assertEqual(plan[0]["run_id"], "p4-g0c-seed211-rep01")
        self.assertEqual(plan[-1]["run_id"], "p4-g0c-seed271-rep03")
        self.assertEqual(
            [(item["seed"], item["repetition"]) for item in plan],
            [(seed, repetition) for seed in [211, 223, 237, 253, 271]
             for repetition in [1, 2, 3]],
        )
        self.assertEqual(
            bundle.registry["protocol_sha256"], bundle.protocol_sha256
        )
        self.assertEqual(
            bundle.protocol["live_fixture"]["sha256"], bundle.fixture_sha256
        )

    def test_protocol_freezes_metrics_only_noise_and_quantile_contract(self):
        bundle = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v1.json",
            REPO / "config/icra27/p4_threshold_registry_v1.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        protocol = bundle.protocol
        effective = protocol["effective_values"]
        self.assertEqual(protocol["minimum_complete_decisions"], 100)
        self.assertEqual(effective["gate"], "G0C")
        self.assertTrue(effective["p4.metrics_only"])
        self.assertFalse(effective["selection_applied"])
        self.assertEqual(effective["p4.max_extra_path_ratio"], 1.30)
        self.assertEqual(effective["p4.per_search_timeout_s"], 0.2)
        self.assertEqual(protocol["numerical_noise_floor"], {
            "value": 1e-12,
            "unit": "risk_cost",
            "derivation": {
                "artifact": "ieee754_binary64_precision_bound_v1",
                "binary64_epsilon": 2.220446049250313e-16,
                "multiplier": 4096,
                "unrounded_bound": 9.094947017729282e-13,
                "rounding": "round_up_to_1e-12",
                "source": "deterministic_numeric_precision_only",
            },
            "calibration_mutable": False,
        })
        self.assertEqual(protocol["quantiles"]["method"], "TYPE_7_LINEAR")
        self.assertEqual(
            protocol["quantiles"]["tie_behavior"],
            "stable_input_row_index",
        )
        self.assertEqual(
            protocol["path_ratio_consistency"]["absolute_tolerance"],
            2e-5,
        )

    def test_shared_decision_csv_schema_matches_production_header_exactly(self):
        self.assertEqual(MODULE.DECISION_CSV_COLUMNS, (
            "schema_version", "stamp", "planning_attempt_id",
            "collision_segment_id", "request_hash",
            "snapshot_generation_id", "snapshot_stamp_s", "snapshot_frame",
            "query_base_time_s", "occupancy_epoch", "status", "reason",
            "selection_applied", "original_hash", "risk_hash",
            "selected_hash", "original_sample_count", "original_valid_count",
            "original_unknown_count", "original_stale_count",
            "original_non_finite_count", "original_mean", "original_max",
            "risk_sample_count", "risk_valid_count", "risk_unknown_count",
            "risk_stale_count", "risk_non_finite_count", "risk_mean",
            "risk_max", "original_path_length", "risk_path_length",
            "path_length_ratio", "original_search_latency_ms",
            "risk_search_latency_ms", "total_search_latency_ms",
        ))

    def test_run_artifact_inventory_schema_is_canonical_complete_and_symlink_free(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-seed211-rep01"
            launch_manifest = run_dir / "exports/test_planner_manifest.json"
            launch_manifest.parent.mkdir(parents=True)
            launch_manifest.write_text("{}\n")
            timing = run_dir / "runtime/profiling/iap_timing.csv"
            timing.parent.mkdir(parents=True)
            timing.write_text("stamp,duration_ms\n")
            (run_dir / "stdout.log").write_text("done\n")
            inventory = MODULE.make_run_artifact_inventory(
                run_dir, "p4-g0c-seed211-rep01"
            )
            entries = MODULE.validate_run_artifact_inventory(
                inventory, run_dir, "p4-g0c-seed211-rep01"
            )
            self.assertEqual(inventory["schema_version"], (
                "p4_g0c_run_artifact_inventory_v1"
            ))
            self.assertEqual(
                [entry["path"] for entry in entries],
                [
                    "exports",
                    "exports/test_planner_manifest.json",
                    "runtime",
                    "runtime/profiling",
                    "runtime/profiling/iap_timing.csv",
                    "stdout.log",
                ],
            )
            self.assertTrue(all(
                set(entry) == {"path", "type", "size_bytes", "sha256"}
                for entry in entries if entry["type"] == "regular"
            ))

            duplicate = json.loads(json.dumps(inventory))
            duplicate["entries"].append(dict(duplicate["entries"][-1]))
            with self.assertRaisesRegex(
                MODULE.ProtocolError, "unordered or duplicate"
            ):
                MODULE.validate_run_artifact_inventory(
                    duplicate, run_dir, "p4-g0c-seed211-rep01"
                )

            (run_dir / "linked.log").symlink_to(run_dir / "stdout.log")
            with self.assertRaisesRegex(MODULE.ProtocolError, "symlink"):
                MODULE.collect_run_artifact_entries(run_dir)

    def test_inventory_does_not_blacklist_production_like_filenames(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-seed211-rep01"
            runtime = run_dir / "runtime"
            runtime.mkdir(parents=True)
            (runtime / "p4_decisions_metrics.csv").write_text(
                "metric,value\nlatency,1\n"
            )
            (runtime / "p4_g0c_export_manifest.json").write_text(
                '{"schema_version":"runtime_export_v1"}\n'
            )

            inventory = MODULE.make_run_artifact_inventory(
                run_dir, "p4-g0c-seed211-rep01"
            )

            self.assertEqual(
                [entry["path"] for entry in inventory["entries"]],
                [
                    "runtime",
                    "runtime/p4_decisions_metrics.csv",
                    "runtime/p4_g0c_export_manifest.json",
                ],
            )

    def test_noncanonical_json_and_calibrated_registry_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            noncanonical = root / "pretty.json"
            noncanonical.write_text(json.dumps({"b": 1, "a": 2}, indent=2))
            with self.assertRaisesRegex(MODULE.ProtocolError, "canonical"):
                MODULE.load_canonical_json(noncanonical)

            registry = {
                "schema_version": "p4_threshold_registry_v1",
                "state": "FROZEN",
                "protocol_sha256": "a" * 64,
                "numerical_noise_floor": {"value": 1e-12, "unit": "risk_cost"},
                "calibration_bundle_sha256": "b" * 64,
                "gates": {
                    "mean_improvement_min": 0.1,
                    "max_improvement_min": 0.1,
                    "path_ratio_max": 1.2,
                    "total_search_timeout_s": 0.3,
                },
                "application_enabled": True,
            }
            with self.assertRaisesRegex(MODULE.ProtocolError, "uncalibrated"):
                MODULE.validate_proposed_registry(registry)


if __name__ == "__main__":
    unittest.main()
