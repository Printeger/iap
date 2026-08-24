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
    @staticmethod
    def _copy_v2_bundle_root(root):
        paths = {
            "protocol": "config/icra27/p4_g0c_protocol_v2.json",
            "registry": "config/icra27/p4_threshold_registry_v2.json",
            "fixture": "config/icra27/p4_g0c_live_fixture_v1.json",
            "dependency": "config/icra27/p4_g0c_runtime_dependencies_v2.json",
            "lineage": "config/icra27/p4_g0c_replacement_lineage_v2.json",
            "v1_protocol": "config/icra27/p4_g0c_protocol_v1.json",
            "raw_manifest": (
                "results/icra27/icra046/preflight/raw_runs_manifest.tsv"
            ),
            "runner_state": (
                "results/icra27/icra046/runs/p4_g0c_runner_state.json"
            ),
        }
        copied = {}
        for name, relative in paths.items():
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((REPO / relative).read_bytes())
            copied[name] = target
        return copied

    def test_v2_replacement_is_exact_unique_and_scientifically_equivalent(self):
        v1 = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v1.json",
            REPO / "config/icra27/p4_threshold_registry_v1.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V1,
        )
        v2 = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        scientific_keys = {
            "effective_values", "matrix_order", "minimum_complete_decisions",
            "no_exclusion", "no_overwrite", "no_retry",
            "numerical_noise_floor", "path_ratio_consistency", "quantiles",
            "repetitions", "seeds", "threshold_formulas", "live_fixture",
        }
        self.assertEqual(
            {key: v2.protocol[key] for key in scientific_keys},
            {key: v1.protocol[key] for key in scientific_keys},
        )
        self.assertEqual(v2.protocol["run_duration_s"], 90)
        run_ids = v2.protocol["registered_run_ids"]
        self.assertEqual(len(run_ids), 15)
        self.assertEqual(len(set(run_ids)), 15)
        self.assertEqual(run_ids[0], "p4-g0c-r2-seed211-rep01")
        self.assertEqual(run_ids[-1], "p4-g0c-r2-seed271-rep03")
        self.assertTrue(all("p4-g0c-seed" not in run_id for run_id in run_ids))

        invalid = json.loads(json.dumps(v2.protocol))
        invalid["registered_run_ids"][0] = "p4-g0c-seed211-rep01"
        with self.assertRaisesRegex(
            MODULE.ProtocolError, "exact immutable matrix"
        ):
            MODULE.validate_protocol(invalid)

    def test_v2_lineage_and_proposed_registry_preserve_icra046_truth(self):
        bundle = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        lineage = MODULE.load_canonical_json(
            REPO / bundle.protocol["replacement_lineage"]["path"]
        )
        failed = lineage["disqualified_execution"]
        self.assertEqual(
            lineage["superseded_protocol"]["sha256"],
            "9e89ea42675459a63853d98845f02b7fe5b9434a9f28fcbd6ef5ba1bc5bd906d",
        )
        self.assertEqual(
            failed["raw_manifest"]["sha256"],
            "f307e61a90707d6da5a38138558a97447c5267ef9a5184f3df92ca8b97079438",
        )
        self.assertEqual(
            failed["runner_state"]["sha256"],
            "a6dba6376b225f2fd00c218bdd19f911b9183e5e53a868f55cb0f1914d474ef1",
        )
        self.assertEqual(failed["failed_run_id"], "p4-g0c-seed211-rep01")
        self.assertEqual(
            (failed["attempted_run_count"], failed["complete_run_count"],
             failed["retry_count"], failed["analyzer_invocations"]),
            (1, 0, 0, 0),
        )
        self.assertEqual(
            failed["replacement_reason"],
            "PRELIVE_DEPENDENCY_GATE_VIOLATION_NO_CALIBRATION_DATA",
        )
        self.assertFalse(failed["threshold_draft_exists"])
        self.assertFalse(failed["threshold_application_possible"])
        self.assertEqual(bundle.registry["state"], "PROPOSED_UNCALIBRATED")
        self.assertTrue(all(
            value is None for value in bundle.registry["gates"].values()
        ))
        self.assertIsNone(bundle.registry["calibration_bundle_sha256"])
        self.assertFalse(bundle.registry["application_enabled"])

    def test_v2_full_file_trust_anchor_rejects_coordinated_and_isolated_drift(self):
        for case in ("coordinated", "isolated_registry"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                paths = self._copy_v2_bundle_root(Path(tmp))
                protocol = json.loads(paths["protocol"].read_text())
                registry = json.loads(paths["registry"].read_text())
                if case == "coordinated":
                    protocol["unreviewed_note"] = "coordinated drift"
                    paths["protocol"].write_bytes(MODULE.canonical_bytes(protocol))
                    registry["protocol_sha256"] = MODULE.sha256_file(
                        paths["protocol"]
                    )
                else:
                    registry["unreviewed_note"] = "coordinated bytes not required"
                paths["registry"].write_bytes(MODULE.canonical_bytes(registry))

                with self.assertRaisesRegex(
                    MODULE.ProtocolError, "v2 trust anchor"
                ):
                    MODULE.load_protocol_bundle(
                        paths["protocol"], paths["registry"], paths["fixture"]
                    )

    def test_v2_trusted_mode_rejects_coordinated_schema_downgrade(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = self._copy_v2_bundle_root(Path(tmp))
            paths["protocol"].write_bytes(
                (REPO / "config/icra27/p4_g0c_protocol_v1.json").read_bytes()
            )
            paths["registry"].write_bytes(
                (REPO / "config/icra27/p4_threshold_registry_v1.json").read_bytes()
            )

            with self.assertRaisesRegex(
                MODULE.ProtocolError, "schema does not match trusted mode"
            ):
                MODULE.load_protocol_bundle(
                    paths["protocol"], paths["registry"], paths["fixture"]
                )

    def test_shared_validator_freezes_complete_scientific_contract(self):
        original = MODULE.load_canonical_json(
            REPO / "config/icra27/p4_g0c_protocol_v2.json"
        )
        mutations = {
            "formula": lambda payload: payload["threshold_formulas"].update({
                "max_improvement_min": "Q50(original_max-risk_max)"
            }),
            "floor_derivation": lambda payload: payload[
                "numerical_noise_floor"
            ]["derivation"].update({"multiplier": 2048}),
            "quantile_definition": lambda payload: payload["quantiles"].update({
                "definition": "nearest rank"
            }),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                payload = json.loads(json.dumps(original))
                mutate(payload)
                with self.assertRaisesRegex(
                    MODULE.ProtocolError, "scientific contract"
                ):
                    MODULE.validate_protocol(payload)

    def test_v2_exact_types_reject_python_equal_substitutions(self):
        original = MODULE.load_canonical_json(
            REPO / "config/icra27/p4_g0c_protocol_v2.json"
        )
        mutations = {
            "seed_float": lambda payload: payload["seeds"].__setitem__(0, 211.0),
            "boolean_integer": lambda payload: payload.update({"no_retry": 1}),
            "effective_boolean_integer": lambda payload: payload[
                "effective_values"
            ].update({"p1.metrics_only": 0}),
            "floor_integer_float": lambda payload: payload[
                "numerical_noise_floor"
            ]["derivation"].update({"multiplier": 4096.0}),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                payload = json.loads(json.dumps(original))
                mutate(payload)
                with self.assertRaises(MODULE.ProtocolError):
                    MODULE.validate_protocol(payload)

    def test_registered_bundle_has_exact_seed_major_matrix_and_hash_binding(self):
        bundle = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v1.json",
            REPO / "config/icra27/p4_threshold_registry_v1.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V1,
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
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V1,
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

    def test_inventory_rejects_secondary_v1_and_v2_run_manifests(self):
        for schema in (
            "p4_g0c_run_manifest_v1", "p4_g0c_run_manifest_v2"
        ):
            with self.subTest(schema=schema), tempfile.TemporaryDirectory() as tmp:
                run_dir = Path(tmp) / "p4-g0c-r2-seed211-rep01"
                run_dir.mkdir(parents=True)
                (run_dir / "secondary.json").write_text(
                    json.dumps({"schema_version": schema}) + "\n"
                )
                with self.assertRaisesRegex(
                    MODULE.ProtocolError, "secondary G0C run manifest"
                ):
                    MODULE.make_run_artifact_inventory(
                        run_dir, "p4-g0c-r2-seed211-rep01"
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
