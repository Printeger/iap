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


EXPECTED_TOP_LEVEL_EFFECTIVE_KEYS = {
    "manager/p1_collision_fanout_clearance_m",
    "manager/p1_collision_fanout_mirror_y",
    "manager/p1_collision_fanout_preserve_homotopies",
    "manager/use_distinctive_trajs",
    "p0.enable_risk_grid",
    "p1.debug_csv_enable",
    "p1.metrics_only",
    "p1.use_integrity_cost",
    "p2.debug_csv_enable",
    "p2.enable_candidate_ranking",
    "p2.metrics_only",
    "p3.debug_csv_enable",
    "p3.enable_global_reference_bias",
    "p3.enable_local_reference_bias",
    "p4.debug_csv_enable",
    "p4.enable_risk_aware_astar",
    "p4.metrics_only",
    "planner_enable_p1",
    "planner_enable_p2",
    "planner_enable_p3_global",
    "planner_enable_p3_local",
    "planner_enable_p4",
    "planner_enable_p5_final",
    "planner_enable_p5_runtime",
    "planner_safety_profile",
    "record_bag",
    "run_validator",
    "start_rviz",
}


class P4G0CProtocolTest(unittest.TestCase):
    def test_v6_extends_only_temporal_and_occupied_support_identity(self):
        v5 = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v5.json",
            REPO / "config/icra27/p4_threshold_registry_v5.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V5,
        )
        v6 = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v6.json",
            REPO / "config/icra27/p4_threshold_registry_v6.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V6,
        )
        effective = dict(v6.protocol["effective_values"])
        self.assertEqual(
            effective.pop("p0.horizons_s"),
            [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
        )
        self.assertEqual(
            effective.pop("p4.cost_query_policy"),
            "CONSERVATIVE_OCCUPIED_COST_SUPPORT",
        )
        self.assertEqual(effective, v5.protocol["effective_values"])
        self.assertEqual(len(v6.protocol["registered_run_ids"]), 15)
        self.assertTrue(all(
            run_id.startswith("p4-g0c-r6-")
            for run_id in v6.protocol["registered_run_ids"]
        ))
        self.assertFalse(
            set(v5.protocol["registered_run_ids"])
            & set(v6.protocol["registered_run_ids"])
        )

    def test_v5_runtime_worker_binding_requires_exact_typed_four_four(self):
        valid = {
            "p0.predictor.requested_worker_count": 4,
            "p0.predictor.effective_worker_count": 4,
        }
        MODULE.validate_v5_runtime_worker_binding(valid)
        for key in valid:
            for bad_value in (None, True, 4.0, "4", 1, 3, 5):
                with self.subTest(key=key, bad_value=bad_value):
                    invalid = dict(valid)
                    invalid[key] = bad_value
                    with self.assertRaisesRegex(
                            MODULE.ProtocolError, key):
                        MODULE.validate_v5_runtime_worker_binding(invalid)

    def test_v5_versions_only_fixture_geometry_and_registers_disjoint_r5(self):
        v4 = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v4.json",
            REPO / "config/icra27/p4_threshold_registry_v4.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V4,
        )
        v5 = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v5.json",
            REPO / "config/icra27/p4_threshold_registry_v5.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V5,
        )
        v1_fixture = json.loads((
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        ).read_text())
        v2_fixture = json.loads(json.dumps(v5.fixture))
        v2_fixture["schema_version"] = "p4_g0c_fixture_v1"
        v2_fixture["map"]["central_obstacle"]["x_m"] = [-8.0, -3.0]
        self.assertEqual(v2_fixture, v1_fixture)
        frozen = {
            "matrix_order", "minimum_complete_decisions",
            "no_exclusion", "no_overwrite", "no_retry",
            "numerical_noise_floor", "p0_profile_binding",
            "path_ratio_consistency", "quantiles", "repetitions",
            "run_duration_s", "seeds", "threshold_formulas",
        }
        self.assertEqual(
            {key: v5.protocol[key] for key in frozen},
            {key: v4.protocol[key] for key in frozen},
        )
        v5_effective = dict(v5.protocol["effective_values"])
        self.assertEqual(v5_effective.pop("p0.predictor.worker_count"), 4)
        self.assertEqual(v5_effective, v4.protocol["effective_values"])
        self.assertEqual(len(v5.protocol["registered_run_ids"]), 15)
        self.assertTrue(all(
            run_id.startswith("p4-g0c-r5-")
            for run_id in v5.protocol["registered_run_ids"]
        ))
        self.assertFalse(
            set(v5.protocol["registered_run_ids"])
            & set(v4.protocol["registered_run_ids"])
        )

    def test_v4_binds_exact_p0_profile_and_disjoint_r4_matrix(self):
        bundle = MODULE.load_protocol_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v4.json",
            REPO / "config/icra27/p4_threshold_registry_v4.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V4,
        )
        effective = bundle.protocol["effective_values"]
        self.assertEqual(effective["p0.predictor.sigma_grow_m_sqrt_s"], 0.01)
        self.assertIs(type(effective["p0.predictor.sigma_grow_m_sqrt_s"]), float)
        self.assertEqual(
            effective["p0.predictor.sigma_growth_profile"],
            "legacy_iap_rq320_baseline_v1",
        )
        self.assertEqual(len(bundle.protocol["registered_run_ids"]), 15)
        self.assertTrue(all(
            run_id.startswith("p4-g0c-r4-")
            for run_id in bundle.protocol["registered_run_ids"]
        ))

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

    def test_v3_replacement_is_disjoint_and_preserves_science_and_failures(self):
        bundles = {
            version: MODULE.load_protocol_bundle(
                REPO / f"config/icra27/p4_g0c_protocol_v{version}.json",
                REPO / f"config/icra27/p4_threshold_registry_v{version}.json",
                REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
                expected_protocol_schema=getattr(
                    MODULE, f"PROTOCOL_SCHEMA_V{version}"
                ),
            )
            for version in (1, 2, 3)
        }
        scientific_keys = {
            "effective_values", "matrix_order", "minimum_complete_decisions",
            "no_exclusion", "no_overwrite", "no_retry",
            "numerical_noise_floor", "path_ratio_consistency", "quantiles",
            "repetitions", "run_duration_s", "seeds", "threshold_formulas",
            "live_fixture",
        }
        self.assertEqual(
            {key: bundles[3].protocol[key] for key in scientific_keys},
            {key: bundles[2].protocol[key] for key in scientific_keys},
        )
        ids = {
            version: set(bundle.protocol["registered_run_ids"])
            for version, bundle in bundles.items()
        }
        self.assertEqual(len(ids[3]), 15)
        self.assertFalse(ids[3] & ids[1])
        self.assertFalse(ids[3] & ids[2])
        self.assertTrue(all(run_id.startswith("p4-g0c-r3-") for run_id in ids[3]))

        lineage = MODULE.load_canonical_json(
            REPO / bundles[3].protocol["replacement_lineage"]["path"]
        )
        failed = {
            item["task_id"]: item for item in lineage["failed_live_executions"]
        }
        self.assertEqual(set(failed), {"ICRA-046", "ICRA-051"})
        icra051 = failed["ICRA-051"]
        self.assertEqual(icra051["failed_run_id"], "p4-g0c-r2-seed211-rep01")
        self.assertEqual(
            icra051["runner_state"]["sha256"],
            "7c3cafc505ad33e7e8631a2ed1534bf5e21c6cf4f4d9eb252319a250989846a7",
        )
        self.assertEqual(
            (
                icra051["attempted_run_count"],
                icra051["complete_run_count"],
                icra051["retry_count"],
            ),
            (1, 0, 0),
        )
        self.assertEqual(
            icra051["failure_classification"],
            "SELF_INDUCED_NON_REPOSITORY_LOCAL_ROS_LOG_ENVIRONMENT",
        )
        self.assertEqual(
            icra051["external_log"]["sha256"],
            "f506e5565d73ad601673c814635797c360f650c7be3c4356e9217449df2458e7",
        )
        registry = bundles[3].registry
        self.assertEqual(registry["state"], "PROPOSED_UNCALIBRATED")
        self.assertFalse(registry["application_enabled"])
        self.assertIsNone(registry["calibration_bundle_sha256"])
        self.assertTrue(all(value is None for value in registry["gates"].values()))

    def test_top_level_production_effective_mapping_is_exact_and_complete(self):
        mapping = MODULE.TEST_PLANNER_TOP_LEVEL_EFFECTIVE_MAP
        self.assertEqual(len(mapping), 28)
        self.assertEqual(set(mapping), EXPECTED_TOP_LEVEL_EFFECTIVE_KEYS)
        self.assertEqual(set(mapping.values()), EXPECTED_TOP_LEVEL_EFFECTIVE_KEYS)
        self.assertTrue(all(key == value for key, value in mapping.items()))
        protocol = MODULE.load_canonical_json(
            REPO / "config/icra27/p4_g0c_protocol_v2.json"
        )
        self.assertTrue(
            EXPECTED_TOP_LEVEL_EFFECTIVE_KEYS
            < set(protocol["effective_values"])
        )

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

    def test_r6_inventory_records_only_exact_safe_iap_logs_latest_alias(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-r6-seed211-rep01"
            logs = run_dir / "runtime/iap_logs"
            target = logs / "20260825T125103Z_278"
            target.mkdir(parents=True)
            (target / "metrics.csv").write_text("metric,value\n")
            (logs / "latest").symlink_to(target.name)

            inventory = MODULE.make_run_artifact_inventory(
                run_dir, run_dir.name
            )
            entries = MODULE.validate_run_artifact_inventory(
                inventory, run_dir, run_dir.name
            )

            self.assertEqual(
                inventory["schema_version"],
                "p4_g0c_run_artifact_inventory_v2",
            )
            self.assertIn({
                "path": "runtime/iap_logs/latest",
                "type": "symlink",
                "target": "20260825T125103Z_278",
            }, entries)
            self.assertIn({
                "path": "runtime/iap_logs/20260825T125103Z_278",
                "type": "directory",
            }, entries)

    def test_r6_inventory_rejects_nonexact_or_unsafe_latest_aliases(self):
        cases = {
            "alternate_name": ("runtime/iap_logs/current", "target", "dir"),
            "absolute_target": (
                "runtime/iap_logs/latest", "/tmp/outside", "none"
            ),
            "nested_target": (
                "runtime/iap_logs/latest", "nested/target", "dir"
            ),
            "dot_target": ("runtime/iap_logs/latest", ".", "none"),
            "dotdot_target": ("runtime/iap_logs/latest", "..", "none"),
            "dangling_target": (
                "runtime/iap_logs/latest", "missing", "none"
            ),
            "regular_file_target": (
                "runtime/iap_logs/latest", "target", "file"
            ),
            "symlink_chain": (
                "runtime/iap_logs/latest", "intermediate", "chain"
            ),
        }
        for label, (relative, target_value, target_kind) in cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                run_dir = Path(tmp) / "p4-g0c-r6-seed211-rep01"
                link = run_dir / relative
                link.parent.mkdir(parents=True)
                target = link.parent / target_value
                if target_kind == "dir":
                    target.mkdir(parents=True)
                elif target_kind == "file":
                    target.write_text("not a directory\n")
                elif target_kind == "chain":
                    (link.parent / "real-target").mkdir()
                    target.symlink_to("real-target")
                link.symlink_to(target_value)
                with self.assertRaisesRegex(MODULE.ProtocolError, "symlink"):
                    MODULE.make_run_artifact_inventory(run_dir, run_dir.name)

    def test_r6_inventory_validation_rechecks_alias_target_type(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-r6-seed211-rep01"
            logs = run_dir / "runtime/iap_logs"
            target = logs / "session"
            target.mkdir(parents=True)
            alias = logs / "latest"
            alias.symlink_to(target.name)
            inventory = MODULE.make_run_artifact_inventory(
                run_dir, run_dir.name
            )

            target.rmdir()
            target.write_text("replaced after inventory\n")
            with self.assertRaisesRegex(MODULE.ProtocolError, "symlink"):
                MODULE.validate_run_artifact_inventory(
                    inventory, run_dir, run_dir.name
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
