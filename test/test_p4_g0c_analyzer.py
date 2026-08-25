import csv
import contextlib
import hashlib
import importlib.util
import io
import json
import math
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock


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

TOP_LEVEL_EFFECTIVE_KEYS = (
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
)


def top_level_effective_values(protocol):
    return {
        key: protocol["effective_values"][key]
        for key in TOP_LEVEL_EFFECTIVE_KEYS
    }


def changed_value(value):
    if isinstance(value, bool):
        return not value
    if isinstance(value, (int, float)):
        return value + 1.0
    return f"{value}-drift"


def wrong_type_value(value):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        return int(value)
    return [value]


class P4G0CAnalyzerTest(unittest.TestCase):
    def setUp(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v1.json",
            REPO / "config/icra27/p4_threshold_registry_v1.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V1,
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

    def _write_artifact_inventory(self, run_dir, run_id):
        inventory_path = run_dir / "p4_g0c_artifact_inventory.json"
        entries = []
        for path in sorted(
            run_dir.rglob("*"), key=lambda item: item.relative_to(run_dir).as_posix()
        ):
            relative = path.relative_to(run_dir).as_posix()
            if path == inventory_path:
                continue
            self.assertFalse(path.is_symlink())
            if path.is_dir():
                entries.append({"path": relative, "type": "directory"})
            else:
                raw = path.read_bytes()
                entries.append({
                    "path": relative,
                    "type": "regular",
                    "size_bytes": len(raw),
                    "sha256": hashlib.sha256(raw).hexdigest(),
                })
        payload = {
            "schema_version": "p4_g0c_run_artifact_inventory_v1",
            "run_id": run_id,
            "excluded_path": "p4_g0c_artifact_inventory.json",
            "entries": entries,
        }
        raw_inventory = (
            json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n"
        )
        inventory_path.write_text(raw_inventory)
        launch_manifests = list(
            (run_dir / "exports").rglob("test_planner_manifest.json")
        )
        self.assertEqual(len(launch_manifests), 1)
        launch_manifest_path = launch_manifests[0]
        return {
            "artifact_inventory_path": str(inventory_path.resolve()),
            "artifact_inventory_sha256": hashlib.sha256(
                raw_inventory.encode()
            ).hexdigest(),
            "test_planner_manifest_path": str(launch_manifest_path.resolve()),
            "test_planner_manifest_sha256": hashlib.sha256(
                launch_manifest_path.read_bytes()
            ).hexdigest(),
        }

    def _make_bundle(self, root, row_counts=None, mutate=None):
        plan = MODULE.expand_run_plan(self.bundle.protocol, root)
        protocol_schema = self.bundle.protocol["schema_version"]
        replacement = protocol_schema in {
            MODULE.PROTOCOL_SCHEMA_V2, MODULE.PROTOCOL_SCHEMA_V3
        }
        hardened = protocol_schema == MODULE.PROTOCOL_SCHEMA_V3
        if hardened:
            for directory in (
                "home", "ros_home", "ros_logs", "tmp", "xdg_runtime",
            ):
                (root / "launch_environment" / directory).mkdir(
                    parents=True, exist_ok=True
                )
            (root / "launch_environment" / "xdg_runtime").chmod(0o700)
        row_counts = row_counts or [7] * 15
        global_index = 0
        inventory_bindings = []
        for run_index, (record, count) in enumerate(zip(plan, row_counts)):
            run_dir = Path(record["run_dir"])
            run_dir.mkdir(parents=True)
            csv_path = run_dir / "p4_decisions.csv"
            rows = []
            for _ in range(count):
                rows.append(self._row(global_index))
                global_index += 1
            manifest = {
                "schema_version": (
                    "p4_g0c_run_manifest_v3" if hardened
                    else "p4_g0c_run_manifest_v2" if replacement
                    else "p4_g0c_run_manifest_v1"
                ),
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
                "experiment": (
                    "p4_g0c_metrics_calibration_v3" if hardened
                    else "p4_g0c_metrics_calibration_v2" if replacement
                    else "p4_g0c_metrics_calibration_v1"
                ),
                "scenario": "p4_g0c_free_corridor_v1",
                "decision_schema_version": "p4_collision_guide_decision_v1",
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
                "test_planner_manifest_path": str(
                    (
                        run_dir / "exports/synthetic_run_token"
                        / "test_planner_manifest.json"
                    ).resolve()
                ),
            }
            if replacement:
                manifest.update({
                    "dependency_manifest_sha256": self.bundle.protocol[
                        "runtime_dependency_manifest"
                    ]["sha256"],
                    "replacement_lineage_sha256": self.bundle.protocol[
                        "replacement_lineage"
                    ]["sha256"],
                })
            if hardened:
                manifest.update(MODULE.expected_launch_environment_binding(
                    root, run_dir
                ))
            if mutate is not None:
                mutate(run_index, manifest, rows)
            with csv_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            (run_dir / "p4_g0c_run_manifest.json").write_text(
                json.dumps(manifest, sort_keys=True) + "\n"
            )
            (run_dir / "launch_command.json").write_text(
                json.dumps(["ros2", "launch"]) + "\n"
            )
            (run_dir / "stdout.log").write_text("controlled shutdown\n")
            launch_manifest = (
                run_dir / "exports/synthetic_run_token"
                / "test_planner_manifest.json"
            )
            launch_manifest.parent.mkdir(parents=True)
            launch_manifest.write_text(json.dumps({
                "schema_version": "test_planner_manifest_v1",
                "run_id": record["run_id"],
                **top_level_effective_values(self.bundle.protocol),
                "p4.g0c": {
                    key: manifest[key] for key in (
                        "schema_version", "protocol_sha256",
                        "registry_sha256", "fixture_sha256",
                        "effective_values", "effective_config_sha256",
                        "selection_applied", "record_bag", "start_rviz",
                        *(
                            ("dependency_manifest_sha256",
                             "replacement_lineage_sha256")
                            if replacement
                            else ()
                        ),
                        *(("child_environment", "mutable_output_paths")
                          if hardened else ()),
                    )
                },
            }) + "\n")
            (run_dir / "exports/runtime_provenance_manifest.json").write_text(
                '{"schema_version":"runtime_provenance_v1"}\n'
            )
            (run_dir / "exports/iap_gnss_factor_debug.csv").write_text(
                "stamp,satellite,residual\n1,3,0.1\n"
            )
            timing = run_dir / "runtime/profiling/iap_timing.csv"
            timing.parent.mkdir(parents=True)
            timing.write_text("stamp,duration_ms\n1,2\n")
            inventory_bindings.append(
                self._write_artifact_inventory(run_dir, record["run_id"])
            )
        run_ids = [record["run_id"] for record in plan]
        runner_state = {
            "schema_version": (
                "p4_g0c_runner_state_v5" if hardened
                else "p4_g0c_runner_state_v4" if replacement
                else "p4_g0c_runner_state_v3"
            ),
            "runner_state": "COMPLETE",
            "protocol_sha256": self.bundle.protocol_sha256,
            "registry_sha256": self.bundle.registry_sha256,
            "fixture_sha256": self.bundle.fixture_sha256,
            "registered_run_ids": run_ids,
            "attempted_run_ids": run_ids,
            "completed_run_ids": run_ids,
            "attempts": [
                {
                    "attempt_index": index,
                    "run_id": run_id,
                    "state": "COMPLETE",
                    **inventory_bindings[index - 1],
                }
                for index, run_id in enumerate(run_ids, start=1)
            ],
            "runs": plan,
            "completed_run_count": 15,
            "launch_invocations": 15,
            "launch_started": True,
            "retries": 0,
            "failure_reason": "",
            "failed_run_id": "",
        }
        if replacement:
            runner_state.update({
                "gpu_preflight_invocations": 1,
                "dependency_preflight": {
                    "dependency_ready": True,
                    "manifest_sha256": self.bundle.protocol[
                        "runtime_dependency_manifest"
                    ]["sha256"],
                },
            })
        if hardened:
            first_binding = MODULE.expected_launch_environment_binding(
                root, Path(plan[0]["run_dir"])
            )
            runner_state["launch_environment"] = {
                "schema_version": MODULE.LAUNCH_ENVIRONMENT_SCHEMA,
                "runs_root": str(root.resolve()),
                "child_environment": first_binding["child_environment"],
                "directory_modes": {"XDG_RUNTIME_DIR": "0700"},
                "run_outputs": [
                    {
                        "run_id": record["run_id"],
                        "mutable_output_paths": (
                            MODULE.expected_launch_environment_binding(
                                root, Path(record["run_dir"])
                            )["mutable_output_paths"]
                        ),
                    }
                    for record in plan
                ],
            }
        (root / "p4_g0c_runner_state.json").write_text(
            json.dumps(runner_state, sort_keys=True) + "\n"
        )

    def _refresh_inventory_binding(self, root, run_index=0):
        record = MODULE.expand_run_plan(self.bundle.protocol, root)[run_index]
        run_dir = Path(record["run_dir"])
        binding = self._write_artifact_inventory(run_dir, record["run_id"])
        state_path = root / "p4_g0c_runner_state.json"
        state = json.loads(state_path.read_text())
        state["attempts"][run_index].update(binding)
        state_path.write_text(json.dumps(state, sort_keys=True) + "\n")

    def _rewrite_inventory_payload(self, root, payload, run_index=0):
        run_id = MODULE.expand_run_plan(
            self.bundle.protocol, root
        )[run_index]["run_id"]
        inventory_path = (
            root / run_id / "p4_g0c_artifact_inventory.json"
        )
        raw = json.dumps(
            payload, sort_keys=True, separators=(",", ":")
        ) + "\n"
        inventory_path.write_text(raw)
        state_path = root / "p4_g0c_runner_state.json"
        state = json.loads(state_path.read_text())
        state["attempts"][run_index]["artifact_inventory_sha256"] = (
            hashlib.sha256(raw.encode()).hexdigest()
        )
        state_path.write_text(json.dumps(state, sort_keys=True) + "\n")

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

    def test_v2_complete_synthetic_bundle_is_draft_eligible(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "DRAFT_ELIGIBLE")
        self.assertEqual(result["schema_version"], "p4_g0c_analysis_v2")
        self.assertEqual(
            result["threshold_draft"]["schema_version"],
            "p4_g0c_threshold_draft_v2",
        )

    def test_v3_environment_and_output_bindings_fail_closed_after_rehash(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v3.json",
            REPO / "config/icra27/p4_threshold_registry_v3.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V3,
        )
        operations = {
            "remove": lambda target, key, _value: target.pop(key),
            "change": lambda target, key, value: target.__setitem__(
                key, f"{value}-changed"
            ),
            "wrong_type": lambda target, key, value: target.__setitem__(
                key, [value]
            ),
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            baseline = MODULE.analyze(self.bundle, root)
            self.assertEqual(baseline["analysis_status"], "DRAFT_ELIGIBLE")
            run_id = self.bundle.protocol["registered_run_ids"][0]
            run_manifest_path = root / run_id / "p4_g0c_run_manifest.json"
            launch_manifest_path = (
                root / run_id
                / "exports/synthetic_run_token/test_planner_manifest.json"
            )
            state_path = root / "p4_g0c_runner_state.json"
            originals = {
                "run": json.loads(run_manifest_path.read_text()),
                "launch": json.loads(launch_manifest_path.read_text()),
                "state": json.loads(state_path.read_text()),
            }
            bindings = (
                ("child_environment", (
                    "HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR",
                    "XDG_RUNTIME_DIR",
                )),
                ("mutable_output_paths", (
                    "bag_output_dir", "decision_csv_path", "export_root_dir",
                    "iap_log_root", "launch_command_path", "run_manifest_path",
                    "runtime_root_dir", "stdout_log_path",
                )),
            )
            adversarial_count = 0
            for section, keys in bindings:
                for key in keys:
                    expected = originals["run"][section][key]
                    for operation, mutate in operations.items():
                        with self.subTest(
                            section=section, key=key, operation=operation
                        ):
                            run_manifest = json.loads(json.dumps(originals["run"]))
                            launch_manifest = json.loads(
                                json.dumps(originals["launch"])
                            )
                            state = json.loads(json.dumps(originals["state"]))
                            state_binding = (
                                state["launch_environment"]["child_environment"]
                                if section == "child_environment"
                                else state["launch_environment"]["run_outputs"][0][
                                    "mutable_output_paths"
                                ]
                            )
                            mutate(run_manifest[section], key, expected)
                            mutate(
                                launch_manifest["p4.g0c"][section], key, expected
                            )
                            mutate(state_binding, key, expected)
                            run_manifest_path.write_text(
                                json.dumps(run_manifest, sort_keys=True) + "\n"
                            )
                            launch_manifest_path.write_text(
                                json.dumps(launch_manifest, sort_keys=True) + "\n"
                            )
                            state_path.write_text(
                                json.dumps(state, sort_keys=True) + "\n"
                            )
                            self._refresh_inventory_binding(root)
                            result = MODULE.analyze(self.bundle, root)
                            self.assertEqual(
                                result["analysis_status"], "REJECTED"
                            )
                            self.assertNotIn("threshold_draft", result)
                            self.assertTrue(any(
                                "launch_environment" in failure
                                for failure in result["failures"]
                            ))
                            adversarial_count += 1
            self.assertEqual(adversarial_count, 39)

    def test_v3_xdg_mode_evidence_rejects_before_draft(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v3.json",
            REPO / "config/icra27/p4_threshold_registry_v3.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROTOCOL_SCHEMA_V3,
        )
        for case in ("filesystem", "runner_state"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                if case == "filesystem":
                    (root / "launch_environment/xdg_runtime").chmod(0o755)
                else:
                    state_path = root / "p4_g0c_runner_state.json"
                    state = json.loads(state_path.read_text())
                    state["launch_environment"]["directory_modes"] = {
                        "XDG_RUNTIME_DIR": "0755"
                    }
                    state_path.write_text(json.dumps(state, sort_keys=True) + "\n")
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertNotIn("threshold_draft", result)
                self.assertTrue(any(
                    "xdg_runtime" in failure
                    or "launch_environment_modes" in failure
                    for failure in result["failures"]
                ))

    def test_v2_launch_effective_disagreement_rejects_before_draft(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            run_id = self.bundle.protocol["registered_run_ids"][0]
            launch_path = (
                root / run_id
                / "exports/synthetic_run_token/test_planner_manifest.json"
            )
            launch_manifest = json.loads(launch_path.read_text())
            launch_manifest["p4.g0c"]["effective_values"][
                "p1.metrics_only"
            ] = True
            launch_manifest["p4.g0c"]["effective_config_sha256"] = (
                MODULE.effective_config_sha256(
                    launch_manifest["p4.g0c"]["effective_values"]
                )
            )
            launch_path.write_text(json.dumps(launch_manifest) + "\n")
            self._refresh_inventory_binding(root)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertNotIn("threshold_draft", result)
        self.assertTrue(any(
            "config_mismatch" in failure
            and "test-planner G0C binding mismatch: effective_values" in failure
            for failure in result["failures"]
        ))

    def test_v2_python_equal_effective_drift_rejects_stale_hash(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            run_id = self.bundle.protocol["registered_run_ids"][0]
            launch_path = (
                root / run_id
                / "exports/synthetic_run_token/test_planner_manifest.json"
            )
            launch_manifest = json.loads(launch_path.read_text())
            launch_manifest["p4.g0c"]["effective_values"][
                "p1.metrics_only"
            ] = 0
            launch_path.write_text(json.dumps(launch_manifest) + "\n")
            self._refresh_inventory_binding(root)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertNotIn("threshold_draft", result)
        self.assertTrue(any(
            "effective hash mismatch" in failure
            for failure in result["failures"]
        ))

    def test_v2_run_manifest_python_equal_drift_rejects_stale_hash(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            run_id = self.bundle.protocol["registered_run_ids"][0]
            manifest_path = root / run_id / "p4_g0c_run_manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["effective_values"]["p1.metrics_only"] = 0
            manifest_path.write_text(json.dumps(manifest) + "\n")
            self._refresh_inventory_binding(root)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertNotIn("threshold_draft", result)
        self.assertTrue(any(
            "effective_config_sha256" in failure
            for failure in result["failures"]
        ))

    def test_all_top_level_effective_disagreements_reject_before_draft(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v2.json",
            REPO / "config/icra27/p4_threshold_registry_v2.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
        )
        operations = {
            "remove": lambda manifest, key, _value: manifest.pop(key),
            "change": lambda manifest, key, value: manifest.__setitem__(
                key, changed_value(value)
            ),
            "wrong_type": lambda manifest, key, value: manifest.__setitem__(
                key, wrong_type_value(value)
            ),
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            run_id = self.bundle.protocol["registered_run_ids"][0]
            launch_path = (
                root / run_id
                / "exports/synthetic_run_token/test_planner_manifest.json"
            )
            original = json.loads(launch_path.read_text())
            nested_before = json.loads(json.dumps(original["p4.g0c"]))
            for key in TOP_LEVEL_EFFECTIVE_KEYS:
                expected = self.bundle.protocol["effective_values"][key]
                for operation, mutate in operations.items():
                    with self.subTest(key=key, operation=operation):
                        launch_manifest = json.loads(json.dumps(original))
                        mutate(launch_manifest, key, expected)
                        launch_path.write_text(
                            json.dumps(launch_manifest) + "\n"
                        )
                        self._refresh_inventory_binding(root)
                        result = MODULE.analyze(self.bundle, root)
                        self.assertEqual(
                            launch_manifest["p4.g0c"], nested_before
                        )
                        self.assertEqual(
                            result["analysis_status"], "REJECTED"
                        )
                        self.assertNotIn("threshold_draft", result)
                        self.assertTrue(any(
                            "config_mismatch" in failure
                            and "test-planner" in failure
                            for failure in result["failures"]
                        ))

    def test_exact_100_boundary_is_eligible_and_draft_uses_frozen_formulas(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root, [7] * 10 + [6] * 5)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "DRAFT_ELIGIBLE")
        self.assertEqual(result["complete_decision_count"], 100)
        self.assertEqual(result["denominator_decision_count"], 100)
        self.assertEqual(result["registered_run_denominator_count"], 15)
        self.assertEqual(result["attempted_run_denominator_count"], 15)
        self.assertEqual(result["completed_run_denominator_count"], 15)
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

    def test_extra_retry_directory_and_header_only_run_are_not_excludable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            (root / "p4-g0c-seed211-rep01-retry").mkdir()
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "root_inventory" in item for item in result["failures"]
        ))

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root, [0] + [8] * 14)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["denominator_decision_count"], 112)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "empty_run_csv" in item for item in result["failures"]
        ))

    def test_absent_partial_failed_reordered_and_duplicate_ledgers_reject(self):
        cases = {
            "runner_state_missing": lambda path, state: path.unlink(),
            "runner_state_attempted_ids": lambda path, state: (
                state.update({
                    "attempted_run_ids": state["attempted_run_ids"][:-1]
                }),
                path.write_text(json.dumps(state)),
            ),
            "runner_state_state": lambda path, state: (
                state.update({"runner_state": "FAILED"}),
                path.write_text(json.dumps(state)),
            ),
            "runner_state_attempt_order": lambda path, state: (
                state["attempted_run_ids"].__setitem__(
                    slice(0, 2), list(reversed(state["attempted_run_ids"][:2]))
                ),
                path.write_text(json.dumps(state)),
            ),
            "runner_state_duplicate_attempt": lambda path, state: (
                state["attempted_run_ids"].__setitem__(
                    1, state["attempted_run_ids"][0]
                ),
                path.write_text(json.dumps(state)),
            ),
            "runner_state_completed_ids": lambda path, state: (
                state.update({
                    "completed_run_ids": state["completed_run_ids"][:-1]
                }),
                path.write_text(json.dumps(state)),
            ),
            "runner_state_launch_invocations": lambda path, state: (
                state.update({"launch_invocations": 14}),
                path.write_text(json.dumps(state)),
            ),
            "runner_state_retries": lambda path, state: (
                state.update({"retries": 1}),
                path.write_text(json.dumps(state)),
            ),
        }
        for expected, mutate in cases.items():
            with self.subTest(expected=expected), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                path = root / "p4_g0c_runner_state.json"
                state = json.loads(path.read_text())
                mutate(path, state)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    expected in item for item in result["failures"]
                ))

    def test_authoritative_attempt_entries_reject_partial_failed_reordered_duplicate(self):
        mutations = {
            "partial": lambda attempts: attempts.pop(),
            "failed": lambda attempts: attempts[0].update({"state": "FAILED"}),
            "reordered": lambda attempts: attempts.__setitem__(
                slice(0, 2), list(reversed(attempts[:2]))
            ),
            "duplicate": lambda attempts: attempts.__setitem__(
                1, dict(attempts[0])
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                path = root / "p4_g0c_runner_state.json"
                state = json.loads(path.read_text())
                mutate(state["attempts"])
                path.write_text(json.dumps(state))
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertIn(
                    "runner_state_attempt_ledger", result["failures"]
                )

    def test_complete_identity_header_and_path_arithmetic_fail_closed(self):
        row_cases = {
            "duplicate_decision_identity": None,
            "path_length": {"original_path_length": "0"},
            "path_ratio_consistency": {"path_length_ratio": "1.100021"},
        }
        for expected, update in row_cases.items():
            def mutate(index, manifest, rows):
                if index != 0:
                    return
                if update is None:
                    for key in (
                        "planning_attempt_id", "collision_segment_id",
                        "request_hash",
                    ):
                        rows[1][key] = rows[0][key]
                else:
                    rows[0].update(update)

            with self.subTest(expected=expected), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root, mutate=mutate)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    expected in item for item in result["failures"]
                ))

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            csv_path = root / "p4-g0c-seed211-rep01" / "p4_decisions.csv"
            row = self._row(0)
            row["unexpected"] = "unexpected"
            with csv_path.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=CSV_FIELDS + ["unexpected"]
                )
                writer.writeheader()
                writer.writerow(row)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "csv_header" in item for item in result["failures"]
        ))

    def test_every_immutable_context_field_is_typed_and_nonblank(self):
        cases = {
            "stamp": ("", "typed_non_finite"),
            "planning_attempt_id": ("", "typed_integer"),
            "collision_segment_id": ("", "typed_integer"),
            "request_hash": ("", "typed_identity"),
            "snapshot_generation_id": ("", "typed_integer"),
            "snapshot_stamp_s": ("nan", "typed_non_finite"),
            "snapshot_frame": ("", "typed_identity"),
            "query_base_time_s": ("inf", "typed_non_finite"),
            "occupancy_epoch": ("", "typed_integer"),
            "original_hash": ("", "typed_identity"),
            "risk_hash": ("", "typed_identity"),
            "selected_hash": ("", "typed_identity"),
            "original_path_length": ("nan", "typed_non_finite"),
            "risk_path_length": ("", "typed_non_finite"),
        }
        for field, (value, expected) in cases.items():
            def mutate(index, manifest, rows):
                if index == 0:
                    rows[0][field] = value

            with self.subTest(field=field), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root, mutate=mutate)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    expected in item for item in result["failures"]
                ))

    def test_snapshot_unavailable_is_rejected_with_typed_p0_diagnosis(self):
        def mutate(index, manifest, rows):
            if index == 0:
                rows[0].update({
                    "reason": "snapshot_unavailable",
                    "snapshot_generation_id": "0",
                    "snapshot_stamp_s": "nan",
                    "snapshot_frame": "",
                })

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root, mutate=mutate)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "p0_riskgrid_snapshot" in item
            and "producer_reason=snapshot_unavailable" in item
            for item in result["failures"]
        ))

    def test_missing_duplicate_reordered_and_unexpected_headers_reject(self):
        headers = {
            "missing": CSV_FIELDS[:-1],
            "duplicate": CSV_FIELDS[:-1] + [CSV_FIELDS[-2]],
            "reordered": [CSV_FIELDS[1], CSV_FIELDS[0]] + CSV_FIELDS[2:],
            "unexpected": CSV_FIELDS + ["unexpected"],
        }
        for label, header in headers.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                csv_path = (
                    root / "p4-g0c-seed211-rep01" / "p4_decisions.csv"
                )
                row = self._row(0)
                if label == "unexpected":
                    row["unexpected"] = "unexpected"
                with csv_path.open("w", newline="") as stream:
                    writer = csv.DictWriter(
                        stream, fieldnames=header, extrasaction="ignore"
                    )
                    writer.writeheader()
                    writer.writerow(row)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    "csv_header" in item for item in result["failures"]
                ))

    def test_unregistered_p4_manifest_and_decision_csv_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            nested = root / "p4-g0c-seed211-rep01" / "retry"
            nested.mkdir()
            (nested / "p4_g0c_retry_manifest.json").write_text("{}")
            (nested / "p4_decisions_retry.csv").write_text("x\n")
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "unregistered nested run directory" in item
            for item in result["failures"]
        ))

    def test_alternate_manifest_and_csv_names_cannot_bypass_inventory(self):
        cases = (
            ("alternate_manifest.json", "{}"),
            ("decisions.csv", "x\n"),
        )
        for name, content in cases:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                run_dir = root / "p4-g0c-seed211-rep01"
                (run_dir / name).write_text(content)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    "artifact inventory does not match" in item
                    for item in result["failures"]
                ))

    def test_registered_production_manifest_and_timing_csv_are_eligible(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            run_dir = root / "p4-g0c-seed211-rep01"
            self.assertTrue(
                (
                    run_dir / "exports/synthetic_run_token"
                    / "test_planner_manifest.json"
                ).is_file()
            )
            self.assertTrue(
                (run_dir / "runtime/profiling/iap_timing.csv").is_file()
            )
            (run_dir / "runtime/p4_decisions_metrics.csv").write_text(
                "metric,value\nlatency,1\n"
            )
            (run_dir / "runtime/p4_g0c_export_manifest.json").write_text(
                '{"schema_version":"runtime_export_v1"}\n'
            )
            self._refresh_inventory_binding(root)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "DRAFT_ELIGIBLE")

    def test_arbitrary_in_root_output_rejects_before_write_or_self_invalidation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            output = root / "arbitrary.json"
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                exit_code = MODULE.main([
                    "--runs-root", str(root), "--output", str(output)
                ])
            reanalysis = MODULE.analyze(self.bundle, root)
        self.assertEqual(exit_code, 2)
        self.assertFalse(output.exists())
        self.assertEqual(reanalysis["analysis_status"], "DRAFT_ELIGIBLE")

    def test_existing_named_analyzer_outputs_are_never_overwritten(self):
        cases = {
            "--output": ("p4_g0c_analysis.json", "analysis-retained\n"),
            "--draft-output": (
                "p4_g0c_threshold_draft.json", "draft-retained\n"
            ),
        }
        for option, (name, retained) in cases.items():
            with self.subTest(option=option), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                output = root / name
                output.write_text(retained)
                with contextlib.redirect_stdout(
                    io.StringIO()
                ), contextlib.redirect_stderr(io.StringIO()):
                    exit_code = MODULE.main([
                        "--runs-root", str(root), option, str(output)
                    ])
                self.assertEqual(exit_code, 2)
                self.assertEqual(output.read_text(), retained)

    def test_post_inventory_add_change_remove_and_symlink_reject(self):
        mutations = {
            "added": lambda run_dir: (
                run_dir / "extra.bin"
            ).write_bytes(b"extra"),
            "changed": lambda run_dir: (
                run_dir / "stdout.log"
            ).write_text("changed\n"),
            "removed": lambda run_dir: (
                run_dir / "stdout.log"
            ).unlink(),
            "symlink": lambda run_dir: (
                run_dir / "linked.log"
            ).symlink_to(run_dir / "stdout.log"),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                mutate(root / "p4-g0c-seed211-rep01")
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    "artifact_inventory_invalid" in item
                    for item in result["failures"]
                ))

    def test_inventory_missing_duplicate_and_escaping_entries_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            inventory_path = (
                root
                / "p4-g0c-seed211-rep01/p4_g0c_artifact_inventory.json"
            )
            inventory_path.unlink()
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "artifact_inventory_missing" in item for item in result["failures"]
        ))

        mutations = {
            "duplicate": lambda payload: payload["entries"].append(
                dict(payload["entries"][-1])
            ),
            "escaping": lambda payload: payload["entries"].insert(
                0, {"path": "../escape", "type": "directory"}
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                inventory_path = (
                    root
                    / "p4-g0c-seed211-rep01/p4_g0c_artifact_inventory.json"
                )
                payload = json.loads(inventory_path.read_text())
                mutate(payload)
                self._rewrite_inventory_payload(root, payload)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    "artifact_inventory_invalid" in item
                    for item in result["failures"]
                ))

    def test_inventory_cannot_authorize_secondary_g0c_manifest_or_decision_csv(self):
        cases = {
            "v1 manifest": (
                "secondary G0C run manifest",
                "secondary_v1.json",
                '{"schema_version":"p4_g0c_run_manifest_v1"}\n',
            ),
            "v2 manifest": (
                "secondary G0C run manifest",
                "secondary_v2.json",
                '{"schema_version":"p4_g0c_run_manifest_v2"}\n',
            ),
            "decision CSV": (
                "secondary P4 decision CSV",
                "secondary.csv", ",".join(CSV_FIELDS) + "\n"
            ),
        }
        for label, (expected, name, content) in cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                run_dir = root / "p4-g0c-seed211-rep01"
                (run_dir / name).write_text(content)
                self._refresh_inventory_binding(root)
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    expected in item for item in result["failures"]
                ))

    def test_launch_manifest_path_hash_and_object_are_bound(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            manifest_path = (
                root / "p4-g0c-seed211-rep01/p4_g0c_run_manifest.json"
            )
            manifest = json.loads(manifest_path.read_text())
            manifest["test_planner_manifest_path"] = str(
                root / "elsewhere/test_planner_manifest.json"
            )
            manifest_path.write_text(json.dumps(manifest, sort_keys=True) + "\n")
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "manifest_truth" in item and "test_planner_manifest_path" in item
            for item in result["failures"]
        ))

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            launch_manifest = (
                root
                / "p4-g0c-seed211-rep01/exports/synthetic_run_token"
                / "test_planner_manifest.json"
            )
            launch_manifest.write_text("[]\n")
            self._refresh_inventory_binding(root)
            result = MODULE.analyze(self.bundle, root)
        self.assertEqual(result["analysis_status"], "REJECTED")
        self.assertTrue(any(
            "launch_manifest_invalid" in item for item in result["failures"]
        ))

        binding_cases = [
            ("launch_manifest_binding",
                "test_planner_manifest_sha256", "0" * 64
            ),
            ("launch_manifest_binding",
                "test_planner_manifest_path", "/tmp/other-launch-manifest.json"
            ),
            ("artifact_inventory_binding",
                "artifact_inventory_sha256", "0" * 64
            ),
            ("artifact_inventory_binding",
                "artifact_inventory_path", "/tmp/other-inventory.json"
            ),
        ]
        for expected, field, value in binding_cases:
            with self.subTest(field=field), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self._make_bundle(root)
                state_path = root / "p4_g0c_runner_state.json"
                state = json.loads(state_path.read_text())
                state["attempts"][0][field] = value
                state_path.write_text(json.dumps(state, sort_keys=True) + "\n")
                result = MODULE.analyze(self.bundle, root)
                self.assertEqual(result["analysis_status"], "REJECTED")
                self.assertTrue(any(
                    expected in item for item in result["failures"]
                ))

    def test_output_swap_alias_and_symlink_reject_before_any_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            cases = [
                [
                    "--output",
                    str(root / "p4_g0c_threshold_draft.json"),
                ],
                [
                    "--draft-output",
                    str(root / "p4_g0c_analysis.json"),
                ],
            ]
            for argv in cases:
                with self.subTest(option=argv[0]), contextlib.redirect_stdout(
                    io.StringIO()
                ), contextlib.redirect_stderr(io.StringIO()):
                    exit_code = MODULE.main([
                        "--runs-root", str(root), *argv
                    ])
                    self.assertEqual(exit_code, 2)
            self.assertFalse((root / "p4_g0c_analysis.json").exists())
            self.assertFalse((root / "p4_g0c_threshold_draft.json").exists())

            shared = Path(tmp) / "shared.json"
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                exit_code = MODULE.main([
                    "--protocol", str(REPO / "config/icra27/p4_g0c_protocol_v1.json"),
                    "--registry", str(REPO / "config/icra27/p4_threshold_registry_v1.json"),
                    "--fixture", str(REPO / "config/icra27/p4_g0c_live_fixture_v1.json"),
                    "--runs-root", str(root),
                    "--output", str(shared),
                    "--draft-output", str(shared),
                ])
            self.assertEqual(exit_code, 2)
            self.assertFalse(shared.exists())

            target = Path(tmp) / "retained.json"
            target.write_text("retained\n")
            symlinked = root / "p4_g0c_analysis.json"
            symlinked.symlink_to(target)
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                exit_code = MODULE.main([
                    "--runs-root", str(root), "--output", str(symlinked)
                ])
            self.assertEqual(exit_code, 2)
            self.assertEqual(target.read_text(), "retained\n")

    def test_lexical_output_aliases_reject_before_analyze_or_write(self):
        cases = (
            (
                "--output",
                lambda root: root / "nonexistent/../p4_g0c_analysis.json",
            ),
            (
                "--draft-output",
                lambda root: root / ".." / root.name
                / "p4_g0c_threshold_draft.json",
            ),
        )
        for option, make_alias in cases:
            with self.subTest(option=option), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "runs"
                self._make_bundle(root)
                alias = make_alias(root)
                with mock.patch.object(
                    MODULE, "analyze", wraps=MODULE.analyze
                ) as analyze_spy, contextlib.redirect_stdout(
                    io.StringIO()
                ), contextlib.redirect_stderr(io.StringIO()):
                    exit_code = MODULE.main([
                        "--runs-root", str(root), option, str(alias)
                    ])

                self.assertEqual(exit_code, 2)
                analyze_spy.assert_not_called()
                self.assertFalse((root / "nonexistent").exists())
                self.assertFalse((root / "p4_g0c_analysis.json").exists())
                self.assertFalse(
                    (root / "p4_g0c_threshold_draft.json").exists()
                )

    def test_canonical_relative_and_absolute_named_outputs_still_work(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            self._make_bundle(root)
            draft_output = root / "p4_g0c_threshold_draft.json"
            with contextlib.chdir(root), contextlib.redirect_stdout(
                io.StringIO()
            ), contextlib.redirect_stderr(io.StringIO()):
                exit_code = MODULE.main([
                    "--protocol", str(REPO / "config/icra27/p4_g0c_protocol_v1.json"),
                    "--registry", str(REPO / "config/icra27/p4_threshold_registry_v1.json"),
                    "--fixture", str(REPO / "config/icra27/p4_g0c_live_fixture_v1.json"),
                    "--runs-root", str(root),
                    "--output", "p4_g0c_analysis.json",
                    "--draft-output", str(draft_output),
                ])

            self.assertEqual(exit_code, 0)
            self.assertTrue((root / "p4_g0c_analysis.json").is_file())
            self.assertTrue(draft_output.is_file())

    def test_named_outputs_are_excluded_from_raw_hash_and_rejected_has_no_draft(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            baseline = MODULE.analyze(self.bundle, root)
            analysis_output = root / "p4_g0c_analysis.json"
            draft_output = root / "p4_g0c_threshold_draft.json"
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                exit_code = MODULE.main([
                    "--protocol", str(REPO / "config/icra27/p4_g0c_protocol_v1.json"),
                    "--registry", str(REPO / "config/icra27/p4_threshold_registry_v1.json"),
                    "--fixture", str(REPO / "config/icra27/p4_g0c_live_fixture_v1.json"),
                    "--runs-root", str(root),
                    "--output", str(analysis_output),
                    "--draft-output", str(draft_output),
                ])
            reanalysis = MODULE.analyze(self.bundle, root)
            self.assertEqual(exit_code, 0)
            self.assertTrue(analysis_output.is_file())
            self.assertTrue(draft_output.is_file())
            self.assertEqual(
                baseline["raw_bundle_sha256"],
                reanalysis["raw_bundle_sha256"],
            )
            self.assertEqual(reanalysis["analysis_status"], "DRAFT_ELIGIBLE")

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root)
            (root / "unregistered.txt").write_text("dirty")
            draft_output = root / "p4_g0c_threshold_draft.json"
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                exit_code = MODULE.main([
                    "--runs-root", str(root),
                    "--draft-output", str(draft_output),
                ])
            self.assertEqual(exit_code, 2)
            self.assertFalse(draft_output.exists())

    def test_ratio_tolerance_boundary_and_raw_hash_are_stable(self):
        def mutate(index, manifest, rows):
            if index == 0:
                rows[0]["path_length_ratio"] = "1.100019"

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._make_bundle(root, [7] * 10 + [6] * 5, mutate=mutate)
            first = MODULE.analyze(self.bundle, root)
            second = MODULE.analyze(self.bundle, root)
        self.assertEqual(first["analysis_status"], "DRAFT_ELIGIBLE")
        self.assertEqual(first["raw_bundle_sha256"], second["raw_bundle_sha256"])
        self.assertEqual(len(first["raw_bundle_sha256"]), 64)

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
            shutil.rmtree(missing)
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
