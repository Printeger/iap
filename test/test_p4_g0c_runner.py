import importlib.util
import csv
import hashlib
import json
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest import mock

from icra_historical_p4_fixture import materialize_p4_r6_test_install


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts/dev_planner/run_p4_g0c_calibration.py"
SPEC = importlib.util.spec_from_file_location("run_p4_g0c_calibration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


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


class P4G0CRunnerTest(unittest.TestCase):
    def test_v6_plan_and_launch_command_use_only_registered_r6_identity(self):
        bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v6.json",
            REPO / "config/icra27/p4_threshold_registry_v6.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            plan = MODULE.expand_run_plan(bundle.protocol, root)
            environment = MODULE.expected_launch_environment_binding(
                root, Path(plan[0]["run_dir"])
            )
            command = MODULE.launch_command(bundle, plan[0], environment)
        self.assertEqual(len(plan), 15)
        self.assertTrue(all(
            record["run_id"].startswith("p4-g0c-r6-") for record in plan
        ))
        self.assertIn("experiment:=p4_g0c_metrics_calibration_v6", command)

    def test_v5_plan_and_launch_command_use_only_registered_r5_identity(self):
        bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v5.json",
            REPO / "config/icra27/p4_threshold_registry_v5.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            plan = MODULE.expand_run_plan(bundle.protocol, root)
            environment = MODULE.expected_launch_environment_binding(
                root, Path(plan[0]["run_dir"])
            )
            command = MODULE.launch_command(bundle, plan[0], environment)
        self.assertEqual(len(plan), 15)
        self.assertTrue(all(
            record["run_id"].startswith("p4-g0c-r5-") for record in plan
        ))
        self.assertIn("experiment:=p4_g0c_metrics_calibration_v5", command)
        self.assertIn(
            "p4.g0c.fixture_path:=" + str(
                (REPO / "config/icra27/p4_g0c_live_fixture_v2.json").resolve()
            ),
            command,
        )

    def test_v4_p0_profile_gate_rejects_bad_sigma_before_gpu_or_launch(self):
        bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v4.json",
            REPO / "config/icra27/p4_threshold_registry_v4.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.PROFILED_PROTOCOL_SCHEMA,
        )
        for bad_sigma in (True, "0.01", float("nan"), -0.01, 0.02):
            with self.subTest(bad_sigma=bad_sigma):
                bundle.protocol["effective_values"][
                    "p0.predictor.sigma_grow_m_sqrt_s"
                ] = bad_sigma
                result = MODULE.validate_p0_profile_binding(bundle)
                self.assertEqual(result["failure_reason"], "P0_SIGMA_BINDING_MISMATCH")

    def test_v5_p0_profile_gate_requires_exact_typed_predictor_worker_four(self):
        bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v5.json",
            REPO / "config/icra27/p4_threshold_registry_v5.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        )
        for bad_worker in (None, True, "4", 1, 3, 5):
            with self.subTest(bad_worker=bad_worker):
                bundle.protocol["effective_values"][
                    "p0.predictor.worker_count"
                ] = bad_worker
                result = MODULE.validate_p0_profile_binding(bundle)
                self.assertEqual(
                    result["failure_reason"], "P0_WORKER_BINDING_MISMATCH"
                )
        bundle.protocol["effective_values"]["p0.predictor.worker_count"] = 4
        result = MODULE.validate_p0_profile_binding(bundle)
        self.assertTrue(result["profile_ready"])
        self.assertEqual(result["predictor_worker_count"], 4)

    def setUp(self):
        self.protocol = REPO / "config/icra27/p4_g0c_protocol_v2.json"
        self.registry = REPO / "config/icra27/p4_threshold_registry_v2.json"
        self.fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        self.bundle = MODULE.load_bundle(
            self.protocol, self.registry, self.fixture
        )
        self.dependency_patch = mock.patch.object(
            MODULE,
            "validate_runtime_dependencies",
            return_value={
                "schema_version": "p4_g0c_dependency_preflight_result_v2",
                "dependency_ready": True,
                "failure_reason": "",
            },
        )
        self.dependency_patch.start()

    def tearDown(self):
        self.dependency_patch.stop()

    def _valid_row(self, index):
        row = {name: "0" for name in MODULE.DECISION_CSV_COLUMNS}
        row.update({
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
            "reason": (
                "metrics_only"
                if self.bundle.protocol["schema_version"]
                == MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA
                else "METRICS_ONLY"
            ),
            "selection_applied": "0",
            "original_hash": f"original-{index}",
            "risk_hash": f"risk-{index}",
            "selected_hash": f"original-{index}",
            "original_sample_count": "200",
            "original_valid_count": "200",
            "original_mean": "2.0",
            "original_max": "3.0",
            "risk_sample_count": "200",
            "risk_valid_count": "200",
            "risk_mean": "1.0",
            "risk_max": "1.0",
            "original_path_length": "10.0",
            "risk_path_length": "11.0",
            "path_length_ratio": "1.1",
            "original_search_latency_ms": "40.0",
            "risk_search_latency_ms": "50.0",
            "total_search_latency_ms": "90.0",
        })
        return row

    def _write_production_outputs(self, record, row_index):
        run_dir = Path(record["run_dir"])
        csv_path = run_dir / "p4_decisions.csv"
        launch_manifest_path = (
            run_dir / "exports/synthetic_run_token/test_planner_manifest.json"
        )
        schema = self.bundle.protocol["schema_version"]
        version = schema.rsplit("_", 1)[-1]
        replacement = schema != MODULE.LEGACY_PROTOCOL_SCHEMA
        hardened = schema in {
            MODULE.HARDENED_PROTOCOL_SCHEMA, MODULE.PROFILED_PROTOCOL_SCHEMA,
            MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
            MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
        }
        manifest = {
            "schema_version": f"p4_g0c_run_manifest_{version}",
            "run_id": record["run_id"],
            "seed": record["seed"],
            "repetition": record["repetition"],
            "protocol_sha256": self.bundle.protocol_sha256,
            "registry_sha256": self.bundle.registry_sha256,
            "fixture_sha256": self.bundle.fixture_sha256,
            "csv_path": str(csv_path.resolve()),
            "gate": "G0C",
            "experiment": f"p4_g0c_metrics_calibration_{version}",
            "scenario": "p4_g0c_free_corridor_v1",
            "decision_schema_version": "p4_collision_guide_decision_v1",
            "effective_values": self.bundle.protocol["effective_values"],
            "effective_config_sha256": MODULE.effective_config_sha256(
                self.bundle.protocol["effective_values"]
            ),
            "required_process_set": list(MODULE.REQUIRED_PROCESSES),
            "selection_applied": False,
            "record_bag": False,
            "start_rviz": False,
            "immutable_run_id": True,
            "overwrite_allowed": False,
            "test_planner_manifest_path": str(launch_manifest_path.resolve()),
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
                run_dir.parent, run_dir
            ))
        if schema in {
            MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
            MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
        }:
            manifest["admission_parameter"] = {
                "requested": True, "effective": True,
            }
        (run_dir / "p4_g0c_run_manifest.json").write_text(
            json.dumps(manifest, sort_keys=True) + "\n"
        )
        with csv_path.open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=MODULE.DECISION_CSV_COLUMNS
            )
            writer.writeheader()
            writer.writerow(self._valid_row(row_index))
        launch_manifest_path.parent.mkdir(parents=True)
        launch_manifest_payload = {
            "schema_version": "test_planner_manifest_v1",
            "run_id": record["run_id"],
            **top_level_effective_values(self.bundle.protocol),
            "p4.g0c": {
                key: manifest[key] for key in (
                    "schema_version", "protocol_sha256", "registry_sha256",
                    "fixture_sha256", "effective_values",
                    "effective_config_sha256", "selection_applied",
                    "record_bag", "start_rviz",
                    *(("dependency_manifest_sha256",
                       "replacement_lineage_sha256") if replacement else ()),
                    *(("child_environment", "mutable_output_paths")
                      if hardened else ()),
                    *(("admission_parameter",) if schema in {
                        MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
                        MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
                    } else ()),
                )
            },
        }
        if schema in {
            MODULE.PROFILED_PROTOCOL_SCHEMA,
            MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
            MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
        }:
            launch_manifest_payload[
                "p4.require_risk_grid_ready_before_planning"
            ] = self.bundle.protocol["effective_values"][
                "p4.require_risk_grid_ready_before_planning"
            ]
        if schema in {
            MODULE.CLOSED_FIXTURE_PROTOCOL_SCHEMA,
            MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
        }:
            launch_manifest_payload.update({
                "p0.predictor.requested_worker_count": 4,
                "p0.predictor.effective_worker_count": 4,
            })
        if schema == MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA:
            launch_manifest_payload.update({
                "p0.horizons_s": [
                    0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0,
                ],
                "p4.cost_query_policy": (
                    "CONSERVATIVE_OCCUPIED_COST_SUPPORT"
                ),
            })
        launch_manifest_path.write_text(
            json.dumps(launch_manifest_payload) + "\n"
        )
        timing = run_dir / "runtime/profiling/iap_timing.csv"
        timing.parent.mkdir(parents=True)
        timing.write_text("stamp,duration_ms\n1,2\n")
        (run_dir / "stdout.log").write_text("controlled shutdown\n")

    def _make_failed_r6_recovery_root(self, root):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v6.json",
            REPO / "config/icra27/p4_threshold_registry_v6.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v2.json",
            expected_protocol_schema=MODULE.TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
        )
        plan = MODULE.expand_run_plan(self.bundle.protocol, root)
        root.mkdir()
        launch_environment = MODULE.prepare_launch_environment(root, plan)
        record = plan[0]
        run_dir = Path(record["run_dir"])
        run_dir.mkdir()
        (run_dir / "launch_command.json").write_text("[]\n")
        self._write_production_outputs(record, 0)
        logs = run_dir / "runtime/iap_logs"
        target = logs / "synthetic-first-session"
        target.mkdir(parents=True)
        (target / "runtime.log").write_text("retained\n")
        (logs / "latest").symlink_to(target.name)
        monitor = {
            "required_processes_ok": True,
            "required_processes": {
                name: {"seen": True, "runtime_failure": False}
                for name in MODULE.REQUIRED_PROCESSES
            },
            "process_failures": [],
        }
        with mock.patch.object(
            MODULE, "make_run_artifact_inventory",
            side_effect=RuntimeError(
                "run artifact cannot be a symlink: runtime/iap_logs/latest"
            ),
        ):
            with self.assertRaisesRegex(
                MODULE.RunnerError, "run artifact cannot be a symlink"
            ):
                MODULE._validate_and_finalize_run(
                    self.bundle, record, monitor,
                    MODULE._run_environment_binding(
                        launch_environment, record["run_id"]
                    ),
                )
        ros_logs = root / "launch_environment/ros_logs"
        ros_target = ros_logs / "synthetic-ros-session"
        ros_target.mkdir()
        (ros_target / "launch.log").write_text("retained launch\n")
        (ros_logs / "latest").symlink_to(ros_target.resolve())
        failure = (
            "run artifact inventory failed: p4-g0c-r6-seed211-rep01:"
            "run artifact cannot be a symlink: runtime/iap_logs/latest"
        )
        state = MODULE._base_result(self.bundle, plan)
        state.update({
            "runner_state": "FAILED",
            "dependency_preflight": {
                "dependency_ready": True,
                "manifest_sha256": self.bundle.protocol[
                    "runtime_dependency_manifest"
                ]["sha256"],
            },
            "p0_profile_preflight": {"profile_ready": True},
            "launch_environment": launch_environment,
            "gpu_preflight_invocations": 1,
            "gpu_preflight": {"gpu_ready": True},
            "launch_started": True,
            "attempted_run_ids": [record["run_id"]],
            "attempts": [{
                "attempt_index": 1, "run_id": record["run_id"],
                "state": "FAILED",
            }],
            "launch_invocations": 1,
            "failure_reason": failure,
            "failed_run_id": record["run_id"],
        })
        state_path = root / "p4_g0c_runner_state.json"
        state_path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
        run_manifest = run_dir / "p4_g0c_run_manifest.json"
        test_manifest = Path(json.loads(
            run_manifest.read_text()
        )["test_planner_manifest_path"])
        contract = MODULE.R6RecoveryContract(
            runs_root=root.resolve(),
            runner_state_sha256=hashlib.sha256(
                state_path.read_bytes()
            ).hexdigest(),
            decisions_sha256=hashlib.sha256(
                (run_dir / "p4_decisions.csv").read_bytes()
            ).hexdigest(),
            run_manifest_sha256=hashlib.sha256(
                run_manifest.read_bytes()
            ).hexdigest(),
            test_planner_manifest_sha256=hashlib.sha256(
                test_manifest.read_bytes()
            ).hexdigest(),
            stdout_sha256=hashlib.sha256(
                (run_dir / "stdout.log").read_bytes()
            ).hexdigest(),
            run_alias_target="synthetic-first-session",
        )
        return contract

    def _ready_r6_recovery_dependency(self):
        return {
            "dependency_ready": True,
            "manifest_sha256": self.bundle.protocol[
                "runtime_dependency_manifest"
            ]["sha256"],
            "validated_prefixes": list(MODULE.R6_RECOVERY_EXACT_PREFIXES),
        }

    @contextmanager
    def _historical_r6_install(self, root):
        install = materialize_p4_r6_test_install(REPO, Path(root))
        prefixes = [str(install), "/opt/ros/jazzy"]
        with mock.patch.object(
            MODULE, "R6_RECOVERY_FINAL_INSTALL", install
        ), mock.patch.object(
            MODULE, "R6_RECOVERY_EXACT_PREFIXES", prefixes
        ):
            yield install

    def test_r6_recovery_validation_only_is_exact_and_nonmutating(self):
        task_tmp = REPO / "results/icra27/icra064"
        task_tmp.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=task_tmp) as tmp:
            root = Path(tmp) / "runs"
            contract = self._make_failed_r6_recovery_root(root)
            state_path = root / "p4_g0c_runner_state.json"
            before = state_path.read_bytes()
            evidence_root = Path(tmp) / "recovery-evidence"
            with self._historical_r6_install(Path(tmp)), mock.patch.object(
                MODULE, "ICRA063_R6_RECOVERY_CONTRACT", contract
            ), mock.patch.object(
                MODULE, "validate_runtime_dependencies",
                return_value=self._ready_r6_recovery_dependency(),
            ):
                result = MODULE.recover_r6_matrix(
                    self.bundle, root, evidence_root,
                    validation_only=True,
                )

            self.assertEqual(result["recovery_state"], "ADOPTION_ELIGIBLE")
            self.assertEqual(
                result["next_run_id"], "p4-g0c-r6-seed211-rep02"
            )
            self.assertEqual(result["remaining_run_count"], 14)
            self.assertEqual(result["recovery_writes"], 0)
            self.assertEqual(result["recovery_launches"], 0)
            self.assertEqual(state_path.read_bytes(), before)
            self.assertFalse(evidence_root.exists())
            self.assertFalse(
                (root / "p4-g0c-r6-seed211-rep01"
                 / "p4_g0c_artifact_inventory.json").exists()
            )

    def test_r6_recovery_continues_only_remaining_fourteen_once(self):
        task_tmp = REPO / "results/icra27/icra064"
        task_tmp.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=task_tmp) as tmp:
            root = Path(tmp) / "runs"
            contract = self._make_failed_r6_recovery_root(root)
            first_run = root / "p4-g0c-r6-seed211-rep01"
            frozen_hashes = {
                name: hashlib.sha256((first_run / name).read_bytes()).hexdigest()
                for name in (
                    "p4_decisions.csv", "p4_g0c_run_manifest.json",
                    "stdout.log",
                )
            }
            launched = []

            def launch(record, *_):
                launched.append(record["run_id"])
                self._write_production_outputs(record, len(launched))
                logs = Path(record["run_dir"]) / "runtime/iap_logs"
                target = logs / f"synthetic-{record['run_id']}"
                target.mkdir(parents=True)
                (target / "runtime.log").write_text("complete\n")
                (logs / "latest").symlink_to(target.name)
                return 0, {
                    "required_processes_ok": True,
                    "required_processes": {
                        name: {"seen": True, "runtime_failure": False}
                        for name in MODULE.REQUIRED_PROCESSES
                    },
                    "process_failures": [],
                }

            evidence_root = Path(tmp) / "recovery-evidence"
            with self._historical_r6_install(Path(tmp)):
                dependency = self._ready_r6_recovery_dependency()
                with mock.patch.object(
                    MODULE, "ICRA063_R6_RECOVERY_CONTRACT", contract
                ), mock.patch.object(
                    MODULE, "validate_runtime_dependencies",
                    return_value=dependency,
                ):
                    result = MODULE.recover_r6_matrix(
                        self.bundle, root, evidence_root,
                        gpu_preflight=lambda _: {"gpu_ready": True},
                        launch_executor=launch,
                    )

            self.assertEqual(
                launched,
                self.bundle.protocol["registered_run_ids"][1:],
            )
            self.assertEqual(result["runner_state"], "COMPLETE")
            self.assertEqual(result["attempted_run_ids"], (
                self.bundle.protocol["registered_run_ids"]
            ))
            self.assertEqual(result["completed_run_ids"], (
                self.bundle.protocol["registered_run_ids"]
            ))
            self.assertEqual(result["launch_invocations"], 15)
            self.assertEqual(result["gpu_preflight_invocations"], 2)
            self.assertEqual(result["retries"], 0)
            self.assertEqual(result["runner_sessions"], [
                {"session_index": 1, "gpu_preflight_invocations": 1,
                 "launch_invocations": 1},
                {"session_index": 2, "gpu_preflight_invocations": 1,
                 "launch_invocations": 14},
            ])
            self.assertTrue((
                first_run / "p4_g0c_artifact_inventory.json"
            ).is_file())
            self.assertTrue((
                evidence_root / "original_terminal_state.json"
            ).is_file())
            self.assertTrue((
                evidence_root / "recovery_transition.json"
            ).is_file())
            self.assertEqual(frozen_hashes, {
                name: hashlib.sha256((first_run / name).read_bytes()).hexdigest()
                for name in frozen_hashes
            })

    def test_r6_recovery_gpu_failure_is_terminal_and_cannot_retry(self):
        task_tmp = REPO / "results/icra27/icra064"
        task_tmp.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=task_tmp) as tmp:
            root = Path(tmp) / "runs"
            contract = self._make_failed_r6_recovery_root(root)
            evidence_root = Path(tmp) / "recovery-evidence"
            never_launch = mock.Mock(
                side_effect=AssertionError("launch must not run")
            )
            with self._historical_r6_install(Path(tmp)):
                dependency = self._ready_r6_recovery_dependency()
                with mock.patch.object(
                    MODULE, "ICRA063_R6_RECOVERY_CONTRACT", contract
                ), mock.patch.object(
                    MODULE, "validate_runtime_dependencies",
                    return_value=dependency,
                ):
                    result = MODULE.recover_r6_matrix(
                        self.bundle, root, evidence_root,
                        gpu_preflight=lambda _: {
                            "gpu_ready": False,
                            "failure_reason": "synthetic_gpu_failure",
                        },
                        launch_executor=never_launch,
                    )
                    with self.assertRaisesRegex(
                        MODULE.RunnerError, "RETAINED_R6_DRIFT"
                    ):
                        MODULE.recover_r6_matrix(
                            self.bundle, root, evidence_root,
                            gpu_preflight=lambda _: {"gpu_ready": True},
                            launch_executor=never_launch,
                        )

            self.assertEqual(result["runner_state"], "FAILED")
            self.assertEqual(result["failure_reason"], "GPU_NOT_READY")
            self.assertEqual(result["gpu_preflight_invocations"], 2)
            self.assertEqual(result["launch_invocations"], 1)
            self.assertEqual(
                result["attempted_run_ids"],
                ["p4-g0c-r6-seed211-rep01"],
            )
            never_launch.assert_not_called()

    def test_r6_recovery_rejects_alternate_install_before_any_write(self):
        task_tmp = REPO / "results/icra27/icra064"
        task_tmp.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=task_tmp) as tmp:
            root = Path(tmp) / "runs"
            contract = self._make_failed_r6_recovery_root(root)
            state_path = root / "p4_g0c_runner_state.json"
            before = state_path.read_bytes()
            evidence_root = Path(tmp) / "recovery-evidence"
            dependency = self._ready_r6_recovery_dependency()
            dependency["validated_prefixes"] = [
                str(Path(tmp) / "alternate-install"), "/opt/ros/jazzy",
            ]

            with mock.patch.object(
                MODULE, "ICRA063_R6_RECOVERY_CONTRACT", contract
            ), mock.patch.object(
                MODULE, "validate_runtime_dependencies",
                return_value=dependency,
            ), self.assertRaisesRegex(
                MODULE.RunnerError, "exact_final_install"
            ):
                MODULE.recover_r6_matrix(
                    self.bundle, root, evidence_root,
                    validation_only=True,
                )

            self.assertEqual(state_path.read_bytes(), before)
            self.assertFalse(evidence_root.exists())
            self.assertFalse((
                root / "p4-g0c-r6-seed211-rep01"
                / "p4_g0c_artifact_inventory.json"
            ).exists())

    def test_plan_only_is_nonmutating_and_seed_major(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs_root = Path(tmp) / "not-created"
            result = MODULE.run(
                self.bundle,
                runs_root,
                plan_only=True,
                preflight_only=False,
            )
            self.assertFalse(runs_root.exists())
        self.assertEqual(result["runner_state"], "PLANNED")
        self.assertEqual(len(result["runs"]), 15)
        self.assertEqual(result["runs"][0]["run_id"], "p4-g0c-r2-seed211-rep01")
        self.assertEqual(result["runs"][-1]["run_id"], "p4-g0c-r2-seed271-rep03")

    def test_plan_only_does_not_touch_or_validate_an_existing_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            root.mkdir()
            retained = root / "retained.txt"
            retained.write_text("unchanged\n")
            result = MODULE.run(self.bundle, root, plan_only=True)
            self.assertEqual(result["runner_state"], "PLANNED")
            self.assertEqual(retained.read_text(), "unchanged\n")
            self.assertEqual(
                sorted(path.name for path in root.iterdir()),
                ["retained.txt"],
            )

    def test_existing_even_empty_run_directory_is_refused_before_preflight(self):
        calls = []
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "p4-g0c-r2-seed211-rep01").mkdir()
            with self.assertRaisesRegex(MODULE.RunnerError, "existing run directory"):
                MODULE.run(
                    self.bundle,
                    root,
                    gpu_preflight=lambda _: calls.append("gpu"),
                )
        self.assertEqual(calls, [])

    def test_existing_runner_state_is_refused_without_overwrite_or_preflight(self):
        calls = []
        retained = '{"runner_state":"FAILED","failure_reason":"GPU_NOT_READY"}\n'
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            state_path = root / "p4_g0c_runner_state.json"
            state_path.write_text(retained)
            with self.assertRaisesRegex(MODULE.RunnerError, "existing runner state"):
                MODULE.run(
                    self.bundle,
                    root,
                    gpu_preflight=lambda _: calls.append("gpu"),
                )
            self.assertEqual(state_path.read_text(), retained)
        self.assertEqual(calls, [])

    def test_every_dirty_or_symlink_root_is_refused_before_gpu_or_launch(self):
        dirty_cases = {
            "arbitrary_file": lambda root: (
                root.mkdir(), (root / "unregistered.txt").write_text("dirty")
            ),
            "retry_directory": lambda root: (
                root.mkdir(), (root / "p4-g0c-retry").mkdir()
            ),
            "old_analyzer_output": lambda root: (
                root.mkdir(), (root / "p4_g0c_analysis.json").write_text("{}")
            ),
        }
        for label, prepare in dirty_cases.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "runs"
                prepare(root)
                calls = []
                with self.assertRaisesRegex(MODULE.RunnerError, "dirty runs root"):
                    MODULE.run(
                        self.bundle,
                        root,
                        gpu_preflight=lambda _: (
                            calls.append("gpu") or {"gpu_ready": True}
                        ),
                        launch_executor=lambda *_: calls.append("launch"),
                    )
                self.assertEqual(calls, [])
                self.assertFalse(
                    (root / "p4_g0c_runner_state.json").exists()
                )

        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "target"
            target.mkdir()
            root = Path(tmp) / "runs"
            root.symlink_to(target, target_is_directory=True)
            calls = []
            with self.assertRaisesRegex(MODULE.RunnerError, "symlink runs root"):
                MODULE.run(
                    self.bundle,
                    root,
                    gpu_preflight=lambda _: (
                        calls.append("gpu") or {"gpu_ready": True}
                    ),
                    launch_executor=lambda *_: calls.append("launch"),
                )
            self.assertEqual(calls, [])
            self.assertFalse(
                (target / "p4_g0c_runner_state.json").exists()
            )

    def test_gpu_failure_emits_not_ready_and_starts_no_launch(self):
        order = []
        with tempfile.TemporaryDirectory() as tmp:
            result = MODULE.run(
                self.bundle,
                Path(tmp) / "runs",
                gpu_preflight=lambda _: (
                    order.append("gpu") or {
                        "gpu_ready": False,
                        "failure_reason": "cuInit_failed_100",
                    }
                ),
                launch_executor=lambda *_: order.append("ros"),
            )
        self.assertEqual(order, ["gpu"])
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertEqual(result["failure_reason"], "GPU_NOT_READY")
        self.assertFalse(result["launch_started"])

    def test_preflight_only_starts_no_launch(self):
        order = []
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            result = MODULE.run(
                self.bundle,
                root,
                preflight_only=True,
                gpu_preflight=lambda _: (
                    order.append("gpu") or {"gpu_ready": True}
                ),
                launch_executor=lambda *_: order.append("ros"),
            )
            with self.assertRaisesRegex(
                MODULE.RunnerError, "existing runner state"
            ):
                MODULE.run(
                    self.bundle,
                    root,
                    gpu_preflight=lambda _: order.append("gpu-reuse"),
                    launch_executor=lambda *_: order.append("ros-reuse"),
                )
        self.assertEqual(order, ["gpu"])
        self.assertEqual(result["runner_state"], "PREFLIGHT_PASS")
        self.assertFalse(result["launch_started"])

    def test_complete_matrix_binds_exact_production_artifact_inventories(self):
        launched = []

        def launch(record, *_):
            launched.append(record["run_id"])
            self._write_production_outputs(record, len(launched) - 1)
            return 0, {
                "required_processes_ok": True,
                "required_processes": {
                    name: {"seen": True, "runtime_failure": False}
                    for name in MODULE.REQUIRED_PROCESSES
                },
                "process_failures": [],
            }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            result = MODULE.run(
                self.bundle,
                root,
                gpu_preflight=lambda _: {"gpu_ready": True},
                launch_executor=launch,
            )
            persisted = json.loads(
                (root / "p4_g0c_runner_state.json").read_text()
            )
            first_inventory_path = Path(
                result["attempts"][0]["artifact_inventory_path"]
            )
            first_inventory = json.loads(first_inventory_path.read_text())
            inventory_raw = first_inventory_path.read_bytes()
        self.assertEqual(len(launched), 15)
        self.assertEqual(result["schema_version"], "p4_g0c_runner_state_v4")
        self.assertEqual(result["runner_state"], "COMPLETE")
        self.assertEqual(result, persisted)
        self.assertTrue(all(
            attempt["state"] == "COMPLETE"
            and set(attempt) == {
                "attempt_index", "run_id", "state",
                "artifact_inventory_path", "artifact_inventory_sha256",
                "test_planner_manifest_path", "test_planner_manifest_sha256",
            }
            for attempt in result["attempts"]
        ))
        self.assertEqual(
            result["attempts"][0]["artifact_inventory_sha256"],
            hashlib.sha256(inventory_raw).hexdigest(),
        )
        self.assertEqual(
            first_inventory["schema_version"],
            "p4_g0c_run_artifact_inventory_v1",
        )
        paths = {entry["path"] for entry in first_inventory["entries"]}
        self.assertTrue({
            "exports/synthetic_run_token/test_planner_manifest.json",
            "runtime/profiling/iap_timing.csv",
            "stdout.log",
            "launch_command.json",
            "p4_g0c_run_manifest.json",
            "p4_decisions.csv",
        }.issubset(paths))
        self.assertNotIn("p4_g0c_artifact_inventory.json", paths)

    def test_required_process_failure_stops_remaining_matrix_without_retry(self):
        launched = []
        persisted_before_executor = []
        root = None

        def launch(record, command, duration_s, required):
            launched.append(record["run_id"])
            state_path = root / "p4_g0c_runner_state.json"
            persisted_before_executor.append(
                json.loads(state_path.read_text()) if state_path.is_file()
                else None
            )
            return 0, {
                "required_processes_ok": False,
                "required_processes": {
                    "iap_rosnode": {"seen": True, "runtime_failure": True},
                    "ego_planner_node": {"seen": True, "runtime_failure": False},
                },
                "process_failures": [{
                    "process_name": "iap_rosnode",
                    "reason": "required_process_died_before_controlled_shutdown",
                }],
            }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            result = MODULE.run(
                self.bundle,
                root,
                gpu_preflight=lambda _: {"gpu_ready": True},
                launch_executor=launch,
            )
        self.assertEqual(launched, ["p4-g0c-r2-seed211-rep01"])
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertEqual(result["completed_run_count"], 0)
        self.assertEqual(result["launch_invocations"], 1)
        self.assertEqual(result["retries"], 0)
        self.assertEqual(
            persisted_before_executor[0]["attempted_run_ids"],
            ["p4-g0c-r2-seed211-rep01"],
        )
        self.assertEqual(result["attempted_run_ids"], [
            "p4-g0c-r2-seed211-rep01"
        ])
        self.assertEqual(result["completed_run_ids"], [])
        self.assertEqual(result["attempts"][0]["state"], "FAILED")
        self.assertEqual(result["attempts"][0]["attempt_index"], 1)
        self.assertNotIn(
            "artifact_inventory_path", result["attempts"][0]
        )
        self.assertNotIn(
            "test_planner_manifest_sha256", result["attempts"][0]
        )

    def test_top_level_success_without_bound_manifest_csv_fails_closed(self):
        def launch(*_):
            return 0, {
                "required_processes_ok": True,
                "required_processes": {
                    name: {"seen": True, "runtime_failure": False}
                    for name in MODULE.REQUIRED_PROCESSES
                },
                "process_failures": [],
            }

        with tempfile.TemporaryDirectory() as tmp:
            result = MODULE.run(
                self.bundle,
                Path(tmp) / "runs",
                gpu_preflight=lambda _: {"gpu_ready": True},
                launch_executor=launch,
            )
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertIn("missing or malformed run manifest", result["failure_reason"])
        self.assertEqual(result["launch_invocations"], 1)
        self.assertEqual(result["retries"], 0)
        self.assertEqual(result["attempted_run_ids"], [
            "p4-g0c-r2-seed211-rep01"
        ])
        self.assertEqual(result["completed_run_ids"], [])
        self.assertEqual(result["attempts"][0]["state"], "FAILED")

    def test_non_object_manifest_persists_failed_attempt(self):
        def launch(record, *_):
            manifest_path = (
                Path(record["run_dir"]) / "p4_g0c_run_manifest.json"
            )
            manifest_path.write_text("[]\n")
            return 0, {
                "required_processes_ok": True,
                "required_processes": {
                    name: {"seen": True, "runtime_failure": False}
                    for name in MODULE.REQUIRED_PROCESSES
                },
                "process_failures": [],
            }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            result = MODULE.run(
                self.bundle,
                root,
                gpu_preflight=lambda _: {"gpu_ready": True},
                launch_executor=launch,
            )
            persisted = json.loads(
                (root / "p4_g0c_runner_state.json").read_text()
            )
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertEqual(result["attempts"][0]["state"], "FAILED")
        self.assertEqual(
            result["failed_run_id"], "p4-g0c-r2-seed211-rep01"
        )
        self.assertEqual(persisted, result)

    def test_finalization_io_failure_persists_failed_attempt(self):
        def launch(*_):
            return 0, {
                "required_processes_ok": True,
                "required_processes": {
                    name: {"seen": True, "runtime_failure": False}
                    for name in MODULE.REQUIRED_PROCESSES
                },
                "process_failures": [],
            }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            with mock.patch.object(
                MODULE,
                "_validate_and_finalize_run",
                side_effect=OSError("manifest write failed"),
            ):
                result = MODULE.run(
                    self.bundle,
                    root,
                    gpu_preflight=lambda _: {"gpu_ready": True},
                    launch_executor=launch,
                )
            persisted = json.loads(
                (root / "p4_g0c_runner_state.json").read_text()
            )
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertEqual(result["attempts"][0]["state"], "FAILED")
        self.assertEqual(
            result["failed_run_id"], "p4-g0c-r2-seed211-rep01"
        )
        self.assertEqual(persisted, result)

    def test_malformed_typed_csv_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = MODULE.expand_run_plan(
                self.bundle.protocol, Path(tmp)
            )[0]
            run_dir = Path(record["run_dir"])
            run_dir.mkdir()
            csv_path = run_dir / "p4_decisions.csv"
            manifest = {
                "schema_version": "p4_g0c_run_manifest_v2",
                "run_id": record["run_id"],
                "seed": record["seed"],
                "repetition": record["repetition"],
                "protocol_sha256": self.bundle.protocol_sha256,
                "registry_sha256": self.bundle.registry_sha256,
                "fixture_sha256": self.bundle.fixture_sha256,
                "csv_path": str(csv_path.resolve()),
                "gate": "G0C",
                "experiment": "p4_g0c_metrics_calibration_v2",
                "scenario": "p4_g0c_free_corridor_v1",
                "decision_schema_version": (
                    "p4_collision_guide_decision_v1"
                ),
                "effective_values": self.bundle.protocol["effective_values"],
                "effective_config_sha256": MODULE.effective_config_sha256(
                    self.bundle.protocol["effective_values"]
                ),
                "required_process_set": list(MODULE.REQUIRED_PROCESSES),
                "selection_applied": False,
                "record_bag": False,
                "start_rviz": False,
                "immutable_run_id": True,
                "dependency_manifest_sha256": self.bundle.protocol[
                    "runtime_dependency_manifest"
                ]["sha256"],
                "replacement_lineage_sha256": self.bundle.protocol[
                    "replacement_lineage"
                ]["sha256"],
                "overwrite_allowed": False,
                "test_planner_manifest_path": str(
                    (run_dir / "exports/test_planner_manifest.json").resolve()
                ),
            }
            (run_dir / "p4_g0c_run_manifest.json").write_text(
                json.dumps(manifest)
            )
            row = {name: "1" for name in MODULE.DECISION_CSV_COLUMNS}
            row.update({
                "schema_version": "p4_collision_guide_decision_v1",
                "request_hash": "request",
                "status": "ORIGINAL_SELECTED",
                "reason": "METRICS_ONLY",
                "selection_applied": "not-an-integer",
                "original_hash": "original",
                "risk_hash": "risk",
                "selected_hash": "original",
            })
            with csv_path.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=MODULE.DECISION_CSV_COLUMNS
                )
                writer.writeheader()
                writer.writerow(row)
            with self.assertRaisesRegex(MODULE.RunnerError, "malformed"):
                MODULE._validate_and_finalize_run(
                    self.bundle,
                    record,
                    {"required_processes_ok": True},
                )

    def test_launch_effective_disagreement_fails_finalization(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = MODULE.expand_run_plan(
                self.bundle.protocol, Path(tmp)
            )[0]
            run_dir = Path(record["run_dir"])
            run_dir.mkdir()
            self._write_production_outputs(record, 0)
            launch_path = (
                run_dir
                / "exports/synthetic_run_token/test_planner_manifest.json"
            )
            launch_manifest = json.loads(launch_path.read_text())
            launch_manifest["p4.g0c"]["effective_values"][
                "p2.metrics_only"
            ] = True
            launch_manifest["p4.g0c"]["effective_config_sha256"] = (
                MODULE.effective_config_sha256(
                    launch_manifest["p4.g0c"]["effective_values"]
                )
            )
            launch_path.write_text(json.dumps(launch_manifest) + "\n")
            with self.assertRaisesRegex(
                MODULE.RunnerError, "launch manifest effective contract mismatch"
            ):
                MODULE._validate_and_finalize_run(
                    self.bundle,
                    record,
                    {"required_processes_ok": True},
                )
            persisted = json.loads(
                (run_dir / "p4_g0c_run_manifest.json").read_text()
            )
            self.assertNotEqual(persisted.get("runner_state"), "COMPLETE")

    def test_run_manifest_python_equal_drift_fails_before_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = MODULE.expand_run_plan(
                self.bundle.protocol, Path(tmp)
            )[0]
            run_dir = Path(record["run_dir"])
            run_dir.mkdir()
            self._write_production_outputs(record, 0)
            manifest_path = run_dir / "p4_g0c_run_manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["effective_values"]["p1.metrics_only"] = 0
            manifest_path.write_text(json.dumps(manifest) + "\n")
            with self.assertRaisesRegex(
                MODULE.RunnerError, "effective_config_sha256"
            ):
                MODULE._validate_and_finalize_run(
                    self.bundle,
                    record,
                    {"required_processes_ok": True},
                )
            persisted = json.loads(manifest_path.read_text())
            self.assertNotEqual(persisted.get("runner_state"), "COMPLETE")

    def test_all_top_level_effective_disagreements_fail_before_complete(self):
        operations = {
            "remove": lambda manifest, key, _value: manifest.pop(key),
            "change": lambda manifest, key, value: manifest.__setitem__(
                key, changed_value(value)
            ),
            "wrong_type": lambda manifest, key, value: manifest.__setitem__(
                key, wrong_type_value(value)
            ),
        }
        for key in TOP_LEVEL_EFFECTIVE_KEYS:
            expected = self.bundle.protocol["effective_values"][key]
            for operation, mutate in operations.items():
                with self.subTest(key=key, operation=operation):
                    with tempfile.TemporaryDirectory() as tmp:
                        record = MODULE.expand_run_plan(
                            self.bundle.protocol, Path(tmp)
                        )[0]
                        run_dir = Path(record["run_dir"])
                        run_dir.mkdir()
                        self._write_production_outputs(record, 0)
                        launch_path = (
                            run_dir
                            / "exports/synthetic_run_token"
                            / "test_planner_manifest.json"
                        )
                        launch_manifest = json.loads(
                            launch_path.read_text()
                        )
                        nested_before = json.loads(json.dumps(
                            launch_manifest["p4.g0c"]
                        ))
                        mutate(launch_manifest, key, expected)
                        launch_path.write_text(
                            json.dumps(launch_manifest) + "\n"
                        )
                        with self.assertRaisesRegex(
                            MODULE.RunnerError,
                            "launch manifest effective contract mismatch",
                        ):
                            MODULE._validate_and_finalize_run(
                                self.bundle,
                                record,
                                {"required_processes_ok": True},
                            )
                        self.assertEqual(
                            launch_manifest["p4.g0c"], nested_before
                        )
                        self.assertFalse(
                            (run_dir / MODULE.RUN_ARTIFACT_INVENTORY_FILENAME)
                            .exists()
                        )
                        persisted = json.loads(
                            (run_dir / "p4_g0c_run_manifest.json").read_text()
                        )
                        self.assertNotEqual(
                            persisted.get("runner_state"), "COMPLETE"
                        )

    def test_launch_command_binds_identity_without_redeclaring_frozen_switches(self):
        record = MODULE.expand_run_plan(self.bundle.protocol, Path("/runs"))[0]
        command = MODULE.launch_command(self.bundle, record)
        joined = " ".join(command)
        self.assertIn("experiment:=p4_g0c_metrics_calibration_v2", command)
        self.assertIn("p4.g0c.run_id:=p4-g0c-r2-seed211-rep01", command)
        self.assertIn(f"p4.g0c.protocol_sha256:={self.bundle.protocol_sha256}", command)
        self.assertNotIn("p4.metrics_only:=", joined)
        self.assertNotIn("start_rviz:=", joined)


if __name__ == "__main__":
    unittest.main()
