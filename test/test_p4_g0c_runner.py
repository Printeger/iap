import importlib.util
import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts/dev_planner/run_p4_g0c_calibration.py"
SPEC = importlib.util.spec_from_file_location("run_p4_g0c_calibration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class P4G0CRunnerTest(unittest.TestCase):
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
            "reason": "METRICS_ONLY",
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
            "dependency_manifest_sha256": self.bundle.protocol[
                "runtime_dependency_manifest"
            ]["sha256"],
            "replacement_lineage_sha256": self.bundle.protocol[
                "replacement_lineage"
            ]["sha256"],
            "test_planner_manifest_path": str(launch_manifest_path.resolve()),
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
        launch_manifest_path.write_text(json.dumps({
            "schema_version": "test_planner_manifest_v1",
            "run_id": record["run_id"],
            "p4.g0c": {
                key: manifest[key] for key in (
                    "schema_version", "protocol_sha256", "registry_sha256",
                    "fixture_sha256", "effective_values",
                    "effective_config_sha256", "selection_applied",
                    "record_bag", "start_rviz",
                    "dependency_manifest_sha256",
                    "replacement_lineage_sha256",
                )
            },
        }) + "\n")
        timing = run_dir / "runtime/profiling/iap_timing.csv"
        timing.parent.mkdir(parents=True)
        timing.write_text("stamp,duration_ms\n1,2\n")
        (run_dir / "stdout.log").write_text("controlled shutdown\n")

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
