import importlib.util
import csv
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
        self.protocol = REPO / "config/icra27/p4_g0c_protocol_v1.json"
        self.registry = REPO / "config/icra27/p4_threshold_registry_v1.json"
        self.fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        self.bundle = MODULE.load_bundle(
            self.protocol, self.registry, self.fixture
        )

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
        self.assertEqual(result["runs"][0]["run_id"], "p4-g0c-seed211-rep01")
        self.assertEqual(result["runs"][-1]["run_id"], "p4-g0c-seed271-rep03")

    def test_existing_even_empty_run_directory_is_refused_before_preflight(self):
        calls = []
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "p4-g0c-seed211-rep01").mkdir()
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
            result = MODULE.run(
                self.bundle,
                Path(tmp) / "runs",
                preflight_only=True,
                gpu_preflight=lambda _: (
                    order.append("gpu") or {"gpu_ready": True}
                ),
                launch_executor=lambda *_: order.append("ros"),
            )
        self.assertEqual(order, ["gpu"])
        self.assertEqual(result["runner_state"], "PREFLIGHT_PASS")
        self.assertFalse(result["launch_started"])

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
        self.assertEqual(launched, ["p4-g0c-seed211-rep01"])
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertEqual(result["completed_run_count"], 0)
        self.assertEqual(result["launch_invocations"], 1)
        self.assertEqual(result["retries"], 0)
        self.assertEqual(
            persisted_before_executor[0]["attempted_run_ids"],
            ["p4-g0c-seed211-rep01"],
        )
        self.assertEqual(result["attempted_run_ids"], [
            "p4-g0c-seed211-rep01"
        ])
        self.assertEqual(result["completed_run_ids"], [])
        self.assertEqual(result["attempts"][0]["state"], "FAILED")
        self.assertEqual(result["attempts"][0]["attempt_index"], 1)

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
            "p4-g0c-seed211-rep01"
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
            result["failed_run_id"], "p4-g0c-seed211-rep01"
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
            result["failed_run_id"], "p4-g0c-seed211-rep01"
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
                "schema_version": "p4_g0c_run_manifest_v1",
                "run_id": record["run_id"],
                "seed": record["seed"],
                "repetition": record["repetition"],
                "protocol_sha256": self.bundle.protocol_sha256,
                "registry_sha256": self.bundle.registry_sha256,
                "fixture_sha256": self.bundle.fixture_sha256,
                "csv_path": str(csv_path.resolve()),
                "gate": "G0C",
                "experiment": "p4_g0c_metrics_calibration_v1",
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
                "overwrite_allowed": False,
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

    def test_launch_command_binds_identity_without_redeclaring_frozen_switches(self):
        record = MODULE.expand_run_plan(self.bundle.protocol, Path("/runs"))[0]
        command = MODULE.launch_command(self.bundle, record)
        joined = " ".join(command)
        self.assertIn("experiment:=p4_g0c_metrics_calibration_v1", command)
        self.assertIn("p4.g0c.run_id:=p4-g0c-seed211-rep01", command)
        self.assertIn(f"p4.g0c.protocol_sha256:={self.bundle.protocol_sha256}", command)
        self.assertNotIn("p4.metrics_only:=", joined)
        self.assertNotIn("start_rviz:=", joined)


if __name__ == "__main__":
    unittest.main()
