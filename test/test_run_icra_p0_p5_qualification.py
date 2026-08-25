import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPO / "scripts/dev_planner/run_icra_p0_p5_qualification.py"
SPEC = importlib.util.spec_from_file_location("icra_p0_p5_live_runner", RUNNER_PATH)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class IcraP0P5LiveRunnerTest(unittest.TestCase):
    def setUp(self):
        self.contract_path = REPO / "config/icra27/icra_p0_p5_qualification_v1.json"
        self.contract = RUNNER.QUALIFICATION.load_contract(self.contract_path)

    def test_fixed_identities_and_full_process_contract(self):
        self.assertEqual(
            RUNNER.LIVE_IDENTITIES,
            (
                ("SAFE_NORMAL", "icra-p0-p5-live-safe-normal-001"),
                ("FINAL_REJECT", "icra-p0-p5-live-final-reject-001"),
                ("RUNTIME_FAIL", "icra-p0-p5-live-runtime-fail-001"),
            ),
        )
        required = set(self.contract["required_processes"])
        self.assertEqual(required, set(RUNNER.REQUIRED_PROCESSES))
        self.assertGreaterEqual(len(required), 15)
        self.assertIn("test_planner_corridor_map_publisher", required)
        self.assertIn("test_planner_so3_control_container", required)
        self.assertIn("test_planner_bag_recorder", required)

    def test_live_config_is_repository_local_and_frozen(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27/icra068") as tmp:
            run_dir = Path(tmp) / "run"
            config = RUNNER.live_config(
                self.contract, "SAFE_NORMAL", "fixed-run", run_dir
            )
        expected = RUNNER.QUALIFICATION.resolve_launch_values(
            self.contract, "SAFE_NORMAL", {}
        )
        for key, value in expected.items():
            self.assertEqual(config[key], value)
        self.assertTrue(config["record_bag"])
        self.assertFalse(config["start_rviz"])
        self.assertEqual(config["run_duration_s"], 90)
        self.assertEqual(config["gate0.evidence_run_id"], "fixed-run")
        self.assertTrue(config["runtime_root_dir"].startswith(str(REPO)))

    def test_normalizer_binds_real_bag_and_p5_rows(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27/icra068") as tmp:
            run_dir = Path(tmp) / "run"
            export = run_dir / "exports/one"
            bag = run_dir / "bags/one"
            export.mkdir(parents=True)
            bag.mkdir(parents=True)
            manifest = RUNNER.QUALIFICATION.build_launch_binding(
                self.contract, self.contract_path, "RUNTIME_FAIL",
                "a" * 40, "fixed-run",
                RUNNER.QUALIFICATION.resolve_launch_values(
                    self.contract, "RUNTIME_FAIL", {}
                ),
            )
            (export / "test_planner_manifest.json").write_text(
                json.dumps({"icra_p0_p5_qualification": manifest}) + "\n"
            )
            (bag / "metadata.yaml").write_text("metadata\n")
            (bag / "evidence.db3").write_bytes(b"bag")
            for name in (
                "process_result.json", "capture_ready.json",
                "launch_command.json", "stdout.log",
            ):
                (run_dir / name).write_text("evidence\n")
            process_result = {
                "required_processes_ok": True,
                "controlled_shutdown": True,
                "orphan_check_passed": True,
                "forced_orphan_cleanup": False,
                "remaining_process_group_pids": [],
                "required_processes": {
                    name: {"seen": True, "runtime_failure": False}
                    for name in RUNNER.REQUIRED_PROCESSES
                },
                "process_failures": [],
            }
            metadata = {"missing": False, "topic_counts": {
                topic: 3 for topic in self.contract["required_topics"]
            }}
            p0_rows = [
                {"ready": True, "stale": False, "generation_id": 4,
                 "predictor_requested_worker_count": 4,
                 "predictor_effective_worker_count": 4,
                 "refresh_duration_ms": 12.0},
                {"ready": True, "stale": False, "generation_id": 5,
                 "predictor_requested_worker_count": 4,
                 "predictor_effective_worker_count": 4,
                 "refresh_duration_ms": 11.0},
            ]
            p5_rows = [
                {"bag_time_s": 1.0, "phase": "final_candidate", "action": "OK",
                 "reason": "ok", "final_candidate_traj_id": 7,
                 "final_candidate_rejected": False, "parse_error": ""},
                {"bag_time_s": 2.0, "phase": "runtime_committed", "action": "OK",
                 "reason": "ok", "final_candidate_traj_id": 7,
                 "parse_error": ""},
                {"bag_time_s": 4.5, "phase": "runtime_committed",
                 "action": "REQUEST_EMERGENCY_STOP_CANDIDATE",
                 "reason": "future_unknown_timeout", "final_candidate_traj_id": 7,
                 "parse_error": ""},
            ]
            bsplines = [{"bag_time_s": 1.5, "traj_id": 7}]
            with mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_bag_metadata", return_value=metadata
            ), mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_p0_bag_artifacts",
                return_value=({"health_rows": p0_rows}, ""),
            ), mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_p5_status_messages",
                return_value=(p5_rows, ""),
            ), mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_bspline_messages",
                return_value=(bsplines, ""),
            ):
                normalized = RUNNER.normalize_live_run(
                    self.contract, self.contract_path, "RUNTIME_FAIL",
                    "fixed-run", run_dir, process_result, "a" * 40,
                )
                process_result["required_processes"][
                    next(iter(RUNNER.REQUIRED_PROCESSES))
                ]["seen"] = False
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "required_process_lifecycle_mismatch"
                ):
                    RUNNER.normalize_live_run(
                        self.contract, self.contract_path, "RUNTIME_FAIL",
                        "fixed-run", run_dir, process_result, "a" * 40,
                    )
        self.assertFalse(normalized["validation_only"])
        self.assertEqual(
            [event["type"] for event in normalized["events"]],
            ["FINAL_ACCEPT", "NORMAL_PUBLISH", "RUNTIME_ACTION"],
        )
        self.assertEqual(normalized["events"][-1]["action"], "EMERGENCY_STOP")
        self.assertEqual(normalized["events"][-1]["reason"], "future_unknown_timeout")
        self.assertTrue(any(path.endswith("evidence.db3") for path in normalized["raw_sources"]))
        self.assertTrue(any(path.endswith("process_result.json") for path in normalized["raw_sources"]))

    def test_event_normalization_rejects_duplicates_and_early_runtime(self):
        accepted = {
            "bag_time_s": 2.0, "phase": "final_candidate", "action": "OK",
            "final_candidate_traj_id": 7, "final_candidate_rejected": False,
            "parse_error": "",
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "final_accept_event_cardinality_mismatch"
        ):
            RUNNER._normalize_events(
                "SAFE_NORMAL", self.contract, [accepted, dict(accepted)],
                [{"bag_time_s": 3.0, "traj_id": 7}],
            )
        early = {
            "bag_time_s": 1.0, "phase": "runtime_committed",
            "action": "REQUEST_EMERGENCY_STOP_CANDIDATE",
            "reason": "future_unknown", "future_unknown_duration_s": 2.0,
            "parse_error": "",
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "future_unknown_emergency_before_threshold"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract, [accepted, early],
                [{"bag_time_s": 3.0, "traj_id": 7}],
            )

    def test_live_environment_rejects_caller_overlay(self):
        with mock.patch.dict(RUNNER.os.environ, {}, clear=True):
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "live_environment_mismatch"
            ):
                RUNNER.validate_live_environment()

    def test_top_level_early_exit_is_runtime_failure_not_success(self):
        launch = mock.Mock(pid=987654)
        launch.wait.return_value = 0
        monitor = mock.Mock()
        monitor.finish.return_value = {
            "required_processes_ok": True,
            "process_failures": [],
            "required_processes": {
                name: {"seen": True, "runtime_failure": False}
                for name in RUNNER.REQUIRED_PROCESSES
            },
        }
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27/icra068") as tmp:
            with mock.patch.object(
                RUNNER.subprocess, "Popen", return_value=launch
            ), mock.patch.object(
                RUNNER.GATE_RUNNER, "RequiredProcessMonitor", return_value=monitor
            ):
                exit_code, result = RUNNER._run_launch(
                    ["ros2", "launch"], Path(tmp) / "stdout.log"
                )
        self.assertEqual(exit_code, 0)
        self.assertFalse(result["required_processes_ok"])
        self.assertFalse(result["controlled_shutdown"])
        self.assertTrue(any(
            item["reason"] == "top_level_launch_exited_before_controlled_shutdown"
            for item in result["process_failures"]
        ))

    def test_run_matrix_stops_after_first_failure_and_never_retries(self):
        calls = []

        def execute(case_id, run_id):
            calls.append((case_id, run_id))
            return {"completed": case_id == "SAFE_NORMAL"}

        result = RUNNER.run_ordered_attempts(execute)
        self.assertEqual(calls, list(RUNNER.LIVE_IDENTITIES[:2]))
        self.assertEqual(result["attempted"], [item[1] for item in calls])
        self.assertEqual(result["completed"], [RUNNER.LIVE_IDENTITIES[0][1]])
        self.assertEqual(result["retries"], 0)

    def test_install_alias_audit_rejects_source_install_mismatch(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27/icra068") as tmp:
            install = Path(tmp)
            for relative, source in RUNNER.INSTALLED_ALIASES.items():
                target = install / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((REPO / source).read_bytes())
            mismatched = install / next(iter(RUNNER.INSTALLED_ALIASES))
            mismatched.write_text("stale\n")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "installed_source_mismatch"
            ):
                RUNNER.verify_installed_aliases(install)


if __name__ == "__main__":
    unittest.main()
