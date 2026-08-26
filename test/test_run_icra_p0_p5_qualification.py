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
EMPTY_DEFAULT_KEYS = {
    "p1.debug_csv_path", "p2.debug_csv_path", "p3.debug_csv_path",
    "p4.debug_csv_path", "p4.profile_trace_path", "p4.g0c.protocol_path",
    "p4.g0c.protocol_sha256", "p4.g0c.registry_path",
    "p4.g0c.registry_sha256", "p4.g0c.fixture_path",
    "p4.g0c.fixture_sha256", "p4.g0c.run_id",
    "p4.g0c.run_manifest_path", "p4.g0c.csv_path", "p4.g0c.child_home",
    "p4.g0c.child_ros_home", "p4.g0c.child_ros_log_dir",
    "p4.g0c.child_tmpdir", "p4.g0c.child_xdg_runtime_dir",
}


class IcraP0P5LiveRunnerTest(unittest.TestCase):
    def setUp(self):
        self.contract_path = REPO / "config/icra27/icra_p0_p5_qualification_v1.json"
        self.contract = RUNNER.QUALIFICATION.load_contract(self.contract_path)

    def test_fixed_identities_and_full_process_contract(self):
        self.assertEqual(
            RUNNER.LIVE_IDENTITIES,
            (
                ("SAFE_NORMAL", "icra-p0-p5-live-safe-normal-002"),
                ("FINAL_REJECT", "icra-p0-p5-live-final-reject-002"),
                ("RUNTIME_FAIL", "icra-p0-p5-live-runtime-fail-002"),
            ),
        )
        required = set(self.contract["required_processes"])
        self.assertEqual(required, set(RUNNER.REQUIRED_PROCESSES))
        self.assertGreaterEqual(len(required), 15)
        self.assertIn("test_planner_corridor_map_publisher", required)
        self.assertIn("test_planner_so3_control_container", required)
        self.assertIn("test_planner_bag_recorder", required)

    def test_live_config_is_repository_local_and_frozen(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
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

    def test_exact_commands_omit_only_registered_empty_defaults(self):
        for case_id, run_id in RUNNER.LIVE_IDENTITIES:
            with self.subTest(case_id=case_id):
                run_dir = REPO / "results/icra27/icra069/live" / run_id
                config = RUNNER.live_config(
                    self.contract, case_id, run_id, run_dir
                )
                command, omitted = RUNNER.render_live_launch_command(
                    config, EMPTY_DEFAULT_KEYS
                )
                self.assertEqual(set(omitted), EMPTY_DEFAULT_KEYS)
                self.assertFalse(any(token.endswith(":=") for token in command))
                rendered_names = [
                    token.split(":=", 1)[0] for token in command[4:]
                ]
                expected_nonempty = {
                    key for key, value in config.items() if value != ""
                }
                self.assertEqual(set(rendered_names), expected_nonempty)
                self.assertEqual(len(rendered_names), len(set(rendered_names)))
                self.assertIn("planner_enable_all_safety:=false", command)
                self.assertIn("p1.lambda_integrity:=0.0", command)

    def test_command_renderer_rejects_unregistered_empty_and_duplicates(self):
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "unregistered_empty_override"
        ):
            RUNNER.render_live_launch_command(
                [("experiment", "")], EMPTY_DEFAULT_KEYS
            )
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "duplicate_override"
        ):
            RUNNER.render_live_launch_command(
                [("record_bag", True), ("record_bag", False)],
                EMPTY_DEFAULT_KEYS,
            )

    def test_adoption_payload_separates_product_and_runner_provenance(self):
        current_commit = "b" * 40
        changed = [
            "scripts/dev_planner/run_icra_p0_p5_qualification.py",
            "test/test_run_icra_p0_p5_qualification.py",
        ]
        payload = RUNNER.build_adoption_payload(current_commit, changed)
        self.assertEqual(
            payload["product_install"]["git_commit"], RUNNER.PRODUCT_COMMIT
        )
        self.assertEqual(
            payload["product_install"]["manifest_sha256"],
            RUNNER.PRODUCT_MANIFEST_SHA256,
        )
        self.assertEqual(payload["runner_analyzer"]["git_commit"], current_commit)
        self.assertEqual(payload["post_product_changed_files"], changed)
        self.assertEqual(payload["installed_runtime_source_overlap"], [])
        self.assertTrue(payload["product_runtime_unchanged"])

    def test_adoption_rejects_wrong_product_manifest_hash(self):
        with mock.patch.object(RUNNER, "_sha256", return_value="0" * 64):
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "product_manifest_sha256_mismatch"
            ):
                RUNNER.build_adoption_payload("b" * 40, [])

    def test_parse_only_command_preserves_rendered_overrides(self):
        rendered = [
            "ros2", "launch", "iap", "test_planner.launch.py",
            "record_bag:=true", "run_duration_s:=90",
        ]
        self.assertEqual(
            RUNNER.parse_only_command(rendered),
            [
                "ros2", "launch", "--show-args", "iap",
                "test_planner.launch.py", "record_bag:=true",
                "run_duration_s:=90",
            ],
        )

    def test_replacement_analysis_reconciles_only_proven_commit_split(self):
        base = {
            "technical_failures": ["install manifest commit mismatch"],
            "behavioral_failures": [],
        }
        accepted = RUNNER.reconcile_replacement_analysis(base, [])
        self.assertEqual(
            accepted["status"], "P5_PROSPECTIVE_QUALIFICATION_PASS"
        )
        self.assertTrue(accepted["qualification_claim"])

        forged = {
            "technical_failures": [
                "install manifest commit mismatch", "raw artifact hash mismatch",
            ],
            "behavioral_failures": [],
        }
        rejected = RUNNER.reconcile_replacement_analysis(forged, [])
        self.assertEqual(
            rejected["status"],
            "P5_PROSPECTIVE_QUALIFICATION_TECHNICAL_BLOCKER",
        )
        self.assertIn("raw artifact hash mismatch", rejected["technical_failures"])

    def test_normalizer_binds_real_bag_and_p5_rows(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
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
                 "future_unknown_duration_s": 2.5,
                 "samples": [{
                     "trajectory_sample_source": "runtime_committed",
                     "fixture_match": True,
                     "fixture_expected_reason": "future_unknown",
                     "reason": "future_unknown", "unknown": True,
                     "x": 1.0, "y": 0.0, "z": 0.0, "query_tau_s": 1.0,
                 }],
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
                process_result["required_processes"][
                    next(iter(RUNNER.REQUIRED_PROCESSES))
                ]["seen"] = True
                process_result["process_failures"] = [{
                    "process_name": next(iter(RUNNER.REQUIRED_PROCESSES)),
                    "phase": "runtime", "reason": "required_process_died",
                }]
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
            "samples": [{
                "trajectory_sample_source": "runtime_committed",
                "fixture_match": True,
                "fixture_expected_reason": "future_unknown",
                "reason": "future_unknown", "unknown": True,
                "x": 1.0, "y": 0.0, "z": 0.0, "query_tau_s": 1.0,
            }],
            "parse_error": "",
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "future_unknown_emergency_before_threshold"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract, [accepted, early],
                [{"bag_time_s": 3.0, "traj_id": 7}],
            )

    def test_event_normalization_rejects_unregistered_fixture_attribution(self):
        rejected = {
            "bag_time_s": 2.0, "phase": "final_candidate", "action": "REPLAN",
            "final_candidate_traj_id": 7, "final_candidate_rejected": True,
            "samples": [{
                "trajectory_sample_source": "final_candidate",
                "fixture_match": True, "fixture_expected_reason": "unrelated",
                "reason": "unrelated", "bad": True,
                "x": -10.0, "y": 0.0, "z": 1.1, "query_tau_s": 1.0,
            }],
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "FINAL_REJECT", self.contract, [rejected], []
            )
        fixture_only = dict(rejected)
        fixture_only.update(
            action="OK", final_candidate_rejected=False,
            final_candidate_traj_id=6,
        )
        unrelated_reject = dict(rejected)
        unrelated_reject["samples"] = []
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "FINAL_REJECT", self.contract,
                [fixture_only, unrelated_reject], [],
            )
        accepted = {
            "bag_time_s": 1.0, "phase": "final_candidate", "action": "OK",
            "final_candidate_traj_id": 9, "final_candidate_rejected": False,
        }
        fixture_runtime = {
            "bag_time_s": 2.0, "phase": "runtime_committed", "action": "OK",
            "samples": [{
                "trajectory_sample_source": "runtime_committed",
                "fixture_match": True,
                "fixture_expected_reason": "future_unknown",
                "reason": "future_unknown", "unknown": True,
                "x": 1.0, "y": 0.0, "z": 0.0, "query_tau_s": 1.0,
            }],
        }
        unrelated_emergency = {
            "bag_time_s": 4.0, "phase": "runtime_committed",
            "action": "REQUEST_EMERGENCY_STOP_CANDIDATE",
            "reason": "future_unknown_timeout", "future_unknown_duration_s": 2.0,
            "samples": [],
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract,
                [accepted, fixture_runtime, unrelated_emergency],
                [{"bag_time_s": 1.5, "traj_id": 9}],
            )
        attributed_emergency = dict(unrelated_emergency)
        attributed_emergency["bag_time_s"] = 5.0
        attributed_emergency["samples"] = fixture_runtime["samples"]
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract,
                [accepted, unrelated_emergency, attributed_emergency],
                [{"bag_time_s": 1.5, "traj_id": 9}],
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
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
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
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
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

    def test_install_manifest_revalidation_rejects_reduced_inventory(self):
        retained = REPO / "results/icra27/icra068/icra068_install_manifest.json"
        manifest = json.loads(retained.read_text())
        manifest["git_commit"] = "a" * 40
        manifest["file_hashes"] = {}
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            path = Path(tmp) / "manifest.json"
            path.write_text(json.dumps(manifest) + "\n")
            with mock.patch.dict(
                RUNNER.os.environ,
                {"AMENT_PREFIX_PATH": ":".join(manifest["active_prefixes"])},
                clear=False,
            ), self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "install_manifest_inventory_mismatch"
            ):
                RUNNER.validate_frozen_install_manifest(path, "a" * 40)

    def test_linkage_inventory_rejects_one_nibble_drift(self):
        libraries = ("lib/libiap.so",)
        with mock.patch.object(
            RUNNER, "_linkage_ready", return_value="a" * 64
        ):
            self.assertFalse(RUNNER.linkage_inventory_matches(
                {libraries[0]: "b" + "a" * 63}, libraries,
                RUNNER.INSTALL_ROOT, {},
            ))


if __name__ == "__main__":
    unittest.main()
