import copy
import hashlib
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
CONTRACT_PATH = REPO / "config" / "icra27" / "icra_p0_p5_qualification_v1.json"
HELPER_PATH = REPO / "launch" / "icra_p0_p5_qualification.py"
SPEC = importlib.util.spec_from_file_location("icra_p0_p5_qualification", HELPER_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def synthetic_bundle(contract, raw_directory):
    digest = hashlib.sha256(CONTRACT_PATH.read_bytes()).hexdigest()
    common = {
        "processes": [
            {"identity": name, "alive_until_controlled_shutdown": True}
            for name in contract["required_processes"]
        ],
        "controlled_shutdown": True,
        "topic_counts": {name: 3 for name in contract["required_topics"]},
        "p0_samples": [
            {"sequence": 1, "ready": True, "stable": True, "refresh_s": 0.2,
             "gnss_epoch_seen": True, "gnss_epoch_valid": True,
             "gnss_epoch_fresh": True, "predictor_gnss_used_count": 4,
             "predictor_lidar_used_count": 4,
             "predictor_horizon_fusion_count": 6},
            {"sequence": 2, "ready": True, "stable": True, "refresh_s": 0.21,
             "gnss_epoch_seen": True, "gnss_epoch_valid": True,
             "gnss_epoch_fresh": True, "predictor_gnss_used_count": 5,
             "predictor_lidar_used_count": 5,
             "predictor_horizon_fusion_count": 6},
        ],
        "integrity_samples": [
            {"sequence": 1, "valid": True, "n_sv_used": 8},
            {"sequence": 2, "valid": True, "n_sv_used": 9},
        ],
    }
    runs = [
        {
            **copy.deepcopy(common),
            "case_id": "SAFE_NORMAL",
            "run_id": "icra-p0-p5-safe-001",
            "fixture_alias": "none_v1",
            "events": [
                {"sequence": 1, "type": "FINAL_ACCEPT", "candidate_id": "safe-candidate"},
                {"sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "safe-candidate"},
            ],
        },
        {
            **copy.deepcopy(common),
            "case_id": "FINAL_REJECT",
            "run_id": "icra-p0-p5-reject-001",
            "fixture_alias": "p5_7_rejected_trajectory_zone_v1",
            "events": [
                {"sequence": 1, "type": "FINAL_REJECT", "candidate_id": "rejected-candidate", "reason": "p5_7_rejected_trajectory"},
            ],
        },
        {
            **copy.deepcopy(common),
            "case_id": "RUNTIME_FAIL",
            "run_id": "icra-p0-p5-runtime-001",
            "fixture_alias": "p5_6_future_unknown_zone_v1",
            "events": [
                {"sequence": 1, "type": "FINAL_ACCEPT", "candidate_id": "runtime-candidate"},
                {"sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "runtime-candidate"},
                {"sequence": 3, "type": "RUNTIME_ACTION", "candidate_id": "runtime-candidate", "action": "EMERGENCY_STOP", "reason": "future_unknown_timeout"},
            ],
        },
    ]
    for run in runs:
        values = MODULE.resolve_launch_values(contract, run["case_id"], {})
        run["launch_binding"] = MODULE.build_launch_binding(
            contract, CONTRACT_PATH, run["case_id"], "1" * 40,
            run["run_id"], values,
        )
    bundle = {
        "schema_version": "icra_p0_p5_synthetic_evidence_v1",
        "validation_only": True,
        "manifest": {
            "route_id": contract["route_id"],
            "git_commit": "1" * 40,
            "contract_sha256": digest,
            "analyzer_version": contract["analyzer_version"],
            "fixture_identities": {
                case_id: contract["cases"][case_id]["fixture_alias"]
                for case_id in MODULE.CASE_IDS
            },
            "run_identities": {
                run["case_id"]: run["run_id"] for run in runs
            },
            "raw_artifact_hashes": {
                run["run_id"]: {} for run in runs
            },
            "normalized_evidence_sha256": {
                run["run_id"]: MODULE.evidence_sha256(run) for run in runs
            },
        },
        "runs": runs,
    }
    return MODULE.bind_run_artifacts(bundle, REPO, raw_directory)


class IcraP0P5QualificationTest(unittest.TestCase):
    def setUp(self):
        self.contract = MODULE.load_contract(CONTRACT_PATH)
        self.raw_tmp = tempfile.TemporaryDirectory(dir=REPO)
        self.addCleanup(self.raw_tmp.cleanup)
        self.raw_directory = Path(self.raw_tmp.name) / "raw"

    def test_profile_values_are_frozen_and_all_other_paths_are_off(self):
        values = MODULE.resolve_launch_values(self.contract, "SAFE_NORMAL", {})
        self.assertEqual(values["planner_safety_profile"], "icra_p0_p5")
        for key in (
            "planner_enable_all_safety", "planner_enable_p1", "planner_enable_p2",
            "planner_enable_p3_local", "planner_enable_p3_global", "planner_enable_p4",
            "p1.use_integrity_cost", "p1.metrics_only", "p2.enable_candidate_ranking",
            "p2.metrics_only", "p3.enable_local_reference_bias",
            "p3.enable_global_reference_bias", "p4.enable_risk_aware_astar",
            "p4.metrics_only", "p4.profile_trace_enable", "manager/use_distinctive_trajs",
        ):
            self.assertFalse(values[key], key)
        for key in (
            "p0.enable_risk_grid", "planner_enable_p5_runtime",
            "planner_enable_p5_final", "p5.enable_runtime_gate",
            "p5.enable_final_gate", "gate0.qualification_evidence_enable",
        ):
            self.assertTrue(values[key], key)
        self.assertEqual(values["p0.predictor.worker_count"], 4)
        self.assertEqual(values["p0.predictor.sigma_grow_m_sqrt_s"], 0.01)
        self.assertEqual(values["p0.predictor.sigma_growth_profile"], "legacy_iap_rq320_baseline_v1")

    def test_all_cases_bind_same_full_sensor_scenario_and_topics(self):
        scenario = "icra_p0_p5_fused_degraded_corridor_v1"
        self.assertEqual(
            [self.contract["cases"][case]["scenario"] for case in MODULE.CASE_IDS],
            [scenario, scenario, scenario],
        )
        self.assertEqual(set(self.contract["required_topics"]), {
            "/planning/risk_grid_health",
            "/planning/integrity_gate_status",
            "/drone_0_planning/bspline",
            "/sim/drone_0/imu",
            "/sim/drone_0/imu_iap",
            "/sim/drone_0/lidar",
            "/sim/drone_0/lidar_body",
            "/ublox_driver/range_meas",
            "/gnss_sim/diagnostics",
            "/iap/integrity",
        })
        self.assertEqual(len(self.contract["required_processes"]), 16)
        self.assertIn("test_planner_gnss_sim_node", self.contract["required_processes"])

    def test_contract_rejects_reduced_sensor_qualification(self):
        reduced = copy.deepcopy(self.contract)
        reduced["qualification_values"]["use_gnss"] = False
        with tempfile.TemporaryDirectory(dir=REPO) as tmp:
            path = Path(tmp) / "contract.json"
            path.write_text(json.dumps(reduced) + "\n")
            with self.assertRaisesRegex(
                MODULE.ContractError, "full-sensor qualification"
            ):
                MODULE.load_contract(path)

    def test_conflicting_profile_and_fixture_overrides_fail_closed(self):
        malicious = (
            ("planner_enable_all_safety", True),
            ("planner_enable_p1", True),
            ("p1.metrics_only", True),
            ("safety_viz.enable_p4_viz", True),
            ("planner_enable_p5_final", False),
            ("p0.enable_risk_grid", False),
            ("manager/use_distinctive_trajs", True),
            ("p5_6.fixture.enabled", True),
        )
        for key, value in malicious:
            with self.subTest(key=key), self.assertRaisesRegex(MODULE.ContractError, key.replace(".", r"\.")):
                MODULE.resolve_launch_values(self.contract, "SAFE_NORMAL", {key: value})

    def test_registered_fixture_case_is_exact_and_unregistered_mix_is_rejected(self):
        values = MODULE.resolve_launch_values(self.contract, "FINAL_REJECT", {})
        self.assertTrue(values["p5_7.fixture.enabled"])
        self.assertFalse(values["p5_6.fixture.enabled"])
        with self.assertRaises(MODULE.ContractError):
            MODULE.resolve_launch_values(
                self.contract,
                "FINAL_REJECT",
                {"p5_6.fixture.enabled": True},
            )

    def test_launch_binding_identity_is_portable_across_install_and_source_paths(self):
        with tempfile.TemporaryDirectory(dir=REPO) as tmp:
            installed = Path(tmp) / "share/iap/config/icra27/contract.json"
            installed.parent.mkdir(parents=True)
            installed.write_bytes(CONTRACT_PATH.read_bytes())
            installed_contract = MODULE.load_contract(installed)
            values = MODULE.resolve_launch_values(self.contract, "SAFE_NORMAL", {})
            source_binding = MODULE.build_launch_binding(
                self.contract, CONTRACT_PATH, "SAFE_NORMAL", "1" * 40, "run-1", values
            )
            installed_binding = MODULE.build_launch_binding(
                installed_contract, installed, "SAFE_NORMAL", "1" * 40, "run-1", values
            )
        self.assertEqual(source_binding, installed_binding)

    def test_all_three_synthetic_cases_pass_without_qualification_claim(self):
        result = MODULE.analyze_bundle(
            self.contract,
            synthetic_bundle(self.contract, self.raw_directory),
            CONTRACT_PATH,
        )
        self.assertEqual(result["status"], "VALIDATION_ONLY_PASS")
        self.assertFalse(result["qualification_claim"])
        self.assertEqual(set(result["case_results"]), {"SAFE_NORMAL", "FINAL_REJECT", "RUNTIME_FAIL"})
        self.assertEqual(set(result["launch_binding_sha256"]), set(MODULE.CASE_IDS))
        self.assertEqual(result["validated_manifest"]["git_commit"], "1" * 40)

    def test_analyzer_rejects_adversarial_evidence(self):
        mutations = {}
        mutations["duplicate identity"] = lambda b: b["runs"].append(copy.deepcopy(b["runs"][0]))
        mutations["process death"] = lambda b: b["runs"][0]["processes"][0].update(alive_until_controlled_shutdown=False)
        mutations["nonfinite"] = lambda b: b["runs"][0]["p0_samples"][0].update(refresh_s=math.nan)
        mutations["topic gap"] = lambda b: b["runs"][0]["topic_counts"].update({"/planning/bspline": 0})
        mutations["unstable p0"] = lambda b: b["runs"][0]["p0_samples"][0].update(stable=False)
        mutations["stale gnss epoch"] = lambda b: b["runs"][0]["p0_samples"][0].update(gnss_epoch_fresh=False)
        mutations["zero gnss prediction"] = lambda b: b["runs"][0]["p0_samples"][0].update(predictor_gnss_used_count=0)
        mutations["zero lidar prediction"] = lambda b: b["runs"][0]["p0_samples"][0].update(predictor_lidar_used_count=0)
        mutations["zero fused horizon"] = lambda b: b["runs"][0]["p0_samples"][0].update(predictor_horizon_fusion_count=0)
        mutations["zero satellites"] = lambda b: b["runs"][0]["integrity_samples"][0].update(n_sv_used=0)
        mutations["wrong config"] = lambda b: b["runs"][0]["launch_binding"]["effective_values"].update({"planner_enable_p1": True})
        mutations["wrong hash"] = lambda b: b["manifest"].update(contract_sha256="0" * 64)
        mutations["wrong raw hash"] = lambda b: b["manifest"]["raw_artifact_hashes"]["icra-p0-p5-safe-001"].update({next(iter(b["manifest"]["raw_artifact_hashes"]["icra-p0-p5-safe-001"])): "0" * 64})
        mutations["unbound raw artifact"] = lambda b: b["manifest"]["raw_artifact_hashes"].update({"icra-p0-p5-safe-001": {str(CONTRACT_PATH.relative_to(REPO)): hashlib.sha256(CONTRACT_PATH.read_bytes()).hexdigest()}})
        mutations["fixture leakage"] = lambda b: b["runs"][0].update(fixture_alias="p5_7_rejected_trajectory_zone_v1")
        mutations["reject with publish"] = lambda b: b["runs"][1]["events"].append({"sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "rejected-candidate"})
        mutations["missing safe publish"] = lambda b: b["runs"][0].update(events=b["runs"][0]["events"][:1])
        mutations["absent runtime action"] = lambda b: b["runs"][2].update(events=b["runs"][2]["events"][:2])
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                bundle = synthetic_bundle(self.contract, self.raw_directory)
                mutate(bundle)
                result = MODULE.analyze_bundle(self.contract, bundle, CONTRACT_PATH)
                self.assertEqual(result["status"], "VALIDATION_ONLY_FAIL")
                self.assertTrue(result["failures"])

    def test_analyzer_rejects_hash_consistent_reduced_sensor_evidence(self):
        mutations = {
            "stale gnss epoch": lambda run: run["p0_samples"][0].update(
                gnss_epoch_fresh=False
            ),
            "zero gnss prediction": lambda run: run["p0_samples"][0].update(
                predictor_gnss_used_count=0
            ),
            "zero lidar prediction": lambda run: run["p0_samples"][0].update(
                predictor_lidar_used_count=0
            ),
            "zero fused horizon": lambda run: run["p0_samples"][0].update(
                predictor_horizon_fusion_count=0
            ),
            "zero satellites": lambda run: run["integrity_samples"][0].update(
                n_sv_used=0
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                bundle = synthetic_bundle(self.contract, self.raw_directory)
                mutate(bundle["runs"][0])
                MODULE.bind_run_artifacts(bundle, REPO, self.raw_directory)
                result = MODULE.analyze_bundle(
                    self.contract, bundle, CONTRACT_PATH
                )
                self.assertEqual(result["status"], "VALIDATION_ONLY_FAIL")
                self.assertTrue(any(
                    "full-sensor" in failure for failure in result["failures"]
                ), result["failures"])

    def test_final_reject_allows_publication_of_an_unrelated_candidate(self):
        bundle = synthetic_bundle(self.contract, self.raw_directory)
        run = bundle["runs"][1]
        run["events"].append({
            "sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "other-candidate"
        })
        MODULE.bind_run_artifacts(bundle, REPO, self.raw_directory)
        result = MODULE.analyze_bundle(self.contract, bundle, CONTRACT_PATH)
        self.assertEqual(result["status"], "VALIDATION_ONLY_PASS", result)

    def test_cli_writes_repository_local_validation_manifest(self):
        bundle = synthetic_bundle(self.contract, self.raw_directory)
        with tempfile.TemporaryDirectory(dir=REPO) as tmp:
            root = Path(tmp)
            source = root / "synthetic.json"
            output = root / "validation.json"
            source.write_text(json.dumps(bundle, allow_nan=False) + "\n")
            exit_code = MODULE.main([
                "analyze", "--contract", str(CONTRACT_PATH),
                "--input", str(source), "--output", str(output),
                "--repository-root", str(REPO),
            ])
            self.assertEqual(exit_code, 0)
            result = json.loads(output.read_text())
            self.assertEqual(result["status"], "VALIDATION_ONLY_PASS")
            self.assertFalse(result["qualification_claim"])

    def test_cli_rejects_caller_selected_external_repository_root(self):
        with self.assertRaisesRegex(MODULE.ContractError, "actual checkout"):
            MODULE.main([
                "emit-synthetic-input", "--contract", str(CONTRACT_PATH),
                "--git-commit", "1" * 40, "--output", "/tmp/not-allowed.json",
                "--repository-root", "/",
            ])

    def test_live_analyzer_rejects_synthetic_validation_bundle(self):
        result = MODULE.analyze_live_bundle(
            self.contract,
            synthetic_bundle(self.contract, self.raw_directory),
            CONTRACT_PATH,
        )
        self.assertEqual(
            result["status"],
            "P5_PROSPECTIVE_QUALIFICATION_TECHNICAL_BLOCKER",
        )
        self.assertFalse(result["qualification_claim"])
        self.assertTrue(any(
            "validation_only" in failure
            for failure in result["technical_failures"]
        ))

    def test_live_lifecycle_rejects_contradictory_runtime_failure(self):
        process = {
            "required_processes_ok": True,
            "controlled_shutdown": True,
            "orphan_check_passed": True,
            "forced_orphan_cleanup": False,
            "remaining_process_group_pids": [],
            "required_processes": {
                name: {"seen": True, "runtime_failure": False}
                for name in self.contract["required_processes"]
            },
            "process_failures": [{
                "process_name": self.contract["required_processes"][0],
                "phase": "runtime", "reason": "required_process_died",
            }],
        }
        self.assertFalse(MODULE.live_process_lifecycle_exact(
            process, self.contract["required_processes"]
        ))

    def test_live_analyzer_accepts_exact_one_shot_real_bundle(self):
        source = synthetic_bundle(self.contract, self.raw_directory)
        runs = source["runs"]
        for run in runs:
            run["run_id"] = MODULE.LIVE_RUN_IDENTITIES[run["case_id"]]
            run["validation_only"] = False
            run["raw_sources"] = {}
            run["launch_binding"] = MODULE.build_launch_binding(
                self.contract, CONTRACT_PATH, run["case_id"], "1" * 40,
                run["run_id"],
                MODULE.resolve_launch_values(self.contract, run["case_id"], {}),
            )
        with tempfile.TemporaryDirectory(dir=REPO) as tmp:
            root = Path(tmp)
            for run in runs:
                raw = root / "sources" / run["run_id"]
                raw.mkdir(parents=True)
                process = {
                    "required_processes_ok": True,
                    "controlled_shutdown": True,
                    "orphan_check_passed": True,
                    "forced_orphan_cleanup": False,
                    "remaining_process_group_pids": [],
                    "required_processes": {
                        name: {"seen": True, "runtime_failure": False}
                        for name in self.contract["required_processes"]
                    },
                    "process_failures": [],
                }
                (raw / "process_result.json").write_text(json.dumps(process) + "\n")
                for name in (
                    "capture_ready.json", "launch_command.json", "stdout.log",
                    "metadata.yaml",
                ):
                    (raw / name).write_text("evidence\n")
                (raw / "evidence.db3").write_bytes(b"bag")
                run["raw_sources"] = [
                    str(path.relative_to(REPO)) for path in sorted(raw.iterdir())
                ]
            install = root / "install_manifest.json"
            retained = json.loads((
                REPO / "results/icra27/icra068/icra068_install_manifest.json"
            ).read_text())
            install_root = REPO / "results/icra27/icra068/install"
            runtime_libraries = sorted(
                str(path.relative_to(install_root))
                for path in (install_root / "lib").glob("*.so")
                if path.is_file() and not path.is_symlink()
            )
            retained["git_commit"] = "1" * 40
            retained["runtime_libraries"] = runtime_libraries
            linkage_environment = dict(__import__("os").environ)
            linkage_environment["LD_LIBRARY_PATH"] = ":".join((
                str(install_root / "lib"),
                "/root/ros2_ws/install/glim_ros/lib",
                "/root/ros2_ws/install/glim/lib", "/opt/ros/jazzy/lib",
                "/opt/ros/jazzy/lib/x86_64-linux-gnu",
            ))
            retained["linkage_output_sha256"] = {
                relative: MODULE.live_linkage_sha(
                    install_root / relative, linkage_environment
                ) for relative in runtime_libraries
            }
            for relative in runtime_libraries:
                retained["file_hashes"][relative] = MODULE._sha256(
                    install_root / relative
                )
            self.assertTrue(MODULE.live_install_manifest_exact(retained, REPO))
            first_linkage = runtime_libraries[0]
            original_linkage = retained["linkage_output_sha256"][first_linkage]
            retained["linkage_output_sha256"][first_linkage] = (
                ("0" if original_linkage[0] != "0" else "1")
                + original_linkage[1:]
            )
            self.assertFalse(MODULE.live_install_manifest_exact(retained, REPO))
            retained["linkage_output_sha256"][first_linkage] = original_linkage
            install.write_text(json.dumps(retained) + "\n")
            runner = root / "runner_state.json"
            identities = list(MODULE.LIVE_RUN_IDENTITIES.values())
            runner.write_text(json.dumps({
                "state": "COMPLETE", "registered": identities,
                "attempted": identities, "completed": identities,
                "retries": 0, "gpu_preflight_invocations": 1,
                "launch_invocations": 3,
                "install_manifest_sha256": MODULE._sha256(install),
            }) + "\n")
            evidence = root / "live.json"
            bundle = MODULE.write_live_bundle(
                self.contract, CONTRACT_PATH, runs, "1" * 40,
                install, evidence,
            )
            result = MODULE.analyze_live_bundle(
                self.contract, bundle, CONTRACT_PATH, REPO
            )
            output = root / "live_result.json"
            self.assertEqual(MODULE.main([
                "analyze-live", "--contract", str(CONTRACT_PATH),
                "--input", str(evidence), "--output", str(output),
                "--repository-root", str(REPO),
            ]), 0)
            with self.assertRaisesRegex(
                MODULE.ContractError, "already exists"
            ):
                MODULE.main([
                    "analyze-live", "--contract", str(CONTRACT_PATH),
                    "--input", str(evidence), "--output", str(output),
                    "--repository-root", str(REPO),
                ])
        self.assertEqual(result["status"], "P5_PROSPECTIVE_QUALIFICATION_PASS")
        self.assertTrue(result["qualification_claim"])


if __name__ == "__main__":
    unittest.main()
