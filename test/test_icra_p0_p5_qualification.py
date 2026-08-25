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


def synthetic_bundle(contract):
    digest = hashlib.sha256(CONTRACT_PATH.read_bytes()).hexdigest()
    effective = {**contract["profile_values"], **contract["p5_thresholds"]}
    common = {
        "processes": [
            {"identity": name, "alive_until_controlled_shutdown": True}
            for name in contract["required_processes"]
        ],
        "controlled_shutdown": True,
        "topic_counts": {name: 3 for name in contract["required_topics"]},
        "p0_samples": [
            {"sequence": 1, "ready": True, "stable": True, "refresh_s": 0.2},
            {"sequence": 2, "ready": True, "stable": True, "refresh_s": 0.21},
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
    return {
        "schema_version": "icra_p0_p5_synthetic_evidence_v1",
        "validation_only": True,
        "manifest": {
            "route_id": contract["route_id"],
            "git_commit": "1" * 40,
            "profile_name": contract["profile_name"],
            "contract_sha256": digest,
            "analyzer_version": contract["analyzer_version"],
            "effective_values": effective,
            "p0_profile": MODULE.p0_profile_binding(contract),
            "p5_thresholds": contract["p5_thresholds"],
            "fixture_identities": {
                case_id: contract["cases"][case_id]["fixture_alias"]
                for case_id in MODULE.CASE_IDS
            },
            "run_identities": {
                run["case_id"]: run["run_id"] for run in runs
            },
            "raw_artifact_hashes": {
                run["run_id"]: MODULE.evidence_sha256(run) for run in runs
            },
        },
        "runs": runs,
    }


class IcraP0P5QualificationTest(unittest.TestCase):
    def setUp(self):
        self.contract = MODULE.load_contract(CONTRACT_PATH)

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

    def test_all_three_synthetic_cases_pass_without_qualification_claim(self):
        result = MODULE.analyze_bundle(self.contract, synthetic_bundle(self.contract), CONTRACT_PATH)
        self.assertEqual(result["status"], "VALIDATION_ONLY_PASS")
        self.assertFalse(result["qualification_claim"])
        self.assertEqual(set(result["case_results"]), {"SAFE_NORMAL", "FINAL_REJECT", "RUNTIME_FAIL"})

    def test_analyzer_rejects_adversarial_evidence(self):
        mutations = {}
        mutations["duplicate identity"] = lambda b: b["runs"].append(copy.deepcopy(b["runs"][0]))
        mutations["process death"] = lambda b: b["runs"][0]["processes"][0].update(alive_until_controlled_shutdown=False)
        mutations["nonfinite"] = lambda b: b["runs"][0]["p0_samples"][0].update(refresh_s=math.nan)
        mutations["topic gap"] = lambda b: b["runs"][0]["topic_counts"].update({"/planning/bspline": 0})
        mutations["unstable p0"] = lambda b: b["runs"][0]["p0_samples"][0].update(stable=False)
        mutations["wrong config"] = lambda b: b["manifest"]["effective_values"].update({"planner_enable_p1": True})
        mutations["wrong hash"] = lambda b: b["manifest"].update(contract_sha256="0" * 64)
        mutations["wrong raw hash"] = lambda b: b["manifest"]["raw_artifact_hashes"].update({"icra-p0-p5-safe-001": "0" * 64})
        mutations["fixture leakage"] = lambda b: b["runs"][0].update(fixture_alias="p5_7_rejected_trajectory_zone_v1")
        mutations["reject with publish"] = lambda b: b["runs"][1]["events"].append({"sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "rejected-candidate"})
        mutations["missing safe publish"] = lambda b: b["runs"][0].update(events=b["runs"][0]["events"][:1])
        mutations["absent runtime action"] = lambda b: b["runs"][2].update(events=b["runs"][2]["events"][:2])
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                bundle = synthetic_bundle(self.contract)
                mutate(bundle)
                result = MODULE.analyze_bundle(self.contract, bundle, CONTRACT_PATH)
                self.assertEqual(result["status"], "VALIDATION_ONLY_FAIL")
                self.assertTrue(result["failures"])

    def test_cli_writes_repository_local_validation_manifest(self):
        bundle = synthetic_bundle(self.contract)
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


if __name__ == "__main__":
    unittest.main()
