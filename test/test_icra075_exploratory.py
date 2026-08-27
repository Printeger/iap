#!/usr/bin/env python3
"""Focused public-seam contracts for ICRA-075 exploratory tooling."""

import copy
import importlib.util
import json
import math
import tempfile
import unittest
from unittest import mock
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts/dev_planner/icra075_exploratory.py"
ANALYZER_PATH = REPO / "scripts/dev_planner/analyze_icra075_exploratory.py"
RUNNER_PATH = REPO / "scripts/dev_planner/run_icra075_exploratory.py"
COMPATIBILITY_PATH = REPO / "scripts/dev_planner/icra075_p5_compatibility.py"


def load_module():
    spec = importlib.util.spec_from_file_location("icra075_exploratory", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_analyzer():
    spec = importlib.util.spec_from_file_location("icra075_analyzer", ANALYZER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_runner():
    spec = importlib.util.spec_from_file_location("icra075_runner", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_compatibility():
    spec = importlib.util.spec_from_file_location(
        "icra075_p5_compatibility", COMPATIBILITY_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class Icra075ExploratoryContractTest(unittest.TestCase):
    def test_capture_readiness_uses_icra075_schema(self):
        runner = load_runner()

        class RunningProcess:
            @staticmethod
            def poll():
                return None

        with tempfile.TemporaryDirectory() as temporary:
            ready = Path(temporary) / "ready.json"
            ready.write_text(json.dumps({
                "schema_version": "icra075_capture_readiness_v1", "ready": True,
            }))
            self.assertTrue(runner._wait_capture_ready(RunningProcess(), ready)["ready"])

    def test_batch_ros_started_requires_successful_process_spawn_manifest(self):
        runner = load_runner()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            row_root = root / "row-1"
            row_root.mkdir()
            matrix = [{"run_id": "row-1"}]
            (row_root / "launch_command.json").write_text("{}\n")
            (row_root / "run_manifest.json").write_text(json.dumps({
                "first_missing_stage": "RUNNER_EXCEPTION",
            }))
            self.assertFalse(runner._batch_ros_started(root, matrix))
            (row_root / "run_manifest.json").write_text(json.dumps({
                "launch_started": True,
            }))
            self.assertTrue(runner._batch_ros_started(root, matrix))

    def test_batch_first_missing_falls_back_to_failed_analyzer_stage(self):
        runner = load_runner()
        self.assertEqual(runner._row_first_missing(
            1, {"result": "PASS", "first_missing_stage": None},
            {"result": "FAIL", "first_missing_stage": "EGO_FINAL_MISSING"}),
            "EGO_FINAL_MISSING")
        self.assertEqual(runner._row_first_missing(
            5, {"result": "FAIL", "first_missing_stage": "CAPTURE_NOT_READY"},
            {"result": "FAIL", "first_missing_stage": "P0_SNAPSHOT_MISSING"}),
            "CAPTURE_NOT_READY")

    def test_successful_analyzer_source_change_is_typed_and_fail_closed(self):
        self._assert_analyzer_source_change_is_fail_closed(0)

    def test_failed_analyzer_source_change_is_typed_and_fail_closed(self):
        self._assert_analyzer_source_change_is_fail_closed(1)

    def _assert_analyzer_source_change_is_fail_closed(self, analyzer_code):
        runner = load_runner()
        baseline = {
            "accepted": True, "head_commit": "a", "origin_dev_icra_commit": "a",
            "known_retained_inventory": [],
        }
        changed = {
            "accepted": True, "head_commit": "b", "origin_dev_icra_commit": "b",
            "known_retained_inventory": [],
        }
        completed = mock.Mock(returncode=analyzer_code, stdout="out", stderr="err")
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(runner.subprocess, "run", return_value=completed), \
                mock.patch.object(runner, "_capture_source_binding", return_value=changed):
            invocation = Path(temporary) / "analyzer_invocation.json"
            result = runner._invoke_analyzer_with_source_admission(
                ["analyzer"], invocation, Path(temporary), baseline,
                "SOURCE_CHANGED_AFTER_ANALYZER")
            retained = json.loads(invocation.read_text())
        self.assertEqual(result["effective_exit_code"], 10)
        self.assertEqual(result["exit_code"], analyzer_code)
        self.assertEqual(result["analyzer_exit_code"], analyzer_code)
        self.assertEqual(result["first_missing_stage"],
                         "SOURCE_CHANGED_AFTER_ANALYZER")
        self.assertFalse(result["source_unchanged"])
        self.assertEqual(retained["first_missing_stage"],
                         "SOURCE_CHANGED_AFTER_ANALYZER")

    def test_power_and_final_batch_source_admission_are_typed(self):
        source = RUNNER_PATH.read_text()
        self.assertIn('"SOURCE_CHANGED_AFTER_POWER_ANALYZER"', source)
        self.assertIn('"SOURCE_CHANGED_AT_FINAL_BATCH_CHECK"', source)
        self.assertIn('source_binding_after_power_analyzer', source)
        self.assertIn('if not power["source_unchanged"] or not final_source_unchanged:',
                      source)

    def test_retained_matrix_002_classifies_frozen_p5_contract_incompatibility(self):
        module = load_compatibility()
        result = module.diagnose(module.DEFAULT_RUN_ROOT)
        self.assertEqual(result["schema_version"],
                         "icra075_p5_compatibility_diagnosis_v1")
        self.assertEqual(result["classification"],
                         "FROZEN_CONTRACT_INCOMPATIBLE")
        self.assertEqual(result["p5_final_status_count"], 2137)
        self.assertEqual(result["unique_candidate_identity_count"], 2117)
        self.assertEqual(result["current_integrity_source_counts"], {"FUSED": 2137})
        self.assertEqual(result["fusion_mode_counts"], {"max_pl": 426})
        self.assertEqual(result["final_hpl_source_counts"], {"GNSS": 426})
        self.assertEqual(result["final_vpl_source_counts"], {"GNSS": 426})
        self.assertAlmostEqual(result["alert_limits_m"]["hal"], 10.0)
        self.assertAlmostEqual(result["alert_limits_m"]["val"], 20.0)
        self.assertAlmostEqual(result["observed_current_pl_m"]["hpl_min"],
                               24.3673612)
        self.assertAlmostEqual(result["observed_current_pl_m"]["hpl_max"],
                               27.733391)
        self.assertAlmostEqual(result["observed_current_pl_m"]["vpl_min"],
                               68.8205779)
        self.assertAlmostEqual(result["observed_current_pl_m"]["vpl_max"],
                               86.6898998)
        self.assertGreater(result["future_margin_m"]["minimum"], 8.5)
        self.assertEqual(result["frame_authority"], "map")
        self.assertEqual(result["stamp_authority"],
                         "IntegrityReport.header.stamp from monitor report stamp")
        self.assertTrue(result["requires_forbidden_contract_change_to_pass"])

    def test_compatibility_diagnosis_refuses_to_overwrite_evidence(self):
        module = load_compatibility()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "diagnosis.json"
            module._write_new(output, {"first": True})
            with self.assertRaises(FileExistsError):
                module._write_new(output, {"second": True})
            self.assertEqual(json.loads(output.read_text()), {"first": True})

    def test_runner_uses_v2_development_scene_not_layer1_trigger(self):
        source = RUNNER_PATH.read_text()
        self.assertIn('"scenario:=icra_p0_p4_v2_p5_dev_fixture_v1"', source)
        self.assertNotIn('"scenario:=icra072_p4_selection_trigger_v1"', source)
        self.assertIn('"--descriptor-sha256", row["descriptor_sha256"]', source)
        self.assertIn("def _capture_source_binding(matrix_root: Path)", source)
        self.assertNotIn("BASE._capture_source_binding()", source)
        self.assertIn("inventory = CORE.known_retained_inventory()", source)
        self.assertIn('first.get("known_retained_inventory") ==', source)
        self.assertIn('payload.get("schema_version") == "icra075_capture_readiness_v1"',
                      source)
        self.assertNotIn("BASE._wait_ready(capture", source)
        self.assertNotIn('batch["ros_started"] = True', source)
        self.assertIn('batch["ros_started"] = _batch_ros_started(root, matrix)', source)
        self.assertIn("first_missing = _row_first_missing", source)
        manager_source = (REPO / "src/iap/planner/plan_manage/src/planner_manager.cpp").read_text()
        lineage_function = manager_source.split(
            "bool EGOPlannerManager::recordP4VerticalSliceLineage", 1)[1].split(
                "bool EGOPlannerManager::", 1)[0]
        self.assertNotIn("config.objective !=", lineage_function)

    def test_protocol_materializes_exact_non_held_out_matrix(self):
        module = load_module()
        protocol = module.load_protocol()
        rows = module.build_matrix(protocol)
        self.assertEqual(len(rows), 40)
        self.assertEqual({row["seed"] for row in rows}, set(range(75001, 75006)))
        formal = [row for row in rows if row["comparison_kind"] == "FORMAL_ARM"]
        exploratory = [row for row in rows if row["comparison_kind"] == "EXPLORATORY_ABLATION"]
        self.assertEqual(len(formal), 30)
        self.assertEqual(len(exploratory), 10)
        self.assertEqual({row["scene"] for row in formal},
                         {"PRIMARY", "EXACT_MIRROR", "FLAT_NULL"})
        self.assertEqual({row["configuration"] for row in formal},
                         {"P0_P5_CONTROL", "P0_P4_V2_P5_TREATMENT"})
        self.assertEqual({row["scene"] for row in exploratory}, {"PRIMARY"})
        self.assertTrue(all(row["held_out_eligible"] is False for row in rows))
        self.assertEqual(len({row["run_id"] for row in rows}), 40)

    def test_scene_assets_bind_exact_v2_hashes_without_forbidden_decision_inputs(self):
        module = load_module()
        binding = module.validate_scene_assets()
        self.assertEqual(binding["result"], "PASS")
        self.assertEqual(binding["descriptor_sha256"], {
            "PRIMARY": "41ab7001c7c60e78cdcf8a670efdcba7545bcd7c93e06081745a090b57f0f06d",
            "EXACT_MIRROR": "3bd25208dba3bbd03857104758878e30d647b7f8bfdfc43e7654f78b569caf33",
            "FLAT_NULL": "89074561a7263268e121c2439ceccaca2b3c7285f665432ebdc07ab5668e8513",
        })
        self.assertEqual(binding["decision_plane_inputs"],
                         ["immutable_p0_snapshot", "ordinary_occupancy"])
        self.assertEqual(binding["forbidden_decision_fields_present"], [])
        self.assertFalse(binding["runtime_asset_contains_route_or_oracle_truth"])
        self.assertFalse(binding["gnss_mask_enabled"]["FLAT_NULL"])
        self.assertTrue(all(item["accepted"] for item in module.known_retained_inventory()))

    @staticmethod
    def final_record():
        return {
            "trajectory_id": 17,
            "start_time_ns": 123456789,
            "final_bspline_identity": "bspline-17-123456789",
            "publication_identity": "publish-17-123456789",
            "order": 1,
            "position_control_points": [[-12.0, 3.5, 1.5], [12.0, 3.5, 1.5]],
            "knots": [0.0, 0.0, 1.0, 1.0],
            "collision_free": True,
            "dynamics_feasible": True,
            "p4_search_latency_ms": 25.0,
        }

    def test_analyzer_uses_exactly_200_equal_arc_committed_final_samples(self):
        module = load_module()
        descriptor = module.load_v2_descriptor("PRIMARY")
        result = module.analyze_committed_final(
            descriptor, self.final_record(),
            {"action": "OK", "min_al_minus_pl_m": 0.8},
            {"action": "OK", "trajectory_id": 17,
             "trajectory_start_time_ns": 123456789})
        self.assertEqual(result["sample_count"], 200)
        self.assertEqual(len(result["samples"]), 200)
        self.assertEqual(result["samples"][0]["position_m"], [-12.0, 3.5, 1.5])
        self.assertEqual(result["samples"][-1]["position_m"], [12.0, 3.5, 1.5])
        self.assertEqual(result["trajectory_source"], "committed_final_bspline_only")
        self.assertEqual(result["trajectory_id"], 17)
        self.assertEqual(result["p5_final_action"], "OK")
        self.assertEqual(result["p5_runtime_action"], "OK")

    def test_curved_equal_arc_sampling_has_a_disclosed_error_bound(self):
        module = load_module()
        final = self.final_record()
        final.update({
            "order": 3,
            "position_control_points": [
                [-12.0, 0.0, 1.5], [-6.0, 7.0, 1.5],
                [6.0, -7.0, 1.5], [12.0, 0.0, 1.5],
            ],
            "knots": [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        })
        samples, evidence = module.equal_arc_samples(final, 200, return_evidence=True)
        lengths = [math.dist(a, b) for a, b in zip(samples, samples[1:])]
        self.assertEqual(len(samples), 200)
        self.assertLess(max(lengths) - min(lengths), 2.0e-4)
        self.assertLessEqual(evidence["arc_position_error_bound_m"], 2.0e-4)

    def test_oracle_values_are_independent_of_p4_output(self):
        module = load_module()
        descriptor = module.load_v2_descriptor("PRIMARY")
        first = self.final_record()
        second = copy.deepcopy(first)
        first["p4_decision_output"] = {"selected": "safe", "risk": 0.0}
        second["p4_decision_output"] = {"selected": "risky", "risk": 999.0}
        p5_final = {"action": "OK", "min_al_minus_pl_m": 0.8}
        p5_runtime = {"action": "OK", "trajectory_id": 17,
                      "trajectory_start_time_ns": 123456789}
        a = module.analyze_committed_final(descriptor, first, p5_final, p5_runtime)
        b = module.analyze_committed_final(descriptor, second, p5_final, p5_runtime)
        for key in ("provider_interior_peak", "provider_interior_mean",
                    "whole_path_provider_peak", "safe_tube_fraction",
                    "risky_tube_fraction", "route_label"):
            self.assertEqual(a[key], b[key])

    def test_actual_row_analyzer_oracle_is_unchanged_when_p4_output_is_deleted(self):
        module = load_module()
        analyzer = load_analyzer()
        row = next(item for item in module.build_matrix(module.load_protocol())
                   if item["scene"] == "PRIMARY" and item["seed"] == 75001 and
                   item["configuration"] == "P0_P4_V2_P5_TREATMENT")
        binding = {"scene": row["scene"], "descriptor_sha256": row["descriptor_sha256"]}
        final = self.final_record()
        capture = [
            {**binding, "kind": "p0_health", "receive_steady_s": 1.0,
             "payload": {"ready": True, "stale": False, "generation_id": 9}},
            {**binding, "kind": "p5_status", "receive_steady_s": 2.0,
             "payload": {"phase": "final", "action": "OK",
                         "final_candidate_traj_id": 17,
                         "final_candidate_start_time_ns": 123456789,
                         "samples": [{"im_min": 0.8}]}},
            {**binding, "kind": "normal_bspline", "receive_steady_s": 3.0,
             "payload": final},
            {**binding, "kind": "p5_status", "receive_steady_s": 4.0,
             "payload": {"phase": "runtime", "action": "OK", "samples": [{
                 "trajectory_sample_source": "runtime_committed", "trajectory_id": 17,
                 "trajectory_start_time_ns": 123456789}]}},
        ]
        manifest = {
            "matrix_row": row, "result": "PASS", "owned_process_groups_cleared": True,
            "launch_contract": {"planner_enable_p4": True, "p4.metrics_only": False,
                                "manager/max_vel": 30.0, "manager/max_acc": 30.0,
                                "manager/feasibility_tolerance": 0.05},
        }
        results_root = REPO / "results/icra27/icra075"
        results_root.mkdir(parents=True, exist_ok=True)
        observed = []
        for include_p4 in (False, True):
            with tempfile.TemporaryDirectory(dir=results_root) as temporary:
                root = Path(temporary)
                (root / "exports").mkdir()
                (root / "run_manifest.json").write_text(json.dumps(manifest))
                (root / "lineage_capture.jsonl").write_text(
                    "\n".join(json.dumps(item) for item in capture) + "\n")
                if include_p4:
                    (root / "exports/planner_p4_risk_astar_debug.csv").write_text(
                        "risk_search_latency_ms,selection_applied,snapshot_generation_id\n25,1,9\n")
                    lineage_header = (
                        "schema_version,stage,selection_applied,trajectory_id,"
                        "trajectory_start_ns,final_bspline_identity,snapshot_generation_id\n")
                    lineage_rows = "".join(
                        f"p4_v2_end_to_end_lineage_v2,{stage},1,17,123456789,prod-id,9\n"
                        for stage in ("final_bspline_before_p5",
                                      "p5_final_pass_before_publish",
                                      "normal_publish_authorized"))
                    (root / "exports/planner_p4_risk_astar_debug.csv.lineage.csv").write_text(
                        lineage_header + lineage_rows)
                result, code = analyzer.analyze_row(root)
                self.assertEqual(code, 0 if include_p4 else 1)
                observed.append(result)
        for key in ("provider_interior_peak", "provider_interior_mean",
                    "whole_path_provider_peak", "route_label", "sample_count"):
            self.assertEqual(observed[0][key], observed[1][key])

    def test_pair_identity_ablation_isolation_and_missing_stage_are_fail_closed(self):
        module = load_module()
        protocol = module.load_protocol()
        rows = module.build_matrix(protocol)
        control = next(row for row in rows if row["scene"] == "PRIMARY" and
                       row["seed"] == 75001 and
                       row["configuration"] == "P0_P5_CONTROL")
        treatment = next(row for row in rows if row["scene"] == "PRIMARY" and
                         row["seed"] == 75001 and
                         row["configuration"] == "P0_P4_V2_P5_TREATMENT")
        self.assertTrue(module.pair_identity_matches(control, treatment))
        changed = copy.deepcopy(treatment)
        changed["goal_m"] = [11.0, 0.0, 1.5]
        self.assertFalse(module.pair_identity_matches(control, changed))
        ablations = [row for row in rows if row["comparison_kind"] == "EXPLORATORY_ABLATION"]
        self.assertTrue(module.ablation_isolation_passes(ablations, protocol))
        contaminated = copy.deepcopy(ablations)
        contaminated[0]["provider_truth_override"] = 2.0
        self.assertFalse(module.ablation_isolation_passes(contaminated, protocol))
        self.assertEqual(module.first_missing_stage({
            "p0_snapshot": True, "p4_selection": True,
            "ego_final": False, "p5_final": False,
            "publication": False, "p5_runtime": False,
        }), "EGO_FINAL_MISSING")

    def test_power_inputs_are_deterministic_and_explicitly_non_freezing(self):
        module = load_module()
        rows = []
        for scene_index, scene in enumerate(("PRIMARY", "EXACT_MIRROR", "FLAT_NULL")):
            for seed_index, seed in enumerate(range(75001, 75006)):
                baseline = 4.0 + 0.1 * scene_index + 0.01 * seed_index
                improvement = 1.0 if scene != "FLAT_NULL" else 0.0
                common = {"scene": scene, "seed": seed, "result": "PASS",
                          "path_length_m": 25.0, "p4_search_latency_ms": 20.0,
                          "minimum_al_minus_pl_m": 0.5, "collision_free": True,
                          "dynamics_feasible": True}
                rows.append({**common, "configuration": "P0_P5_CONTROL",
                             "provider_interior_peak": baseline,
                             "provider_interior_mean": baseline - 0.2})
                rows.append({**common, "configuration": "P0_P4_V2_P5_TREATMENT",
                             "provider_interior_peak": baseline - improvement,
                             "provider_interior_mean": baseline - 0.2 - improvement})
        first = module.compute_power_inputs(rows)
        second = module.compute_power_inputs(copy.deepcopy(rows))
        self.assertEqual(first, second)
        self.assertEqual(first["schema_version"], "icra075_exploratory_power_inputs_v1")
        self.assertFalse(first["freezes_sesoi"])
        self.assertFalse(first["freezes_sample_size"])
        self.assertEqual(first["candidate_confirmatory_sample_size_range"], [30, 60])
        self.assertEqual(first["formal_pair_count"], 15)
        self.assertIn("sensitivity", first)
        for summary in first["scene_summaries"].values():
            self.assertIn("D_peak_zero_mass", summary)
            self.assertIn("repeatability", summary)
            self.assertEqual(summary["intra_run_correlation"]["status"],
                             "UNAVAILABLE_SINGLE_TERMINAL_EVENT_PER_RUN")
            self.assertIn("matched_arm_cross_seed_correlation", summary)
            self.assertEqual(summary["candidate_n_range"], [30, 60])

    def test_safety_is_computed_and_incomplete_inputs_fail_closed(self):
        module = load_module()
        final = self.final_record()
        final["position_control_points"] = [[-12.0, 0.0, 1.5], [12.0, 0.0, 1.5]]
        safety = module.evaluate_committed_final_safety(
            final, module.load_map_asset(), max_velocity_mps=30.0,
            max_acceleration_mps2=30.0, feasibility_tolerance=0.05)
        self.assertFalse(safety["collision_free"])
        self.assertEqual(safety["collision_source"], "independent_v2_geometry_evaluator")
        with self.assertRaises(ValueError):
            module.validate_complete_analysis({
                "minimum_al_minus_pl_m": None, "collision_free": True,
                "dynamics_feasible": True, "p4_search_latency_ms": 0.0,
            })

    def test_power_rejects_incomplete_safety_rows(self):
        module = load_module()
        with self.assertRaises(ValueError):
            module.compute_power_inputs([{
                "scene": "PRIMARY", "seed": 75001, "result": "PASS",
                "configuration": "P0_P5_CONTROL",
                "minimum_al_minus_pl_m": None,
            }])


if __name__ == "__main__":
    unittest.main()
