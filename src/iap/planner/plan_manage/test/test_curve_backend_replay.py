"""Captured dynamic symptom; free-map fixture is mechanism evidence only."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(REPO / "launch/_includes"))
from run_directory import resolve_run_directory


class CurveReplayTest(unittest.TestCase):
    def test_captured_guide_correction_and_late_failure_own_current_evidence(self):
        fixture=json.loads((Path(__file__).parent /
                            "fixtures/curve_attempt12_gen55_geometry.json").read_text())
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            data=json.loads(json.dumps(fixture))
            # Captured geometry and acquisition time; observed-free physics is
            # explicitly a mechanism fixture, never the original forest.
            data.update(kind="attempt_failure_curve",origin_m=[-6.,-6.,0.],
                        max_boundary_m=[6.,6.,6.],dimensions=[120,120,60],resolution_m=.1,
                        virtual_ceiling_height_m=-1.,inflation_radius_m=0.,frame_id="map",
                        cell_flags_file="cells.bin")
            for p in [data["real_start_p_m"],*data["guide_m"]]: p[0]+=16.
            for stage in data["curve_stages"]:
                stage["target_p_m"][0]+=16.
                for key in ("guide_m","control_points_m"):
                    for p in stage[key]: p[0]+=16.
            (root/"cells.bin").write_bytes(bytes([4])*(120*120*60))
            parameters=root/"parameters.yaml"
            parameters.write_text("""/**:
  ros__parameters:
    optimization/order: 3
    optimization/lambda_smooth: 1.0
    optimization/lambda_collision: 0.5
    optimization/lambda_feasibility: 0.1
    optimization/lambda_fitness: 1.0
    optimization/dist0: 0.5
    optimization/swarm_clearance: 0.5
    optimization/max_vel: 0.5
    optimization/max_acc: 2.0
    planning/advisory_guidance_enabled: false
""")
            snapshot=root/"snapshot.json"
            for scenario in ("normal","late_failure","quota_denied"):
                current=json.loads(json.dumps(data))
                if scenario=="quota_denied": current["curve_stages"][0]["repairs"]=3
                snapshot.write_text(json.dumps(current))
                previous=os.environ.get("IAP_RUN_ROOT")
                os.environ["IAP_RUN_ROOT"]=temporary
                try:
                    run=resolve_run_directory(entrypoint="curve_backend_replay")
                finally:
                    if previous is None: os.environ.pop("IAP_RUN_ROOT",None)
                    else: os.environ["IAP_RUN_ROOT"]=previous
                binary=os.environ["IAP_CURVE_REPLAY_FAILURE_BIN" if scenario=="late_failure"
                                  else "IAP_CURVE_REPLAY_BIN"]
                process=subprocess.run([binary,str(snapshot),"backend","--ros-args",
                                        "--params-file",str(parameters)],
                                       env={**os.environ,"IAP_RUN_DIR":str(run)},
                                       capture_output=True,text=True,timeout=15)
                self.assertEqual(process.returncode,1 if scenario=="late_failure" else 0,
                                 process.stderr+process.stdout)
                result=json.loads((run/"export/planner/curve_replay/result.json").read_text())
                self.assertEqual(result["original_time_s"],fixture["planning_time_s"])
                self.assertEqual(result["original_cloud_stamp_s"],fixture["cloud_stamp_s"])
                self.assertFalse(result["execution_authorized"])
                self.assertEqual(result["risk_evidence"],"NOT_AVAILABLE")
                stages=[stage["stage"] for stage in result["curve_stages"]]
                self.assertIn("dynamics_pass",stages)
                if scenario=="normal":
                    self.assertTrue(result["physical_geometric_candidate_valid"])
                    self.assertEqual(result["added_repairs"],1)
                    self.assertGreater(result["guide_max_deviation_m"],result["guide_corridor_m"])
                    self.assertFalse(result["guide_route_preserved"])
                elif scenario=="late_failure":
                    self.assertIn("correction_failed",stages)
                    self.assertFalse(result["physical_geometric_candidate_valid"])
                    self.assertTrue(result["dynamics_feasible"])
                    self.assertEqual(result["final_check_state"],"incomplete")
                    self.assertIn("budget_expired",result["solver_reason"])
                else:
                    self.assertTrue(result["dynamics_feasible"])
                    self.assertTrue(result["physical_executable"])
                    self.assertFalse(result["guide_route_preserved"])
                    self.assertTrue(result["physical_geometric_candidate_valid"])
                    self.assertEqual(result["added_repairs"],0)
                    self.assertEqual(result["termination"],"quality_correction_skipped")

    def test_real_route_loss_is_visible_before_final_check(self):
        fixture = json.loads((Path(__file__).parent /
                              "fixtures/curve_attempt45_gen209_geometry.json").read_text())
        # Geometry evidence only. These free cells never replace the captured
        # forest in a physical or execution replay.
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            parameters = root / "parameters.yaml"
            parameters.write_text("""/**:
  ros__parameters:
    optimization/order: 3
    optimization/lambda_smooth: 1.0
    optimization/lambda_collision: 0.5
    optimization/lambda_feasibility: 0.1
    optimization/lambda_fitness: 1.0
    optimization/dist0: 0.5
    optimization/swarm_clearance: 0.5
    optimization/max_vel: 0.5
    optimization/max_acc: 2.0
    manager/control_points_distance: 0.4
""")
            for offset in (0., 12.):
                data = json.loads(json.dumps(fixture))
                data.update(kind="attempt_failure_curve", origin_m=[-20.+offset, -6., 0.],
                            max_boundary_m=[10.+offset, 6., 6.], dimensions=[150, 60, 30],
                            resolution_m=.1, virtual_ceiling_height_m=-1., inflation_radius_m=0.,
                            frame_id="map", cloud_stamp_s=fixture["planning_time_s"]-.1,
                            motion_quality=1, motion_allow_bridged=False,
                            motion_stamp_s=fixture["planning_time_s"]-.1, motion_error_proxy_m=.01,
                            motion_body_radius_m=.35, motion_tracking_reserve_m=.1,
                            motion_budget_m=.55, motion_max_age_s=.5, environment_max_age_s=.5,
                            cell_flags_file="cells.bin")
                # The original corridor is 0.1366 m: keep its 0.1 m lattice.
                data.update(resolution_m=.1, dimensions=[300, 120, 60])
                (root / "cells.bin").write_bytes(bytes([4]) * (300 * 120 * 60))
                for p in data["guide_m"]:
                    p[0] += offset
                for stage in data["curve_stages"]:
                    stage["target_p_m"][0] += offset
                    for key in ("guide_m", "control_points_m"):
                        for p in stage.get(key, []):
                            p[0] += offset
                data["real_start_p_m"][0] += offset
                snapshot = root / "snapshot.json"
                snapshot.write_text(json.dumps(data))
                previous = os.environ.get("IAP_RUN_ROOT")
                os.environ["IAP_RUN_ROOT"] = temporary
                try:
                    run = resolve_run_directory(entrypoint="curve_backend_replay")
                finally:
                    if previous is None: os.environ.pop("IAP_RUN_ROOT", None)
                    else: os.environ["IAP_RUN_ROOT"] = previous
                process = subprocess.run([os.environ["IAP_CURVE_REPLAY_BIN"], str(snapshot), "audit",
                                          "--ros-args", "--params-file", str(parameters)],
                                         env={**os.environ, "IAP_RUN_DIR": str(run)},
                                         capture_output=True, text=True, timeout=15)
                self.assertEqual(process.returncode, 1, process.stderr + process.stdout)
                result = json.loads((run / "export/planner/curve_replay/result.json").read_text())
                self.assertTrue(result["all_stages_checked"])
                self.assertFalse(result["all_stages_preserve_route"])
                self.assertFalse(result["execution_authorized"])
                self.assertEqual(result["risk_evidence"], "NOT_AVAILABLE")
                self.assertEqual(result["original_time_s"], fixture["planning_time_s"])
                stages = result["curve_stages"]
                self.assertEqual([s["stage"] for s in stages], [s["stage"] for s in fixture["curve_stages"]])
                self.assertEqual(len(stages), 7)
                for stage, expected in zip(stages[:4], (.14971, .34638, .34638, .59006)):
                    self.assertTrue(stage["route_lost"])
                    self.assertAlmostEqual(stage["max_deviation_m"], expected, delta=.0001)
                self.assertLess(stages[0]["max_deviation_m"], stages[1]["max_deviation_m"])
                self.assertLess(stages[1]["max_deviation_m"], stages[3]["max_deviation_m"])
            base_parameters=parameters.read_text()
            fine_capture = None
            for scenario in ("same_input", "voxel_sampling", "voxel_sampling_missing_nominal",
                             "unsupported_sampling_model", "zero_without_policy", "unowned",
                             "explicit_stop", "guidance_without_predictor"):
                parameters.write_text(base_parameters + ("\n    planning/advisory_guidance_enabled: true\n"
                    if scenario=="guidance_without_predictor" else ""))
                current = json.loads(json.dumps(data))
                first = current["curve_stages"][0]
                if scenario.startswith("voxel_sampling") or scenario == "unsupported_sampling_model":
                    self.assertIsNotNone(fine_capture)
                    first.update(interval_s=fine_capture['interval_s'],control_points_m=fine_capture['control_points_m'],
                                 target_v_mps=fine_capture['target_v_mps'],ends_at_rest=True,nominal_interval_s=1.2000000000000002,
                                 guide_sampling_model='guide_arc_voxel_diagonal_v1')
                    if scenario == 'voxel_sampling_missing_nominal':del first['nominal_interval_s']
                    if scenario == 'unsupported_sampling_model':first['guide_sampling_model']='unsupported'
                if scenario in ("zero_without_policy", "explicit_stop"):
                    first["target_v_mps"] = [0., 0., 0.]
                if scenario == "explicit_stop":
                    first["ends_at_rest"] = True
                elif scenario == "unowned":
                    del first["guide_m"]
                snapshot.write_text(json.dumps(current))
                existing = set(root.iterdir())
                process = subprocess.run([sys.executable, str(REPO / "scripts/dev_planner/replay_curve_backend.py"),
                                          str(snapshot), "--mode", "initialize", "--parameters", str(parameters),
                                          "--binary", os.environ["IAP_CURVE_REPLAY_BIN"]],
                                         env={**os.environ, "IAP_RUN_ROOT": temporary},
                                         capture_output=True, text=True, timeout=15)
                run, = set(root.iterdir()) - existing
                rejected = scenario in ("zero_without_policy", "unowned", "guidance_without_predictor", "voxel_sampling_missing_nominal", "unsupported_sampling_model")
                self.assertEqual(process.returncode, 1 if rejected else 0, process.stderr + process.stdout)
                output = run / "export/planner/curve_replay/result.json"
                if rejected:
                    self.assertFalse(output.exists())
                    continue
                result = json.loads(output.read_text())
                self.assertTrue(result["physical_executable"])
                self.assertTrue(result["dynamics_feasible"])
                self.assertTrue(result["guide_route_preserved"])
                self.assertTrue(result["physical_geometric_candidate_valid"])
                self.assertFalse(result["execution_authorized"])
                self.assertEqual(result["original_time_s"], fixture["planning_time_s"])
                self.assertEqual(result["captured_target_velocity_mps"], first["target_v_mps"])
                self.assertEqual(result["curve_stages"][0]["stage"], "captured_initial")
                self.assertEqual(result["curve_stages"][0]["control_points_m"], first["control_points_m"])
                self.assertEqual(result["curve_stages"][1]["stage"], "guide_fit_replayed")
                if scenario == 'same_input':
                    fine_capture={**result['curve_stages'][1],'target_v_mps':result['replayed_target_velocity_mps']}
                    fine_capture['ends_at_rest']=True
                if scenario == 'voxel_sampling':
                    self.assertEqual(result['nominal_interval_source'],'EXPLICIT_STAGE_NOMINAL_INTERVAL')
                    self.assertEqual(result['nominal_interval_s'],first['nominal_interval_s'])
                    self.assertEqual(result['captured_guide_sampling_model'],first['guide_sampling_model'])
                    self.assertEqual(result['curve_stages'][1]['interval_s'],first['interval_s'])
                    self.assertEqual(result['curve_stages'][1]['control_points_m'],first['control_points_m'])
                self.assertTrue(result["terminal_stop"])
                self.assertEqual(result["replayed_target_velocity_mps"],[0.,0.,0.])
                self.assertEqual(result["added_repairs"], 0)
            del data["curve_stages"][1]["guide_m"]
            snapshot.write_text(json.dumps(data))
            existing = set(root.iterdir())
            process = subprocess.run([sys.executable, str(REPO / "scripts/dev_planner/replay_curve_backend.py"),
                                      str(snapshot), "--mode", "audit", "--parameters", str(parameters),
                                      "--binary", os.environ["IAP_CURVE_REPLAY_BIN"]],
                                     env={**os.environ, "IAP_RUN_ROOT": temporary},
                                     capture_output=True, text=True, timeout=15)
            self.assertEqual(process.returncode, 1, process.stderr + process.stdout)
            run, = set(root.iterdir()) - existing
            manifest = json.loads((run / "metadata/run_manifest.json").read_text())
            self.assertEqual(manifest["lifecycle"], "failed")
            self.assertFalse((run / "export/planner/curve_replay/result.json").exists())

    def test_captured_boundary_neighbourhood_requires_refine(self):
        fixture = json.loads((Path(__file__).parent / "fixtures/curve_attempt12_gen260.json").read_text())
        stage = fixture["captured_stage"]
        # Translation leaves every derivative and the failure invariant unchanged.
        def move(p):
            return [p[0] + 12., p[1], p[2]]
        data = {"kind": "attempt_failure_curve", "planning_attempt_id": fixture["planning_attempt_id"],
                "generation": fixture["generation"], "origin_m": [-6., -6., 0.],
                "max_boundary_m": [6., 6., 6.], "dimensions": [60, 60, 30], "resolution_m": .2,
                "virtual_ceiling_height_m": -1., "inflation_radius_m": 0.,
                "frame_id": "map", "cloud_stamp_s": 100., "planning_time_s": 100.1,
                "motion_quality": 1, "motion_allow_bridged": False, "motion_stamp_s": 100.,
                "motion_error_proxy_m": .01, "motion_body_radius_m": .35, "motion_tracking_reserve_m": .1,
                "motion_budget_m": .55, "motion_max_age_s": .5, "environment_max_age_s": .5,
                "cell_flags_file": "cells.bin", "guide_m": [move(p) for p in fixture["guide_m"]],
                "real_start_p_m": move(fixture["real_start_p_m"]),
                "real_start_v_mps": fixture["real_start_v_mps"], "real_start_a_mps2": fixture["real_start_a_mps2"],
                "curve_stages": [{**stage, "control_points_m": [move(p) for p in stage["control_points_m"]],
                                  "target_p_m": move(stage["target_p_m"])}]}
        parameters = """/**:
  ros__parameters:
    optimization/lambda_smooth: 1.0
    optimization/lambda_collision: 0.5
    optimization/lambda_feasibility: 0.1
    optimization/lambda_fitness: 1.0
    optimization/dist0: 0.5
    optimization/swarm_clearance: 0.5
    optimization/max_vel: 0.5
    optimization/max_acc: 2.0
    optimization/order: 3
"""
        with tempfile.TemporaryDirectory() as temporary:
            snapshot = Path(temporary) / "snapshot.json"
            snapshot.write_text(json.dumps(data))
            flags = bytearray([4]) * (60 * 60 * 30)
            flags[(59 * 60 + 59) * 30] = 5  # Far raw witness; never intersects the curve.
            (Path(temporary) / "cells.bin").write_bytes(flags)
            params = Path(temporary) / "parameters.yaml"
            params.write_text(parameters)
            for scenario, mode, expected, isolated in (("retime", "retime", False, True),
                    ("refine", "refine", True, True), ("obstacle_budget_denied", "refine", False, False),
                    ("backend_early_return", "backend", False, False), ("mismatched_guide", "refine", False, False)):
                current = json.loads(json.dumps(data))
                current_flags = bytearray(flags)
                if scenario == "obstacle_budget_denied":
                    q = current["curve_stages"][0]["control_points_m"]
                    p = [(q[7][axis] + 4 * q[8][axis] + q[9][axis]) / 6 for axis in range(3)]
                    index = [int((p[axis] - current["origin_m"][axis]) / .2) for axis in range(3)]
                    current_flags[(index[0] * 60 + index[1]) * 30 + index[2]] = 5
                    current["curve_stages"][0]["repairs"] = 3
                elif scenario == "backend_early_return":
                    current["curve_stages"][0]["stage"] = "guide_bound"
                    current["curve_stages"][0]["elapsed_s"] = 1.5
                elif scenario == "mismatched_guide":
                    current["guide_m"][-1][0] += 1.
                snapshot.write_text(json.dumps(current))
                (Path(temporary) / "cells.bin").write_bytes(current_flags)
                previous = os.environ.get("IAP_RUN_ROOT")
                os.environ["IAP_RUN_ROOT"] = temporary
                try:
                    run = resolve_run_directory(entrypoint="curve_backend_replay")
                finally:
                    if previous is None: os.environ.pop("IAP_RUN_ROOT", None)
                    else: os.environ["IAP_RUN_ROOT"] = previous
                process = subprocess.run([os.environ["IAP_CURVE_REPLAY_BIN"], str(snapshot), mode,
                                          *(["isolated-budget"] if isolated else []),
                                          "--ros-args", "--params-file", str(params)],
                                         env={**os.environ, "IAP_RUN_DIR": str(run)},
                                         capture_output=True, text=True, timeout=15)
                if scenario == "mismatched_guide":
                    self.assertEqual(process.returncode, 2, process.stderr + process.stdout)
                    self.assertIn("stage target/guide mismatch", process.stderr)
                    self.assertFalse((run / "export/planner/curve_replay/result.json").exists())
                    continue
                # A lost guide is preference quality; physical/dynamic checks
                # still own candidate validity, with no execution authorization.
                self.assertEqual(process.returncode, 0 if expected else 1, process.stderr + process.stdout)
                result = json.loads((run / "export/planner/curve_replay/result.json").read_text())
                self.assertEqual(result["dynamics_feasible"], expected)
                self.assertEqual(result["original_time_s"], 100.1)
                self.assertEqual(result["original_cloud_stamp_s"], 100.)
                self.assertEqual(result["physical_executable"], expected)
                self.assertEqual(result["physical_geometric_candidate_valid"],expected)
                self.assertFalse(result["execution_authorized"])
                if scenario=="refine":
                    self.assertFalse(result["guide_route_preserved"])
                if scenario == "obstacle_budget_denied":
                    self.assertGreater(result["constraint_samples"], 0)
                    self.assertEqual(result["termination"], "curve_correction_budget_denied")
                    self.assertEqual(result["added_repairs"], 0)
                elif scenario == "backend_early_return":
                    self.assertEqual(result["termination"], "backend_rejected")
                    self.assertEqual(result["final_check_state"], "not_checked")
                    self.assertIn("optimized_failed", [s["stage"] for s in result["curve_stages"]])
                if expected:
                    import numpy as np
                    curve = result["curve_stages"][-1]
                    q = np.array(curve["control_points_m"])
                    dt = curve["interval_s"]
                    for knots, p, v, a in ((q[:3], data["real_start_p_m"], data["real_start_v_mps"], data["real_start_a_mps2"]),
                                           (q[-3:], data["curve_stages"][0]["target_p_m"], stage["target_v_mps"], stage["target_a_mps2"])):
                        np.testing.assert_allclose((knots[0] + 4 * knots[1] + knots[2]) / 6, p, atol=1e-9)
                        np.testing.assert_allclose((knots[2] - knots[0]) / (2 * dt), v, atol=1e-9)
                        np.testing.assert_allclose((knots[0] - 2 * knots[1] + knots[2]) / dt**2, a, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
