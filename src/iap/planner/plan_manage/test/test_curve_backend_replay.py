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
                self.assertEqual(process.returncode, 0 if expected else 1, process.stderr + process.stdout)
                result = json.loads((run / "export/planner/curve_replay/result.json").read_text())
                self.assertEqual(result["dynamics_feasible"], expected)
                self.assertEqual(result["original_time_s"], 100.1)
                self.assertEqual(result["original_cloud_stamp_s"], 100.)
                self.assertEqual(result["physical_executable"], expected)
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
