#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = (Path(__file__).parents[1] / "scripts" / "dev_planner" /
          "analyze_control_capability.py")
SPEC = importlib.util.spec_from_file_location("control_capability", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ControlCapabilityAnalysisTest(unittest.TestCase):
    def test_selects_fastest_configuration_passing_every_partition(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            captures = {}
            for speed in MODULE.SPEEDS:
                path = root / f"trace-{speed}.jsonl"
                saturated = speed == 2.0
                rows = []
                for index, trajectory_id in enumerate((1, 1, 2, 2)):
                    position = 0.01 * index
                    rows.append({
                        "event": "controller_trace",
                        "payload": {
                            "execution_instance_id": 1,
                            "trajectory_id": trajectory_id,
                            "start_time_ns": trajectory_id * 100,
                            "curve_hash": f"curve-{trajectory_id}",
                            "position_xyz": [position, 0.0, 1.0],
                            "feedback_position_xyz": [
                                position + 0.05, 0.0, 1.0],
                            "velocity_xyz": [speed, 0.0, 0.0],
                            "feedback_velocity_xyz": [speed - 0.1, 0.0, 0.0],
                            "acceleration_xyz": [0.0, 0.0, 0.0],
                            "saturated": saturated,
                        }})
                path.write_text("".join(json.dumps(row) + "\n" for row in rows),
                                encoding="utf-8")
                captures[speed] = path.name
            cases = []
            for speed in MODULE.SPEEDS:
                for acceleration in MODULE.ACCELERATIONS:
                    for jerk in MODULE.JERKS:
                        for maneuver in MODULE.MANEUVERS:
                            for partition in MODULE.PARTITIONS:
                                cases.append({
                                    "capture_path": captures[speed],
                                    "speed_mps": speed,
                                    "acceleration_mps2": acceleration,
                                    "jerk_mps3": jerk,
                                    "maneuver": maneuver,
                                    "partition": partition,
                                })
            manifest = {
                "measured_latency_bound_s": 0.12,
                "controller_identity": "controller-fixture",
                "simulator_identity": "simulator-fixture",
                "code_version": "commit-fixture",
                "cases": cases,
            }
            result = MODULE.qualify(manifest, root)
            self.assertEqual(result["maximum_velocity_mps"], [1.5] * 3)
            self.assertEqual(result["maximum_acceleration_mps2"], [3.0] * 3)
            self.assertEqual(result["maximum_jerk_mps3"], [4.0] * 3)
            self.assertLessEqual(
                max(result["position_tracking_bound_m"]) * 1.2, 0.15)


if __name__ == "__main__":
    unittest.main()
