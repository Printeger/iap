import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts/dev_planner/analyze_continuous_flight.py"
SPEC = importlib.util.spec_from_file_location("analyze_continuous_flight", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ContinuousFlightAnalysisTest(unittest.TestCase):
    def test_correlates_curve_identity_and_keeps_missing_controller_delay_unknown(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            capture = [
                {"kind": "normal_bspline", "receive_steady_s": 10.0,
                 "payload": {"execution_instance_id": 7, "trajectory_id": 25,
                             "start_time_ns": 1_000_000_000,
                             "curve_hash": "nominal-25",
                             "control_points_xyz": [[0, 0, 0], [1, 0, 0]],
                             "knots": [0, 0, 1, 1]}},
                {"kind": "trajectory_status", "receive_steady_s": 10.01,
                 "payload": {"state": "ACTIVATED", "execution_instance_id": 7,
                             "trajectory_id": 25, "start_time_ns": 1_000_000_000,
                             "curve_hash": "nominal-25", "event_time_ns": 1_000_000_000}},
                {"kind": "poscmd", "receive_steady_s": 10.02,
                 "payload": {"execution_instance_id": 7, "trajectory_id": 25,
                             "start_time_ns": 1_000_000_000,
                             "curve_hash": "nominal-25", "stamp_s": 1.0,
                             "position_xyz": [0, 0, 0], "velocity_xyz": [1, 0, 0],
                             "acceleration_xyz": [0, 0, 0]}},
                {"kind": "iap_odom", "receive_steady_s": 10.03,
                 "payload": {"stamp_s": 1.0, "position_xyz": [0, 0.1, 0],
                             "velocity_xyz": [0.8, 0, 0]}},
                {"kind": "pending_guard_bspline", "receive_steady_s": 10.8,
                 "payload": {"execution_instance_id": 7, "trajectory_id": 26,
                             "start_time_ns": 2_000_000_000,
                             "curve_hash": "guard-26",
                             "control_points_xyz": [[0.8, 0, 0], [1.0, 0, 0]],
                             "knots": [0, 0, 1, 1]}},
                {"kind": "trajectory_status", "receive_steady_s": 10.9,
                 "payload": {"state": "ACTIVATED", "execution_instance_id": 7,
                             "trajectory_id": 26, "start_time_ns": 2_000_000_000,
                             "curve_hash": "guard-26", "event_time_ns": 2_000_000_000}},
                {"kind": "normal_bspline", "receive_steady_s": 11.0,
                 "payload": {"execution_instance_id": 7, "trajectory_id": 26,
                             "start_time_ns": 2_000_000_000,
                             "curve_hash": "emergency-26",
                             "control_points_xyz": [[0.8, 0, 0], [0.8, 0, 0]],
                             "knots": [0, 0, 1, 1]}},
            ]
            (run / "capture.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in capture), encoding="utf-8")
            exports = run / "exports"
            exports.mkdir()
            with (exports / "planner_p4_risk_astar_debug.csv.forward_candidates.csv").open(
                    "w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "decision_event_id", "candidate_id", "channel_id", "selected",
                    "path_hash", "reason"])
                writer.writeheader()
                writer.writerow({"decision_event_id": 3, "candidate_id": 1,
                                 "channel_id": 10, "selected": 0,
                                 "path_hash": "left", "reason": "ready"})
                writer.writerow({"decision_event_id": 3, "candidate_id": 2,
                                 "channel_id": 11, "selected": 1,
                                 "path_hash": "right", "reason": "ready"})

            report = MODULE.analyze_run(run)

            self.assertEqual(report["identity_conflict_count"], 1)
            self.assertEqual(report["baseline_reproduction"]["trajectory_sequence"], [25, 26])
            self.assertEqual(report["controller_delay_status"], "uncertain")
            self.assertAlmostEqual(report["tracking_error"]["cross_track_max_m"], 0.1)
            self.assertEqual(len(report["channel_decisions"]), 1)

    def test_controller_receipt_closes_delay_chain_and_cli_writes_fixed_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            rows = [
                {"kind": "normal_bspline", "receive_steady_s": 20.0,
                 "payload": {"execution_instance_id": 8, "trajectory_id": 1,
                             "start_time_ns": 3_000_000_000, "curve_hash": "curve"}},
                {"kind": "controller_trace", "receive_steady_s": 20.04,
                 "payload": {"execution_instance_id": 8, "trajectory_id": 1,
                             "start_time_ns": 3_000_000_000, "curve_hash": "curve",
                             "receive_steady_ns": 20_040_000_000}},
            ]
            (run / "capture.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")

            exit_code = MODULE.main(["--run", str(run)])

            self.assertEqual(exit_code, 0)
            self.assertTrue((run / "continuous_flight_timeline.csv").exists())
            self.assertTrue((run / "channel_comparison.csv").exists())
            report = json.loads((run / "continuous_flight_report.json").read_text())
            self.assertEqual(report["controller_delay_status"], "measured")
            self.assertAlmostEqual(report["controller_delay_ms"]["max"], 40.0)


if __name__ == "__main__":
    unittest.main()
