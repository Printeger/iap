import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts/dev_planner/calibrate_local_surface_error.py"
SPEC = importlib.util.spec_from_file_location("local_surface_calibration", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class LocalSurfaceErrorCalibrationTest(unittest.TestCase):
    def test_uses_q999_floor_and_margin_then_requires_held_out_pass(self):
        calibration = [
            {"pose_errors_m": [0.01, 0.03], "surface_offsets_m": [0.02, 0.04]}
            for _ in range(3)
        ]
        held_out = {"pose_errors_m": [0.03], "surface_offsets_m": [0.045]}
        result = MODULE.calibrate(calibration, held_out)
        self.assertTrue(result["held_out_passed"])
        self.assertGreater(result["local_surface_error_bound_m"], 0.049)
        self.assertTrue(result["calibration_id"].startswith("local-surface-v1-"))

    def test_held_out_exceedance_fails_instead_of_lowering_bound(self):
        calibration = [
            {"pose_errors_m": [0.01], "surface_offsets_m": [0.02]}
            for _ in range(3)
        ]
        held_out = {"pose_errors_m": [0.08], "surface_offsets_m": [0.02]}
        result = MODULE.calibrate(calibration, held_out)
        self.assertFalse(result["held_out_passed"])
        self.assertEqual(result["status"], "FAIL_HELD_OUT_EXCEEDED_BOUND")

    def test_relative_pose_error_uses_error_change_not_absolute_global_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "truth.csv"
            path.write_text(
                "est_stamp,truth_x,truth_y,truth_z,est_x,est_y,est_z\n"
                "0.0,10,0,0,9,0,0\n"
                "0.1,10.1,0,0,9.08,0,0\n",
                encoding="utf-8",
            )
            errors = MODULE.relative_pose_errors(path, (0.1,))
            self.assertAlmostEqual(errors["0.1"][0], 0.02)

    def test_cli_rejects_reusing_a_run_as_held_out(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = [root / f"run-{index}" for index in range(3)]
            for run in runs:
                run.mkdir()
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--calibration-run", str(runs[0]),
                    "--calibration-run", str(runs[1]),
                    "--calibration-run", str(runs[2]),
                    "--held-out-run", str(runs[2]),
                    "--output", str(root / "result.json"),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("must be distinct directories", completed.stderr)

    def test_correlation_aligns_samples_by_stamp_not_row_order(self):
        lhs = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)]
        rhs = [(2.01, 30.0), (0.01, 10.0), (1.01, 20.0)]
        self.assertAlmostEqual(MODULE._aligned_pearson(lhs, rhs), 1.0)


if __name__ == "__main__":
    unittest.main()
