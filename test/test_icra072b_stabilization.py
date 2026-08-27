import json
import importlib.util
import os
import subprocess
import sys
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
RUNNER = REPO / "scripts/dev_planner/run_icra072b_stabilization.py"


class Icra072bStabilizationTest(unittest.TestCase):
    @staticmethod
    def _load_runner():
        spec = importlib.util.spec_from_file_location(
            "icra072b_stabilization", RUNNER)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    def test_cli_describes_each_required_matrix_row_exactly_once(self):
        completed = subprocess.run(
            [sys.executable, str(RUNNER), "--describe-matrix"],
            cwd=REPO, capture_output=True, text=True, check=False)

        self.assertEqual(
            completed.returncode, 0, completed.stdout + completed.stderr)
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["schema_version"],
                         "icra072b_stabilization_matrix_v1")
        names = [row["name"] for row in payload["rows"]]
        self.assertEqual(names, [
            "happy_path",
            "occupancy_epoch",
            "attempt_request_identity",
            "snapshot_guide_lineage",
            "final_trajectory_identity",
            "p5_final_authority",
            "p5_runtime_authority",
            "operational_closure",
        ])
        self.assertEqual(len(names), len(set(names)))
        for row in payload["rows"]:
            self.assertTrue(row["assertions"])
            self.assertTrue(all(assertion["suite"] and assertion["name"]
                                for assertion in row["assertions"]))

    def test_summary_fails_closed_when_a_named_assertion_is_not_observed(self):
        module = self._load_runner()
        suites = [{
            "name": name,
            "argv": [name],
            "exit_code": 0,
            "test_count": 1,
            "expected_test_count": 1,
            "observed_assertions": sorted(
                assertion["name"]
                for row in module.MATRIX_ROWS
                for assertion in row["assertions"]
                if assertion["suite"] == name),
            "log_path": f"logs/{name}.log",
        } for name in module.REQUIRED_SUITES]

        passed = module.build_summary("a" * 40, suites)
        self.assertEqual(passed["result"], "PASS")
        self.assertEqual([row["result"] for row in passed["matrix_rows"]],
                         ["PASS"] * 8)

        declaration = module.MATRIX_ROWS[0]["assertions"][0]
        missing = declaration["name"]
        expected_suite = next(
            suite for suite in suites
            if suite["name"] == declaration["suite"])
        expected_suite["observed_assertions"].remove(missing)
        failed = module.build_summary("a" * 40, suites)
        self.assertEqual(failed["result"], "FAIL")
        failed_rows = [row for row in failed["matrix_rows"]
                       if row["result"] == "FAIL"]
        self.assertTrue(failed_rows)
        self.assertIn(missing, {
            item for row in failed_rows
            for item in row["missing_assertions"]})

        wrong_suite = next(
            suite for suite in suites
            if suite["name"] != declaration["suite"])
        wrong_suite["observed_assertions"].append(missing)
        misattributed = module.build_summary("a" * 40, suites)
        self.assertEqual(misattributed["result"], "FAIL")
        self.assertIn(missing, misattributed["matrix_rows"][0][
            "missing_assertions"])

        expected_suite["observed_assertions"].append(missing)
        missing_count = [dict(suite) for suite in suites]
        missing_count[0].pop("expected_test_count")
        self.assertEqual(module.build_summary(
            "a" * 40, missing_count)["result"], "FAIL")

        original_rows = module.MATRIX_ROWS
        try:
            module.MATRIX_ROWS = original_rows[:-1] + (original_rows[0],)
            duplicate = module.build_summary("a" * 40, suites)
        finally:
            module.MATRIX_ROWS = original_rows
        self.assertEqual(duplicate["result"], "FAIL")
        self.assertEqual(duplicate["required_row_cardinality"]["happy_path"],
                         2)
        self.assertEqual(
            duplicate["required_row_cardinality"]["operational_closure"], 0)

    def test_canonical_outputs_are_new_and_repository_local(self):
        module = self._load_runner()
        suffix = str(os.getpid())
        output = REPO / f"results/icra27/icra072b/contract_{suffix}.json"
        logs = REPO / f"results/icra27/icra072b/contract_logs_{suffix}"
        self.assertFalse(output.exists())
        self.assertFalse(logs.exists())
        resolved_output, resolved_logs = module.validate_output_paths(
            output, logs)
        self.assertEqual(resolved_output, output)
        self.assertEqual(resolved_logs, logs)

        for bad_output, bad_logs in (
                (Path("/tmp/icra072b.json"), logs),
                (output, Path("/tmp/icra072b_logs")),
                (REPO / "results/icra27/icra072/final.json", logs)):
            with self.subTest(output=bad_output, logs=bad_logs), \
                    self.assertRaisesRegex(SystemExit, "icra072b"):
                module.validate_output_paths(bad_output, bad_logs)


if __name__ == "__main__":
    unittest.main()
