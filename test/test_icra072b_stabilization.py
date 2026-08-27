import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock


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

    def test_isolated_suite_environment_uses_exact_command_local_git_trust(self):
        module = self._load_runner()
        environment_root = (
            REPO / "results/icra27/icra072b" /
            f"trust_contract_{os.getpid()}")
        self.assertFalse(environment_root.exists())
        configuration_files = [
            REPO / ".git/config",
            REPO / ".git/config.worktree",
        ]
        before_hashes = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in configuration_files if path.is_file()}

        with mock.patch.dict(os.environ, {
                "GIT_CONFIG_COUNT": "2",
                "GIT_CONFIG_KEY_0": "safe.directory",
                "GIT_CONFIG_VALUE_0": "*",
                "GIT_CONFIG_KEY_1": "user.name",
                "GIT_CONFIG_VALUE_1": "ambient-user",
                "GIT_CONFIG_GLOBAL": "/root/.gitconfig",
                "GIT_CONFIG_SYSTEM": "/etc/gitconfig",
                "GIT_CONFIG_PARAMETERS": "'safe.directory'='*'",
                "GTEST_ALSO_RUN_DISABLED_TESTS": "1",
        }):
            environment, trust = module.build_suite_environment(
                environment_root)

        self.assertEqual(trust, {
            "schema_version": "icra072b_command_local_git_trust_v1",
            "mechanism": "git_config_environment",
            "safe_directory": str(REPO),
            "config_count": 1,
            "system_config_disabled": True,
            "accepted": True,
        })
        self.assertEqual(environment["HOME"],
                         str(environment_root / "home"))
        self.assertEqual(environment["GIT_CONFIG_COUNT"], "1")
        self.assertEqual(environment["GIT_CONFIG_KEY_0"], "safe.directory")
        self.assertEqual(environment["GIT_CONFIG_VALUE_0"], str(REPO))
        self.assertEqual(environment["GIT_CONFIG_NOSYSTEM"], "1")
        self.assertEqual(environment["XDG_CONFIG_HOME"],
                         str(environment_root / "xdg_config"))
        self.assertNotIn("GIT_CONFIG_KEY_1", environment)
        self.assertNotIn("GIT_CONFIG_VALUE_1", environment)
        self.assertNotIn("GIT_CONFIG_GLOBAL", environment)
        self.assertNotIn("GIT_CONFIG_SYSTEM", environment)
        self.assertNotIn("GIT_CONFIG_PARAMETERS", environment)
        self.assertNotIn("GTEST_ALSO_RUN_DISABLED_TESTS", environment)
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, env=environment,
            capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0,
                         completed.stdout + completed.stderr)
        origins = subprocess.run(
            ["git", "config", "--show-origin", "--get-all",
             "safe.directory"], cwd=REPO, env=environment,
            capture_output=True, text=True, check=False)
        self.assertEqual(origins.returncode, 0,
                         origins.stdout + origins.stderr)
        self.assertEqual(len(origins.stdout.splitlines()), 1)
        self.assertTrue(origins.stdout.rstrip().endswith("\t" + str(REPO)))
        self.assertNotIn("*", origins.stdout)
        source_binding = module._source_binding(environment, trust)
        self.assertTrue(all(
            check["exit_code"] == 0 for check in source_binding["checks"]))
        self.assertEqual(source_binding["command_local_git_trust"], trust)
        self.assertNotIn("dubious ownership", "".join(
            check["stderr"] for check in source_binding["checks"]))
        self.assertFalse(environment_root.exists())
        self.assertEqual({
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in configuration_files if path.is_file()
        }, before_hashes)

        missing = dict(environment)
        for key in ("GIT_CONFIG_COUNT", "GIT_CONFIG_KEY_0",
                    "GIT_CONFIG_VALUE_0"):
            missing.pop(key)
        missing_completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, env=missing,
            capture_output=True, text=True, check=False)
        self.assertNotEqual(missing_completed.returncode, 0)
        self.assertFalse(
            module.command_local_git_trust(missing)["accepted"])
        wrong = dict(environment)
        wrong["GIT_CONFIG_VALUE_0"] = str(REPO.parent)
        wrong_completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, env=wrong,
            capture_output=True, text=True, check=False)
        self.assertNotEqual(wrong_completed.returncode, 0)
        self.assertFalse(module.command_local_git_trust(wrong)["accepted"])

    def test_summary_fails_closed_when_a_named_assertion_is_not_observed(self):
        module = self._load_runner()
        suites = [{
            "name": name,
            "argv": [name],
            "exit_code": 0,
            "test_count": 1,
            "expected_test_count": 1,
            "skipped_test_count": 0,
            "skipped_assertions": [],
            "disabled_test_count": 0,
            "disabled_assertions": [],
            "command_local_git_trust": {
                "schema_version": "icra072b_command_local_git_trust_v1",
                "mechanism": "git_config_environment",
                "safe_directory": str(REPO),
                "config_count": 1,
                "system_config_disabled": True,
                "accepted": True,
            },
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

        missing_trust = [dict(suite) for suite in suites]
        missing_trust[0].pop("command_local_git_trust")
        self.assertEqual(module.build_summary(
            "a" * 40, missing_trust)["result"], "FAIL")
        wrong_trust = [dict(suite) for suite in suites]
        wrong_trust[0]["command_local_git_trust"] = {
            **wrong_trust[0]["command_local_git_trust"],
            "safe_directory": str(REPO.parent),
            "accepted": False,
        }
        self.assertEqual(module.build_summary(
            "a" * 40, wrong_trust)["result"], "FAIL")

        skipped = [dict(suite) for suite in suites]
        skipped[0]["skipped_test_count"] = 1
        skipped[0]["skipped_assertions"] = [
            skipped[0]["observed_assertions"][0]]
        skipped_summary = module.build_summary("a" * 40, skipped)
        self.assertEqual(skipped_summary["result"], "FAIL")
        self.assertEqual(skipped_summary["suites"][0]["result"], "FAIL")
        self.assertIn("required_test_skipped",
                      skipped_summary["suites"][0]["failure_reasons"])
        self.assertTrue(any(
            row["result"] == "FAIL" and
            skipped[0]["name"] in row["required_suites"]
            for row in skipped_summary["matrix_rows"]))
        disabled = [dict(suite) for suite in suites]
        disabled[0]["disabled_test_count"] = 1
        disabled[0]["disabled_assertions"] = [
            disabled[0]["observed_assertions"][0]]
        disabled_summary = module.build_summary("a" * 40, disabled)
        self.assertEqual(disabled_summary["result"], "FAIL")
        self.assertEqual(disabled_summary["suites"][0]["result"], "FAIL")
        self.assertIn("required_test_disabled",
                      disabled_summary["suites"][0]["failure_reasons"])

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

    def test_observations_type_python_and_gtest_skips_and_disabled(self):
        module = self._load_runner()
        python_output = """\
test_required (__main__.ToolsTest.test_required) ... ok
test_skipped (__main__.ToolsTest.test_skipped) ... skipped 'not ready'

----------------------------------------------------------------------
Ran 2 tests in 0.001s

OK (skipped=1)
"""
        self.assertEqual(module._observations(python_output), {
            "test_count": 2,
            "observed_assertions": ["ToolsTest.test_required"],
            "skipped_test_count": 1,
            "skipped_assertions": ["ToolsTest.test_skipped"],
            "disabled_test_count": 0,
            "disabled_assertions": [],
        })

        gtest_output = """\
[==========] Running 2 tests from 1 test suite.
[ RUN      ] ProductTest.Required
[       OK ] ProductTest.Required (0 ms)
[ RUN      ] ProductTest.Skipped
[  SKIPPED ] ProductTest.Skipped (0 ms)
[==========] 2 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 1 test.
[  SKIPPED ] 1 test, listed below:
[  SKIPPED ] ProductTest.Skipped

  YOU HAVE 1 DISABLED TEST
"""
        self.assertEqual(module._observations(gtest_output), {
            "test_count": 2,
            "observed_assertions": ["ProductTest.Required"],
            "skipped_test_count": 1,
            "skipped_assertions": ["ProductTest.Skipped"],
            "disabled_test_count": 1,
            "disabled_assertions": [],
        })
        forced_disabled_output = """\
[==========] Running 1 test from 1 test suite.
[ RUN      ] ProductTest.DISABLED_Required
[       OK ] ProductTest.DISABLED_Required (0 ms)
[==========] 1 test from 1 test suite ran. (0 ms total)
[  PASSED  ] 1 test.
"""
        self.assertEqual(module._observations(forced_disabled_output), {
            "test_count": 1,
            "observed_assertions": [],
            "skipped_test_count": 0,
            "skipped_assertions": [],
            "disabled_test_count": 1,
            "disabled_assertions": ["ProductTest.DISABLED_Required"],
        })

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
