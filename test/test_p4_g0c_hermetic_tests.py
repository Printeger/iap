import ast
import json
import os
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO / "scripts" / "dev_planner"
BOOTSTRAP = SCRIPT_DIR / "run_p4_g0c_tests.py"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_p4_g0c_tests import (  # noqa: E402
    ENVIRONMENT_PATHS,
    compare_inventories,
    external_log_inventory,
)


class P4G0CHermeticTests(unittest.TestCase):
    @staticmethod
    def _external_log_inventory():
        return external_log_inventory()

    @staticmethod
    def _run_bootstrap(task_root, suite_root):
        return subprocess.run(
            [
                sys.executable,
                str(BOOTSTRAP),
                "--task-root",
                str(task_root),
                "unittest",
                "--",
                "discover",
                "-s",
                str(suite_root),
                "-p",
                "no_tests_here.py",
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

    def test_guard_rejects_missing_environment_before_launch_import(self):
        environment = dict(os.environ)
        for name in (*ENVIRONMENT_PATHS, "P4_G0C_HERMETIC_TEST_ROOT"):
            environment.pop(name, None)
        probe = (
            "import sys; "
            f"sys.path.insert(0, {str(SCRIPT_DIR)!r}); "
            "from run_p4_g0c_tests import require_hermetic_test_environment; "
            "require_hermetic_test_environment(); "
            "import launch; "
            "print('UNSAFE_LAUNCH_IMPORT')"
        )
        completed = subprocess.run(
            [sys.executable, "-c", probe],
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertNotIn("UNSAFE_LAUNCH_IMPORT", completed.stdout)

    def test_guard_rejects_each_external_binding_before_launch_import(self):
        before = self._external_log_inventory()
        probe = (
            "import sys; "
            f"sys.path.insert(0, {str(SCRIPT_DIR)!r}); "
            "from run_p4_g0c_tests import require_hermetic_test_environment; "
            "require_hermetic_test_environment(); "
            "import launch; "
            "print('UNSAFE_LAUNCH_IMPORT')"
        )
        for name in ENVIRONMENT_PATHS:
            with self.subTest(name=name):
                environment = dict(os.environ)
                environment[name] = "/root/.ros/log"
                completed = subprocess.run(
                    [sys.executable, "-c", probe],
                    env=environment,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertNotEqual(completed.returncode, 0)
                self.assertNotIn("UNSAFE_LAUNCH_IMPORT", completed.stdout)
        self.assertEqual(self._external_log_inventory(), before)

    def test_bootstrap_rejects_unsafe_roots_without_external_log_delta(self):
        hermetic_root = Path(os.environ["P4_G0C_HERMETIC_TEST_ROOT"])
        before = self._external_log_inventory()
        with tempfile.TemporaryDirectory() as tmp:
            parent = Path(tmp)
            wrong_type = parent / "not-a-directory"
            wrong_type.write_text("not a directory\n")
            symlink = parent / "escape-link"
            symlink.symlink_to(Path("/root/.ros/log"), target_is_directory=True)
            cases = {
                "outside": "/root/.ros/log",
                "lexical_parent": f"{parent}/child/../child",
                "lexical_alias": str(hermetic_root).replace(
                    "/results/", "//results/", 1
                ),
                "wrong_type": wrong_type,
                "symlink_escape": symlink,
            }
            for case, root in cases.items():
                with self.subTest(case=case):
                    completed = self._run_bootstrap(root, parent)
                    self.assertEqual(completed.returncode, 2)
                    self.assertIn(
                        "HERMETIC_VERIFICATION_NOT_READY", completed.stderr
                    )
        self.assertEqual(self._external_log_inventory(), before)

    def test_bootstrap_rejects_existing_unsafe_xdg_mode_without_chmod(self):
        before = self._external_log_inventory()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "unsafe-xdg-root"
            root.mkdir()
            for relative in ENVIRONMENT_PATHS.values():
                (root / relative).mkdir()
            xdg = root / ENVIRONMENT_PATHS["XDG_RUNTIME_DIR"]
            xdg.chmod(0o755)
            completed = self._run_bootstrap(root, Path(tmp))
            self.assertEqual(completed.returncode, 2)
            self.assertIn("XDG_RUNTIME_DIR:unsafe_mode", completed.stderr)
            self.assertEqual(stat.S_IMODE(xdg.stat().st_mode), 0o755)
        self.assertEqual(self._external_log_inventory(), before)

    def test_inventory_comparator_detects_every_required_delta_class(self):
        regular = {
            "type": "regular", "mode": 0o644, "uid": 0, "gid": 0,
            "size": 3, "mtime_ns": 10, "ctime_ns": 11,
            "symlink_target": None, "sha256": "aaa",
        }
        symlink = {
            "type": "symlink", "mode": 0o777, "uid": 0, "gid": 0,
            "size": 6, "mtime_ns": 10, "ctime_ns": 11,
            "symlink_target": "target", "sha256": None,
        }
        cases = {
            "added": ({"kept": regular}, {"kept": regular, "new": regular}),
            "removed": ({"kept": regular, "old": regular}, {"kept": regular}),
            "metadata": (
                {"kept": regular},
                {"kept": {**regular, "mode": 0o600}},
            ),
            "symlink_target": (
                {"link": symlink},
                {"link": {**symlink, "symlink_target": "other"}},
            ),
            "same_name_content": (
                {"kept": regular},
                {"kept": {**regular, "sha256": "bbb"}},
            ),
        }
        for name, (before, after) in cases.items():
            with self.subTest(name=name):
                delta = compare_inventories(before, after)
                self.assertEqual(len(delta), 1)
                self.assertNotEqual(delta[0]["change"], "unchanged")
        self.assertEqual(compare_inventories({"kept": regular}, {"kept": regular}), [])

    def test_every_launch_import_has_the_hermetic_guard_first(self):
        for path in sorted((REPO / "test").glob("*.py")):
            tree = ast.parse(path.read_text(), filename=str(path))
            launch_import_lines = [
                node.lineno for node in ast.walk(tree)
                if (
                    isinstance(node, ast.Import)
                    and any(alias.name == "launch" for alias in node.names)
                ) or (
                    isinstance(node, ast.ImportFrom)
                    and node.module is not None
                    and (node.module == "launch" or node.module.startswith("launch."))
                )
            ]
            if not launch_import_lines:
                continue
            guard_lines = [
                node.lineno for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "require_hermetic_test_environment"
            ]
            with self.subTest(path=path.name):
                self.assertTrue(guard_lines, "launch import lacks hermetic guard")
                self.assertLess(min(guard_lines), min(launch_import_lines))

    def test_child_failure_exit_and_external_result_are_distinct(self):
        before = self._external_log_inventory()
        with tempfile.TemporaryDirectory() as tmp:
            parent = Path(tmp)
            suite = parent / "failing-suite"
            suite.mkdir()
            (suite / "test_failure.py").write_text(
                "import unittest\n"
                "class ExpectedFailure(unittest.TestCase):\n"
                "    def test_fails(self):\n"
                "        self.fail('expected child failure')\n"
            )
            root = parent / "failure-root"
            completed = subprocess.run(
                [
                    sys.executable, str(BOOTSTRAP), "--task-root", str(root),
                    "unittest", "--", "discover", "-s", str(suite),
                    "-p", "test_failure.py",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 1)
            self.assertIn("EXTERNAL_ROS_LOG_UNCHANGED", completed.stdout)
            result_paths = sorted(
                (root / "external_inventory").glob("*-result.json")
            )
            self.assertEqual(len(result_paths), 1)
            result = json.loads(result_paths[0].read_text())
            self.assertEqual(result["child_exit"], 1)
            self.assertEqual(result["final_exit"], 1)
            self.assertEqual(result["external_delta"], [])
            self.assertEqual(result["result"], "CHILD_FAILED")
        self.assertEqual(self._external_log_inventory(), before)

    def test_bootstrap_contains_launch_context_artifacts_below_task_root(self):
        before = self._external_log_inventory()
        with tempfile.TemporaryDirectory() as tmp:
            parent = Path(tmp)
            suite = parent / "probe-suite"
            suite.mkdir()
            probe = suite / "test_launch_context_probe.py"
            probe.write_text(
                "import sys\n"
                "import unittest\n"
                f"sys.path.insert(0, {str(SCRIPT_DIR)!r})\n"
                "from run_p4_g0c_tests import "
                "require_hermetic_test_environment\n"
                "ROOT = require_hermetic_test_environment()\n"
                "from launch import LaunchContext\n"
                "class LaunchContextProbe(unittest.TestCase):\n"
                "    def test_construct_context(self):\n"
                "        self.assertIsNotNone(LaunchContext())\n"
            )
            root = parent / "bootstrap-root"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(BOOTSTRAP),
                    "--task-root",
                    str(root),
                    "unittest",
                    "--",
                    "discover",
                    "-s",
                    str(suite),
                    "-p",
                    "test_launch_context_probe.py",
                    "-v",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("HERMETIC_VERIFICATION_READY", completed.stdout)
            self.assertIn(str(root), completed.stdout)
            for name, relative in ENVIRONMENT_PATHS.items():
                path = root / relative
                self.assertTrue(path.is_dir())
                self.assertFalse(path.is_symlink())
                if name == "XDG_RUNTIME_DIR":
                    self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o700)
            for path in root.rglob("*"):
                path.resolve().relative_to(root.resolve())
        self.assertEqual(self._external_log_inventory(), before)


if __name__ == "__main__":
    unittest.main()
