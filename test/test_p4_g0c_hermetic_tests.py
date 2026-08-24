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

from run_p4_g0c_tests import ENVIRONMENT_PATHS  # noqa: E402


class P4G0CHermeticTests(unittest.TestCase):
    @staticmethod
    def _external_log_inventory():
        root = Path("/root/.ros/log")
        return {
            str(path.relative_to(root))
            for path in root.rglob("*")
        }

    @staticmethod
    def _run_bootstrap(task_root, suite_root):
        return subprocess.run(
            [
                sys.executable,
                str(BOOTSTRAP),
                "--task-root",
                str(task_root),
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
                        "HERMETIC_TEST_ENVIRONMENT_NOT_READY", completed.stderr
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
            self.assertIn("HERMETIC_TEST_ENVIRONMENT_READY", completed.stdout)
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
