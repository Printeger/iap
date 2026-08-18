import importlib.util
import subprocess
import sys
import time
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "run_gate0_qualification.py"
)
SPEC = importlib.util.spec_from_file_location("run_gate0_qualification", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _wait_until(predicate, timeout_s: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


class Gate0RunnerTest(unittest.TestCase):
    def test_gate0a_matrix_is_fixed_to_three_scenarios_three_repeats(self):
        matrix = MODULE.gate0a_matrix()
        self.assertEqual(len(matrix), 9)
        self.assertEqual({row[0] for row in matrix}, set(MODULE.SCENARIOS))
        for label in MODULE.SCENARIOS:
            self.assertEqual(
                [row[2] for row in matrix if row[0] == label], [1, 2, 3]
            )

    def test_gate0a_effective_config_is_isolated_and_manifest_complete(self):
        config = MODULE.gate0a_effective_config(
            "mirror-r1", "p1_fork_fused_mirror_v1", Path("/tmp/gate0")
        )
        self.assertEqual(config["forest_random_seed"], 11)
        self.assertEqual(config["gnss_random_seed"], 20260011)
        self.assertEqual(config["terminal_wall_feature_seed"], 11022)
        self.assertFalse(config["manager/p1_collision_fanout_mirror_y"])
        for field in (
            "planner_enable_p1",
            "planner_enable_p2",
            "planner_enable_p3_local",
            "planner_enable_p3_global",
            "planner_enable_p4",
            "planner_enable_p5_runtime",
            "planner_enable_p5_final",
            "p1.use_integrity_cost",
            "p2.enable_candidate_ranking",
            "p3.enable_local_reference_bias",
            "p3.enable_global_reference_bias",
            "p4.enable_risk_aware_astar",
        ):
            self.assertIs(config[field], False, field)
        self.assertTrue(config["gate0.qualification_evidence_enable"])
        self.assertFalse(config["record_bag"])
        self.assertTrue(config["run_validator"])

    def test_p0_config_has_exact_query_shape(self):
        config = MODULE.p0_effective_config(Path("/tmp/p0"))
        self.assertEqual(config["iap_mapping_backend"], "cpu")
        self.assertEqual(config["p0.size_x_m"], 30.0)
        self.assertEqual(config["p0.size_y_m"], 30.0)
        self.assertEqual(config["p0.size_z_m"], 6.0)
        self.assertEqual(config["p0.resolution_m"], 0.75)
        self.assertEqual(config["p0.horizons_s"].split(","), [
            "0.0", "0.5", "1.0", "1.5", "2.0", "2.5"
        ])
        self.assertEqual(config["p0.predictor.worker_count"], 1)

    def test_runner_exit_requires_launch_capture_finalize_and_process_ok(self):
        self.assertEqual(
            MODULE._gate0b_runner_exit(0, 0, 0, {"required_processes_ok": True}),
            0,
        )
        self.assertEqual(
            MODULE._gate0b_runner_exit(1, 0, 0, {"required_processes_ok": True}),
            2,
        )
        self.assertEqual(
            MODULE._gate0b_runner_exit(0, 1, 0, {"required_processes_ok": True}),
            2,
        )
        self.assertEqual(
            MODULE._gate0b_runner_exit(0, 0, 1, {"required_processes_ok": True}),
            2,
        )
        self.assertEqual(
            MODULE._gate0b_runner_exit(0, 0, 0, {"required_processes_ok": False}),
            2,
        )

    def test_monitor_never_started_required_process(self):
        monitor = MODULE.RequiredProcessMonitor(
            999999, {"iap_rosnode": ["iap_rosnode"]}, 1.0
        )
        monitor.start()
        monitor.launch_running = True
        time.sleep(0.15)
        result = monitor.finish()
        self.assertFalse(result["required_processes_ok"])
        self.assertFalse(
            result["required_processes"]["iap_rosnode"]["seen"]
        )
        self.assertTrue(any(
            item.get("phase") == "launch"
            for item in result["process_failures"]
        ))

    def test_monitor_ignores_unrelated_same_name_process(self):
        unrelated = subprocess.Popen([
            sys.executable,
            "-c",
            "import time; time.sleep(30) # iap_rosnode",
        ])
        parent = subprocess.Popen([
            sys.executable,
            "-c",
            "import time; time.sleep(30)",
        ])
        try:
            monitor = MODULE.RequiredProcessMonitor(
                parent.pid, {"iap_rosnode": ["iap_rosnode"]}, 1.0
            )
            monitor.start()
            monitor.launch_running = True
            time.sleep(0.3)
            result = monitor.finish()
            self.assertFalse(
                result["required_processes"]["iap_rosnode"]["seen"]
            )
        finally:
            unrelated.terminate()
            parent.terminate()
            unrelated.wait(timeout=5)
            parent.wait(timeout=5)

    def test_monitor_records_runtime_child_death(self):
        parent = subprocess.Popen([
            sys.executable,
            "-c",
            (
                "import subprocess,sys,time\n"
                "p=subprocess.Popen([sys.executable,'-c',"
                "'import time; time.sleep(0.4) # iap_rosnode'])\n"
                "time.sleep(30)\n"
            ),
        ])
        try:
            monitor = MODULE.RequiredProcessMonitor(
                parent.pid, {"iap_rosnode": ["iap_rosnode"]}, 10.0
            )
            monitor.start()
            monitor.launch_running = True
            _wait_until(lambda: monitor.result()["required_processes"][
                "iap_rosnode"
            ]["seen"], 2.0)
            _wait_until(lambda: any(
                item.get("phase") == "runtime"
                for item in monitor.result()["process_failures"]
            ), 3.0)
            result = monitor.finish()
            self.assertFalse(result["required_processes_ok"])
            self.assertTrue(any(
                item.get("phase") == "runtime"
                for item in result["process_failures"]
            ))
        finally:
            parent.terminate()
            parent.wait(timeout=5)
            if parent.stdout:
                parent.stdout.close()

    def test_monitor_classifies_controlled_shutdown_separately(self):
        parent = subprocess.Popen(
            [
                sys.executable,
                "-c",
                (
                    "import subprocess,sys,time\n"
                    "p=subprocess.Popen([sys.executable,'-c',"
                    "'import time; time.sleep(30) # iap_rosnode'])\n"
                    "print(p.pid, flush=True)\n"
                    "time.sleep(30)\n"
                ),
            ],
            stdout=subprocess.PIPE,
            text=True,
        )
        try:
            child_pid_line = parent.stdout.readline().strip()
            self.assertTrue(child_pid_line.isdigit())
            child_pid = int(child_pid_line)
            monitor = MODULE.RequiredProcessMonitor(
                parent.pid, {"iap_rosnode": ["iap_rosnode"]}, 10.0
            )
            monitor.start()
            monitor.launch_running = True
            _wait_until(lambda: monitor.result()["required_processes"][
                "iap_rosnode"
            ]["seen"], 2.0)
            monitor.mark_controlled_shutdown()
            child = MODULE.psutil.Process(child_pid)
            child.terminate()
            _wait_until(lambda: not child.is_running(), 5.0)
            _wait_until(lambda: any(
                item.get("phase") == "controlled_shutdown"
                for item in monitor.result()["process_failures"]
            ), 3.0)
            result = monitor.finish()
            self.assertTrue(result["required_processes_ok"])
            self.assertTrue(any(
                item.get("phase") == "controlled_shutdown"
                for item in result["process_failures"]
            ))
        finally:
            parent.terminate()
            parent.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
