import importlib.util
import json
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock


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
    @staticmethod
    def _nvidia_smi_runner(returncodes=(0, 0)):
        calls = []

        def run(command, **kwargs):
            index = len(calls)
            calls.append((command, kwargs))
            stdout = (
                "GPU 0: Test GPU (UUID: GPU-test)\n"
                if "-L" in command
                else "0, Test GPU, GPU-test, 555.55\n"
            )
            return subprocess.CompletedProcess(
                command, returncodes[index], stdout=stdout, stderr=""
            )

        return run, calls

    def test_gpu_preflight_missing_command_fails_closed(self):
        def missing(command, **kwargs):
            raise FileNotFoundError("nvidia-smi")

        with tempfile.TemporaryDirectory() as directory:
            result = MODULE.run_gpu_preflight(
                Path(directory), command_runner=missing,
                cuda_probe=lambda: {"cuInit_result": 0,
                                    "cuDeviceGetCount_result": 0,
                                    "device_count": 1},
            )
            persisted = json.loads(
                (Path(directory) / "gpu_preflight.json").read_text()
            )
        self.assertFalse(result["gpu_ready"])
        self.assertEqual(result, persisted)
        self.assertIn("nvidia_smi_missing", result["failure_reason"])

    def test_gpu_preflight_nonzero_nvml_fails_closed(self):
        runner, _ = self._nvidia_smi_runner((1, 0))
        with tempfile.TemporaryDirectory() as directory:
            result = MODULE.run_gpu_preflight(
                Path(directory), command_runner=runner,
                cuda_probe=lambda: {"cuInit_result": 0,
                                    "cuDeviceGetCount_result": 0,
                                    "device_count": 1},
            )
        self.assertFalse(result["gpu_ready"])
        self.assertIn("nvidia_smi_list_exit_1", result["failure_reason"])

    def test_gpu_preflight_zero_cuda_devices_fails_closed(self):
        runner, _ = self._nvidia_smi_runner()
        with tempfile.TemporaryDirectory() as directory:
            result = MODULE.run_gpu_preflight(
                Path(directory), command_runner=runner,
                cuda_probe=lambda: {"cuInit_result": 0,
                                    "cuDeviceGetCount_result": 0,
                                    "device_count": 0},
            )
        self.assertFalse(result["gpu_ready"])
        self.assertEqual(result["failure_reason"], "cuda_device_count_zero")

    def test_gpu_preflight_cuda_initialization_failure_fails_closed(self):
        runner, _ = self._nvidia_smi_runner()
        with tempfile.TemporaryDirectory() as directory:
            result = MODULE.run_gpu_preflight(
                Path(directory), command_runner=runner,
                cuda_probe=lambda: {"cuInit_result": 999,
                                    "cuDeviceGetCount_result": None,
                                    "device_count": None},
            )
        self.assertFalse(result["gpu_ready"])
        self.assertEqual(result["failure_reason"], "cuInit_failed_999")

    def test_gpu_preflight_pass_records_bounded_commands_and_cuda(self):
        runner, calls = self._nvidia_smi_runner()
        with tempfile.TemporaryDirectory() as directory:
            result = MODULE.run_gpu_preflight(
                Path(directory), command_runner=runner,
                cuda_probe=lambda: {"cuInit_result": 0,
                                    "cuDeviceGetCount_result": 0,
                                    "device_count": 1},
            )
        self.assertTrue(result["gpu_ready"])
        self.assertEqual(result["failure_reason"], "")
        self.assertEqual(len(result["commands"]), 2)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result["cuda"]["device_count"], 1)

    def test_failed_preflight_never_calls_ros_launch_path(self):
        failed = {
            "schema_version": "iap_gpu_preflight_v1",
            "gpu_ready": False,
            "failure_reason": "cuInit_failed_999",
        }
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            MODULE, "run_gpu_preflight", return_value=failed
        ), mock.patch.object(MODULE, "run_gate0_smoke") as smoke, mock.patch.object(
            sys, "argv", [str(MODULE_PATH), "--output-root", directory, "--smoke"]
        ):
            self.assertNotEqual(MODULE.main(), 0)
        smoke.assert_not_called()

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

    def test_capture_readiness_is_required_before_launch(self):
        class Capture:
            def poll(self):
                return None

        with tempfile.TemporaryDirectory() as directory:
            ready_path = Path(directory) / "capture_ready.json"
            ready_path.write_text(json.dumps({
                "schema_version": "gate0_capture_readiness_v1",
                "ready": True,
                "subscriptions": [
                    {"topic": "/planning/risk_grid_health"},
                    {"topic": "/iap/integrity"},
                ],
            }))
            readiness = MODULE._wait_for_capture_ready(
                Capture(), ready_path, timeout_s=0.1
            )
        self.assertTrue(readiness["ready"])

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
