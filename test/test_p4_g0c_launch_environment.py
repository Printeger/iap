import copy
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts" / "dev_planner" / "run_p4_g0c_calibration.py"
SPEC = importlib.util.spec_from_file_location(
    "run_p4_g0c_calibration_environment", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class P4G0CLaunchEnvironmentTest(unittest.TestCase):
    def setUp(self):
        self.bundle = MODULE.load_bundle(
            REPO / "config/icra27/p4_g0c_protocol_v3.json",
            REPO / "config/icra27/p4_threshold_registry_v3.json",
            REPO / "config/icra27/p4_g0c_live_fixture_v1.json",
            expected_protocol_schema=MODULE.HARDENED_PROTOCOL_SCHEMA,
        )
        self.dependency_patch = mock.patch.object(
            MODULE,
            "validate_runtime_dependencies",
            return_value={
                "schema_version": "p4_g0c_dependency_preflight_result_v3",
                "dependency_ready": True,
                "failure_reason": "",
            },
        )
        self.dependency_patch.start()

    def tearDown(self):
        self.dependency_patch.stop()

    def test_runner_derives_all_child_values_and_complete_output_inventory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            plan = MODULE.expand_run_plan(self.bundle.protocol, root.resolve())
            with mock.patch.dict(os.environ, {
                "HOME": "/outside/home",
                "ROS_HOME": "relative-ros-home",
                "ROS_LOG_DIR": "/outside/ros-logs",
                "TMPDIR": "../outside-tmp",
            }, clear=False):
                inventory = MODULE.derive_launch_environment_inventory(root, plan)
                child = MODULE.child_launch_environment(None, inventory)

            expected = MODULE.expected_launch_environment_binding(
                root, Path(plan[0]["run_dir"])
            )
            self.assertEqual(
                inventory["child_environment"], expected["child_environment"]
            )
            self.assertEqual(
                {key: child[key] for key in MODULE.LAUNCH_ENVIRONMENT_KEYS},
                expected["child_environment"],
            )
            self.assertEqual(
                set(inventory["run_outputs"][0]["mutable_output_paths"]),
                set(MODULE.MUTABLE_OUTPUT_KEYS),
            )
            self.assertIn(
                "bag_output_dir",
                inventory["run_outputs"][0]["mutable_output_paths"],
            )
            self.assertEqual(len(inventory["run_outputs"]), 15)

            for absent_key in MODULE.LAUNCH_ENVIRONMENT_KEYS:
                with self.subTest(absent_key=absent_key):
                    caller = {
                        key: f"/caller/{key.lower()}"
                        for key in MODULE.LAUNCH_ENVIRONMENT_KEYS
                        if key != absent_key
                    }
                    child = MODULE.child_launch_environment(caller, inventory)
                    self.assertEqual(
                        child[absent_key],
                        expected["child_environment"][absent_key],
                    )
                    self.assertEqual(
                        {
                            key: child[key]
                            for key in MODULE.LAUNCH_ENVIRONMENT_KEYS
                        },
                        expected["child_environment"],
                    )

    def test_malicious_and_unknown_paths_fail_before_gpu_launch_or_attempt(self):
        cases = (
            "missing_environment_key",
            "outside",
            "relative",
            "lexical_parent",
            "symlink",
            "conflicting_output",
            "unknown_output_key",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "runs"

                def malicious_factory(runs_root, plan):
                    inventory = MODULE.derive_launch_environment_inventory(
                        runs_root, plan
                    )
                    inventory = copy.deepcopy(inventory)
                    outputs = inventory["run_outputs"][0][
                        "mutable_output_paths"
                    ]
                    if case == "missing_environment_key":
                        del inventory["child_environment"]["HOME"]
                    elif case == "outside":
                        outputs["export_root_dir"] = str(
                            Path(tmp).resolve() / "outside"
                        )
                    elif case == "relative":
                        outputs["runtime_root_dir"] = "relative/runtime"
                    elif case == "lexical_parent":
                        outputs["iap_log_root"] = str(
                            runs_root / "p4-g0c-r3-seed211-rep01"
                            / "runtime" / ".." / "escape"
                        )
                    elif case == "symlink":
                        target = Path(tmp) / "outside-target"
                        target.mkdir()
                        alias = runs_root / "alias"
                        alias.symlink_to(target, target_is_directory=True)
                        outputs["export_root_dir"] = str(alias / "exports")
                    elif case == "conflicting_output":
                        conflict = Path(outputs["stdout_log_path"])
                        conflict.parent.mkdir(parents=True)
                        conflict.write_text("conflict\n")
                    elif case == "unknown_output_key":
                        outputs["unregistered_writable_output"] = str(
                            runs_root / "unregistered"
                        )
                    return inventory

                gpu_calls = []
                launch_calls = []
                result = MODULE.run(
                    self.bundle,
                    root,
                    gpu_preflight=lambda _: gpu_calls.append(True),
                    launch_executor=lambda *args: launch_calls.append(args),
                    launch_environment_factory=malicious_factory,
                )
                self.assertEqual(result["runner_state"], "FAILED")
                self.assertTrue(
                    result["failure_reason"].startswith(
                        "LAUNCH_ENVIRONMENT_NOT_READY"
                    )
                )
                self.assertEqual(result["gpu_preflight_invocations"], 0)
                self.assertEqual(result["launch_invocations"], 0)
                self.assertEqual(result["attempted_run_ids"], [])
                self.assertFalse(result["launch_started"])
                self.assertEqual(gpu_calls, [])
                self.assertEqual(launch_calls, [])
                first_run = root / "p4-g0c-r3-seed211-rep01"
                if case != "conflicting_output":
                    self.assertFalse(first_run.exists())
                persisted = json.loads(
                    (root / "p4_g0c_runner_state.json").read_text()
                )
                self.assertEqual(persisted["gpu_preflight_invocations"], 0)
                self.assertEqual(persisted["launch_invocations"], 0)
                self.assertEqual(persisted["attempted_run_ids"], [])

    def test_nominal_run_propagates_exact_environment_to_launch_child(self):
        observed = {}

        def launch_executor(record, command, duration, required, environment):
            observed.update({
                "record": record,
                "command": command,
                "duration": duration,
                "required": required,
                "environment": environment,
            })
            return 19, {"required_processes_ok": False}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            result = MODULE.run(
                self.bundle,
                root,
                dependency_environment={
                    "PATH": "/registered/bin",
                    "HOME": "/caller/home",
                    "ROS_HOME": "/caller/ros-home",
                    "ROS_LOG_DIR": "/caller/ros-log",
                    "TMPDIR": "/caller/tmp",
                },
                gpu_preflight=lambda _: {
                    "gpu_ready": True,
                    "failure_reason": "",
                },
                launch_executor=launch_executor,
            )
            expected = MODULE.expected_launch_environment_binding(
                root, Path(result["runs"][0]["run_dir"])
            )

        self.assertEqual(result["gpu_preflight_invocations"], 1)
        self.assertEqual(result["launch_invocations"], 1)
        self.assertEqual(len(result["attempted_run_ids"]), 1)
        self.assertTrue(result["launch_started"])
        self.assertEqual(
            {
                key: observed["environment"][key]
                for key in MODULE.LAUNCH_ENVIRONMENT_KEYS
            },
            expected["child_environment"],
        )
        self.assertEqual(observed["environment"]["PATH"], "/registered/bin")
        command_values = {
            item.split(":=", 1)[0]: item.split(":=", 1)[1]
            for item in observed["command"]
            if ":=" in item
        }
        self.assertEqual(
            command_values["bag_output_dir"],
            expected["mutable_output_paths"]["bag_output_dir"],
        )
        self.assertEqual(
            command_values["p4.g0c.child_ros_log_dir"],
            expected["child_environment"]["ROS_LOG_DIR"],
        )


if __name__ == "__main__":
    unittest.main()
