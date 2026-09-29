"""Private stable full-stack simulation graph.

This boundary exposes none of the validation launch's paper/fixture argument
surface. It initializes the maintained runtime with fixed canonical defaults,
forces test processes off, and calls only its graph builder. Runtime helpers
remain shared temporarily so historical experiment behavior does not fork.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable


def _runtime_module():
    path = Path(__file__).resolve().parent / "full_stack_runtime.py"
    spec = importlib.util.spec_from_file_location("iap_full_stack_sim_runtime", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load full-stack simulation runtime: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context):
    runtime = _runtime_module()
    supplied = dict(context.launch_configurations)
    for name, default in runtime.ARG_DEFAULTS:
        # Reset the entire legacy surface. Undeclared raw CLI values must not
        # leak through the shared runtime into canonical safety settings.
        context.launch_configurations[name] = str(default)

    for required in (
        "scenario",
        "runtime_root_dir",
        "export_root_dir",
        "iap_log_root",
        "bag_output_dir",
    ):
        if not str(supplied.get(required, "")).strip():
            raise RuntimeError(f"canonical simulation internal argument is empty: {required}")

    # These values are an internal contract, never caller-selectable switches.
    context.launch_configurations.update(
        {
            "experiment": "canonical_full_stack_sim",
            "scenario": str(supplied["scenario"]),
            "start_rviz": str(supplied.get("start_rviz", "true")),
            "record_bag": "false",
            "run_validator": "false",
            "runtime_root_dir": str(supplied["runtime_root_dir"]),
            "export_root_dir": str(supplied["export_root_dir"]),
            "iap_log_root": str(supplied["iap_log_root"]),
            "bag_output_dir": str(supplied["bag_output_dir"]),
            "run_duration_s": str(supplied.get("run_duration_s", "0.0")),
            "planner_start_delay_s": str(
                supplied.get("planner_start_delay_s", "10.0")
            ),
        }
    )
    return runtime._launch_setup(context)


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    return LaunchDescription(
        [
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                os.path.join(
                    iap_share, "config", "sim_ego", "fastdds_udp_only.xml"
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
