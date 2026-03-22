import os
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_compute_mode(mode: str) -> str:
    normalized = mode.strip().lower()
    if normalized in ("cpu", "gpu"):
        return normalized

    if normalized != "auto":
        raise RuntimeError("Invalid compute_mode. Use one of: auto, gpu, cpu")

    result = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, timeout=2, check=False)
    if result.returncode == 0 and "GPU" in result.stdout:
        return "gpu"

    return "cpu"


def _prepare_runtime_config(context):
    source_config_path = LaunchConfiguration("config_path").perform(context)
    requested_mode = LaunchConfiguration("compute_mode").perform(context)
    resolved_mode = _resolve_compute_mode(requested_mode)

    runtime_config_path = f"/tmp/iap_launch_config_{resolved_mode}"
    shutil.rmtree(runtime_config_path, ignore_errors=True)
    shutil.copytree(
        source_config_path,
        runtime_config_path,
        symlinks=True,
        ignore_dangling_symlinks=True,
    )

    config_json_path = os.path.join(runtime_config_path, "config.json")
    with open(config_json_path, "r", encoding="utf-8") as f:
        config_text = f.read()

    if resolved_mode == "cpu":
        config_text = config_text.replace("config_odometry_gpu.json", "config_odometry_cpu.json")
        config_text = config_text.replace("config_sub_mapping_gpu.json", "config_sub_mapping_cpu.json")
        config_text = config_text.replace("config_global_mapping_gpu.json", "config_global_mapping_cpu.json")
    else:
        config_text = config_text.replace("config_odometry_cpu.json", "config_odometry_gpu.json")
        config_text = config_text.replace("config_sub_mapping_cpu.json", "config_sub_mapping_gpu.json")
        config_text = config_text.replace("config_global_mapping_cpu.json", "config_global_mapping_gpu.json")

    with open(config_json_path, "w", encoding="utf-8") as f:
        f.write(config_text)

    return [
        SetLaunchConfiguration("resolved_config_path", runtime_config_path),
        LogInfo(msg=f"[iap_rosnode.launch] compute_mode={requested_mode} -> resolved={resolved_mode}"),
        LogInfo(msg=f"[iap_rosnode.launch] runtime config path: {runtime_config_path}"),
    ]


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    mode = LaunchConfiguration("mode")
    resolved_config_path = LaunchConfiguration("resolved_config_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([FindPackageShare("iap"), "config"]),
            description="Path to IAP config directory",
        ),
        DeclareLaunchArgument(
            "bag_path",
            default_value="/home/dev/code/ws_iap/src/iap/data/realsense_ros2",
            description="Path to rosbag2 directory",
        ),
        DeclareLaunchArgument(
            "bag_rate",
            default_value="1.0",
            description="Playback rate passed to ros2 bag play --rate",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="bag",
            description="Run mode: 'bag' to play rosbag, 'realtime' for live sensor input",
        ),
        DeclareLaunchArgument(
            "compute_mode",
            default_value="gpu",
            description="Compute mode for mapping modules: auto, gpu, or cpu",
        ),
        OpaqueFunction(function=_prepare_runtime_config),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="iap_rosnode",
            output="screen",
            parameters=[{"config_path": resolved_config_path}],
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag_path, "--rate", bag_rate],
            output="screen",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'bag'"])),
        ),
    ])
