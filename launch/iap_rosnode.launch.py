import os
import shutil
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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

    try:
        result = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, timeout=2, check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return "cpu"

    if result.returncode == 0 and "GPU" in result.stdout:
        return "gpu"

    return "cpu"


def _resolve_bag_path(bag_path: str) -> str:
    requested = bag_path.strip()
    if requested:
        return os.path.abspath(os.path.expanduser(requested))

    candidates = []

    # Prefer workspace source bag path: <ws>/src/iap/data/realsense_ros2
    candidates.append(os.path.join(os.getcwd(), "src", "iap", "data", "realsense_ros2"))

    try:
        share_dir = get_package_share_directory("iap")
        candidates.append(os.path.join(share_dir, "data", "realsense_ros2"))
    except PackageNotFoundError:
        pass

    launch_dir_pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidates.append(os.path.join(launch_dir_pkg_root, "data", "realsense_ros2"))

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    # Fall back to the first candidate even if it does not exist so ros2 bag
    # reports a concrete path in error output.
    return candidates[0] if candidates else ""


def _prepare_runtime_config(context):
    source_config_path = LaunchConfiguration("config_path").perform(context)
    run_mode = LaunchConfiguration("mode").perform(context).strip().lower()
    requested_bag_path = LaunchConfiguration("bag_path").perform(context)
    resolved_bag_path = _resolve_bag_path(requested_bag_path)
    requested_mode = LaunchConfiguration("compute_mode").perform(context)
    resolved_mode = _resolve_compute_mode(requested_mode)

    if run_mode not in ("bag", "realtime"):
        raise RuntimeError("Invalid mode. Use one of: bag, realtime")

    if run_mode == "bag" and (not resolved_bag_path or not os.path.isdir(resolved_bag_path)):
        raise RuntimeError(
            "[iap_rosnode.launch] mode=bag but rosbag2 path does not exist: "
            f"{resolved_bag_path}. Set bag_path:=<rosbag2_directory>."
        )

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

    actions = [
        SetLaunchConfiguration("resolved_config_path", runtime_config_path),
        SetLaunchConfiguration("resolved_bag_path", resolved_bag_path),
        LogInfo(msg=f"[iap_rosnode.launch] compute_mode={requested_mode} -> resolved={resolved_mode}"),
        LogInfo(msg=f"[iap_rosnode.launch] runtime config path: {runtime_config_path}"),
        LogInfo(msg=f"[iap_rosnode.launch] bag_path={requested_bag_path or '<auto>'} -> resolved={resolved_bag_path}"),
    ]

    if run_mode == "bag":
        actions.append(LogInfo(msg=f"[iap_rosnode.launch] auto-play enabled: {resolved_bag_path}"))

    return actions


def generate_launch_description():
    bag_rate = LaunchConfiguration("bag_rate")
    mode = LaunchConfiguration("mode")
    resolved_config_path = LaunchConfiguration("resolved_config_path")
    resolved_bag_path = LaunchConfiguration("resolved_bag_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([FindPackageShare("iap"), "config"]),
            description="Path to IAP config directory",
        ),
        DeclareLaunchArgument(
            "bag_path",
            default_value="",
            description="Path to rosbag2 directory. Empty means auto-resolve from package path.",
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
            cmd=["ros2", "bag", "play", resolved_bag_path, "--rate", bag_rate],
            output="screen",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'bag'"])),
        ),
    ])
