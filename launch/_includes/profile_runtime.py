"""Runtime configuration materialization for canonical IAP launch files."""

from __future__ import annotations

import json
import re
import warnings
from pathlib import Path
from typing import Any


INTEGRITY_PROFILES = {
    "fused": (True, True, "max_pl"),
    "gnss_only": (True, False, "gnss_only"),
    "lidar_only": (False, True, "lidar_only"),
    "fallback_only": (False, False, "fallback_only"),
}


def _load_json(path: Path) -> dict[str, Any]:
    source = path.read_text(encoding="utf-8")
    # GLIM configuration files conventionally allow C++ line comments.
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    source = re.sub(r"(^|\s)//.*$", r"\1", source, flags=re.MULTILINE)
    value = json.loads(source)
    if not isinstance(value, dict):
        raise RuntimeError(f"expected a JSON object: {path}")
    return value


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _resolve_reference(config_dir: Path, value: str, field: str) -> Path:
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = config_dir / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise RuntimeError(f"IAP profile {field} does not exist: {candidate}")
    return candidate


def _redirect_artifact_paths(
    value: Any, *, export_dir: Path, log_dir: Path, dump_dir: Path
) -> Any:
    """Redirect output-bearing fields without rewriting input resources."""

    if isinstance(value, dict):
        redirected: dict[str, Any] = {}
        for key, child in value.items():
            if isinstance(child, str) and key.endswith("_csv_path"):
                configured = Path(child)
                if configured.parent != Path("."):
                    warnings.warn(
                        f"legacy {key} directory is ignored; only its basename is used",
                        DeprecationWarning,
                        stacklevel=2,
                    )
                redirected[key] = str(
                    export_dir / (configured.name or f"{key}.csv")
                )
            elif isinstance(child, str) and key == "log_dir":
                if child and Path(child).parent != Path("."):
                    warnings.warn(
                        "legacy log_dir is ignored; runtime logs are run-scoped",
                        DeprecationWarning,
                        stacklevel=2,
                    )
                redirected[key] = str(log_dir)
            elif isinstance(child, str) and key == "dump_path":
                configured = Path(child)
                if configured.parent != Path("."):
                    warnings.warn(
                        "legacy dump_path directory is ignored; only its basename is used",
                        DeprecationWarning,
                        stacklevel=2,
                    )
                redirected[key] = str(
                    dump_dir.parent / (configured.name or dump_dir.name)
                )
            else:
                redirected[key] = _redirect_artifact_paths(
                    child,
                    export_dir=export_dir,
                    log_dir=log_dir,
                    dump_dir=dump_dir,
                )
        return redirected
    if isinstance(value, list):
        return [
            _redirect_artifact_paths(
                child,
                export_dir=export_dir,
                log_dir=log_dir,
                dump_dir=dump_dir,
            )
            for child in value
        ]
    return value


def materialize_profile(
    *,
    source_config_dir: str,
    output_dir: str,
    contract: str,
    integrity_profile: str = "fused",
    forbid_sim_extensions: bool = False,
    simulation_scenario: dict[str, Any] | None = None,
) -> tuple[str, dict[str, Any]]:
    """Create a run-local config and validate its module contract.

    The source profile remains immutable. All generated logs and CSVs are
    redirected below the explicitly supplied run directory.
    """

    source = Path(source_config_dir).expanduser().resolve()
    run_root = Path(output_dir).expanduser()
    if not run_root.is_absolute():
        raise RuntimeError("output_dir must be an absolute path")
    run_root = run_root.resolve()
    if run_root == Path("/"):
        raise RuntimeError("output_dir cannot be filesystem root")
    if not source.is_dir():
        raise RuntimeError(f"config_path is not a directory: {source}")

    runtime_dir = run_root / "metadata" / "config" / "iap"
    export_dir = run_root / "export" / "glio"
    integrity_export_dir = run_root / "export" / "current_integrity"
    simulation_export_dir = run_root / "export" / "simulation"
    log_dir = run_root
    dump_dir = export_dir / "dump"
    profiling_dir = run_root / "profiling"
    manifest_dir = run_root / "metadata" / "manifests"
    for directory in (
        runtime_dir,
        export_dir,
        integrity_export_dir,
        simulation_export_dir,
        dump_dir,
        profiling_dir,
        manifest_dir,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    root_path = source / "config.json"
    root_config = _load_json(root_path)
    global_config = root_config.setdefault("global", {})
    if not isinstance(global_config, dict):
        raise RuntimeError(f"global must be an object: {root_path}")

    ros_source = _resolve_reference(source, str(global_config["config_ros"]), "config_ros")
    gnss_source = _resolve_reference(source, str(global_config["config_gnss"]), "config_gnss")
    logging_source = _resolve_reference(
        source, str(global_config["config_logging"]), "config_logging"
    )
    ros_config = _redirect_artifact_paths(
        _load_json(ros_source), export_dir=export_dir, log_dir=log_dir, dump_dir=dump_dir
    )
    gnss_config = _redirect_artifact_paths(
        _load_json(gnss_source), export_dir=export_dir, log_dir=log_dir, dump_dir=dump_dir
    )
    logging_config = _redirect_artifact_paths(
        _load_json(logging_source),
        export_dir=export_dir,
        log_dir=log_dir,
        dump_dir=dump_dir,
    )

    glim_ros = ros_config.get("glim_ros")
    if not isinstance(glim_ros, dict):
        raise RuntimeError(f"glim_ros must be an object: {ros_source}")
    if simulation_scenario is not None:
        if forbid_sim_extensions or contract != "full_stack":
            raise RuntimeError("simulation configuration requires the simulation full_stack contract")
        initial = list(simulation_scenario["initial"])
        extent = list(simulation_scenario["map_size"])
        glim_ros["extension_modules"] = [
            "libgnss_extension.so", "libintegrity_extension.so",
            "libplanner_local_map_extension.so", "libsim_extension.so",
        ]
        glim_ros["acc_scale"] = 1.0
        glim_ros["imu_topic"] = "/sim/drone_0/imu_iap"
        glim_ros["points_topic"] = "/sim/drone_0/lidar_body"
        glim_ros["sim"] = {
            "truth_odom_topic": "/sim/drone_0/truth_odom",
            "planner_odom_topic": "/drone_0_visual_slam/odom",
            "planner_odom_frame_id": "map",
            "planner_body_frame_id": "imu",
            "align_planner_odom_to_truth": False,
            "static_planner_alignment_enabled": True,
            "static_planner_translation_m": initial,
            "enable_metrics_csv": False,
        }
        glim_ros["planner_local_map"].update({
            "frame_contract_id": "ego_grid_map_sim_v1",
            "beam_evidence_topic": "/iap/simulator/lidar_beam_evidence",
            "static_planner_translation_m": initial,
            "planning_lattice_resolution_m": 0.1,
            "planning_lattice_origin_m": [-extent[0] / 2, -extent[1] / 2, 0.0],
            "planning_lattice_extent_m": extent,
            "publish_current_hits_map": True,
        })
    modules = list(glim_ros.get("extension_modules", []))
    if "libgnss_extension.so" not in modules:
        raise RuntimeError(f"{contract} requires libgnss_extension.so")
    if contract == "glio" and "libintegrity_extension.so" in modules:
        raise RuntimeError("GLIO-only profile must not load libintegrity_extension.so")
    if contract in ("glio_integrity", "full_stack") and "libintegrity_extension.so" not in modules:
        raise RuntimeError(f"{contract} requires libintegrity_extension.so")
    if contract == "full_stack" and "libplanner_local_map_extension.so" not in modules:
        raise RuntimeError("full_stack requires libplanner_local_map_extension.so")
    if forbid_sim_extensions:
        forbidden = {
            "libsim_extension.so",
            "libdemo8_truth_araim_extension.so",
        }
        found = sorted(forbidden.intersection(modules))
        if found:
            raise RuntimeError(
                "flight profile contains forbidden simulation extensions: "
                + ", ".join(found)
            )
    if contract == "full_stack":
        local_map = glim_ros.get("planner_local_map")
        if not isinstance(local_map, dict):
            raise RuntimeError("full_stack requires glim_ros.planner_local_map config")
        frame_contract_id = str(local_map.get("frame_contract_id", "")).strip()
        if not frame_contract_id:
            raise RuntimeError("full_stack planner local-map frame_contract_id is empty")

    if integrity_profile not in INTEGRITY_PROFILES:
        valid = ", ".join(sorted(INTEGRITY_PROFILES))
        raise RuntimeError(f"unknown integrity_profile '{integrity_profile}'; valid: {valid}")
    gnss_enabled, lidar_enabled, fusion_mode = INTEGRITY_PROFILES[integrity_profile]
    integrity = gnss_config.setdefault("integrity", {})
    if not isinstance(integrity, dict):
        raise RuntimeError(f"integrity must be an object: {gnss_source}")
    integrity_module_loaded = "libintegrity_extension.so" in modules
    integrity["enable"] = integrity_module_loaded
    integrity["enable_araim"] = bool(integrity_module_loaded and gnss_enabled)
    integrity["enable_gnss_integrity"] = bool(integrity_module_loaded and gnss_enabled)
    integrity["enable_gnss_araim"] = bool(integrity_module_loaded and gnss_enabled)
    integrity["enable_lidar_integrity"] = bool(integrity_module_loaded and lidar_enabled)
    integrity["integrity_fusion_mode"] = fusion_mode

    gnss = gnss_config.setdefault("gnss", {})
    if isinstance(gnss, dict):
        gnss["debug_csv_path"] = str(export_dir / "iap_gnss_factor_debug.csv")
    integrity["araim_csv_path"] = str(integrity_export_dir / "iap_araim.csv")
    integrity["araim_pl_decomp_csv_path"] = str(
        integrity_export_dir / "iap_araim_pl_decomp.csv"
    )
    integrity["lidar_araim_stage0_csv_path"] = str(
        integrity_export_dir / "iap_lidar_araim_stage0.csv"
    )
    integrity["traj_csv_path"] = str(integrity_export_dir / "traj_with_gnss.csv")

    glim_ros["dump_path"] = str(dump_dir)
    sim_config = glim_ros.get("sim")
    if isinstance(sim_config, dict):
        sim_config["metrics_csv_path"] = str(
            simulation_export_dir / "iap_sim_truth_vs_est.csv"
        )

    logging_section = logging_config.setdefault("logging", {})
    if isinstance(logging_section, dict):
        logging_section["log_dir"] = str(log_dir)
    root_logging = root_config.setdefault("logging", {})
    if isinstance(root_logging, dict):
        root_logging["log_dir"] = str(log_dir)
    global_config["timing_csv_path"] = str(profiling_dir / "iap_timing.csv")

    runtime_ros = runtime_dir / "config_ros.json"
    runtime_gnss = runtime_dir / "config_gnss.json"
    runtime_logging = runtime_dir / "config_logging.json"
    _write_json(runtime_ros, ros_config)
    _write_json(runtime_gnss, gnss_config)
    _write_json(runtime_logging, logging_config)

    materialized_secondary_configs: dict[str, str] = {}
    for key, value in list(global_config.items()):
        if not key.startswith("config_") or key in {
            "config_path",
            "config_ros",
            "config_gnss",
            "config_logging",
        }:
            continue
        secondary_source = _resolve_reference(source, str(value), key)
        secondary = _redirect_artifact_paths(
            _load_json(secondary_source),
            export_dir=export_dir,
            log_dir=log_dir,
            dump_dir=dump_dir,
        )
        if simulation_scenario is not None and key == "config_odometry":
            secondary["odometry_estimation"]["initialization_mode"] = "NAIVE"
        secondary_name = f"{key}.json"
        _write_json(runtime_dir / secondary_name, secondary)
        global_config[key] = secondary_name
        materialized_secondary_configs[key] = str(runtime_dir / secondary_name)
    global_config["config_path"] = str(runtime_dir)
    global_config["config_ros"] = runtime_ros.name
    global_config["config_gnss"] = runtime_gnss.name
    global_config["config_logging"] = runtime_logging.name
    _write_json(runtime_dir / "config.json", root_config)

    manifest = {
        "schema_version": "iap_canonical_launch_profile_v1",
        "contract": contract,
        "source_config_dir": str(source),
        "runtime_config_dir": str(runtime_dir),
        "output_dir": str(run_root),
        "extension_modules": modules,
        "integrity_profile": integrity_profile,
        "imu_topic": glim_ros.get("imu_topic", ""),
        "points_topic": glim_ros.get("points_topic", ""),
        "materialized_secondary_configs": materialized_secondary_configs,
    }
    _write_json(manifest_dir / "launch_profile_manifest.json", manifest)
    return str(runtime_dir), manifest
