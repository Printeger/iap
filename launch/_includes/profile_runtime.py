"""Runtime configuration materialization for canonical IAP launch files."""

from __future__ import annotations

import json
import re
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
                redirected[key] = str(export_dir / (Path(child).name or f"{key}.csv"))
            elif isinstance(child, str) and key == "log_dir":
                redirected[key] = str(log_dir)
            elif isinstance(child, str) and key == "dump_path":
                redirected[key] = str(dump_dir)
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

    runtime_dir = run_root / "runtime_config"
    export_dir = run_root / "export"
    log_dir = run_root / "logs"
    dump_dir = run_root / "dump"
    for directory in (runtime_dir, export_dir, log_dir, dump_dir):
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
    integrity["araim_csv_path"] = str(export_dir / "iap_araim.csv")
    integrity["traj_csv_path"] = str(export_dir / "traj_with_gnss.csv")

    glim_ros["dump_path"] = str(dump_dir)
    sim_config = glim_ros.get("sim")
    if isinstance(sim_config, dict):
        sim_config["metrics_csv_path"] = str(export_dir / "iap_sim_truth_vs_est.csv")

    logging_section = logging_config.setdefault("logging", {})
    if isinstance(logging_section, dict):
        logging_section["log_dir"] = str(log_dir)
    root_logging = root_config.setdefault("logging", {})
    if isinstance(root_logging, dict):
        root_logging["log_dir"] = str(log_dir)
    global_config["timing_csv_path"] = str(export_dir / "iap_timing.csv")

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
    _write_json(run_root / "launch_profile_manifest.json", manifest)
    return str(runtime_dir), manifest
