#!/usr/bin/env python3
"""Run and summarize v2.0 demo11 acceptance validation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
LOG_ROOT = PACKAGE_ROOT / "log"
SETUP_BASH = WORKSPACE_ROOT / "install" / "setup.bash"

FINAL_PASS = "v2.0 full demo11 acceptance: PASS"
FINAL_FAIL = "v2.0 full demo11 acceptance: FAIL"

KEY_CSVS = (
    "planner_traj.csv",
    "planner_cmd.csv",
    "desired_vs_truth.csv",
    "iap_sim_truth_vs_est.csv",
    "iap_araim.csv",
    "integrity_along_planner_traj.csv",
    "future_integrity_snapshot.csv",
    "phase2_integrity_eval_aligned.csv",
    "planner_integrity_cost_debug.csv",
    "pl_grid_voxels.csv",
    "urg_grid_voxels.csv",
)

COMMON_LAUNCH_ARGS = {
    "start_rviz": "false",
    "allow_truth_alignment": "false",
    "use_so3_dynamics": "true",
    "use_iap_odom_for_planner": "true",
    "use_gnss": "true",
    "use_araim": "true",
    "planner_use_integrity_cost": "true",
    "planner_use_integrity_front_search": "true",
    "planner_use_integrity_global_search": "true",
}

LEGACY_ARGS = {
    "phase2_use_advisory_fim_add": "false",
    "phase2_use_lidar_advisory_fim": "false",
    "phase2_pi_use_unified_advisory_pl": "false",
    "phase2_use_unified_risk_grid": "false",
}

ENABLED_ARGS = {
    "phase2_pl_model": "fused_fim_grid",
    "phase2_use_advisory_fim_add": "true",
    "phase2_use_lidar_advisory_fim": "true",
    "phase2_pi_use_unified_advisory_pl": "true",
    "phase2_use_unified_risk_grid": "true",
}


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def finite_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def count_csv_rows(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            next(reader)
        except StopIteration:
            return 0
        return sum(1 for _ in reader)


def run_command(
    cmd: list[str],
    cwd: Path,
    log_path: Path,
    timeout_s: int | None = None,
    env: dict[str, str] | None = None,
) -> dict[str, Any]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8", errors="replace") as f:
        f.write("$ " + " ".join(cmd) + "\n\n")
        f.flush()
        try:
            proc = subprocess.run(
                cmd,
                cwd=str(cwd),
                stdout=f,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
                env=env,
            )
            return {
                "command": cmd,
                "returncode": proc.returncode,
                "timed_out": False,
                "log": str(log_path),
                "output_tail": tail_file(log_path),
            }
        except subprocess.TimeoutExpired:
            f.write(f"\nTIMEOUT after {timeout_s}s\n")
            return {
                "command": cmd,
                "returncode": 124,
                "timed_out": True,
                "log": str(log_path),
                "output_tail": tail_file(log_path),
            }


def run_bash(command: str, cwd: Path, log_path: Path, timeout_s: int | None = None) -> dict[str, Any]:
    return run_command(["bash", "-lc", command], cwd, log_path, timeout_s=timeout_s)


def tail_file(path: Path, lines: int = 40) -> str:
    if not path.exists():
        return ""
    text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(text[-lines:])


def latest_run_dir(before: set[Path]) -> Path | None:
    if not LOG_ROOT.exists():
        return None
    candidates = [
        p for p in LOG_ROOT.iterdir()
        if p.is_dir() and p.name != "latest" and p.resolve() not in before and (p / "export").exists()
    ]
    if not candidates:
        latest = LOG_ROOT / "latest"
        if latest.exists():
            return latest.resolve()
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime).resolve()


def current_log_dirs() -> set[Path]:
    if not LOG_ROOT.exists():
        return set()
    return {p.resolve() for p in LOG_ROOT.iterdir() if p.is_dir() and p.name != "latest"}


def build_launch_args(mode: str, run_duration_s: float, urg_export_voxels: bool) -> dict[str, str]:
    args = dict(COMMON_LAUNCH_ARGS)
    args["run_duration_s"] = f"{run_duration_s:g}"
    if mode == "legacy":
        args.update(LEGACY_ARGS)
    elif mode == "enabled":
        args.update(ENABLED_ARGS)
        args["phase2_urg_export_voxels"] = "true" if urg_export_voxels else "false"
    else:
        raise ValueError(f"unknown mode {mode}")
    return args


def launch_demo11(
    mode: str,
    run_duration_s: float,
    urg_export_voxels: bool,
    launch_timeout_s: int,
    mode_dir: Path,
) -> tuple[Path | None, dict[str, Any]]:
    before = current_log_dirs()
    launch_args = build_launch_args(mode, run_duration_s, urg_export_voxels)
    rendered_args = " ".join(f"{key}:={value}" for key, value in launch_args.items())
    command = (
        f"source {SETUP_BASH} && "
        "ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py "
        f"{rendered_args}"
    )
    result = run_bash(command, WORKSPACE_ROOT, mode_dir / "launch.log", timeout_s=launch_timeout_s)
    run_dir = latest_run_dir(before)
    result["launch_args"] = launch_args
    result["rendered_command"] = command
    result["detected_run_dir"] = str(run_dir) if run_dir else ""
    return run_dir, result


def parse_reuse(values: list[str]) -> dict[str, Path]:
    out: dict[str, Path] = {}
    for value in values:
        if "=" not in value:
            raise SystemExit(f"--reuse-run-dir must be MODE=PATH, got {value!r}")
        mode, path = value.split("=", 1)
        mode = mode.strip()
        if mode not in ("legacy", "enabled"):
            raise SystemExit(f"--reuse-run-dir mode must be legacy or enabled, got {mode!r}")
        out[mode] = Path(path).expanduser().resolve()
    return out


def validator_cmds(
    run_dir: Path,
    mode: str,
    urg_mean_update_ms_max: float,
) -> tuple[list[str], list[str], list[str]]:
    ana = [
        sys.executable,
        str(PACKAGE_ROOT / "tools" / "ana_log.py"),
        "--run",
        str(run_dir),
        "--skip-external-tools",
        "--no-plots",
    ]
    phase1 = [
        sys.executable,
        str(PACKAGE_ROOT / "tools" / "phase1" / "validate_phase1_closed_loop.py"),
        "--run-dir",
        str(run_dir),
        "--official",
    ]
    expect_mode = "fim_add_urg" if mode == "enabled" else "legacy"
    phase2 = [
        sys.executable,
        str(PACKAGE_ROOT / "tools" / "phase2" / "validate_phase2_integrity_eval.py"),
        "--run-dir",
        str(run_dir),
        "--expect-mode",
        expect_mode,
        "--urg-mean-update-ms-max",
        f"{urg_mean_update_ms_max:g}",
    ]
    return ana, phase1, phase2


def parse_mux_odom_stats(run_dir: Path, extra_logs: list[Path] | None = None) -> dict[str, Any]:
    stats: dict[str, Any] = {
        "available": False,
        "mode": "",
        "accepted_iap": None,
        "hist_accepted": None,
        "hist_stale": None,
        "hist_non_increasing": None,
        "hist_zero_stamp": None,
        "valid_iap_streak": None,
        "last_reason": "",
        "rejection_dominance_ratio": None,
    }
    extra_logs = extra_logs or []
    pattern = re.compile(
        r"status mode=(?P<mode>\S+).*?accepted_iap=(?P<accepted>\d+).*?"
        r"valid_iap_streak=(?P<streak>-?\d+).*?last_reason=(?P<reason>\S+).*?"
        r"hist_accepted=(?P<hist_accepted>\d+)\s+hist_stale=(?P<hist_stale>\d+)\s+"
        r"hist_non_increasing=(?P<hist_non_increasing>\d+)\s+hist_zero_stamp=(?P<hist_zero_stamp>\d+)"
    )
    matches: list[dict[str, str]] = []
    paths: list[Path] = []
    runtime = run_dir / "runtime"
    if runtime.exists():
        paths.extend(runtime.glob("*.log"))
    paths.extend(path for path in extra_logs if path.exists())
    for path in paths:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "demo3_odom_mux" not in text and "demo9_preflight_control_odom_mux" not in text:
            continue
        matches.extend(match.groupdict() for match in pattern.finditer(text))
    if not matches:
        return stats
    last = matches[-1]
    accepted = int(last["accepted"])
    hist_stale = int(last["hist_stale"])
    hist_non_increasing = int(last["hist_non_increasing"])
    hist_zero_stamp = int(last["hist_zero_stamp"])
    rejected = hist_stale + hist_non_increasing + hist_zero_stamp
    denom = accepted + rejected
    stats.update(
        {
            "available": True,
            "mode": last["mode"],
            "accepted_iap": accepted,
            "hist_accepted": int(last["hist_accepted"]),
            "hist_stale": hist_stale,
            "hist_non_increasing": hist_non_increasing,
            "hist_zero_stamp": hist_zero_stamp,
            "valid_iap_streak": int(last["streak"]),
            "last_reason": last["reason"],
            "rejection_dominance_ratio": rejected / denom if denom > 0 else None,
        }
    )
    return stats


def collect_artifacts(run_dir: Path, urg_export_voxels: bool, mode_dir: Path | None = None) -> dict[str, Any]:
    export = run_dir / "export"
    phase1_summary = load_json(export / "phase1_summary.json")
    phase2_summary = load_json(export / "phase2_summary.json")
    csvs: dict[str, Any] = {}
    for name in KEY_CSVS:
        path = export / name
        required = name != "urg_grid_voxels.csv" or urg_export_voxels
        csvs[name] = {
            "path": str(path),
            "exists": path.exists() and path.stat().st_size > 0,
            "rows": count_csv_rows(path),
            "required": required,
        }
    return {
        "run_dir": str(run_dir),
        "phase1_summary": phase1_summary,
        "phase2_summary": phase2_summary,
        "advisory_fim": phase2_summary.get("advisory_fim", {}),
        "pi_stage3": phase2_summary.get("pi_stage3", {}),
        "urg": phase2_summary.get("urg", {}),
        "csvs": csvs,
        "planner_trajectory_count": int(phase1_summary.get("planner_trajectory_count") or 0),
        "planner_command_count": int(phase1_summary.get("planner_command_count") or 0),
        "truth_odom_count": int(phase1_summary.get("truth_odom_count") or 0),
        "iap_odom_count": int(phase1_summary.get("iap_odom_count") or 0),
        "odom_acceptance": parse_mux_odom_stats(
            run_dir,
            [mode_dir / "launch.log"] if mode_dir is not None else [],
        ),
    }


def evaluate_mode(
    mode: str,
    collected: dict[str, Any],
    results: dict[str, Any],
    min_run_duration_s: float,
    urg_mean_update_ms_max: float,
    max_odom_reject_ratio: float,
) -> list[str]:
    failures: list[str] = []
    prefix = f"{mode}: "
    if results.get("phase1", {}).get("returncode") != 0:
        failures.append(prefix + "Phase 1 official validator failed")
    if results.get("phase2", {}).get("returncode") != 0:
        failures.append(prefix + "Phase 2 validator failed")
    phase1 = collected.get("phase1_summary") or {}
    run_duration = finite_float(phase1.get("run_duration_s"))
    if run_duration is None or run_duration < min_run_duration_s:
        rendered = "non-finite" if run_duration is None else f"{run_duration:.2f}"
        failures.append(prefix + f"run_duration_s {rendered} < {min_run_duration_s:.2f}")
    if int(phase1.get("iap_odom_count") or 0) < 50:
        failures.append(prefix + f"iap_odom_count {int(phase1.get('iap_odom_count') or 0)} < 50")
    odom = collected.get("odom_acceptance") or {}
    ratio = finite_float(odom.get("rejection_dominance_ratio"))
    if ratio is not None and ratio > max_odom_reject_ratio:
        failures.append(
            prefix
            + f"IAP odom rejection dominance ratio {ratio:.3f} > {max_odom_reject_ratio:.3f}"
        )
    for name, info in (collected.get("csvs") or {}).items():
        if info.get("required") and not info.get("exists"):
            failures.append(prefix + f"missing or empty export/{name}")
    if mode == "enabled":
        fim = collected.get("advisory_fim") or {}
        if int(fim.get("query_count") or 0) <= 0:
            failures.append(prefix + "advisory_fim.query_count <= 0")
        if fim.get("fusion_mode") != "fim_add":
            failures.append(prefix + f"advisory_fim.fusion_mode is {fim.get('fusion_mode')!r}, expected 'fim_add'")
        pi = collected.get("pi_stage3") or {}
        hist = pi.get("selected_source_histogram") if isinstance(pi, dict) else {}
        if not as_bool(pi.get("enabled", False)):
            failures.append(prefix + "pi_stage3.enabled is not true")
        if int((hist or {}).get("fim_add") or 0) <= 0:
            failures.append(prefix + "pi_stage3.selected_source_histogram.fim_add <= 0")
        urg = collected.get("urg") or {}
        if not as_bool(urg.get("urg_enabled", False)):
            failures.append(prefix + "urg.urg_enabled is not true")
        if not as_bool(urg.get("urg_active", False)):
            failures.append(prefix + "urg.urg_active is not true")
        if int(urg.get("urg_query_count") or 0) <= 0:
            failures.append(prefix + "urg.urg_query_count <= 0")
        mean_update = finite_float(urg.get("urg_mean_update_ms"))
        if mean_update is None:
            failures.append(prefix + "urg.urg_mean_update_ms is not finite")
        elif mean_update > urg_mean_update_ms_max:
            failures.append(
                prefix
                + f"urg.urg_mean_update_ms {mean_update:.3f} > {urg_mean_update_ms_max:.3f}"
            )
    return failures


def write_reports(report: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "acceptance_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# v2.0 Full Demo11 Acceptance Report",
        "",
        f"- Status: {'PASS' if report['passed'] else 'FAIL'}",
        f"- Generated: {report['generated_at']}",
        f"- Output directory: `{output_dir}`",
        "",
        "## Failed Checks",
    ]
    failures = report.get("failed_checks") or []
    if failures:
        lines.extend(f"- {failure}" for failure in failures)
    else:
        lines.append("- None")
    for mode, data in (report.get("modes") or {}).items():
        collected = data.get("collected") or {}
        phase1 = data.get("commands", {}).get("phase1", {})
        phase2 = data.get("commands", {}).get("phase2", {})
        lines.extend(
            [
                "",
                f"## {mode.title()}",
                f"- Run directory: `{collected.get('run_dir', '')}`",
                f"- Phase 1 validator return code: {phase1.get('returncode')}",
                f"- Phase 2 validator return code: {phase2.get('returncode')}",
                f"- Planner trajectories: {collected.get('planner_trajectory_count')}",
                f"- Planner commands: {collected.get('planner_command_count')}",
                f"- IAP odom count: {collected.get('iap_odom_count')}",
                f"- Odom acceptance: `{json.dumps(collected.get('odom_acceptance', {}), sort_keys=True)}`",
                f"- Advisory FIM: `{json.dumps(collected.get('advisory_fim', {}), sort_keys=True)}`",
                f"- PI Stage 3: `{json.dumps(collected.get('pi_stage3', {}), sort_keys=True)}`",
                f"- URG: `{json.dumps(collected.get('urg', {}), sort_keys=True)}`",
                "",
                "### CSV Artifacts",
            ]
        )
        for name, info in (collected.get("csvs") or {}).items():
            lines.append(
                f"- `{name}`: exists={info.get('exists')} rows={info.get('rows')} required={info.get('required')}"
            )
    (output_dir / "acceptance_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def selected_modes(mode: str) -> list[str]:
    if mode == "both":
        return ["legacy", "enabled"]
    return [mode]


def main() -> int:
    parser = argparse.ArgumentParser(description="Run v2.0 full demo11 acceptance validation.")
    parser.add_argument("--mode", choices=("legacy", "enabled", "both"), default="both")
    parser.add_argument("--run-duration-s", type=float, default=90.0)
    parser.add_argument("--urg-export-voxels", action="store_true", default=False)
    parser.add_argument("--urg-mean-update-ms-max", type=float, default=1000.0)
    parser.add_argument("--launch-timeout-s", type=int, default=0)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--reuse-run-dir", action="append", default=[], metavar="MODE=PATH")
    parser.add_argument("--max-odom-reject-ratio", type=float, default=0.50)
    args = parser.parse_args()

    if args.run_duration_s < 90.0:
        print(f"{FINAL_FAIL}")
        print(f"  - run_duration_s {args.run_duration_s:.2f} < 90.00")
        return 2

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else (LOG_ROOT / f"v2_acceptance_{timestamp}").resolve()
    )
    reuse = parse_reuse(args.reuse_run_dir)
    launch_timeout_s = args.launch_timeout_s or int(args.run_duration_s + 240)

    modes: dict[str, Any] = {}
    failed_checks: list[str] = []

    for mode in selected_modes(args.mode):
        mode_dir = output_dir / mode
        mode_dir.mkdir(parents=True, exist_ok=True)
        commands: dict[str, Any] = {}
        if mode in reuse:
            run_dir = reuse[mode]
            commands["launch"] = {
                "skipped": True,
                "reuse_run_dir": str(run_dir),
                "launch_args": build_launch_args(mode, args.run_duration_s, args.urg_export_voxels),
            }
        else:
            run_dir, launch_result = launch_demo11(
                mode,
                args.run_duration_s,
                args.urg_export_voxels,
                launch_timeout_s,
                mode_dir,
            )
            commands["launch"] = launch_result
            if launch_result.get("returncode") != 0:
                failed_checks.append(
                    f"{mode}: launch command returned {launch_result.get('returncode')}"
                )
            if run_dir is None:
                failed_checks.append(f"{mode}: launch did not produce a run directory")
                modes[mode] = {"commands": commands, "collected": {}}
                continue

        if not run_dir.exists():
            failed_checks.append(f"{mode}: run directory does not exist: {run_dir}")
            modes[mode] = {"commands": commands, "collected": {}}
            continue

        ana_cmd, phase1_cmd, phase2_cmd = validator_cmds(
            run_dir, mode, args.urg_mean_update_ms_max
        )
        commands["ana_log"] = run_command(ana_cmd, PACKAGE_ROOT, mode_dir / "ana_log.log")
        commands["phase1"] = run_command(phase1_cmd, PACKAGE_ROOT, mode_dir / "phase1_validator.log")
        commands["phase2"] = run_command(phase2_cmd, PACKAGE_ROOT, mode_dir / "phase2_validator.log")
        collected = collect_artifacts(
            run_dir,
            args.urg_export_voxels if mode == "enabled" else False,
            mode_dir,
        )
        failed_checks.extend(
            evaluate_mode(
                mode,
                collected,
                commands,
                args.run_duration_s,
                args.urg_mean_update_ms_max,
                args.max_odom_reject_ratio,
            )
        )
        modes[mode] = {"commands": commands, "collected": collected}

    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "passed": not failed_checks,
        "failed_checks": failed_checks,
        "config": {
            "mode": args.mode,
            "run_duration_s": args.run_duration_s,
            "urg_export_voxels": args.urg_export_voxels,
            "urg_mean_update_ms_max": args.urg_mean_update_ms_max,
            "launch_timeout_s": launch_timeout_s,
            "max_odom_reject_ratio": args.max_odom_reject_ratio,
        },
        "modes": modes,
    }
    write_reports(report, output_dir)

    if report["passed"]:
        print(FINAL_PASS)
        return 0
    print(FINAL_FAIL)
    for failure in failed_checks:
        print(f"  - {failure}")
    print(f"acceptance_report.md: {output_dir / 'acceptance_report.md'}")
    print(f"acceptance_report.json: {output_dir / 'acceptance_report.json'}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
