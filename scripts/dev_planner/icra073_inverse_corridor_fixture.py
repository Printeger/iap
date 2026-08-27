#!/usr/bin/env python3
"""Build immutable ICRA-073 inverse-corridor fixture descriptors."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import subprocess
import sys
from pathlib import Path


SCHEMA_VERSION = "p4_v2_inverse_corridor_fixture_v1"
SCHEMA_VERSION_V2 = "p4_v2_inverse_corridor_fixture_v2"
VARIANTS = ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL")
FIXTURE_SEED = 73001
REPOSITORY = Path(__file__).resolve().parents[2]
RESULTS_ROOT = (REPOSITORY / "results/icra27/icra073").resolve()
KNOWN_RETAINED_UNTRACKED = (
    {
        "path": ".claude/settings.local.json",
        "size_bytes": 72,
        "sha256": (
            "27aac0ccca0ad0ab573578864cf27b9560d3f819bdeeae62378f8c20e62a8f64"),
    },
    {
        "path": "docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf",
        "size_bytes": 243368,
        "sha256": (
            "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6"),
    },
    {
        "path": (
            "src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~"),
        "size_bytes": 2962,
        "sha256": (
            "29a73228c1d58d5c4983dc217230d6cbc6ff6e295a77a004d8e6a207fd242028"),
    },
    {
        "path": "results/icra27/icra073/forged-source-1338727.json",
        "size_bytes": 5600,
        "sha256": (
            "3a22a352eb9329f660384e901e42bfd3bce11ab74ff3718c7d1a0c35c87ac8dc"),
        "disposition": "REJECTED_PRE_FIX_RED_EVIDENCE_RETAINED",
    },
)


def _canonical_bytes(value: dict) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"),
        ensure_ascii=True).encode("utf-8")


def _sha256(value: dict) -> str:
    return hashlib.sha256(_canonical_bytes(value)).hexdigest()


def build_descriptor(variant: str) -> dict:
    """Return one deterministic descriptor at the frozen public seam."""
    if variant not in VARIANTS:
        raise ValueError(f"unknown ICRA-073 scene variant: {variant}")
    mirror = variant == "EXACT_MIRROR"
    safe_amplitude = -3.5 if mirror else 3.5
    risky_amplitude = 2.1 if mirror else -2.1
    provider_values = (
        {"safe": 1.0, "risky": 1.0}
        if variant == "FLAT_NULL"
        else {"safe": 1.0, "risky": 4.0})
    descriptor = {
        "schema_version": SCHEMA_VERSION,
        "design_record": "ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1",
        "scene_variant": variant,
        "frame_id": "map",
        "units": "m",
        "deterministic_seed": FIXTURE_SEED,
        "start_m": [-12.0, 0.0, 1.5],
        "goal_m": [12.0, 0.0, 1.5],
        "straight_seed": {
            "kind": "closed_line_segment",
            "start_reference": "start_m",
            "goal_reference": "goal_m",
            "equation": "(-12+24*u, 0, 1.5)",
            "u_interval_inclusive": [0.0, 1.0],
        },
        "centre_lines": {
            "safe": {
                "equation": "(-12+24*u, amplitude_y_m*sin(pi*u), 1.5)",
                "u_interval_inclusive": [0.0, 1.0],
                "amplitude_y_m": safe_amplitude,
            },
            "risky": {
                "equation": "(-12+24*u, amplitude_y_m*sin(pi*u), 1.5)",
                "u_interval_inclusive": [0.0, 1.0],
                "amplitude_y_m": risky_amplitude,
            },
        },
        "tube_radius_m": 0.75,
        "guard_band_width_m": 0.5,
        "protected_centre_line_radius_m": 1.25,
        "polyline": {
            "segment_count": 240,
            "max_bidirectional_hausdorff_error_m": 0.01,
        },
        "central_cuboid": {
            "kind": "occupied_cuboid",
            "bounds_inclusive": True,
            "bounds_m": {
                "x": [-2.0, 2.0],
                "y": [-0.75, 0.75],
                "z": [0.0, 2.8],
            },
        },
        "gnss_only_overhead_mask": {
            "kind": "provider_occluder_not_occupancy",
            "follows_route": "risky",
            "x_interval_inclusive_m": [-8.0, 8.0],
            "z_interval_inclusive_m": [7.30, 7.55],
            "projection_half_width_m": 0.75,
        },
        "lidar_landmarks": {
            "policy": "equal mirrored landmark pairs outside_both_guards",
            "pairs_m": [
                [[-8.0, -5.25, 1.5], [-8.0, 5.25, 1.5]],
                [[0.0, -5.25, 1.5], [0.0, 5.25, 1.5]],
                [[8.0, -5.25, 1.5], [8.0, 5.25, 1.5]],
            ],
        },
        "outer_trees": {
            "policy": "close_third_homotopy_only_preserve_both_guards",
            "trunk_radius_m": 0.10,
            "centres_m": [
                [-6.0, -6.0, 0.0], [-2.0, -6.0, 0.0],
                [2.0, -6.0, 0.0], [6.0, -6.0, 0.0],
                [-6.0, 6.0, 0.0], [-2.0, 6.0, 0.0],
                [2.0, 6.0, 0.0], [6.0, 6.0, 0.0],
            ],
            "mirror_applied": mirror,
        },
        "motion_feasibility_identity": {
            "uav_radius_m": 0.35,
            "occupancy_inflation_m": 0.099,
            "optimizer_swarm_clearance_m": 0.5,
            "authority": "occupancy_and_ego",
        },
        "provider_truth": {
            "schema_version": "icra073_provider_truth_v1",
            "policy": "analytic_frozen_overhead_mask_visibility_truth",
            "finite_complete_support": True,
            "controllable_x_interval_inclusive_m": [-8.0, 8.0],
            "route_interior_values": provider_values,
            "flat_null": variant == "FLAT_NULL",
        },
        "independent_oracle": {
            "schema_version": "p4_v2_inverse_corridor_analysis_v1",
            "sample_count": 200,
            "sampling": "deterministic_equal_arc_including_endpoints",
            "truth_source": "this_frozen_descriptor_only",
            "trajectory_source": "committed_final_bspline_only",
            "forbidden_p4_inputs": [
                "selected_guide_risk", "p4_route_label", "p4_objective",
                "p4_decision_output", "p4_evidence_output",
            ],
        },
        "decision_plane": {
            "allowed_inputs": [
                "ordinary_occupancy", "immutable_p0_snapshot"],
            "forbidden_inputs": [
                "corridor_label", "centre_line", "tube_membership",
                "oracle_risk", "expected_route", "analysis_output"],
        },
        "interval_policy": "all_declared_bounds_are_inclusive",
        "scene_identity": f"icra073-{variant.lower()}-seed-{FIXTURE_SEED}",
    }
    descriptor["descriptor_hash_input_fields"] = sorted(
        (*descriptor.keys(), "descriptor_hash_input_fields"))
    descriptor["descriptor_sha256"] = _sha256(descriptor)
    return descriptor


def build_v2_descriptor(variant: str) -> dict:
    """Return the user-amended V2 descriptor while retaining V1 bytes."""
    descriptor = build_descriptor(variant)
    descriptor["schema_version"] = SCHEMA_VERSION_V2
    descriptor["design_record"] = "ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V2"
    descriptor["centre_lines"]["risky"]["amplitude_y_m"] = (
        2.20 if variant == "EXACT_MIRROR" else -2.20)
    descriptor["scene_identity"] = (
        f"icra074-{variant.lower()}-seed-{FIXTURE_SEED}")
    descriptor.pop("descriptor_sha256")
    descriptor["descriptor_sha256"] = _sha256(descriptor)
    return descriptor


def _centre_point(descriptor: dict, route: str, u: float) -> tuple[float, ...]:
    amplitude = descriptor["centre_lines"][route]["amplitude_y_m"]
    return (-12.0 + 24.0 * u, amplitude * math.sin(math.pi * u), 1.5)


def _point_aabb_distance(point: tuple[float, ...], bounds: dict) -> float:
    squared = 0.0
    for coordinate, axis in zip(point, ("x", "y", "z")):
        low, high = bounds[axis]
        delta = low - coordinate if coordinate < low else (
            coordinate - high if coordinate > high else 0.0)
        squared += delta * delta
    return math.sqrt(squared)


def _point_segment_distance(
        point: tuple[float, ...], start: tuple[float, ...],
        end: tuple[float, ...]) -> float:
    delta = tuple(b - a for a, b in zip(start, end))
    length_squared = sum(value * value for value in delta)
    if length_squared == 0.0:
        return math.dist(point, start)
    projection = sum(
        (value - a) * direction
        for value, a, direction in zip(point, start, delta)) / length_squared
    projection = min(1.0, max(0.0, projection))
    closest = tuple(a + projection * direction
                    for a, direction in zip(start, delta))
    return math.dist(point, closest)


def _polyline_error(descriptor: dict, route: str) -> float:
    segments = descriptor["polyline"]["segment_count"]
    if not isinstance(segments, int) or segments <= 0:
        return math.inf
    maximum = 0.0
    for index in range(segments):
        start = _centre_point(descriptor, route, index / segments)
        end = _centre_point(descriptor, route, (index + 1) / segments)
        for subdivision in range(1, 10):
            u = (index + subdivision / 10.0) / segments
            maximum = max(maximum, _point_segment_distance(
                _centre_point(descriptor, route, u), start, end))
    return maximum


def _minimum_curve_cuboid_clearance(
        descriptor: dict, route: str) -> float:
    bounds = descriptor["central_cuboid"]["bounds_m"]
    return min(
        _point_aabb_distance(_centre_point(descriptor, route, index / 4800),
                             bounds)
        for index in range(4801))


def _minimum_xy_curve_point_clearance(
        descriptor: dict, route: str, point: list[float]) -> float:
    return min(
        math.hypot(
            _centre_point(descriptor, route, index / 2400)[0] - point[0],
            _centre_point(descriptor, route, index / 2400)[1] - point[1])
        for index in range(2401))


def _result(accepted: bool, **observations) -> dict:
    return {"accepted": bool(accepted), **observations}


def preflight_descriptors(descriptors: dict[str, dict]) -> dict:
    """Prove the frozen geometry and data-plane contract without ROS."""
    checks = {}
    variants_exact = set(descriptors) == set(VARIANTS)
    descriptor_hashes = {}
    hashes_ok = variants_exact
    for variant in VARIANTS:
        descriptor = descriptors.get(variant, {})
        hash_input = dict(descriptor)
        observed = hash_input.pop("descriptor_sha256", None)
        expected = _sha256(hash_input) if hash_input else None
        descriptor_hashes[variant] = {
            "observed": observed, "recomputed": expected}
        hashes_ok = hashes_ok and observed == expected
    checks["descriptor_hash_exact"] = _result(
        hashes_ok, descriptors=descriptor_hashes)

    if not variants_exact:
        checks["common_endpoints_and_nonstraight"] = _result(False)
        for name in (
                "straight_seed_closed_collision_with_free_entry_exit",
                "curved_tubes_and_guards_occupancy_clear",
                "curved_routes_reachable",
                "polyline_hausdorff_within_0_01_m",
                "lidar_landmarks_symmetric",
                "outer_trees_close_only_third_homotopy",
                "primary_provider_support_finite_complete_and_ordered",
                "exact_mirror_geometric_y_negation",
                "flat_null_truth_finite_complete_identical",
                "decision_and_oracle_data_planes_isolated"):
            checks[name] = _result(False, reason="variant_set_incomplete")
        return _preflight_record(checks)

    endpoints_ok = all(
        descriptor["start_m"] == [-12.0, 0.0, 1.5]
        and descriptor["goal_m"] == [12.0, 0.0, 1.5]
        and all(abs(descriptor["centre_lines"][route][
            "amplitude_y_m"]) > 0.0 for route in ("safe", "risky"))
        for descriptor in descriptors.values())
    checks["common_endpoints_and_nonstraight"] = _result(endpoints_ok)

    straight_results = {}
    for variant, descriptor in descriptors.items():
        bounds = descriptor["central_cuboid"]["bounds_m"]
        intersects = (
            bounds["x"][0] <= 0.0 <= bounds["x"][1]
            and bounds["y"][0] <= 0.0 <= bounds["y"][1]
            and bounds["z"][0] <= 1.5 <= bounds["z"][1])
        entry_exit_free = (
            _point_aabb_distance((-12.0, 0.0, 1.5), bounds) > 0.0
            and _point_aabb_distance((12.0, 0.0, 1.5), bounds) > 0.0)
        straight_results[variant] = {
            "collision": intersects, "entry_exit_free": entry_exit_free}
    checks["straight_seed_closed_collision_with_free_entry_exit"] = _result(
        all(item["collision"] and item["entry_exit_free"]
            for item in straight_results.values()), variants=straight_results)

    clearance_results = {}
    clearance_ok = True
    for variant, descriptor in descriptors.items():
        required = (
            descriptor["protected_centre_line_radius_m"]
            + descriptor["motion_feasibility_identity"]
            ["occupancy_inflation_m"])
        clearance_results[variant] = {}
        for route in ("safe", "risky"):
            observed = _minimum_curve_cuboid_clearance(descriptor, route)
            clearance_results[variant][route] = {
                "minimum_raw_occupancy_clearance_m": observed,
                "required_guard_plus_inflation_m": required,
            }
            clearance_ok = clearance_ok and observed >= required
    checks["curved_tubes_and_guards_occupancy_clear"] = _result(
        clearance_ok, variants=clearance_results)
    if not clearance_ok:
        checks["curved_routes_reachable"] = {
            "accepted": None,
            "status": "NOT_EVALUATED_BLOCKED_BY_GEOMETRY",
            "reason": "full_occupancy_including_outer_trees_not_evaluated",
        }
        for name in (
                "polyline_hausdorff_within_0_01_m",
                "lidar_landmarks_symmetric",
                "outer_trees_close_only_third_homotopy",
                "primary_provider_support_finite_complete_and_ordered",
                "exact_mirror_geometric_y_negation",
                "flat_null_truth_finite_complete_identical",
                "decision_and_oracle_data_planes_isolated"):
            checks[name] = {
                "accepted": None,
                "status": "NOT_EVALUATED_BLOCKED_BY_GEOMETRY",
            }
        return _preflight_record(checks)

    checks["curved_routes_reachable"] = {
        "accepted": None,
        "status": "NOT_EVALUATED_PENDING_FULL_OCCUPANCY_PROOF",
    }

    polyline_errors = {
        variant: {
            route: _polyline_error(descriptor, route)
            for route in ("safe", "risky")}
        for variant, descriptor in descriptors.items()}
    maximum_error = max(
        value for by_route in polyline_errors.values()
        for value in by_route.values())
    polyline_ok = all(
        value <= descriptor["polyline"]
        ["max_bidirectional_hausdorff_error_m"]
        for variant, descriptor in descriptors.items()
        for value in polyline_errors[variant].values())
    checks["polyline_hausdorff_within_0_01_m"] = _result(
        polyline_ok, maximum_observed_error_m=maximum_error,
        per_variant_route_m=polyline_errors)

    lidar_ok = True
    lidar_clearances = {}
    for variant, descriptor in descriptors.items():
        pairs = descriptor["lidar_landmarks"]["pairs_m"]
        symmetric = all(
            left[0] == right[0] and left[1] == -right[1]
            and left[2] == right[2] for left, right in pairs)
        minimum = min(
            _minimum_xy_curve_point_clearance(descriptor, route, point)
            for pair in pairs for point in pair for route in ("safe", "risky"))
        lidar_clearances[variant] = minimum
        lidar_ok = lidar_ok and symmetric and minimum >= (
            descriptor["protected_centre_line_radius_m"])
    checks["lidar_landmarks_symmetric"] = _result(
        lidar_ok, minimum_route_clearance_m=lidar_clearances)

    tree_ok = True
    tree_clearances = {}
    for variant, descriptor in descriptors.items():
        trees = descriptor["outer_trees"]
        required = (
            descriptor["protected_centre_line_radius_m"]
            + descriptor["motion_feasibility_identity"]
            ["occupancy_inflation_m"] + trees["trunk_radius_m"])
        minimum = min(
            _minimum_xy_curve_point_clearance(descriptor, route, point)
            for point in trees["centres_m"] for route in ("safe", "risky"))
        tree_clearances[variant] = {
            "minimum_m": minimum, "required_m": required}
        tree_ok = tree_ok and (
            trees["policy"] ==
            "close_third_homotopy_only_preserve_both_guards"
            and minimum >= required)
    checks["outer_trees_close_only_third_homotopy"] = _result(
        tree_ok, variants=tree_clearances)

    primary_truth = descriptors["PRIMARY"]["provider_truth"]
    primary_values = primary_truth["route_interior_values"]
    primary_ok = (
        primary_truth["finite_complete_support"] is True
        and all(math.isfinite(value) for value in primary_values.values())
        and primary_values["risky"] > primary_values["safe"])
    checks["primary_provider_support_finite_complete_and_ordered"] = _result(
        primary_ok, values=primary_values)

    primary = descriptors["PRIMARY"]
    mirror = descriptors["EXACT_MIRROR"]
    mirror_ok = (
        mirror["start_m"] == primary["start_m"]
        and mirror["goal_m"] == primary["goal_m"]
        and all(mirror["centre_lines"][route]["amplitude_y_m"] ==
                -primary["centre_lines"][route]["amplitude_y_m"]
                for route in ("safe", "risky"))
        and {tuple(point) for point in mirror["outer_trees"]["centres_m"]} ==
        {(point[0], -point[1], point[2])
         for point in primary["outer_trees"]["centres_m"]})
    checks["exact_mirror_geometric_y_negation"] = _result(mirror_ok)

    null = descriptors["FLAT_NULL"]
    null_values = null["provider_truth"]["route_interior_values"]
    null_ok = (
        null["centre_lines"] == primary["centre_lines"]
        and null["central_cuboid"] == primary["central_cuboid"]
        and null["provider_truth"]["finite_complete_support"] is True
        and all(math.isfinite(value) for value in null_values.values())
        and null_values["safe"] == null_values["risky"])
    checks["flat_null_truth_finite_complete_identical"] = _result(
        null_ok, values=null_values)

    expected_forbidden = {
        "selected_guide_risk", "p4_route_label", "p4_objective",
        "p4_decision_output", "p4_evidence_output"}
    isolation_ok = all(
        set(descriptor["independent_oracle"]["forbidden_p4_inputs"])
        == expected_forbidden
        and set(descriptor["decision_plane"]["allowed_inputs"])
        == {"ordinary_occupancy", "immutable_p0_snapshot"}
        and "analysis_output" in descriptor["decision_plane"]
        ["forbidden_inputs"] for descriptor in descriptors.values())
    checks["decision_and_oracle_data_planes_isolated"] = _result(
        isolation_ok)
    return _preflight_record(checks)


def _preflight_record(checks: dict) -> dict:
    failures = [name for name, check in checks.items()
                if check.get("accepted") is False]
    incomplete = [name for name, check in checks.items()
                  if check.get("accepted") is None]
    return {
        "schema_version": "icra073_fixture_preflight_v1",
        "checks": checks,
        "failure_reasons": failures,
        "incomplete_checks": incomplete,
        "result": "PASS" if not failures and not incomplete else "FAIL",
    }


def _git_environment() -> dict[str, str]:
    environment = {
        key: value for key, value in os.environ.items()
        if not (
            key == "GIT_CONFIG_PARAMETERS"
            or key == "GIT_CONFIG_SYSTEM"
            or key == "GIT_CONFIG_GLOBAL"
            or re.fullmatch(r"GIT_CONFIG_(KEY|VALUE)_\d+", key)
            or key == "GIT_CONFIG_COUNT")
    }
    environment.update({
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_COUNT": "1",
        "GIT_CONFIG_KEY_0": "safe.directory",
        "GIT_CONFIG_VALUE_0": str(REPOSITORY),
    })
    return environment


def _git(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *arguments], cwd=REPOSITORY, env=_git_environment(),
        check=False, capture_output=True, text=True)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bind_source(requested_head: str) -> dict:
    """Bind evidence to an aligned, tracked-clean Git source fail-closed."""
    commands = {
        "head": _git("rev-parse", "HEAD"),
        "origin_head": _git("rev-parse", "origin/dev/icra"),
        "divergence": _git(
            "rev-list", "--left-right", "--count",
            "HEAD...origin/dev/icra"),
        "status": _git(
            "status", "--porcelain=v1", "--untracked-files=all"),
    }
    command_ok = all(command.returncode == 0 for command in commands.values())
    head = commands["head"].stdout.strip()
    origin_head = commands["origin_head"].stdout.strip()
    divergence = commands["divergence"].stdout.strip()
    status_lines = commands["status"].stdout.splitlines()
    tracked_changes = [
        line for line in status_lines if not line.startswith("?? ")]
    visible_untracked = [
        line[3:] for line in status_lines if line.startswith("?? ")]
    retained_paths = {entry["path"] for entry in KNOWN_RETAINED_UNTRACKED}
    unexpected_visible_untracked = [
        path for path in visible_untracked if path not in retained_paths]

    required_tracked_paths = (
        "scripts/dev_planner/icra073_inverse_corridor_fixture.py",
        "test/test_icra073_inverse_corridor.py",
    )
    required_tracked = {}
    for relative in required_tracked_paths:
        tracked = _git("ls-files", "--error-unmatch", "--", relative)
        required_tracked[relative] = (
            tracked.returncode == 0 and tracked.stdout.splitlines() == [relative])
    required_tracked_ok = all(required_tracked.values())

    retained = []
    retained_ok = True
    for expected in KNOWN_RETAINED_UNTRACKED:
        relative = expected["path"]
        path = REPOSITORY / relative
        observed = {
            "path": relative,
            "expected_size_bytes": expected["size_bytes"],
            "expected_sha256": expected["sha256"],
            "disposition": expected.get("disposition", "PROTECTED_RETAINED"),
        }
        try:
            metadata = path.lstat()
            observed.update({
                "exists": True,
                "regular_non_symlink": (
                    stat.S_ISREG(metadata.st_mode) and not path.is_symlink()),
                "observed_size_bytes": metadata.st_size,
                "observed_sha256": _file_sha256(path),
            })
        except OSError as exc:
            observed.update({
                "exists": False,
                "regular_non_symlink": False,
                "error": type(exc).__name__,
            })
        ignore_blind = _git("ls-files", "--others", "--", relative)
        observed["ignore_blind_untracked"] = (
            ignore_blind.returncode == 0
            and ignore_blind.stdout.splitlines() == [relative])
        observed["accepted"] = (
            observed.get("exists") is True
            and observed.get("regular_non_symlink") is True
            and observed.get("observed_size_bytes") == expected["size_bytes"]
            and observed.get("observed_sha256") == expected["sha256"]
            and observed["ignore_blind_untracked"])
        retained_ok = retained_ok and observed["accepted"]
        retained.append(observed)

    accepted = (
        command_ok
        and re.fullmatch(r"[0-9a-f]{40}", requested_head or "") is not None
        and requested_head == head == origin_head
        and divergence.split() == ["0", "0"]
        and not tracked_changes
        and not unexpected_visible_untracked
        and required_tracked_ok
        and retained_ok)
    return {
        "accepted": accepted,
        "requested_head": requested_head,
        "observed_head": head,
        "observed_origin_dev_icra": origin_head,
        "divergence_left_right": divergence,
        "tracked_status_entries": tracked_changes,
        "visible_untracked_entries": visible_untracked,
        "unexpected_visible_untracked_entries": unexpected_visible_untracked,
        "required_head_tracked_paths": required_tracked,
        "known_retained_untracked": retained,
        "git_commands_ok": command_ok,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--variant", choices=VARIANTS)
    mode.add_argument("--preflight-all", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--source-head")
    args = parser.parse_args()
    if args.preflight_all:
        if args.output is None:
            parser.error("--preflight-all requires --output")
        output = (args.output if args.output.is_absolute()
                  else REPOSITORY / args.output).resolve()
        try:
            output.relative_to(RESULTS_ROOT)
        except ValueError as exc:
            raise SystemExit(
                "ICRA-073 preflight output must be repository-local") from exc
        if output.exists():
            raise SystemExit("ICRA-073 preflight output must be new")
        source_binding = bind_source(args.source_head or "")
        if not source_binding["accepted"]:
            print("SOURCE_BINDING_NOT_READY", file=sys.stderr)
            return 3
        descriptors = {variant: build_descriptor(variant)
                       for variant in VARIANTS}
        payload = {
            **preflight_descriptors(descriptors),
            "task_id": "ICRA-073",
            "milestone": (
                "ICRA-073_LAYER3_INVERSE_CORRIDOR_EFFECT_DIAGNOSTICS"),
            "development_only": True,
            "effect_claim": False,
            "qualification_claim": False,
            "campaign_claim": False,
            "source_head": args.source_head,
            "source_binding": source_binding,
            "argv": list(sys.argv),
            "cwd": str(Path.cwd().resolve()),
            "descriptor_bindings": {
                variant: {
                    "scene_identity": descriptor["scene_identity"],
                    "descriptor_sha256": descriptor["descriptor_sha256"],
                } for variant, descriptor in descriptors.items()},
            "icra072b_status": "BLOCKED_USER_ACCEPTED_BYPASS_NOT_PASS",
            "gpu_preflight": "NOT_RUN_FIXTURE_PREFLIGHT_FAILED",
            "ros_main_flow": "NOT_STARTED",
        }
        output.parent.mkdir(parents=True, exist_ok=True)
    else:
        payload = build_descriptor(args.variant)
    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        output = output if args.preflight_all else args.output
        output.write_text(rendered)
    return 0 if payload.get("result", "PASS") == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
