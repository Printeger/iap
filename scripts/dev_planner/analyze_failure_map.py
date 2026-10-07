#!/usr/bin/env python3
"""Replay a frozen failure map through the planner's C++ clearance and A*."""

import argparse
import csv
import hashlib
import tempfile
from collections import Counter

import numpy as np
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

from replay_failure_map import load_snapshot


def _backend_path():
    override = os.environ.get("IAP_FAILURE_MAP_REPLAY_BIN")
    if override:
        return Path(override)
    installed = (Path(__file__).resolve().parents[4] / "install" /
                 "ego_planner/lib/ego_planner/failure_map_replay")
    if installed.is_file():
        return installed
    executable = shutil.which("failure_map_replay")
    if executable:
        return Path(executable)
    raise FileNotFoundError("build ego_planner to install failure_map_replay")


def _point(value):
    if len(value) != 3 or not all(isinstance(x, (int, float)) and
                                  math.isfinite(x) for x in value):
        raise ValueError("invalid three-dimensional snapshot coordinate")
    return " ".join(format(float(x), ".17g") for x in value)


def _number(value):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("snapshot has unavailable motion or time evidence")
    return format(float(value), ".17g")


def _inconclusive(directory, reason):
    return {"schema_version": "iap_failure_reachability_v2",
            "classification": reason, "snapshot": str(directory)}


def inspect(directory, budget_s=120.0, backend=None, benchmark_repeats=None, benchmark_diagnostics=True, differential=False, full_epoch=False, attribution=False, output=None, plots=False):
    initial_hashes = _snapshot_hashes(Path(directory)) if attribution else None
    directory, meta, _ = load_snapshot(directory)
    if attribution and initial_hashes != _snapshot_hashes(directory):
        raise RuntimeError("immutable snapshot changed while reading context")
    if meta.get("schema_version") not in ("iap_gridmap_failure_v2", "iap_gridmap_failure_v3"):
        raise ValueError("same-rule replay requires a v2 failure snapshot")
    if meta.get("kind") == "candidate":
        return _inconclusive(directory, "INCONCLUSIVE_CANDIDATE_ONLY")
    if not math.isfinite(budget_s) or budget_s <= 0:
        raise ValueError("budget must be a finite positive number")
    required = ("planning_time_s", "cloud_stamp_s", "motion_stamp_s",
                "motion_error_proxy_m", "motion_body_radius_m",
                "motion_tracking_reserve_m", "motion_budget_m",
                "motion_max_age_s", "environment_max_age_s")
    try:
        motion_values = [_number(meta[name]) for name in required]
    except (KeyError, ValueError):
        return _inconclusive(directory,
                             "INCONCLUSIVE_STALE_OR_INVALID_EVIDENCE")
    points = meta.get("control_points_m")
    if attribution:
        goals = meta.get("planning_goals_m")
        if not isinstance(goals, list) or not goals or "real_start_p_m" not in meta:
            return _inconclusive(directory, "INCONCLUSIVE_MISSING_GOAL_SET")
        points = points or [meta["real_start_p_m"], goals[0]]
    if (not isinstance(points, list) or not points or
            meta.get("search_failure") is None):
        return _inconclusive(directory,
                             "INCONCLUSIVE_MISSING_SEGMENT_CONTEXT")
    if len(points) > 100000:
        raise ValueError("too many control points in failure snapshot")
    try:
        lines = [" ".join(str(int(x)) for x in meta["dimensions"]),
                 _point(meta["origin_m"]), _point(meta["max_boundary_m"]),
                 " ".join((_number(meta["resolution_m"]),
                           motion_values[1], str(int(meta["generation"])),
                           json.dumps(meta["frame_id"]), motion_values[0])),
                 " ".join((str(int(meta["motion_quality"])),
                           str(int(bool(meta["motion_allow_bridged"]))),
                           *motion_values[2:8], motion_values[8])),
                 " ".join((*map(str, meta["search_pool_dimensions"]),
                           _number(meta["search_step_size_m"]))),
                 _point(meta["search_pool_center_m"]),
                 _point(meta["real_start_p_m"] if attribution else meta["search_requested_start_m"]),
                 _point(meta["search_requested_end_m"]),
                 " ".join((str(int(meta.get("segment_start_index", -1))),
                           str(int(meta.get("segment_end_index", -1))),
                           str(len(points))))]
        lines.extend(_point(point) for point in points)
        lines.append(f"{meta['search_failure']} {_number(budget_s)}")
    except (KeyError, TypeError, ValueError):
        return _inconclusive(directory,
                             "INCONCLUSIVE_MISSING_SEGMENT_CONTEXT")
    binary = Path(backend) if backend else _backend_path()
    command = [str(binary), str(directory / meta["cell_flags_file"])]
    if attribution:
        lines.append(f"{_number(meta['guide_fitting_reserve_m'])} {_number(meta['guide_reserve_taper_distance_m'])} {len(goals)}")
        lines.extend(_point(goal) for goal in goals)
        frame = meta.get("current_frame")
        source_path = ""
        if meta.get("observation_evidence_available") and frame and 0 <= (
                meta["planning_time_s"]-frame.get("scan_end_stamp_s", frame["stamp_s"])) <= meta["environment_max_age_s"]:
            source_path = str(directory/meta["observation_sources_file"])
        lines.append(json.dumps(source_path))
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(output) if output else Path(temporary)
            if destination.resolve() == directory or directory in destination.resolve().parents:
                raise ValueError("attribution output must be outside the immutable snapshot")
            destination.mkdir(parents=True, exist_ok=True)
            if output and any(destination.iterdir()):
                raise ValueError("attribution output directory must be empty")
            before = _snapshot_hashes(directory)
            if before != initial_hashes:
                raise RuntimeError("immutable snapshot changed before C++ replay")
            try:
                completed = subprocess.run(command + ["--attribution", str(destination)],
                    input="\n".join(lines)+"\n", text=True, capture_output=True,
                    timeout=max(10., budget_s*(4+len(goals))+30.))
            except subprocess.TimeoutExpired:
                report = _inconclusive(directory, "INCONCLUSIVE_OFFLINE_BUDGET")
                report.update(input_sha256=before, snapshot_bytes_unchanged=before==_snapshot_hashes(directory))
                if output:
                    (destination/"report.json").write_text(json.dumps(report,indent=2)+"\n")
                return report
            (destination/"replay_input.txt").write_text("\n".join(lines)+"\n")
            (destination/"backend_report.json").write_text(completed.stdout)
            if completed.returncode:
                raise RuntimeError(completed.stderr.strip() or "C++ attribution failed")
            report = json.loads(completed.stdout)
            report["backend_stderr"] = completed.stderr
            if before != _snapshot_hashes(directory):
                raise RuntimeError("immutable snapshot bytes changed during attribution")
            report.update(snapshot=str(directory), input_sha256=before, snapshot_bytes_unchanged=True,
                generation=meta["generation"], planning_attempt_id=meta.get("planning_attempt_id"),
                scope="original production lattice, connectors, complete edge checks, frozen motion and full saved goal set; expanded lattice aligned by integer shifts",
                advisory="OFF; counterfactual geometry never grants execution authorization",
                backend_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
            if meta.get("run_manifest"):
                manifest_path = (directory/meta["run_manifest"]).resolve()
                if manifest_path.is_file():
                    source_manifest = json.loads(manifest_path.read_text())
                    report["source_run_manifest"] = {"path": str(manifest_path),
                        "sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                        "source": source_manifest.get("source")}
            if "observed" in report:
                report.update(_component_witness(meta, destination, report["observed"]))
                report.update(_boundary_evidence(directory, meta, destination, report["geometry_only"]["path_m"]))
                report["causes"] = []
                original, geometry, expanded = (report[k] for k in ("observed", "geometry_only", "observed_expanded"))
                if original["exhausted"] and not original["goals_reachable"]:
                    report["causes"].append("ORIGINAL_GOAL_SET_DISCONNECTED")
                    if geometry["goals_reachable"]:
                        report["causes"].append("OBSERVATION_AUTHORIZATION_BARRIER")
                    elif geometry["exhausted"]:
                        report["causes"].append("RECORDED_GEOMETRY_SUFFICIENT_TO_CUT_ORIGINAL_POOL")
                    if report["unthinned_evidence_available"] and report["unthinned_counterfactual"]["goals_reachable"]:
                        report["causes"].append("CURRENT_UNTHINNED_SUPPORT_LOSS_SUFFICIENT_TO_DISCONNECT_GOALS")
                    if expanded["goals_reachable"]:
                        report["causes"].append("ORIGINAL_POOL_TRUNCATES_OBSERVED_PATH")
                report["classification"] = ("ATTRIBUTED_WITH_WITNESSES" if all(
                    stage["exhausted"] or stage["goals_reachable"] for stage in (original, geometry, expanded))
                    else "PARTIAL_ATTRIBUTION_OFFLINE_BUDGET_OR_INVALID_ENDPOINT")
                report["goal_component_labels"] = {"-2": "ORIGINAL_TERMINAL_CONNECTOR_INELIGIBLE",
                    "-1": "UNRESOLVED", "0": "REAL_START_COMPONENT", "positive": "DISCONNECTED_COMPONENT_ID"}
                if plots and output:
                    _plots(directory, meta, destination, report)
                report["witness_files"] = sorted(path.name for path in destination.glob("*.csv")) if output else []
            if output:
                (destination/"report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
            return report
    if benchmark_repeats is not None:
        command.extend((str(benchmark_repeats), str(int(benchmark_diagnostics)), str(int(differential))))
        if full_epoch:
            command.append("1")
    try:
        completed = subprocess.run(
            command,
            input="\n".join(lines) + "\n", text=True, capture_output=True,
            check=False, timeout=max(10.0, budget_s * (1 + (benchmark_repeats or 0)) + 30.0))
    except subprocess.TimeoutExpired:
        return _inconclusive(directory, "INCONCLUSIVE_OFFLINE_BUDGET")
    if completed.returncode:
        raise RuntimeError(completed.stderr.strip() or "C++ replay failed")
    report = ({"samples": [json.loads(line) for line in completed.stdout.splitlines()],
               "schema_version": "iap_search_benchmark_v1"}
              if benchmark_repeats is not None else json.loads(completed.stdout))
    report.update(snapshot=str(directory), generation=meta["generation"],
                  online_failure=meta["search_failure"],
                  search_stage=meta.get("search_stage"),
                  required_clearance_m=meta.get("required_clearance_m"),
                  scope="original A* pool; observed physical map and saved motion; "
                        "minimum physical clearance only, guide fitting reserve and goal set are not replayed",
                  advisory="saved queried PL samples are diagnostic only")
    return report



def _snapshot_hashes(directory):
    return {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(directory.iterdir()) if path.is_file()}



def _component_witness(meta, destination, stage):
    """Export the complete outgoing cut, plus a checked forward path in the component."""
    node_path = destination/"observed_nodes.csv"
    if node_path.stat().st_size == len("x,y,z,parent_x,parent_y,parent_z\n"):
        (destination/"observed_cut.csv").write_text((destination/"observed_boundary.csv").read_text())
        return {"alternative_forward_target": None, "cut_complete": False, "cut_edges": 0}
    nodes = np.atleast_2d(np.loadtxt(node_path, delimiter=",", skiprows=1))
    center = np.array(meta["search_pool_center_m"])
    step = meta["search_step_size_m"]
    indices = np.rint((nodes[:,:3]-center)/step).astype(int)
    membership = {tuple(i): j for j, i in enumerate(indices)}
    cut_edges = 0
    cut_path = destination/"observed_cut.csv"
    with (destination/"observed_boundary.csv").open() as stream, cut_path.open("w") as output:
        reader = csv.DictReader(stream)
        writer = csv.DictWriter(output, reader.fieldnames)
        writer.writeheader()
        for row in reader:
            point = np.array([float(row["to_"+a]) for a in "xyz"])
            key = tuple(np.rint((point-center)/step).astype(int))
            if key not in membership:
                writer.writerow(row)
                cut_edges += 1
    start = np.array(meta["real_start_p_m"])
    goals = np.array(meta["planning_goals_m"])
    direction = goals[0]-start
    if np.linalg.norm(direction)<1e-12:
        return {"alternative_forward_target": None, "cut_complete": stage["exhausted"], "cut_edges": cut_edges}
    direction /= np.linalg.norm(direction)
    progress = (nodes[:,:3]-start)@direction
    distance = np.linalg.norm(nodes[:,:3]-goals[0], axis=1)
    eligible = progress > .2
    alternative = None
    if np.any(eligible):
        j = int(np.argmin(np.where(eligible, distance, np.inf)))
        target_j = j
        path = []
        visited = set()
        while j not in visited:
            visited.add(j)
            path.append(nodes[j,:3].tolist())
            parent_key = tuple(np.rint((nodes[j,3:]-center)/step).astype(int))
            if parent_key == tuple(indices[j]):
                break
            j = membership[parent_key]
        path = [start.tolist()]+list(reversed(path))
        alternative = {"target_m": nodes[target_j,:3].tolist(),
            "progress_along_saved_goal_direction_m": float(progress[target_j]),
            "distance_from_original_first_goal_m": float(distance[target_j]),
            "path_m": path, "execution_authorized": False,
            "scope": "production edge/connectivity witness; direction is saved start-to-goal, full FSM reference/terminal velocity qualification still required"}
        (destination/"alternative_forward_path.json").write_text(json.dumps(alternative,indent=2)+"\n")
    return {"alternative_forward_target": alternative, "cut_complete": stage["exhausted"], "cut_edges": cut_edges}

def _boundary_evidence(directory, meta, destination, geometry_path=()):
    """Bind unknown cut edges to raw, same-scan evidence without changing authority."""
    origin = np.asarray(meta["origin_m"], dtype=float)
    resolution = float(meta["resolution_m"])
    dims = tuple(meta["dimensions"])
    unique = {}
    counts = Counter()
    with (destination/"observed_cut.csv").open() as stream:
        for row in csv.DictReader(stream):
            counts[row["reason"]] += 1
            if row["reason"] != "ENVIRONMENT_UNOBSERVED":
                continue
            p = np.array([float(row["sample_"+a]) for a in "xyz"])
            index = tuple(np.floor((p-origin)*(1./resolution)).astype(int))
            if all(0 <= i < d for i, d in zip(index, dims)):
                if index not in unique:
                    unique[index] = {"voxel_index": list(map(int, index)), "sample_m": p.tolist(),
                        "edge_from_m": [float(row["from_"+a]) for a in "xyz"],
                        "edge_to_m": [float(row["to_"+a]) for a in "xyz"], "incident_edges": 0}
                unique[index]["incident_edges"] += 1
    sources = None
    if meta.get("observation_evidence_available"):
        sources = np.fromfile(directory/meta["observation_sources_file"], dtype=np.uint8).reshape(dims)
    frame = meta.get("current_frame")
    beams = hits = None
    fresh = False
    if frame:
        age = meta["planning_time_s"]-frame.get("scan_end_stamp_s", frame["stamp_s"])
        fresh = 0 <= age <= meta["environment_max_age_s"]
        beams = np.genfromtxt(directory/frame["beams_file"], delimiter=",", names=True)
        hits = np.genfromtxt(directory/frame["hits_file"], delimiter=",", names=True)
        beams = np.atleast_1d(beams)
        direction = np.column_stack([beams["map_d"+a] for a in "xyz"])
        direction /= np.linalg.norm(direction, axis=1)[:, None]
        sensor = np.array(frame["sensor_position_m"])
        ranges = np.where(beams["outcome"] == 1, beams["range_m"], frame["max_range_m"])
    classifications = Counter()
    for index, record in unique.items():
        flags = int(sources[index]) if sources is not None else None
        record["source_flags"] = flags
        record["execution_authorized"] = False
        record["loss_producer"] = ({0: None, 1: "current_replace", 2: "active_delta", 3: "active_replace"}
                                   [(flags & 48) >> 4] if flags is not None else None)
        record["loss_producer_time_known"] = False
        record["unthinned_current_support"] = bool(flags is not None and flags & 128)
        record["classification"] = "MISSING_CURRENT_FRAME_EVIDENCE"
        record["beam_witnesses"] = []
        if frame and fresh and frame.get("beam_evidence_complete"):
            # Slab intersection gives physical first-hit occlusion evidence. The
            # stored unthinned mask remains the registered producer's traversal fact.
            lower = origin+np.array(index)*resolution
            upper = lower+resolution
            with np.errstate(divide="ignore", invalid="ignore"):
                first = (lower-sensor)/direction
                last = (upper-sensor)/direction
            enter = np.maximum(np.min(np.stack((first,last)),axis=0).max(axis=1), 0.)
            leave = np.max(np.stack((first,last)),axis=0).min(axis=1)
            intersects = (leave > enter+1e-10) & (beams["outcome"] != 0) & (enter < frame["max_range_m"])
            covered = np.flatnonzero(intersects & (enter < ranges-1e-8))
            occluded = np.flatnonzero(intersects & (enter >= ranges) & (beams["outcome"] == 1))
            for row_id in list(covered[:2])+list(occluded[:2]):
                row_id = int(row_id)
                endpoint = sensor+direction[row_id]*ranges[row_id]
                hit_distance = None
                if beams["outcome"][row_id] == 1 and hits.size:
                    hit_points = np.column_stack([hits["map_"+a] for a in "xyz"])
                    hit_distance = float(np.linalg.norm(hit_points-endpoint,axis=1).min())
                record["beam_witnesses"].append({"csv_data_row": row_id+1,
                    "outcome": int(beams["outcome"][row_id]), "range_m": float(ranges[row_id]),
                    "voxel_entry_range_m": float(enter[row_id]), "voxel_exit_range_m": float(leave[row_id]),
                    "endpoint_m": endpoint.tolist(), "nearest_saved_hit_distance_m": hit_distance,
                    "relation": "PREFIX_INTERSECTION" if row_id in covered else "BEHIND_FIRST_HIT"})
            record["prefix_intersection_beams"] = len(covered)
            record["occluded_beams"] = len(occluded)
            record["classification"] = ("OBSERVATION_MASK_INCONSISTENT" if flags is not None and flags & 15
                else "PROCESSING_SUPPORT_LOSS" if flags is not None and flags & 128
                else "FIRST_HIT_OCCLUSION" if len(occluded) and not len(covered)
                else "COVERAGE_OR_TRAVERSAL_GAP" if len(covered) else "NO_RECORDED_BEAM_COVERAGE")
        elif frame and not fresh:
            record["classification"] = "STALE_CURRENT_FRAME_EVIDENCE"
        classifications[record["classification"]] += 1
    (destination/"unknown_boundary_evidence.json").write_text(json.dumps({
        "scope": "all unique unknown samples on reachable outgoing edges; historical loss is not current coverage",
        "frame": frame, "records": list(unique.values())}, indent=2, allow_nan=False)+"\n")
    ordered = sorted(unique.values(), key=lambda r: (not r["unthinned_current_support"], -r["incident_edges"]))
    key_evidence = ordered[:12]
    for classification in classifications:
        representative = next(record for record in ordered if record["classification"] == classification)
        if representative not in key_evidence:
            key_evidence.append(representative)
    path_boundary = []
    for point in geometry_path:
        index = tuple(np.floor((np.asarray(point)-origin)*(1./resolution)).astype(int))
        if index in unique:
            path_boundary.append(unique[index])
    for record in path_boundary[:3]:
        if record not in key_evidence:
            key_evidence.append(record)
    return {"boundary_reasons": dict(counts), "unknown_boundary_voxels": len(unique),
            "unknown_boundary_classifications": dict(classifications),
            "geometry_path_boundary_evidence": path_boundary,
            "key_ray_evidence": key_evidence}


def _plots(directory, meta, destination, report):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    _, _, cells = load_snapshot(directory)
    nodes = np.atleast_2d(np.loadtxt(destination/"observed_nodes.csv",delimiter=",",skiprows=1))
    origin = np.array(meta["origin_m"])
    center = np.array(meta["search_pool_center_m"])
    resolution = meta["resolution_m"]
    start = np.array(meta["real_start_p_m"])
    goals = np.array(meta["planning_goals_m"])
    fig, axes = plt.subplots(1,3,figsize=(17,5.5),constrained_layout=True)
    for ax,z in zip(axes,(.85,1.55,2.35)):
        k = int(np.clip(np.floor((z-origin[2])/resolution),0,cells.shape[2]-1))
        plane = cells[:,:,k]
        labels = np.where(plane&4,1,0)
        labels = np.where(plane&2,2,labels)
        labels = np.where(plane&1,3,labels)
        ax.imshow(labels.T,origin="lower",extent=[origin[0],meta["max_boundary_m"][0],origin[1],meta["max_boundary_m"][1]],
                  cmap=ListedColormap(["#bdbdbd","#ffffff","#797979","#181818"]),vmin=0,vmax=3,interpolation="nearest")
        if nodes.size:
            layer = nodes[np.abs(nodes[:,2]-z)<meta["search_step_size_m"]/2,:3]
            ax.scatter(layer[:,0],layer[:,1],s=.25,color="#64c7dd",alpha=.7,label="real-start component")
        ax.scatter(goals[:,0],goals[:,1],marker="x",c="red",s=35,label="saved goals")
        ax.scatter(start[0],start[1],c="blue",s=45,label="real start")
        path = np.array(report["geometry_only"]["path_m"])
        if len(path):ax.plot(path[:,0],path[:,1],"--",c="orange",label="geometry path: UNAUTHORIZED")
        alternative = report.get("alternative_forward_target")
        if alternative:
            path = np.array(alternative["path_m"])
            ax.plot(path[:,0],path[:,1],c="#004577",lw=1.2,label="observed forward witness")
        ax.set(xlim=(center[0]-5.1,center[0]+5.1),ylim=(center[1]-5.1,center[1]+5.1),
               xlabel="map X (m)",ylabel="map Y (m)",title=f"voxel slice z={z:.2f} m")
        ax.set_aspect("equal")
    axes[0].legend(fontsize=7,loc="upper left")
    fig.suptitle(f"attempt {meta.get('planning_attempt_id')} / generation {meta['generation']}: unknown grey; recorded obstacle black")
    fig.savefig(destination/"components_barrier.png",dpi=180)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,6),constrained_layout=True)
    frame=meta.get("current_frame")
    if frame:
        sensor=np.array(frame["sensor_position_m"])
        ax.scatter(sensor[0],sensor[1],c="blue",label="same-scan sensor")
        representatives=[]
        seen=set()
        for record in report["key_ray_evidence"]:
            cls=record["classification"]
            if cls not in seen and record["beam_witnesses"]:
                representatives.append(record); seen.add(cls)
        if not representatives and report["key_ray_evidence"]:
            representatives=report["key_ray_evidence"][:1]
        for record in representatives:
            p=np.array(record["sample_m"])
            ax.scatter(p[0],p[1],marker="x",s=50,label=record["classification"])
            for beam in record["beam_witnesses"][:1]:
                endpoint=np.array(beam["endpoint_m"])
                ax.plot([sensor[0],endpoint[0]],[sensor[1],endpoint[1]],lw=1.5,label=f"beam row {beam['csv_data_row']} / {beam['relation']}")
    ax.set(xlabel="map X (m)",ylabel="map Y (m)",title="Raw same-frame ray witnesses (XY projection; 3D evidence in JSON)")
    ax.legend(fontsize=8);ax.set_aspect("equal");ax.grid(alpha=.2)
    fig.savefig(destination/"key_rays.png",dpi=180);plt.close(fig)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--budget-s", type=float, default=120.0)
    parser.add_argument("--attribution", action="store_true")
    parser.add_argument("--output", type=Path, help="existing run export/analysis directory")
    parser.add_argument("--plots", action="store_true", help="export component/barrier and raw ray PNGs")
    args = parser.parse_args()
    if args.attribution and args.output is None:
        parser.error("--attribution requires --output below a resolved run")
    report = inspect(args.snapshot, args.budget_s, attribution=args.attribution, output=args.output, plots=args.plots)
    print(json.dumps(report, indent=2))
    directory = args.snapshot.resolve()
    if not args.attribution and directory.parents[2].name == "export":
        run = directory.parents[3]
        destination = run / "export" / "analysis" / (
            "failure_map_" + directory.name + ".json")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
