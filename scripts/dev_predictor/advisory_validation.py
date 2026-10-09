#!/usr/bin/env python3
"""Record full read-only inputs, replay the production Predictor, report evidence."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "launch/_includes"))
from run_directory import (adopt_run_directory, resolve_run_directory, finalize_run,
                           write_subordinate_manifest, canonical_run_uses_sim_time)

SCENARIO = "icra_dense_forest_four_fork_v2"
CODEC = "iap_prediction_input_v9/boost_binary/zlib_length_u64le"
READABLE_CODECS={CODEC,"iap_prediction_input_v8/boost_binary/zlib_length_u64le"}
SOURCES = ["src/iap/planner/plan_manage/include/ego_planner/prediction_input.h",
           "src/iap/planner/plan_manage/src/prediction_input.cpp",
           "src/iap/planner/plan_manage/src/prediction_input_codec.cpp",
           "src/iap/planner/plan_manage/src/planner_risk.cpp",
           "src/iap/planner/plan_manage/src/advisory_validation.cpp"]
SOURCES += [str(p.relative_to(REPO)) for p in sorted((REPO / "src/iap/predictor").glob("*.cpp"))]
SOURCES += [str(p.relative_to(REPO)) for p in sorted((REPO / "include/iap/predictor").glob("*.hpp"))]
SOURCES += ["src/iap/gnss/visibility_predictor.cpp", "include/iap/gnss/visibility_predictor.hpp",
            "src/iap/planner/plan_env/src/grid_map.cpp", "src/iap/planner/plan_env/src/grid_map_risk.cpp",
            "src/iap/planner/plan_env/include/plan_env/grid_map.h",
            "src/iap/planner/plan_env/src/local_evidence_snapshot.cpp",
            "src/iap/planner/plan_env/include/plan_env/local_evidence_snapshot.h"]
SOURCES += ["include/iap/gnss/gnss_types.hpp", "include/iap/gnss/gnss_epoch_wire.hpp", "msg/AdvisoryGnssObservation.msg", "include/iap/planner/integrity_snapshot.hpp",
            "include/iap/util/shared_state.hpp", "msg/IntegrityReport.msg",
            "src/iap/gnss/gnss_extension.cpp", "src/iap/integrity/integrity_extension.cpp"]
SOURCES += ["scripts/dev_predictor/advisory_validation.py", "scripts/dev_predictor/compare_advisory_error.py",
            "scripts/dev_predictor/advisory_coordinate_evidence.py", "scripts/dev_predictor/advisory_live_capture.py"]
SOURCES += ["include/iap/gnss/postopt_evidence.hpp", "msg/GnssPostoptEvidence.msg",
            "include/iap/gnss/broadcast_ephemeris.hpp",
            "include/iap/gnss/constellation_clock.hpp", "src/iap/gnss/gnss_handler.cpp",
            "include/iap/gnss/gnss_handler.hpp", "include/iap/gnss/gnss_extension.hpp"]
SOURCES += ["src/iap/odometry/odometry_estimation_imu.cpp",
            "include/iap/gnss/clock_geometry.hpp", "src/iap/integrity/araim.cpp",
            "include/iap/integrity/araim.hpp", "include/iap/integrity/araim_types.hpp",
            "src/iap/integrity/integrity_monitor.cpp"]
SOURCES += ["src/uav_simulator/gnss_sim/src/gnss_sim_node.cpp",
            "launch/_includes/simulation_environment.launch.py",
            "scripts/dev_planner/run_curve_channel_live.py",
            "src/iap/odometry/odometry_estimation_gpu.cpp",
            "src/iap/gnss/pseudorange_factor.cpp","include/iap/gnss/pseudorange_factor.hpp",
            "src/iap/gnss/doppler_factor.cpp","include/iap/gnss/doppler_factor.hpp",
            "src/iap/common/imu_integration.cpp","include/iap/common/imu_integration.hpp",
            "scripts/dev_predictor/epoch_motion_reference.py"]
SOURCES += ["srv/GetGridMapPredictionInput.srv",
            "scripts/dev_predictor/fusion_scientific_audit.py",
            "scripts/dev_predictor/fusion_actual_error_audit.py",
            "scripts/dev_predictor/check_fusion_fixed_route.py",
            "scripts/dev_predictor/recheck_fusion_route.py",
            "src/iap/planner/plan_manage/src/failure_map_replay.cpp"]
SOURCES += [str(p.relative_to(REPO)) for directory in
            (REPO/"include/iap/odometry/gpu_evidence",REPO/"src/iap/odometry/gpu_evidence")
            for p in sorted(directory.glob("*")) if p.is_file()]


def git(*args):
    return subprocess.check_output(["git", "-C", str(REPO), *args], text=True).strip()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe_label(label):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", label):
        raise ValueError("label must be one safe path component")
    return label


def artifact(run, relative):
    p = Path(relative)
    if p.is_absolute() or ".." in p.parts or not p.parts or p.parts[0] not in (
            "runtime", "profiling", "export", "metadata"):
        raise ValueError("invalid artifact path")
    result = (run / p).resolve()
    result.relative_to(run.resolve())
    result.parent.mkdir(parents=True, exist_ok=True)
    return result


def json_write(path, value):
    with path.open("x", encoding="utf-8") as out:
        json.dump(value, out, indent=2, allow_nan=False, ensure_ascii=False)
        out.write("\n")


def source_identity():
    return {"revision": git("rev-parse", "HEAD"), "dirty": git("status", "--porcelain"),
            "source_sha256": {p: sha(REPO / p) for p in SOURCES}}


def model_audit(run, dataset):
    name = "advisory_model_audit_" + safe_label(dataset or "report")
    path = artifact(run, "metadata/manifests/" + name + ".json")
    if path.exists(): return path
    seams = [("src/iap/planner/plan_manage/src/planner_risk.cpp", "Advisory approximation", 13),
             ("src/iap/integrity/fgo_information_manager.cpp", "marginalCovariance(X(frame_id))", 28),
             ("src/iap/integrity/fgo_information_manager.cpp", "smoother.getFactors()", 30),
             ("src/iap/integrity/integrity_monitor.cpp", "report.current_motion_error_proxy_m =", 4),
             ("config/sim_ego/config_gnss.json", '"K_pl":', 1),
             ("src/iap/predictor/fusion_advisory_predictor.cpp", "out.joint_pose_information=(gnss_pose+lidar_pose)", 9),
             ("src/iap/sim/sim_extension.cpp", "T_truth_est_ = T_truth * T_est.inverse()", 22)]
    evidence = []
    for file, needle, count in seams:
        lines = (REPO / file).read_text().splitlines()
        start = next(i for i, line in enumerate(lines) if needle in line)
        evidence.append({"file": file, "sha256": sha(REPO / file), "line": start+1,
                         "excerpt": "\n".join(lines[start:start+count])})
    manifest(run, name, {**source_identity(), "evidence": evidence,
             "observed": "canonical posterior proxy is disabled; pose constraints join before attitude marginalization with a Cauchy cross-source covariance bound",
             "unresolved": "map/alignment/extrinsic uncertainty, within-source correlation, state-time propagation, real noise calibration and fused faults remain unqualified",
             "production_model_changed": False})
    return path


def validate_record(payload, sidecar):
    info = json.loads(sidecar.read_text())
    if info.get("codec") not in READABLE_CODECS or info.get("identity") != "REAL_FROZEN":
        raise ValueError("complete real recording identity required; failure_map/fixture cannot substitute")
    if info["payload_sha256"] != sha(payload):
        raise ValueError("payload checksum mismatch")
    current = source_identity()
    # Documentation/report commits may advance HEAD without changing the model.
    # The recording retains its clean revision; source and producer hashes bind
    # replay behavior. A different production source is still rejected.
    try:
        git("cat-file", "-e", info["revision"] + "^{commit}")
    except subprocess.CalledProcessError as error:
        raise ValueError("cross-version input replay rejected") from error
    if info["source_sha256"] != current["source_sha256"]:
        raise ValueError("cross-version input replay rejected")
    if info.get("dirty"):
        raise ValueError("recording is not bound to a clean live revision")
    return info


def manifest(run, name, value, owner=False):
    value = {"schema": "iap_advisory_validation_manifest_v1", **value}
    if owner:
        write_subordinate_manifest(run, name, value)
    else:
        json_write(artifact(run, "metadata/manifests/" + safe_label(name) + ".json"), value)


def binary_identity(binary):
    libs = {}
    output = subprocess.check_output(["ldd", str(binary)], text=True)
    for line in output.splitlines():
        fields = line.split()
        for field in fields:
            if field.startswith("/") and Path(field).is_file():
                libs[field] = sha(field)
    import platform
    flags = binary.parent / "CMakeFiles" / (binary.name + ".dir/flags.make")
    return {"path": str(binary), "sha256": sha(binary), "libraries_sha256": libs,
            "machine": platform.machine(), "byteorder": sys.byteorder,
            "compile_flags": flags.read_text() if flags.exists() else None}


def installed_build_identity(installed, built):
    """Permit byte identity or exactly CMake's removal of an ELF64 build RPATH."""
    import struct
    installed, built = Path(installed), Path(built)
    source, actual = built.read_bytes(), installed.read_bytes()
    kind = "exact_bytes"
    if source != actual:
        if source[:6] != b"\x7fELF\x02\x01" or len(source) < 64:
            raise ValueError("installed binary differs from workspace build")
        offset = struct.unpack_from("<Q", source, 40)[0]
        entry_size, count, strings_index = struct.unpack_from("<HHH", source, 58)
        if entry_size != 64 or strings_index >= count or offset + count * entry_size > len(source):
            raise ValueError("invalid build ELF section table")
        headers = [struct.unpack_from("<IIQQQQIIQQ", source, offset + i * entry_size) for i in range(count)]
        string_header = headers[strings_index]
        names = source[string_header[4]:string_header[4] + string_header[5]]
        sections = {names[h[0]:].split(b"\0", 1)[0]: h for h in headers}
        dynamic, strings = sections[b".dynamic"], sections[b".dynstr"]
        if dynamic[5] % 16 or dynamic[4] + dynamic[5] > len(source) or strings[4] + strings[5] > len(source):
            raise ValueError("invalid build ELF dynamic sections")
        entries = list(struct.iter_unpack("<QQ", source[dynamic[4]:dynamic[4] + dynamic[5]]))
        paths = [value for tag, value in entries if tag in (15, 29)]
        if not paths:
            raise ValueError("installed binary differs beyond an RPATH removal")
        expected = bytearray(source)
        for path in paths:
            if path >= strings[5]:
                raise ValueError("invalid build ELF RPATH offset")
            start = strings[4] + path
            end = source.index(b"\0", start, strings[4] + strings[5])
            expected[start:end + 1] = bytes(end + 1 - start)
        remaining = [entry for entry in entries if entry[0] not in (15, 29)]
        remaining += [(0, 0)] * (len(entries) - len(remaining))
        expected[dynamic[4]:dynamic[4] + dynamic[5]] = b"".join(struct.pack("<QQ", *entry) for entry in remaining)
        if actual != expected:
            raise ValueError("installed binary differs beyond an RPATH removal")
        kind = "cmake_build_rpath_removed"
    return {"match": kind, "workspace_release_binary": str(built),
            "workspace_release_sha256": sha(built), "installed_sha256": sha(installed)}


def backend(run, binary, args, name):
    stem = "runtime/ros/advisory_" + safe_label(name)
    attempt = 0
    output = artifact(run, stem + ".log")
    while output.exists():
        attempt += 1
        output = artifact(run, stem + f"_{attempt:03}.log")
    with output.open("x") as out:
        subprocess.run([str(binary), *args], env={**os.environ, "IAP_RUN_DIR": str(run),
                       "ROS_LOG_DIR": str(run / "runtime/ros")}, stdout=out,
                       stderr=subprocess.STDOUT, check=True)


def save_record(run, label, payload, identity, service_identity):
    directory = artifact(run, "export/advisory/validation/recordings/" + safe_label(label) + "/input.bin").parent
    path = directory / "input.bin"
    with path.open("xb") as out:
        out.write(bytes(payload))
    info = {**identity, **service_identity, "identity": "REAL_FROZEN", "codec": CODEC,
            "payload_sha256": sha(path), "payload_bytes": path.stat().st_size,
            "payload": str(path.relative_to(run)), "time_policy": "preserve saved reference time"}
    json_write(directory / "record.json", info)
    return path


def record(run, args):
    # A recorder adopts the canonical launch; it never launches sensors or a map.
    primary = json.loads((run / "metadata/run_manifest.json").read_text())
    identity = source_identity()
    if (identity["dirty"] or primary.get("source", {}).get("git_commit") != identity["revision"] or
            not primary.get("source", {}).get("git_worktree_clean")):
        raise RuntimeError("LIVE_BLOCKED_BY_UNRELATED_DIRTY_WORKTREE or revision mismatch")
    if primary.get("entrypoint") != "iap_sim" or primary.get("scenario") != SCENARIO:
        raise RuntimeError("record requires the canonical iap_sim four-fork run")
    from ament_index_python.packages import get_package_prefix
    producer = args.producer_binary or (Path(get_package_prefix("ego_planner")) / "lib/ego_planner/ego_planner_node")
    identity["producer_binary"] = binary_identity(producer.resolve())
    import rclpy
    from iap.srv import GetGridMapPredictionInput
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=[])
    from rclpy.parameter import Parameter
    node = rclpy.create_node("advisory_validation_recorder", parameter_overrides=[
        Parameter("use_sim_time", value=canonical_run_uses_sim_time(run))])
    client = node.create_client(GetGridMapPredictionInput, args.service)
    requests = artifact(run, "export/advisory/validation/recordings/" + args.label + "_requests.csv")
    odometry = artifact(run, "export/advisory/validation/recordings/" + args.label + "_odometry.csv")
    start = time.monotonic()
    request_ids=[]
    try:
        with requests.open("x") as req, odometry.open("x") as odom:
            w = csv.writer(req); w.writerow(["request", "request_id", "run_id", "available", "reason", "elapsed_s", "payload"])
            ow = csv.writer(odom); ow.writerow(["source", "stamp", "frame", "body", "x", "y", "z", "qx", "qy", "qz", "qw"])
            def on_odom(msg, source):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                ow.writerow([source, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             msg.header.frame_id, msg.child_frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
                odom.flush()
            subscriptions = [node.create_subscription(Odometry, topic, lambda msg, s=source: on_odom(msg, s), 100)
                             for source, topic in (("glio", args.glio_topic), ("truth", args.truth_topic))]
            for i in range(args.count):
                request_id=f"{run.name}:{args.label}:{i}"
                request_ids.append(request_id)
                stamp = time.monotonic()
                payload_path = ""
                if not client.wait_for_service(timeout_sec=args.timeout):
                    available, reason = False, "input_service_unavailable"
                else:
                    request=GetGridMapPredictionInput.Request()
                    request.planning_input=args.planning_input
                    request.planning_attempt_id=args.planning_attempt_id
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
                    response = future.result() if future.done() else None
                    available = bool(response and response.available)
                    reason = response.reason if response else "input_service_timeout"
                    if available:
                        path = save_record(run, f"{args.label}_{i:04}", response.payload, identity,
                                           {"frame_id": response.frame_id, "geometry_id": response.geometry_id,
                                            "generation": response.generation, "service": args.service,
                                            "request_id":request_id,"planning_input":args.planning_input,
                                            "planning_attempt_id":response.planning_attempt_id,
                                            "risk_version":response.risk_version})
                        payload_path = str(path.relative_to(run))
                w.writerow([i, request_id, run.name, available, reason, time.monotonic() - stamp, payload_path]); req.flush()
                end = time.monotonic() + args.interval
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=min(.1, end - time.monotonic()))
            del subscriptions
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Intentional owned stop still closes and registers evidence.
    finally:
        node.destroy_node(); rclpy.try_shutdown()
        request_manifest=artifact(run,"export/advisory/validation/recordings/"+args.label+"_requests_manifest.json")
        json_write(request_manifest,{"identity":"LIVE_MEASUREMENT","run_id":run.name,"request_ids":request_ids,
                   "requests_csv":str(requests.relative_to(run)),"requests_csv_sha256":sha(requests)})
        manifest(run, "advisory_record_" + args.label, {**identity, "duration_s": time.monotonic()-start,
                 "artifacts_sha256":{str(p.relative_to(run)):sha(p) for p in (requests,odometry,request_manifest)}})


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def preflight(run):
    import ctypes
    from datetime import datetime, timezone
    result = {**source_identity(), "checked_at_utc": datetime.now(timezone.utc).isoformat(),
              "entrypoint": "iap_sim.launch.py", "scenario": SCENARIO, "live_started": False}
    try:
        nv = subprocess.run(["nvidia-smi"], capture_output=True, text=True)
        with artifact(run, "runtime/ros/advisory_nvidia_smi.log").open("x") as out:
            out.write(nv.stdout + nv.stderr)
        result["nvidia_smi_exit"] = nv.returncode
    except OSError as e:
        result.update(nvidia_smi_exit=None, nvidia_smi_reason=str(e))
    try:
        cuda = ctypes.CDLL("libcuda.so.1")
        result["cuInit"] = cuda.cuInit(0)
        count = ctypes.c_int()
        result["cuDeviceGetCount"] = cuda.cuDeviceGetCount(ctypes.byref(count)) if result["cuInit"] == 0 else None
        result["cuda_device_count"] = count.value
    except OSError as e:
        result.update(cuInit=None, cuda_device_count=0, cuda_reason=str(e))
    result["gpu_status"] = "READY" if result.get("nvidia_smi_exit") == 0 and result.get("cuInit") == 0 and result.get("cuDeviceGetCount") == 0 and result["cuda_device_count"] > 0 else "GPU_NOT_READY"
    result["live_status"] = "LIVE_BLOCKED_BY_UNRELATED_DIRTY_WORKTREE" if result["dirty"] else "GPU_NOT_READY" if result["gpu_status"] != "READY" else "READY"
    manifest(run, "advisory_preflight", result)
    return result


def numeric(row, key):
    import math
    try:
        value = float(row[key])
        return value if math.isfinite(value) else float("nan")
    except (ValueError, KeyError):
        return float("nan")


def summary_for(run, dataset=None):
    import math
    data_root = run / "export/advisory/validation"
    if dataset: data_root = data_root / safe_label(dataset)
    tables = {p.parent.name: rows(p) for p in sorted(data_root.glob("*/points.csv"))}
    all_rows = [r for table in tables.values() for r in table]
    real = [r for r in all_rows if r["identity"] == "REAL_REPLAY"]
    synthetic = [r for r in all_rows if r["identity"] == "SYNTHETIC_MECHANISM"]
    availability = {}
    for label, table in tables.items():
        reasons = {}
        for r in table:
            key = r["status"] + ":" + r["reason"]
            reasons[key] = reasons.get(key, 0) + 1
        availability[label] = {"requested": len(table), "valid": sum(r["valid"] == "1" for r in table),
                               "wrapper_calls": sum(r["wrapper_called"] == "1" for r in table), "reasons": reasons}
    mechanism = {}
    for group in ("S1", "S2", "S3", "S4", "S3_no_prior"):
        series = [tables.get(f"{group}_{i}", [{}])[0] for i in range(3)]
        h, v = ([numeric(r, key) for r in series] for key in ("fused_hpl", "fused_vpl"))
        finite = all(math.isfinite(x) for x in h+v)
        monotone = finite and all(a <= b + 1e-12 for values in (h, v) for a, b in zip(values, values[1:]))
        mechanism[group] = {"hpl": [x if math.isfinite(x) else None for x in h],
                            "vpl": [x if math.isfinite(x) else None for x in v],
                            "status": "PASS" if monotone else "FAIL" if finite else "INCONCLUSIVE_SOURCE_UNAVAILABLE"}
    ratios = {}
    for key in ("hpl", "vpl"):
        a, b = mechanism["S3"][key], mechanism["S3_no_prior"][key]
        ratios[key] = (a[-1]-a[0])/(b[-1]-b[0]) if None not in a+b and abs(b[-1]-b[0]) > 1e-12 else None
    gate = tables.get("S5_missing_gnss", [{}])[0]
    gate_failure = gate.get("wrapper_bound") == "0" and gate.get("module_valid") == "1"
    spatial = {}
    for label, table in tables.items():
        if len(table) <= 1: continue
        spatial[label] = {}
        for source in ("gnss", "lidar", "prior", "fused"):
            for metric in ("hpl", "vpl"):
                values = sorted(numeric(r, source+"_"+metric) for r in table if math.isfinite(numeric(r, source+"_"+metric)))
                spatial[label][source+"_"+metric] = {"valid": len(values), "min": min(values) if values else None,
                    "max": max(values) if values else None, "range_m": max(values)-min(values) if values else None,
                    "range_relative_to_min": (max(values)-min(values))/min(values) if values and min(values)>0 else None}
    return {"schema": "iap_advisory_validation_summary_v1", "scenario": SCENARIO,
            "mechanism_identity": "SYNTHETIC_MECHANISM",
            "input_availability": "INCONCLUSIVE_INPUT_UNAVAILABLE" if not real else "INCONCLUSIVE_INPUT_REVIEW_REQUIRED",
            "spatial_sensitivity": "INCONCLUSIVE_INPUT_UNAVAILABLE" if not real else "INCONCLUSIVE_SPATIAL_CONTRAST",
            "actual_error_conformity": "INCONCLUSIVE_LIVE_NOT_RUN",
            "live_status": "LIVE_BLOCKED_BY_UNRELATED_DIRTY_WORKTREE" if git("status", "--porcelain") else "NOT_RUN",
            "real_requested": len(real), "synthetic_requested": len(synthetic), "availability": availability,
            "mechanism": mechanism, "signal_retention_ratio_diagnostic": ratios,
            "spatial_ranges": spatial,
            "source_admission_discrepancy_reproduced": gate_failure,
            "prior_observation_correlation": "UNRESOLVED_POSTERIOR_PLUS_REUSED_OBSERVATIONS",
            "independent_live_runs": 0, "tables": tables}


def report(run, dataset=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import Normalize
    summary = summary_for(run, dataset)
    tables = summary.pop("tables")
    prefix = "export/analysis/advisory_validation/" + (safe_label(dataset) + "/" if dataset else "")
    out = artifact(run, prefix + "report.md").parent
    if (out / "summary.json").exists():
        raise FileExistsError("report evidence already exists")
    audit = model_audit(run, dataset)
    identity = "SYNTHETIC MECHANISM; real forest replay / live errors NOT RUN" if summary["real_requested"] == 0 else "Real replay and synthetic mechanism shown separately"
    def save(fig, name, detail=""):
        fig.suptitle(identity + ("\n"+detail if detail else ""), fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, .96)); fig.savefig(out / name, dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(13, 11))
    labels = list(tables)
    axes[0].bar(labels, [sum(r["valid"] == "1" for r in tables[k])/len(tables[k]) for k in labels])
    axes[0].set_ylabel("valid / requested"); axes[0].tick_params(axis="x", rotation=90)
    for key in ("cloud_stamp_s", "pose_stamp_s", "current_stamp_s", "gnss_stamp_s"):
        axes[1].plot(labels, [numeric(tables[k][0], "reference_time_s")-numeric(tables[k][0], key) for k in labels], ".", label=key)
    axes[1].set_ylabel("saved reference - source stamp (s)"); axes[1].legend(); axes[1].tick_params(axis="x", rotation=90)
    invalid = [r for t in tables.values() for r in t if r["valid"] != "1"]
    reasons = sorted({r["status"] + ":" + r["reason"] for r in invalid})
    axes[2].barh(reasons, [sum(r["status"]+":"+r["reason"] == reason for r in invalid) for reason in reasons])
    axes[2].set_xlabel("requests (missing is never zero PL)")
    save(fig, "input_availability.png")
    # Fixed online physical scales are shared across sources, never autoscaled.
    scans = [(k, t) for k, t in tables.items() if len(t) > 1]
    for label, table in scans:
        fig, axes = plt.subplots(2, 4, figsize=(15, 7), sharex=True, sharey=True)
        for j, source in enumerate(("gnss", "lidar", "prior", "fused")):
            for i, metric in enumerate(("hpl", "vpl")):
                values = np.array([numeric(r, source+"_"+metric) for r in table])
                x = [numeric(r, "x") for r in table]; y = [numeric(r, "y") for r in table]
                norm = Normalize(.25 if i == 0 else .20, .65 if i == 0 else .55)
                plot = axes[i, j].scatter(x, y, c=np.ma.masked_invalid(values), cmap="viridis", norm=norm, marker="s", s=32)
                for px, py, value in zip(x, y, values):
                    if not np.isfinite(value): axes[i, j].scatter(px, py, marker="x", c="gray", s=32)
                axes[i, j].set_title(f"{source} {metric.upper()} m"); axes[i, j].set_aspect("equal")
                fig.colorbar(plot, ax=axes[i, j], extend="both")
        detail = f"{label}: ref={table[0]['reference_time_s']} s; generation={table[0]['generation']}; fused valid={sum(r['valid']=='1' for r in table)}/{len(table)}; 0.1 m voxels"
        save(fig, f"spatial_layers_{label}.png", detail)
        # Distribution plot gives raw values even when all are below display floor.
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for i, metric in enumerate(("hpl", "vpl")):
            for source in ("gnss", "lidar", "prior", "fused"):
                values = sorted(v for r in table if np.isfinite(v := numeric(r, source+"_"+metric)))
                if values: axes[i].plot(values, label=source)
            axes[i].set_ylabel(metric.upper()+" (m)"); axes[i].set_xlabel("ordered valid values"); axes[i].legend()
        save(fig, f"spatial_distribution_{label}.png", detail)
    for name, groups, xlabel in (("degradation_response.png", ("S1", "S2", "S3", "S3_no_prior"), "noise sigma multiplier"),
                                 ("prior_ablation.png", ("S4",), "diagnostic prior alpha")):
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for group in groups:
            for i, key in enumerate(("hpl", "vpl")):
                values = summary["mechanism"][group][key]
                axes[i].plot([1, .1, 0] if group == "S4" else [1, 10, 100], [np.nan if v is None else v for v in values], "o-", label=group)
                axes[i].set_ylabel(key.upper()+" (m)"); axes[i].set_xlabel(xlabel); axes[i].legend()
                if group != "S4": axes[i].set_xscale("log")
        save(fig, name)
    fig, ax = plt.subplots(figsize=(10, 4))
    error_csv = run / "export/advisory/validation/error_requests.csv"
    error_rows = rows(error_csv) if error_csv.exists() else []
    usable = [r for r in error_rows if r["valid"] == "True"]
    if usable:
        for key in ("error_h", "error_v", "hpl", "vpl"):
            ax.plot([numeric(r, "pose_stamp_s") for r in usable], [numeric(r, key) for r in usable], ".", label=key)
        ax.legend();ax.set_ylabel("m");ax.set_xlabel("saved pose stamp s")
        ax.set_title(f"LIVE MEASUREMENT; runs={len({r['run_id'] for r in usable})}; unmatched={len(error_rows)-len(usable)}")
    else:
        ax.axis("off"); ax.text(.02, .65, "LIVE MEASUREMENT — NOT RUN\nNo paired GLIO/truth/prediction samples; independent runs = 0.\nThree repeats, route legality, time/extrinsic alignment and block analysis pending.")
    fig.tight_layout(); fig.savefig(out / "actual_error_vs_pl.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    timings = [r for p in sorted((run / "profiling").glob("advisory_validation_" + (dataset+"_" if dataset else "") + "*.csv")) for r in rows(p)]
    for ax, key in zip(axes, ("preparation_s", "query_total_s")):
        ax.bar([r["label"] for r in timings], [numeric(r, key) for r in timings]); ax.set_ylabel(key)
        ax.tick_params(axis="x", rotation=90)
    save(fig, "timing.png")
    data_root = run / "export/advisory/validation"
    if dataset: data_root = data_root / dataset
    matrices = [json.loads(line) for p in sorted(data_root.glob("*/matrices.jsonl")) for line in p.read_text().splitlines()]
    lookup = {r["label"]: r for r in matrices if r["label"] != "S0"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for group in ("S1", "S2", "S3"):
        series = [lookup.get(f"{group}_{i}", {}) for i in range(3)]
        for source in ("gnss", "lidar"):
            values = [min(r[source]["eigenvalues"]) if r.get(source) else np.nan for r in series]
            axes[0].plot([1,10,100], values, "o-", label=group+":"+source)
        axes[1].plot([1,10,100], [r.get("weak_direction_prior_fraction", np.nan) for r in series], "o-", label=group)
    axes[0].set_ylabel("source minimum information eigenvalue (1/m²)")
    axes[1].set_ylabel("prior fraction along fused weakest direction")
    for ax in axes: ax.set_xscale("log");ax.set_xlabel("diagnostic sigma multiplier");ax.legend()
    save(fig, "weak_direction_response.png")
    fractions = [r["weak_direction_prior_fraction"] for r in matrices if r.get("label") == "S3_0" and r["weak_direction_prior_fraction"] is not None]
    summary["S3_baseline_weak_direction_prior_fraction"] = fractions
    error_summary = run / "export/analysis/advisory_validation/error_summary.json"
    if error_summary.exists():
        actual = json.loads(error_summary.read_text())
        summary["actual_error_conformity"] = actual["status"]
        summary["independent_live_runs"] = actual["independent_runs"]
    json_write(out / "summary.json", summary)
    raw = [(p.parent.name, p) for p in sorted(data_root.glob("*/points.csv"))]
    text = ["# Advisory PL 空间退化验证：实际执行报告", "", f"场景目标：`{SCENARIO}`。现场状态：`{summary['live_status']}`。",
            "", ("本轮真实四分叉冻结输入、真实扫描和 GLIO 重复运行均未取得；已有日志和解析图未冒充本轮实测。" if not summary["real_requested"] else
                   f"本报告包含 {summary['real_requested']} 个真实冻结重放请求；覆盖仅限这些输入和查询点，不能据此宣称全部地图或实际误差尺度已验证。"),
            "", "| 独立结论 | 状态 |", "|---|---|"]
    text += [f"| {k} | `{summary[k]}` |" for k in ("input_availability", "spatial_sensitivity", "actual_error_conformity")]
    text += ["", "[原始 summary.json](summary.json)", "", "## 已完成的合成机制实验", "",
             "使用真实 PredictorModule、共享生产冻结准备与 v7 codec；固定参考时间 100 s，tau=0。S0 为合成六平面几何，最多 100 个唯一 0.1 m 体素中心，范围约 10×10 m。",
             "S1/S2/S3 分别将 GNSS、LiDAR 或两者测量 sigma 乘以 1/10/100；不删除物理障碍。S4 的 alpha=1/0.1/0 仅为诊断。GNSS 合成 epoch 的身份同步重算，固定合成监测 anchor；因此还导出 raw GNSS PL，不能将其解释为重新计算的认证监测 PL。",
             "弱法向变体仅过滤离线 primitive 支持，不修改物理 flags，包装器原结果与该诊断分列；其 wrapper_equal 为 N/A。",
             "", "| 组别 | HPL (m) | VPL (m) | 单调检查 |", "|---|---|---|---|"]
    text += [f"| {k} | {v['hpl']} | {v['vpl']} | {v['status']} |" for k, v in summary["mechanism"].items()]
    text += ["", f"双源退化信号保留比（alpha=1 / alpha=0）：`{summary['signal_retention_ratio_diagnostic']}`。",
             f"S3 基线融合弱方向上的先验信息占比：`{fractions}`。这证明该合成输入的先验主导程度，不证明森林真实位置的风险排序。",
             "", "## 来源准入与相关性证据", "",
             f"缺 GNSS 且 LiDAR 可用的包装器/模块不一致已复现：`{summary['source_admission_discrepancy_reproduced']}`，",
             f"见 [S5 原始请求]({os.path.relpath(data_root / 'S5_missing_gnss/points.csv', out)}) 和 [矩阵]({os.path.relpath(data_root / 'S5_missing_gnss/matrices.jsonl', out)})。包装器未绑定时 fused PL 留空，module_hpl 单列诊断；没有把零调用解释为零风险。",
             "`planner_risk.cpp` 从同帧 FGO 后验误差代理构造 `(3/e)^2 I`；`fgo_information_manager.cpp` 从 smoother 的 marginalCovariance 提取包含当前 GNSS/ICP 因子的后验。融合再直接相加 prior + GNSS + LiDAR，没有交叉协方差/来源去重。结构上存在相关观测重复使用，具体数值偏差尚未由实测量化。",
             f"[代码证据、行号与 SHA256]({os.path.relpath(audit, out)})。本配置 K_pl=3 与先验公式 3 一致；该先验只保留最大特征值的各向同性近似，不能将它当成已证明独立的完整后验信息。",
             "修正建议：先定义条件先验和来源独立性（排除再次加入的因子，或保守相关融合），核对 3 与 K_pl 的尺度及局部坐标旋转；对准入使用 PredictorModule 的有效来源策略，并保持物理/时间检查。须另行论证、回归和实测，本任务未更换生产模型或准入规则。",
             "", "## 坐标、时间与真实误差待测", "",
             "代码核对发现 sim_extension 使用第一对真值/GLIO 帧求一次 SE(3) 并固定用于 planner odom；`T_lidar_imu` 配置为单位变换，仿真 truth body 与预测 imu 的关系仍须按运行消息确认。这个初始真值对齐已经存在，应声明为坐标标定，不能重新逐段拟合或据此用起点零误差证明尺度正确。",
             "本轮未新增真值反馈。现场误差评估须冻结并登记单个外参与坐标变换，用 pose_stamp 对齐真值、保留 reference_time-pose_stamp 和插值窗口；同参考时刻与未来到达预测分别统计。覆盖不足单列；至少三次相同合法路线和配对 seed，按运行/轨迹块分析。当前独立现场样本数为 0。",
             "", "## 图表（身份已标注，缺失值为叉号）", ""]
    images = ["input_availability.png"] + [p.name for p in sorted(out.glob("spatial_*.png"))] + ["degradation_response.png", "weak_direction_response.png", "prior_ablation.png", "actual_error_vs_pl.png", "timing.png"]
    text += [f"![{name}]({name})\n" for name in images]
    text += ["## 原始数值、矩阵与完整输入", "", "每个目录含 input.bin / input.json / variant.json / points.csv / matrices.jsonl；缺物理输入的变体没有可编码 payload，保留缺失原因。参数完整身份由 payload 与 manifest SHA256 绑定。重复、batch、codec 和生产包装器对齐容差固定为 1e-12；没有计算的检查为 N/A。所有无效请求保留，统计分母含全部请求。耗时包含诊断重复查询、batch、包装器和 codec 对照；total 包含编码/解码，不能当成在线单次查询时延或独立重复运行。", ""]
    text += [f"- {label}：[CSV]({os.path.relpath(p, out)})、[矩阵]({os.path.relpath(p.parent / 'matrices.jsonl', out)})、[输入身份]({os.path.relpath(p.parent / 'input.json', out)})" for label, p in raw]
    if error_csv.exists():
        text += [f"- 现场误差请求：[原始 CSV]({os.path.relpath(error_csv, out)})、[状态 JSON]({os.path.relpath(error_summary, out)})；本轮没有现场输入时 CSV 仅有表头，绝不当成零误差。"]
    preflight_path = run / "metadata/manifests/advisory_preflight.json"
    if preflight_path.exists():
        text += [f"- 提交后干净工作树 / GPU 预检：[JSON]({os.path.relpath(preflight_path, out)})。"]
    text += ["", "## 未完成与阻止原因", "", "真实 start/middle/stop 冻结输入、S0 四分叉空间扫描、真实退化对照、固定合法路线三次重复和 GLIO 误差校准均待测。无关 RViz 修改保留，未启动现场，也未通过另建工作树绕过规则。合成机制 PASS 只表示该机制对照通过，不代替三个真实结论。", ""]
    text += ["## 已执行的复现入口", "", "在工作区 source ROS 和 install/setup.bash 后执行下列入口。新运行须由 resolver 分配；同名证据存在时拒绝覆盖。完整命令、源 hash、二进制与动态库 hash 见本运行的 metadata/manifests/advisory_fixture_*.json。", "", "```bash",
             "python3 src/iap/scripts/dev_predictor/advisory_validation.py fixture --binary build/ego_planner/advisory_validation --label committed",
             "ctest --test-dir build/ego_planner -R '^(test_advisory_validation|test_ego_baseline|test_ego_pipeline)$' --output-on-failure",
             "ctest --test-dir build/iap -R '^test_predictor_module$' --output-on-failure", "```", ""]
    (out / "report.md").write_text("\n".join(text), encoding="utf-8")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("fixture", "replay", "record", "report", "preflight"))
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--producer-binary", type=Path, help="record the installed production executable identity")
    parser.add_argument("--payload", type=Path)
    parser.add_argument("--label", default="start")
    parser.add_argument("--dataset", help="select one fixture campaign for reporting")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--interval", type=float, default=1.)
    parser.add_argument("--timeout", type=float, default=10.)
    parser.add_argument("--service", default="/grid_map/prediction_input")
    parser.add_argument("--planning-input",action="store_true",help="read retained planning input instead of a new display capture")
    parser.add_argument("--planning-attempt-id",type=int,default=0,help="exact retained planning attempt; 0 selects latest")
    parser.add_argument("--glio-topic", default="/drone_0_visual_slam/odom")
    parser.add_argument("--truth-topic", default="/sim/drone_0/truth_odom")
    args = parser.parse_args(); safe_label(args.label)
    if args.count < 1 or args.interval <= 0 or args.timeout <= 0 or args.planning_attempt_id < 0:
        parser.error("positive recording count/interval/timeout required")
    inherited = os.environ.get("IAP_RUN_DIR")
    if args.mode == "record" and not inherited:
        parser.error("record must adopt IAP_RUN_DIR from canonical iap_sim")
    owner = not inherited
    run = resolve_run_directory(entrypoint="advisory_validation", scenario=SCENARIO) if owner else adopt_run_directory(inherited)
    os.environ["IAP_RUN_DIR"] = str(run); os.environ["ROS_LOG_DIR"] = str(run / "runtime/ros")
    try:
        identity = source_identity()
        if args.mode in ("fixture", "replay"):
            if not args.binary or not args.binary.is_file(): parser.error("--binary must name built advisory_validation")
            binary = args.binary.resolve()
            if args.mode == "replay":
                if not args.payload: parser.error("--payload required")
                recorded = validate_record(args.payload, args.payload.with_name("record.json"))
                producer = recorded.get("producer_binary")
                if not producer: raise ValueError("recording lacks production binary/library identity")
                replay_libs = binary_identity(binary)["libraries_sha256"]
                for path, expected in producer["libraries_sha256"].items():
                    if path in replay_libs and replay_libs[path] != expected:
                        raise ValueError("compiled library identity mismatch: " + path)
                backend(run, binary, ["replay", args.label, str(args.payload.resolve())], args.label)
                decoded = json.loads((run / "export/advisory/validation" / args.label / "input.json").read_text())
                if any(decoded[k] != recorded[k] for k in ("frame_id", "geometry_id", "generation")):
                    raise ValueError("service/payload geometry identity mismatch")
            else: backend(run, binary, ["fixture", args.label], "fixture_" + args.label)
            output = report(run, args.label) if args.mode == "fixture" else run / "export/advisory/validation" / args.label
            raw_root = run / "export/advisory/validation" / args.label
            hashes = {str(p.relative_to(run)): sha(p) for p in sorted(raw_root.rglob("*")) if p.is_file()}
            if args.mode == "fixture":
                hashes.update({str(p.relative_to(run)): sha(p) for p in sorted(output.rglob("*")) if p.is_file()})
            hashes.update({str(p.relative_to(run)): sha(p) for p in sorted((run / "profiling").glob("advisory_validation_"+args.label+"*.csv"))})
            manifest(run, "advisory_" + args.mode + "_" + args.label,
                     {**identity, "binary": binary_identity(binary), "codec": CODEC,
                      "identity": "SYNTHETIC_MECHANISM" if args.mode == "fixture" else "REAL_REPLAY",
                      "command": sys.argv, "artifacts_sha256": hashes}, owner)
        elif args.mode == "preflight":
            output = preflight(run)
        elif args.mode == "record":
            record(run, args); output = run / "export/advisory/validation/recordings"
        else:
            output = report(run, args.dataset)
            manifest(run, "advisory_report", {**identity, "report": str((output / "report.md").relative_to(run))}, owner)
        if owner: finalize_run(run, lifecycle="completed", safety_outcome="not_applicable")
        print(output)
    except Exception:
        if owner: finalize_run(run, lifecycle="failed", safety_outcome="unknown")
        raise


if __name__ == "__main__":
    main()
