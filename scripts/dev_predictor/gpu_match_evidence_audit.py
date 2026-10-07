#!/usr/bin/env python3
"""Audit actual CUDA postopt samples; never fit/promote a noise model."""
import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
import sys

import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/"launch/_includes"))
from run_directory import adopt_run_directory, write_subordinate_manifest


def quantiles(values):
    values=np.asarray(values,dtype=float)
    return None if not len(values) else {"n":len(values),"min":float(np.min(values)),
        "median":float(np.median(values)),"p95":float(np.quantile(values,.95)),"max":float(np.max(values))}


def sha(path):
    digest=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""): digest.update(block)
    return digest.hexdigest()


def audit_sample(record, transform):
    """Independent host algebra from actual exported GPU identities and bytes."""
    e=np.asarray(record["residual_m"],float)
    covariance=np.asarray(record["covariance_m2_row_major"],float).reshape(3,3)
    ca=np.asarray(record["source_covariance_m2_row_major"],float).reshape(3,3)
    cb=np.asarray(record["target_covariance_m2_row_major"],float).reshape(3,3)
    a=np.asarray(record["source_mean_m"],float);b=np.asarray(record["target_mean_m"],float)
    if not all(np.isfinite(x).all() for x in (e,covariance,ca,cb,a,b,transform)):
        raise ValueError("nonfinite actual matching sample")
    if not np.allclose(covariance,covariance.T,rtol=1e-6,atol=1e-7):
        raise ValueError("asymmetric actual covariance")
    chol=np.linalg.cholesky(covariance) # No epsilon/repair.
    whitened=np.linalg.solve(chol,e)
    mahalanobis=float(whitened@whitened)
    gpu=float(record["mahalanobis_squared"])
    if not np.isfinite(gpu) or gpu<0:
        raise ValueError("invalid actual GPU cost")
    if not np.isclose(mahalanobis,gpu,rtol=1e-3,atol=1e-5):
        raise ValueError("actual GPU Mahalanobis mismatch")
    residual_error=float(np.max(np.abs(e-(b-transform[:3,:3]@a-transform[:3,3]))))
    covariance_error=float(np.max(np.abs(covariance-(cb+transform[:3,:3]@ca@transform[:3,:3].T))))
    if residual_error>1e-5 or covariance_error>1e-5:
        raise ValueError("actual transform/residual/covariance mismatch")
    return e,whitened,mahalanobis,residual_error,covariance_error


def audit(run):
    raw=run/"export/glio/gpu_match_residuals.jsonl"
    ledger=run/"export/glio/gpu_match_requests.csv"
    manifest=run/"metadata/manifests/gpu_match_evidence.json"
    summary=json.loads(manifest.read_text())
    with ledger.open() as stream: entries=list(csv.DictReader(stream))
    ledger_ids=[int(row["request_id"]) for row in entries]
    if ledger_ids!=list(range(1,len(entries)+1)): raise ValueError("missing/repeated request identity")
    accepted={int(row["request_id"]):row for row in entries if row["queue_accepted"]=="1"}
    if (summary["entries"]!=len(entries) or summary["requested"]!=sum(int(row["capture_requested"]) for row in entries)
        or summary["available"]!=sum(int(row["available"]) for row in entries)
        or summary["queue_dropped"]!=len(entries)-len(accepted) or summary["write_failures"]):
        raise ValueError("request/drop/write denominator mismatch")
    if any(row["available"]=="1" and row["capture_requested"]!="1" for row in entries):
        raise ValueError("unrequested available ledger capture")
    raw_ids=set();residuals=[];norms=[];whitened_all=[];sample_fractions=[];gaps=[]
    residual_errors=[];covariance_errors=[];transform_errors=[];statuses=Counter();owners=Counter()
    source_matches=Counter();voxel_matches=Counter();multi_level=defaultdict(dict)
    frames=set();total_source=0;total_inliers=0;sample_count=0
    with raw.open() as stream:
        for line in stream:
            p=json.loads(line);identity=p["request_id"]
            if p["schema"]!="iap_gpu_match_residual_v1" or p["model"]!="actual_cuda_postopt_quality_linearization_v1":
                raise ValueError("unsupported actual residual model")
            if identity not in accepted or identity in raw_ids: raise ValueError("raw request identity mismatch")
            raw_ids.add(identity);row=accepted[identity]
            if (int(row["source_frame_id"])!=p["source_frame_id"] or int(row["target_frame_id"])!=p["target_frame_id"]
                or int(row["available"])!=int(p["available"]) or int(row["samples"])!=len(p["samples"])):
                raise ValueError("request/raw owner/count mismatch")
            integer_fields={"level_id":"level_id","capture_requested":"capture_requested","target_is_fixed":"target_is_fixed",
                "sequence":"sequence","source_count":"original_source_count","inlier_count":"original_inlier_count","stride":"sampling_stride"}
            for field,target_field in integer_fields.items():
                if int(row[field])!=int(p[target_field]): raise ValueError("request/raw identity mismatch: "+field)
            for field,target_field in {"source_stamp":"source_stamp","target_stamp":"target_stamp","voxel_resolution_m":"voxel_resolution_m","cost":"original_cost"}.items():
                if not np.isfinite(float(row[field])) or float(row[field])!=p[target_field]:
                    raise ValueError("request/raw original scalar mismatch: "+field)
            if row["failure_reason"]!=p["failure_reason"]: raise ValueError("request/raw failure mismatch")
            keys=[int(row["key0"]),int(row["key1"])][:int(row["key_count"])]
            if keys!=p["factor_keys"]: raise ValueError("request/raw factor identity mismatch")
            g=p["gnss"]
            for field,target_field in {"gnss_frame_id":"frame_id","gnss_update_sequence":"update_sequence","gnss_epoch_identity":"epoch_source_identity","gnss_owner_matches":"owner_matches"}.items():
                if int(row[field])!=int(g[target_field]): raise ValueError("request/raw GNSS identity mismatch: "+field)
            for field,target_field in {"gnss_state_stamp":"state_stamp","gnss_epoch_stamp":"epoch_stamp"}.items():
                if not np.isfinite(float(row[field])) or float(row[field])!=g[target_field]:
                    raise ValueError("request/raw GNSS original time mismatch")
            if row["used_constellations"]!=g["used_constellations"] or g["propagation"]!="NOT_PROPAGATED":
                raise ValueError("request/raw GNSS model mismatch")
            if p["available"] and not p["capture_requested"]: raise ValueError("unrequested available capture")
            statuses["AVAILABLE" if p["available"] else p["failure_reason"]]+=1
            if not p["available"]:
                if p["samples"]: raise ValueError("unavailable capture borrowed samples")
                continue
            frames.add(p["source_frame_id"]);total_source+=p["original_source_count"];total_inliers+=p["original_inlier_count"]
            samples=p["samples"];sample_count+=len(samples)
            if len(samples)>128 or len(samples)>p["original_inlier_count"]: raise ValueError("invalid sample bound")
            stride=max(1,(p["original_source_count"]+127)//128)
            if p["sampling_stride"]!=stride: raise ValueError("density/stride mismatch")
            sample_fractions.append(len(samples)/max(1,p["original_inlier_count"]))
            transform=np.asarray(p["linearization_transform_row_major"],float).reshape(4,4)
            source=np.asarray(p["T_world_source_row_major"],float).reshape(4,4)
            target=np.asarray(p["T_world_target_row_major"],float).reshape(4,4)
            error=float(np.max(np.abs(transform-np.linalg.inv(target)@source)))
            if error>1e-5: raise ValueError("optimized pose/actual transform mismatch")
            transform_errors.append(error)
            owners["MATCHED" if g["owner_matches"] else "UNMATCHED"]+=1
            if g["owner_matches"]:
                if g["frame_id"]!=p["source_frame_id"] or abs(g["state_stamp"]-p["source_stamp"])>1e-9:
                    raise ValueError("GNSS owner falsely matched")
                gaps.append(g["state_stamp"]-g["epoch_stamp"])
            cost=0.;indices=set()
            for s in samples:
                if s["source_index"] in indices or s["source_index"]%stride or not 0<=s["source_index"]<p["original_source_count"]:
                    raise ValueError("source sample identity mismatch")
                indices.add(s["source_index"])
                if s["target_index"]<0 or s["target_point_count"]<=0: raise ValueError("invalid target identity")
                e,w,m,re,ce=audit_sample(s,transform);cost+=m
                world_e=target[:3,:3]@e
                residuals.append(world_e);norms.append(float(np.linalg.norm(e)));whitened_all.append(w)
                residual_errors.append(re);covariance_errors.append(ce)
                sk=(p["source_frame_id"],s["source_index"]);source_matches[sk]+=1
                vk=(p["target_frame_id"],p["level_id"],s["target_index"]);voxel_matches[vk]+=1
                key=(p["source_frame_id"],p["target_frame_id"],s["source_index"])
                multi_level[key][p["level_id"]]=world_e
            if stride==1 and not np.isclose(cost,p["original_cost"],rtol=1e-3,atol=1e-5):
                raise ValueError("full fixture cost mismatch; no point-count weight allowed")
    if raw_ids!=set(accepted) or summary["written"]!=len(raw_ids) or summary["samples_written"]!=sample_count:
        raise ValueError("raw missing accepted requests/samples")
    paired=[(value[0],value[1]) for value in multi_level.values() if 0 in value and 1 in value]
    correlation=None
    if len(paired)>2:
        pairs=np.asarray(paired)
        correlation=[float(np.corrcoef(pairs[:,0,axis],pairs[:,1,axis])[0,1])
                     if np.std(pairs[:,0,axis])*np.std(pairs[:,1,axis])>0 else None for axis in range(3)]
    report={"identity":"ACTUAL_GPU_POSTOPT_DIAGNOSTIC","entries":len(entries),"requested":summary["requested"],
        "queue_dropped":summary["queue_dropped"],"available_requested":summary["available"],
        "available_exported":statuses["AVAILABLE"],"export_statuses":dict(statuses),
        "request_statuses":dict(Counter("AVAILABLE" if row["available"]=="1" else row["failure_reason"] for row in entries)),
        "source_frames":len(frames),"samples":sample_count,"total_source_count":total_source,"total_inlier_count":total_inliers,
        "sample_fraction":quantiles(sample_fractions),"actual_residual_norm_m":quantiles(norms),
        "residual_reconstruction_error_m":quantiles(residual_errors),"covariance_reconstruction_error_m2":quantiles(covariance_errors),
        "optimized_transform_error":quantiles(transform_errors),"gnss_owners":dict(owners),"original_state_epoch_gap_s":quantiles(gaps),
        "repeated_source_match_count":quantiles(list(source_matches.values())),"repeated_target_voxel_count":quantiles(list(voxel_matches.values())),
        "paired_multi_level_sources":len(paired),"multi_level_world_component_correlation_diagnostic":correlation,
        "world_component_std_m":np.std(residuals,axis=0).tolist() if residuals else None,
        "whitened_component_std":np.std(whitened_all,axis=0).tolist() if whitened_all else None,
        "qualified_noise":False,"qualified_joint_model":False,"qualified_space_or_meters":False,
        "note":"Actual 3D VGICP postopt residuals; covariance is matcher geometry. Repeated source/voxel/levels are correlated; no independent-noise or physical-Up qualification."}
    return report,np.asarray(residuals),np.asarray(whitened_all)


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--run-dir",required=True)
    args=parser.parse_args();run=adopt_run_directory(args.run_dir)
    report,residuals,whitened=audit(run)
    output=run/"export/analysis/gpu_match_evidence";output.mkdir(parents=True,exist_ok=True)
    result=output/"audit.json"
    with result.open("x") as stream: json.dump(report,stream,ensure_ascii=False,allow_nan=False,indent=2)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    if len(residuals):
        for axis,label in enumerate("XYZ"):
            axes[0].hist(residuals[:,axis],bins=80,histtype="step",label=label)
            axes[1].hist(whitened[:,axis],bins=80,histtype="step",label=label)
        for ax in axes: ax.legend();ax.set_ylabel("Exported sample count")
    axes[0].set_xlabel("World component residual (m; Up unqualified)")
    axes[1].set_xlabel("Matcher covariance whitened residual (dimensionless)")
    fig.suptitle("Actual CUDA postopt diagnostic; repeated samples, noise unqualified")
    fig.tight_layout();plot=output/"actual_residuals.png";fig.savefig(plot,dpi=160);plt.close(fig)
    files=[run/"export/glio/gpu_match_residuals.jsonl",run/"export/glio/gpu_match_requests.csv",
           run/"metadata/manifests/gpu_match_evidence.json",result,plot,Path(__file__)]
    write_subordinate_manifest(run,"gpu_match_evidence_audit",{"command":sys.argv,"result":report,
        "sha256":{str(p):sha(p) for p in files}})
    print(json.dumps(report,ensure_ascii=False,allow_nan=False))
    return 0


if __name__=="__main__": raise SystemExit(main())
