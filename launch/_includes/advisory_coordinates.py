"""Shared static coordinate/extrinsic/time evidence checks for Advisory trials."""
import numpy as np


def rigid(value):
    transform=np.asarray(value,dtype=float)
    if (transform.shape!=(4,4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3],[0,0,0,1],atol=1e-12,rtol=0) or
            not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-9,rtol=0) or
            abs(np.linalg.det(transform[:3,:3])-1)>1e-9):
        raise ValueError("fixed transform must be rigid SE(3)")
    return transform


def checked_coordinates(value):
    if (value.get("verified") is not True or not value.get("provenance") or
            value.get("alignment_policy")!="known_fixed_transform"):
        raise ValueError("verified static coordinates/extrinsic/time evidence required")
    for key in ("prediction_frame","prediction_body","truth_frame","truth_body"):
        if not isinstance(value.get(key),str) or not value[key].strip():
            raise ValueError("coordinate frame/body missing: "+key)
    for key in ("T_truth_map","T_truthbody_predictionbody"):
        if key not in value: raise ValueError("fixed transform missing: "+key)
        rigid(value[key])
    for key in ("world_to_enu_verified","body_extrinsics_verified","time_verified"):
        if value.get(key) is not True: raise ValueError("coordinate evidence missing: "+key)
    time=value.get("time_contract",{})
    if (time.get("clock")!="ros_system_time" or time.get("reference")!="saved_pose_stamp" or
            time.get("max_truth_bracket_dt_s")!=.05 or time.get("max_reference_pose_dt_s")!=.05):
        raise ValueError("canonical saved-time alignment evidence required")
    return value
