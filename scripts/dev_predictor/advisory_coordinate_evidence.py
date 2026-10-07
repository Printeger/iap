#!/usr/bin/env python3
"""Audit real frozen coordinate evidence; truth is used only by error analysis."""
import argparse
import json
import math
import os
from pathlib import Path
import numpy as np
from advisory_validation import adopt_run_directory, artifact, json_write, manifest, sha, validate_record, safe_label, binary_identity
from advisory_coordinates import rigid


def audit(meta):
    c=meta['coordinates']
    if (meta.get('recording_codec_version')!=7 or
        meta.get('clock_model')!='per_constellation_pseudorange_bias_v1' or
        meta.get('gnss_fault_model')!='single_satellite_and_constellation_v1') or not c.get('required') or not c.get('valid'):
        raise ValueError('production_coordinate_evidence_unavailable')
    matrix=lambda key,n:np.asarray(c[key],dtype=float).reshape(n,n)
    R_ew,R_en,R_mn=(matrix(key,3) for key in ('R_ecef_world','R_ecef_enu','R_map_enu'))
    T_mw,T_wi,T_li=(rigid(matrix(key,4)) for key in ('T_map_world','T_world_imu','T_lidar_imu'))
    for R in (R_ew,R_en,R_mn):
        T=np.eye(4);T[:3,:3]=R;rigid(T)
    if c.get('reason') or c['map_frame']!=meta['frame_id'] or c['body_frame']!='imu':
        raise ValueError('coordinate_frame_or_body_mismatch')
    if (abs(c['stamp']-meta['current_stamp'])>1e-6 or abs(c['stamp']-meta['pose_stamp'])>1e-6 or
        c['frame_id']!=meta['estimation_frame_id']):
        raise ValueError('coordinate_estimator_time_mismatch')
    if not meta['has_epoch']:raise ValueError('gnss_epoch_missing_from_frozen_input')
    if c['epoch_source_identity']!=meta['epoch_source_identity']:raise ValueError('coordinate_epoch_identity_mismatch')
    algebra=float(np.linalg.norm(R_mn-T_mw[:3,:3]@R_ew.T@R_en))
    if algebra>1e-9:raise ValueError('direction_rotation_mismatch')
    lever=np.asarray(c['lever_arm_imu'])
    antenna=np.asarray(c['anchor_ecef'])+R_ew@(T_wi[:3,3]+T_wi[:3,:3]@lever)
    errors=[]
    for sat in meta['gnss_satellites']:
        if sat['excluded']:continue
        delta=np.asarray(sat['ecef'])-antenna
        if not np.isfinite(delta).all() or np.linalg.norm(delta)<1:
            raise ValueError('invalid_satellite_ecef')
        direct=T_mw[:3,:3]@R_ew.T@(delta/np.linalg.norm(delta))
        az,el=sat['azimuth'],sat['elevation']
        projected=R_mn@np.array([math.cos(el)*math.sin(az),math.cos(el)*math.cos(az),math.sin(el)])
        errors.append(float(np.linalg.norm(direct-projected)))
    if not errors:raise ValueError('no_gnss_directions_to_verify')
    if max(errors)>1e-9:raise ValueError('direct_ecef_direction_mismatch')
    return {'valid':True,'rotation_algebra_error':algebra,'max_direction_error':max(errors),
            'directions_checked':len(errors),'coordinate_frame_id':c['frame_id'],
            'reference_pose_delta_s':meta['reference_time_s']-meta['pose_stamp'],
            'calibration_time_qualified':abs(meta['reference_time_s']-meta['pose_stamp'])<=.05,
            'gnss_pose_delta_s':meta['pose_stamp']-meta['gnss_stamp'],
            'gpst_utc_delta_s':meta['gps_sec']-meta['gnss_stamp'],
            'map_up_enu_angle_deg':math.degrees(math.acos(float(np.clip(R_mn[2,2],-1,1)))),
            'antenna_offset_map':(T_mw[:3,:3]@T_wi[:3,:3]@lever).tolist(),
            'T_map_lidar':(T_mw@T_wi@np.linalg.inv(T_li)).tolist(),
            'PL_axes':'map XY horizontal; map Z vertical; GNSS raw/anchored remain legacy ENU diagnostics',
            'conditioning':'FGO rotation and attitude treated as fixed; their uncertainty is not propagated',
            'truth_used_by_predictor':False}


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--label',required=True);p.add_argument('--record',type=Path,required=True)
    args=p.parse_args();safe_label(args.label);run=adopt_run_directory(os.environ['IAP_RUN_DIR'])
    meta=json.loads(args.input.read_text())
    result={'identity':'UNVERIFIED_INPUT_DIAGNOSTIC','input':str(args.input),'input_sha256':sha(args.input)}
    try:
        recorded=validate_record(args.record.parent/'input.bin',args.record)
        producer=recorded.get('producer_binary')
        if not producer or not producer.get('libraries_sha256'):
            raise ValueError('production_binary_identity_missing')
        actual=binary_identity(Path(producer['path']))
        if actual['sha256']!=producer['sha256'] or actual['libraries_sha256']!=producer['libraries_sha256']:
            raise ValueError('production_binary_or_library_identity_mismatch')
        if sha(args.record.parent/'input.bin')!=sha(args.input.parent/'input.bin'):
            raise ValueError('replay_payload_does_not_match_real_record')
        if json.loads((args.input.parent/'variant.json').read_text())['identity']!='REAL_REPLAY':
            raise ValueError('real_replay_identity_required')
        result['identity']='REAL_REPLAY';result['record_sha256']=sha(args.record)
        result.update(audit(meta))
    except (ValueError,KeyError,TypeError) as e:result.update(valid=False,reason=str(e))
    target=artifact(run,'export/analysis/advisory_validation/coordinates/'+args.label+'.json')
    json_write(target,result);manifest(run,'advisory_coordinates_'+args.label,{'artifact':str(target.relative_to(run)),'sha256':sha(target)})
    print(json.dumps(result,ensure_ascii=False));return 0 if result['valid'] else 2

if __name__=='__main__':raise SystemExit(main())
