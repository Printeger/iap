"""Offline actual-source pose correction against the SAME frozen production GridMap.
This is an actual captured-measurement diagnostic, not a GLIO replay or coverage bound.
Original stamps are retained; real IMU motion transports source geometry to GNSS epoch.
"""
import csv,json,sys,hashlib
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(REPO/'scripts/dev_predictor'),str(REPO/'launch/_includes')]
from compare_advisory_error import rotation_matrix,quaternion
from epoch_motion_reference import log as so3log
from run_directory import adopt_run_directory,write_subordinate_manifest
from advisory_validation import sha,safe_label,artifact
cfgpath=Path(sys.argv[1]);cfg=json.loads(cfgpath.read_text());run=adopt_run_directory(cfg['owner']);label=safe_label(sys.argv[2]);out=artifact(run,'export/analysis/'+label+'/summary.json').parent
assert not (out/'summary.json').exists(),'diagnostic evidence already exists'
root=Path(cfg['decoded_root']);raw=Path(cfg['gpu']);truthpath=Path(cfg['truth']);qualified=set(cfg['qualified_frames'])
meta={}
for p in sorted(root.glob(cfg['decoded_glob'])):
 m=json.loads(p.read_text());k=m['estimation_frame_id']
 if k in qualified:meta[k]=(p,m)
packets={};considered=0
for line in raw.open():
 p=json.loads(line);k=p['source_frame_id']
 if k not in meta:continue
 considered+=1
 if p['available'] and p['samples'] and p['level_id']==0 and p['target_is_fixed']:
  if k not in packets or p['target_frame_id']>packets[k]['target_frame_id']:packets[k]=p
truth=sorted([v for v in csv.DictReader(truthpath.open()) if v['source']=='truth' and v['frame']=='map' and v['body']=='drone_0'],key=lambda v:float(v['stamp']))
ts=np.array([float(v['stamp']) for v in truth]);ps=np.array([[float(v[k]) for k in ['x','y','z']] for v in truth]);qs=np.array([quaternion(v) for v in truth])
def pose(t):
 i=int(np.searchsorted(ts,t));assert 0<i<len(ts),'truth_not_bracketed'
 assert max(t-ts[i-1],ts[i]-t)<=cfg['truth_gate_s'],'truth_original_50ms_gate'
 u=(t-ts[i-1])/(ts[i]-ts[i-1]);a,b=qs[i-1],qs[i]
 if a@b<0:b=-b
 dot=np.clip(a@b,-1.,1.)
 if dot>.9995:q=(1-u)*a+u*b;q/=np.linalg.norm(q)
 else:
  z=np.arccos(dot);q=(np.sin((1-u)*z)*a+np.sin(u*z)*b)/np.sin(z)
 T=np.eye(4);T[:3,:3]=rotation_matrix(q);T[:3,3]=(1-u)*ps[i-1]+u*ps[i];return T

def surface_rows(centers,normals,confidence,weights,position,radius,sigma,scale,voxel):
 keys=np.column_stack((np.floor(centers/voxel+1e-9).astype('int64'),np.argmax(abs(normals),axis=1)))
 rows=np.column_stack((normals,np.cross(centers-position,normals)))
 w=scale*np.exp(-np.sum((centers-position)**2,axis=1)/(2*radius**2))*confidence*weights/sigma**2
 return keys,rows,w

def grouped(A,y,w,keys):
 h=np.zeros((A.shape[1],A.shape[1]));g=np.zeros(A.shape[1]);aw=[];bw=[]
 for key in sorted(set(map(tuple,keys))):
  idx=np.array([tuple(k)==key for k in keys]);weight=w[idx]/np.sum(idx)
  h+=A[idx].T@(weight[:,None]*A[idx]);g+=A[idx].T@(weight*y[idx])
  aw.extend(A[idx]*np.sqrt(weight)[:,None]);bw.extend(y[idx]*np.sqrt(weight))
 return h,g,np.array(aw),np.array(bw)

def solve(A,b):
 u,s,vt=np.linalg.svd(A,full_matrices=False)
 if len(s)<A.shape[1] or s[-1]<max(1e-10,s[0]*1e-8):return {'valid':False,'reason':'rank_deficient','singular_values':s.tolist()}
 x=np.linalg.lstsq(A,b,rcond=1e-8)[0];direct=np.linalg.solve(A.T@A,A.T@b)
 assert np.max(abs(x-direct))<1e-5,'independent_local_solve_mismatch'
 C=np.linalg.inv(A.T@A)[:3,:3];ev,v=np.linalg.eigh(C)
 return {'valid':True,'increment':x[:6].tolist(),'clock_increment':x[6:].tolist(),'position_metric_eigenvalues':ev.tolist(),'weak_axis':v[:,-1].tolist(),'h_metric_m':float(5*np.sqrt(np.linalg.eigvalsh(C[:2,:2])[-1])),'v_metric_m':float(5*np.sqrt(C[2,2])),'solve_crosscheck_max':float(np.max(abs(x-direct))),'postfit_rms':float(np.sqrt(np.mean((A@x-b)**2)))}

results=[];inputs={str(cfgpath):sha(cfgpath),str(raw):sha(raw),str(truthpath):sha(truthpath)}
for k,(mp,m) in sorted(meta.items()):
 row={'frame_id':k,'reference_s':m['reference_time_s'],'original_state_s':m['postopt_evidence']['state_stamp'],'map_cloud_s':m['cloud_stamp'],'map_generation':m['generation'],'same_production_fixed_map':True,'same_reference':True,'truth_used_in_estimation':False,'model_fit':False,'sparse_sample_limitation':True}
 try:
  assert k in packets,'no_fixed_target_source_packet';p=packets[k]
  row.update(sparse_sample_limitation=p['sampling_stride']!=1,full_original_inliers=p['sampling_stride']==1 and len(p['samples'])==p['original_inlier_count'],original_source_count=p['original_source_count'],original_inlier_count=p['original_inlier_count'],sampling_stride=p['sampling_stride'],capture_sample_limit=p.get('sample_limit',128),capture_request_id=p['request_id'],position=m['position'])
  assert abs(p['source_stamp']-row['original_state_s'])<1e-6,'source_owner_mismatch'
  Tws=np.array(p['T_world_source_row_major']).reshape(4,4);opt=np.array(m['postopt_evidence']['optimized_means'][:16]).reshape(4,4)
  assert np.max(abs(Tws-opt))<1e-9,'postopt_source_pose_mismatch'
  Tmw=np.array(m['coordinates']['T_map_world']).reshape(4,4);Twr=np.array(m['coordinates']['T_world_imu']).reshape(4,4);ext=np.array(m['coordinates']['T_lidar_imu']).reshape(4,4)
  assert np.max(abs(ext-np.array(p['T_lidar_imu_row_major']).reshape(4,4)))<1e-9,'extrinsics_mismatch'
  assert 0<=m['reference_time_s']-p['source_stamp']<=.5,'original_propagation_validity'
  assert m['postopt_evidence']['propagation']=='IMU_TO_GNSS_EPOCH','missing_real_IMU_reference'
  # Body means are genuine deskewed source samples. Transport by the RECORDED
  # IMU relative transform, without replacing source/reference timestamps.
  means=np.array([v['source_mean_m'] for v in p['samples']]);world=(Tws[:3,:3]@means.T).T+Tws[:3,3]
  refbody=(Twr[:3,:3].T@(world-Twr[:3,3]).T).T
  Tmr=Tmw@Twr;points=(Tmr[:3,:3]@refbody.T).T+Tmr[:3,3]
  assert np.max(abs(Tmr[:3,3]-np.array(m['position'])))<1e-9,'linearization_pivot_mismatch'
  pp=mp.with_name('lidar_primitives.csv');dd=list(csv.DictReader(pp.open()));c=np.array([[float(v[z]) for z in ['x','y','z']] for v in dd]);n=np.array([[float(v[z]) for z in ['nx','ny','nz']] for v in dd]);cf=np.array([float(v['confidence']) for v in dd]);ww=np.array([float(v['weight']) for v in dd]);position=np.array(m['position']);radius=m['lidar_radius_m']
  inside=np.sum((c-position)**2,axis=1)<=radius**2;original_primitive_ids=np.flatnonzero(inside);c,n,cf,ww=c[inside],n[inside],cf[inside],ww[inside]
  assert len(c)>0,'no_fixed_map_support'
  keys,modelrows,mw=surface_rows(c,n,cf,ww,position,radius,m['lidar_sigma'],m['lidar_weight_scale'],m['lidar_support_voxel_m'])
  hl,_,_,_=grouped(modelrows,np.zeros(len(c)),mw,keys)
  matp=mp.with_name('matrices.jsonl');mat=json.loads(matp.read_text().splitlines()[0]);native=np.array(mat['lidar_pose_information']['row_major']).reshape(6,6)
  model_difference=float(np.max(abs(hl-native)));assert model_difference<1e-7,'independent_lidar_information_mismatch'
  # Chunk only the diagnostic lookup; deterministic original order breaks ties.
  match=np.concatenate([np.argmin(np.sum((part[:,None,:]-c[None,:,:])**2,axis=2),axis=1) for part in np.array_split(points,max(1,(len(points)+255)//256))])
  distance=np.linalg.norm(points-c[match],axis=1);within_scan_radius=np.sum((points-position)**2,axis=1)<=radius**2
  keep=(distance<=cfg['association_distance_m']) & within_scan_radius
  correspondence=[{'source_index':s['source_index'],'original_gpu_target_index':s['target_index'],'original_gpu_target_frame_id':p['target_frame_id'],'original_gpu_level_id':p['level_id'],'production_primitive_index':int(original_primitive_ids[j]),'radius_subset_index':int(j),'support_family':keys[j].tolist(),'association_distance_m':float(d),'used':bool(ok),'missing_reason':None if ok else ('SOURCE_OUTSIDE_MODEL_RADIUS' if not inside_scan else 'NO_SURFACE_WITHIN_ASSOCIATION_RADIUS')} for s,j,d,ok,inside_scan in zip(p['samples'],match,distance,keep,within_scan_radius)]
  (out/f'frame_{k}_correspondence.json').write_text(json.dumps({'schema':'iap_actual_source_production_support_correspondence_v1','source_frame_id':k,'request_id':p['request_id'],'source_stamp':p['source_stamp'],'reference_stamp':m['reference_time_s'],'map_generation':m['generation'],'primitive_file_sha256':sha(pp),'raw_valid_matches':len(p['samples']),'original_source_count':p['original_source_count'],'original_inlier_count':p['original_inlier_count'],'capture_stride':p['sampling_stride'],'invalid_or_unmatched_source_slots':p.get('source_slots',(p['original_source_count']+p['sampling_stride']-1)//p['sampling_stride'])-len(p['samples']),'uncaptured_source_slots':p['original_source_count']-p.get('source_slots',(p['original_source_count']+p['sampling_stride']-1)//p['sampling_stride']),'rows':correspondence})+'\n')
  row.update(unique_source=len(set(s['source_index'] for s in p['samples'])),unique_original_gpu_target=len(set(s['target_index'] for s in p['samples'])),unique_production_target=len(set(original_primitive_ids[match[keep]].tolist())),missing_association=int(np.sum(~keep)),source_outside_radius=int(np.sum(~within_scan_radius)))
  assert np.sum(keep)>=6,'too_few_real_associations'
  points=points[keep];match=match[keep];normals=n[match]
  residual=np.einsum('ni,ni->n',normals,c[match]-points)
  lidarA=np.column_stack((normals,np.cross(points-position,normals)))
  hl_real,gl_real,la,lb=grouped(lidarA,residual,mw[match],keys[match])
  gnssp=mp.with_name('gnss_information_rows.jsonl');gnss=json.loads(gnssp.read_text().splitlines()[0]);sats={(v['constellation'],v['id']):v for v in m['gnss_satellites']};systems=sorted(set(v['constellation'] for v in gnss['rows']));ga=np.zeros((len(gnss['rows']),6+len(systems)));gb=[]
  for i,v in enumerate(gnss['rows']):
   sat=sats[(v['constellation'],v['sat_id'])];assert sat['measurement_residual_available'],'missing_actual_pseudorange_residual'
   ga[i,:3]=-np.array(v['los_map']);ga[i,6+systems.index(v['constellation'])]=1.;ga[i]/=v['final_sigma_m'];gb.append(sat['pr_residual_m']/v['final_sigma_m'])
  gb=np.array(gb);hg=ga.T@ga;gg=hg[:6,:6]-hg[:6,6:]@np.linalg.inv(hg[6:,6:])@hg[6:,:6]
  assert np.max(abs(gg-np.array(mat['gnss_pose_information']['row_major']).reshape(6,6)))<1e-8,'independent_clock_elimination_mismatch'
  # Independent local estimator uses original nuisance clocks. The production
  # factor-two envelope inflates uncertainty, not the minimizer; no regularizer.
  lidar=np.pad(la,((0,0),(0,len(systems))));both=np.vstack((lidar,ga));b=np.r_[lb,gb]
  fusion=solve(both,b);lidars=solve(la,lb)
  # GNSS constrains position plus constellation clocks but no attitude for zero lever arm.
  ga_position=np.column_stack((ga[:,:3],ga[:,6:]));gpssv=np.linalg.svd(ga_position,compute_uv=False)
  gps_valid=len(gpssv)==ga_position.shape[1] and gpssv[-1]>max(1e-10,gpssv[0]*1e-8)
  if gps_valid:
   gpsx=np.linalg.lstsq(ga_position,gb,rcond=1e-8)[0];gpsC=np.linalg.inv(ga_position.T@ga_position)[:3,:3]
   assert np.linalg.eigvalsh(gpsC)[0]>0,'invalid_GNSS_position_covariance'
  true=pose(m['reference_time_s']);expected=np.r_[true[:3,3]-position,so3log(true[:3,:3]@Tmr[:3,:3].T)]
  C=np.array(mat['covariance']['row_major']).reshape(3,3);weak=np.linalg.eigh(C)[1][:,-1]
  for solution in [fusion,lidars]:
   if solution['valid']:
    e=np.array(solution['increment'])-expected;solution.update(position_error_m=e[:3].tolist(),position_error_norm_m=float(np.linalg.norm(e[:3])),rotation_error_rad=float(np.linalg.norm(e[3:])),model_weak_error_m=float(weak@e[:3]),weak_axis_alignment=float(abs(weak@np.array(solution['weak_axis']))))
  doubled=grouped(np.vstack((lidarA,lidarA)),np.r_[residual,residual],np.r_[mw[match],mw[match]],np.vstack((keys[match],keys[match])))
  assert np.max(abs(doubled[0]-hl_real))<1e-9,'duplicate_information_inflation'
  # Deterministic map sensitivity, not a fabricated covariance/correlation model.
  sensitivities={}
  for name,shift in [('map_z_plus_005',np.array([0,0,.05])),('map_x_plus_005',np.array([.05,0,0]))]:
   _,_,_,shiftb=grouped(lidarA,residual+normals@shift,mw[match],keys[match]);s=solve(both,np.r_[shiftb,gb]);sensitivities[name]=s
  remove_lidar={'valid':bool(gps_valid),'attitude_unobservable':True,'actual_design_rows':ga_position.shape[0],'nuisance_plus_position_columns':ga_position.shape[1],'singular_values':gpssv.tolist()}
  if gps_valid:remove_lidar.update(position_increment=gpsx[:3].tolist(),position_error_norm_m=float(np.linalg.norm(gpsx[:3]-expected[:3])),position_covariance_m2=gpsC.tolist())
  else:remove_lidar.update(reason='GNSS_POSITION_UNOBSERVABLE_AFTER_CLOCK_NUISANCE')
  row.update(qualified=True,model_h_m=float(5*np.sqrt(np.linalg.eigvalsh(C[:2,:2])[-1])),model_v_m=float(5*np.sqrt(C[2,2])),production_support_groups=mat['lidar_support_groups'],captured_means=len(means),real_associations=int(np.sum(keep)),matched_support_groups=len(set(map(tuple,keys[match]))),matched_model_information_trace_ratio=float(np.trace(hl_real)/np.trace(hl)),independent_model_matrix_max=model_difference,source_transport_s=m['reference_time_s']-p['source_stamp'],source_pose_identity_max=float(np.max(abs(Tws-opt))),propagation_F_Q_preserved=True,gnss_admitted_rows=gnss['rows'],clock_systems=systems,gnss_residual_semantics='actual postopt unwhitened pr_meas-minus-pr_pred at original IMU-propagated GNSS reference; NOT raw measurement error',true_local_increment=expected.tolist(),fusion=fusion,lidar_only=lidars,gnss_only=remove_lidar,map_sensitivity=sensitivities,duplicate_max=float(np.max(abs(doubled[0]-hl_real))),correlation_assumption='one averaged row family per original spatial support; cross-frame/shared-map covariance not established; complete original inliers only when stride=1 and count closes',map_condition='same frozen registered geometry; raw scan inclusion requires actual frame/source provenance, not inferred voxel deletion; map bias and shared optimizer history remain conditional limitations')
  arrays={'reference_body_points':refbody.tolist(),'source_packet':p,'matched_primitive_indices':original_primitive_ids[match].tolist(),'matched_radius_subset_indices':match.tolist(),'matched_source_indices':[s['source_index'] for s,ok in zip(p['samples'],keep) if ok],'matched_gpu_target_indices':[s['target_index'] for s,ok in zip(p['samples'],keep) if ok],'actual_residual_m':residual.tolist(),'actual_lidar_rows':lidarA.tolist(),'actual_group_keys':keys[match].tolist(),'actual_lidar_information':hl_real.tolist(),'admitted_gnss_design':ga.tolist(),'admitted_gnss_residual_whitened':gb.tolist(),'fixed_production_lidar_information':hl.tolist()}
  ap=out/f'frame_{k}.json';ap.write_text(json.dumps(arrays)+'\n')
  for ip in [mp,pp,matp,gnssp]:inputs[str(ip)]=sha(ip)
 except (AssertionError,KeyError,ValueError,np.linalg.LinAlgError) as exc:row.update(qualified=False,reason=str(exc))
 results.append(row)
valid=[v for v in results if v.get('qualified') and v['fusion']['valid']]
def ranks(a):
 a=np.asarray(a);return np.array([np.sum(a<v)+.5*(np.sum(a==v)-1) for v in a])
rho=float(np.corrcoef(ranks([v['model_h_m'] for v in valid]),ranks([v['fusion']['position_error_norm_m'] for v in valid]))[0,1]) if len(valid)>2 else None
summary={'schema':'actual_same_fixed_gridmap_local_increment_v1','rows':results,'requested':len(meta),'valid_local_fusion_solves':len(valid),'raw_packets_considered':considered,'prediction_object':'LOCAL_LINEARIZED_POSE_INCREMENT_FIXED_MAP','estimator':'real captured deskewed means point-to-plane WLS against identical production PCA geometry, same reference; real admitted pseudorange with constellation clock nuisance; unregularized full pose solve','spearman_model_H_vs_error':None if rho is None else float(rho),'claims':{'same_fixed_map_geometry':True,'same_reference_epoch':True,'same_full_observation_set_as_predictor':False,'independent_truth':True,'independent_run':cfg.get('independent_run',False),'95_percent_coverage':False,'GLIO_absolute_bound':False,'production_model_changed':False},'limits':['Original per-factor capture stride and unmatched slots retained; full level-0 latest fixed target capture still omits other factor targets and levels','raw scan inclusion is not established without original frame/source provenance; shared optimizer history is a separate correlation question','postfit GNSS residuals are genuine factor inputs but not independent raw noise','IMU mean transports original scan geometry; F/Q uncertainty retained but not turned into added covariance','shared-map/cross-frame correlation and map accuracy not qualified','one temporally correlated diagnostic run, not independent trials'],'fit':False,'config_sha256':sha(cfgpath),'script_sha256':sha(Path(__file__))}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');write_subordinate_manifest(run,sys.argv[2],{'inputs_sha256':inputs,'artifacts_sha256':{str(p.relative_to(run)):sha(p) for p in out.glob('*')},'qualification':'same geometric object and epoch, actual recorded measurements, independent truth; limited model ordering test, not covariance calibration'})
print(json.dumps({k:v for k,v in summary.items() if k not in ['rows','limits']}));print([(v['frame_id'],v.get('reason', ''),v.get('real_associations'),round(v.get('fusion',{}).get('position_error_norm_m',-1),3)) for v in results])
