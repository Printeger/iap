import importlib.util
from pathlib import Path
import unittest
import numpy as np
import csv
import json
import tempfile

path=Path(__file__).resolve().parents[1]/"scripts/dev_predictor/gpu_match_evidence_audit.py"
spec=importlib.util.spec_from_file_location("gpu_audit",path)
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)

class ActualSampleAudit(unittest.TestCase):
    def fixture(self):
        transform=np.eye(4);transform[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];transform[:3,3]=[.2,-.1,.3]
        a=np.array([1.,2.,3.]);e=np.array([.1,-.2,.3]);b=transform[:3,:3]@a+transform[:3,3]+e
        ca=np.diag([.01,.02,.03]);cb=np.diag([.04,.05,.06]);cov=cb+transform[:3,:3]@ca@transform[:3,:3].T
        record={"residual_m":e.tolist(),"covariance_m2_row_major":cov.ravel().tolist(),
            "source_covariance_m2_row_major":ca.ravel().tolist(),"target_covariance_m2_row_major":cb.ravel().tolist(),
            "source_mean_m":a.tolist(),"target_mean_m":b.tolist(),"mahalanobis_squared":float(e@np.linalg.solve(cov,e))}
        return record,transform

    def test_rotated_target_frame_independent_metric(self):
        record,transform=self.fixture();e,w,m,re,ce=audit.audit_sample(record,transform)
        self.assertAlmostEqual(m,.01/.06+.04/.06+.09/.09)
        self.assertLess(re,1e-12);self.assertLess(ce,1e-12)

    def test_wrong_pose_or_count_weight_rejected(self):
        record,transform=self.fixture();transform[0,3]+=.1
        with self.assertRaisesRegex(ValueError,"transform"):audit.audit_sample(record,transform)
        record,transform=self.fixture();record["mahalanobis_squared"]*=2
        with self.assertRaisesRegex(ValueError,"Mahalanobis"):audit.audit_sample(record,transform)

    def test_singular_nonfinite_and_invalid_cost_rejected(self):
        record,transform=self.fixture();record["covariance_m2_row_major"]=[0.]*9
        with self.assertRaises(np.linalg.LinAlgError):audit.audit_sample(record,transform)
        record,transform=self.fixture();record["residual_m"][0]=float('nan')
        with self.assertRaisesRegex(ValueError,"nonfinite"):audit.audit_sample(record,transform)
        for cost in (-1.,float('inf')):
            record,transform=self.fixture();record["mahalanobis_squared"]=cost
            with self.assertRaisesRegex(ValueError,"invalid actual GPU cost"):audit.audit_sample(record,transform)

    def test_raw_identity_must_match_request_ledger(self):
        record,transform=self.fixture();record.update(source_index=0,target_index=0,target_point_count=2)
        original={"schema":"iap_gpu_match_residual_v1","model":"actual_cuda_postopt_quality_linearization_v1",
            "request_id":1,"capture_requested":True,"source_frame_id":2,"target_frame_id":1,
            "source_stamp":1001.,"target_stamp":1000.,"level_id":0,"target_is_fixed":False,"voxel_resolution_m":.5,
            "factor_keys":[(ord('x')<<56)+1,(ord('x')<<56)+2],"sequence":1,
            "available":True,"failure_reason":"","original_source_count":1,"original_inlier_count":1,
            "original_cost":record["mahalanobis_squared"],"sampling_stride":1,"samples":[record],
            "linearization_transform_row_major":transform.ravel().tolist(),"T_world_source_row_major":transform.ravel().tolist(),
            "T_world_target_row_major":np.eye(4).ravel().tolist(),"T_lidar_imu_row_major":np.eye(4).ravel().tolist(),
            "gnss":{"owner_matches":False,"frame_id":-1,"update_sequence":0,"epoch_source_identity":0,
                "state_stamp":0.,"epoch_stamp":0.,"used_constellations":"","propagation":"NOT_PROPAGATED"}}
        with tempfile.TemporaryDirectory() as temporary:
            run=Path(temporary);(run/'export/glio').mkdir(parents=True);(run/'metadata/manifests').mkdir(parents=True)
            summary={"entries":1,"requested":1,"available":1,"queue_dropped":0,"write_failures":0,"written":1,"samples_written":1}
            (run/'metadata/manifests/gpu_match_evidence.json').write_text(json.dumps(summary))
            fields=['request_id','source_frame_id','source_stamp','target_frame_id','level_id','capture_requested','available','samples','queue_accepted','failure_reason',
                'target_stamp','target_is_fixed','voxel_resolution_m','key_count','key0','key1','gnss_frame_id','gnss_update_sequence',
                'gnss_epoch_identity','gnss_state_stamp','gnss_epoch_stamp','gnss_owner_matches','used_constellations','sequence','source_count','inlier_count','stride','cost']
            row={key:original[key] for key in fields if key in original}
            row.update(capture_requested=1,available=1,samples=1,queue_accepted=1,target_is_fixed=0,key_count=2,
                key0=original['factor_keys'][0],key1=original['factor_keys'][1],gnss_frame_id=-1,gnss_update_sequence=0,
                gnss_epoch_identity=0,gnss_state_stamp=0.,gnss_epoch_stamp=0.,gnss_owner_matches=0,used_constellations='',
                source_count=1,inlier_count=1,stride=1,cost=record['mahalanobis_squared'])
            with (run/'export/glio/gpu_match_requests.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerow(row)
            raw=run/'export/glio/gpu_match_residuals.jsonl';raw.write_text(json.dumps(original)+'\n')
            self.assertEqual(audit.audit(run)[0]['samples'],1)
            for field,value in [('source_stamp',1002.),('target_stamp',1001.),('level_id',1),('capture_requested',False),
                ('failure_reason','borrowed'),('target_is_fixed',True),('voxel_resolution_m',1.),('factor_keys',[0,1]),('sequence',2)]:
                changed=dict(original);changed[field]=value;raw.write_text(json.dumps(changed)+'\n')
                with self.subTest(field=field),self.assertRaises(ValueError):audit.audit(run)
            for field,value in [('frame_id',2),('update_sequence',2),('epoch_source_identity',9),('state_stamp',1001.),
                ('epoch_stamp',1000.),('owner_matches',True),('used_constellations','CG')]:
                changed=dict(original);changed['gnss']={**original['gnss'],field:value};raw.write_text(json.dumps(changed)+'\n')
                with self.subTest(gnss_field=field),self.assertRaises(ValueError):audit.audit(run)

if __name__=="__main__":unittest.main()
