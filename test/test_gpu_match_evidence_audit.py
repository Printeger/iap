import importlib.util
from pathlib import Path
import unittest
import numpy as np
import csv
import json
import tempfile
import os

path=Path(__file__).resolve().parents[1]/"scripts/dev_predictor/gpu_match_evidence_audit.py"
spec=importlib.util.spec_from_file_location("gpu_audit",path)
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)

class ActualSampleAudit(unittest.TestCase):
    def test_unmatched_native_sources_are_separate_from_original_matches(self):
        match, _ = self.fixture()
        match["source_index"] = 0
        packet = {"schema":"iap_gpu_match_residual_v2", "available":True,
            "source_geometry_model":"actual_cuda_source_gaussians_v1",
            "source_geometry_available":True, "source_slots":2,
            "original_source_count":2, "sampling_stride":1, "sample_limit":128,
            "samples":[match], "source_samples":[
                {"source_index":0,"mean_m":match["source_mean_m"],
                 "covariance_m2_row_major":match["source_covariance_m2_row_major"]},
                {"source_index":1,"mean_m":[10.,20.,30.],
                 "covariance_m2_row_major":np.eye(3).ravel().tolist()}]}
        self.assertEqual(audit.audit_source_geometry(packet),2)
        self.assertEqual(audit.audit_source_geometry({**packet,"samples":[]}),2)
        self.assertEqual(audit.audit_source_geometry({**packet,"schema":"iap_gpu_match_residual_v1"}),0)
        for field, value in [("available",False),("source_geometry_available",False),
                             ("source_slots",1),("source_samples",packet["source_samples"][:1])]:
            with self.subTest(field=field),self.assertRaises(ValueError):
                audit.audit_source_geometry({**packet,field:value})
        for field, value in [("source_index",0),("mean_m",[float('nan'),0.,0.]),
                             ("covariance_m2_row_major",[-1.]+[0.]*8)]:
            changed=json.loads(json.dumps(packet));changed["source_samples"][1][field]=value
            with self.subTest(source_field=field),self.assertRaises(ValueError):
                audit.audit_source_geometry(changed)
        changed=json.loads(json.dumps(packet));changed["source_samples"][0]["mean_m"][0]+=.1
        with self.assertRaisesRegex(ValueError,"differs from original"):
            audit.audit_source_geometry(changed)

    def test_captured_dense_float_covariance_is_not_an_interface_error(self):
        path=os.environ.get('IAP_DENSE_COVARIANCE_COUNTEREXAMPLE')
        if not path:self.skipTest('requires immutable actual dense counterexample')
        cases=json.loads(Path(path).read_text())
        inherited=Path(path).with_name('input_symmetry_rejections.json')
        if inherited.exists():cases+=json.loads(inherited.read_text())
        for case in cases:
            record=case['record'];transform=np.array(case['transform']);before=json.dumps(record,sort_keys=True)
            with self.subTest(run=case['source_run']):
                audit.audit_sample(record,transform)
                self.assertEqual(json.dumps(record,sort_keys=True),before)
                changed={**record,'covariance_m2_row_major':list(record['covariance_m2_row_major'])}
                changed['covariance_m2_row_major'][1]+=1e-3
                with self.assertRaisesRegex(ValueError,'covariance'):audit.audit_sample(changed,transform)

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
        record["source_covariance_m2_row_major"]=[0.]*9
        record["target_covariance_m2_row_major"]=[0.]*9
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
            # Full source capture may exceed the old 128 sample cap. Its declared
            # slots and actual reduction still have to close exactly.
            dense={**original,"original_source_count":150,"original_inlier_count":150,
                "sample_limit":16384,"source_slots":150,"original_cost":150*record["mahalanobis_squared"],
                "samples":[{**record,"source_index":i} for i in range(150)]}
            fields+=['sample_limit','source_slots']
            row.update(source_count=150,inlier_count=150,samples=150,cost=dense['original_cost'],sample_limit=16384,source_slots=150)
            with (run/'export/glio/gpu_match_requests.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerow(row)
            summary['samples_written']=150;(run/'metadata/manifests/gpu_match_evidence.json').write_text(json.dumps(summary))
            raw.write_text(json.dumps(dense)+'\n');self.assertEqual(audit.audit(run)[0]['samples'],150)
            dense['source_slots']=149;raw.write_text(json.dumps(dense)+'\n')
            with self.assertRaisesRegex(ValueError,'slot ledger'):audit.audit(run)
            dense.update(schema="iap_gpu_match_residual_v2",source_slots=150,
                         source_geometry_available=True,source_geometry_model="actual_cuda_source_gaussians_v1",
                         source_samples=[{"source_index":i,"mean_m":record["source_mean_m"],
                             "covariance_m2_row_major":record["source_covariance_m2_row_major"]} for i in range(150)])
            fields += ['source_geometry_available','source_samples']
            row.update(source_geometry_available=1,source_samples=150)
            with (run/'export/glio/gpu_match_requests.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerow(row)
            summary['source_samples_written']=150
            (run/'metadata/manifests/gpu_match_evidence.json').write_text(json.dumps(summary))
            raw.write_text(json.dumps(dense)+'\n')
            self.assertEqual(audit.audit(run)[0]['native_source_samples'],150)
            self.assertEqual(audit.audit(run)[0]['complete_native_source_captures'],1)

if __name__=="__main__":unittest.main()
