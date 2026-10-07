import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from advisory_coordinate_evidence import audit, audit_postopt
from advisory_forest_report import truth_at, csv_write, analyze, information_diagnostics, unique_trials, validated_replays, digest, araim_rows


class ForestReportContract(unittest.TestCase):
    def coordinate_input(self):
        eye3=np.eye(3).tolist();eye4=np.eye(4).tolist()
        return {'recording_codec_version':8,'clock_model':'per_constellation_pseudorange_bias_v1',
                'gnss_fault_model':'single_satellite_and_constellation_v1',
                'frame_id':'map','estimation_frame_id':'optimized','current_stamp':100.,'pose_stamp':100.,
                'reference_time_s':100.,'gnss_stamp':99.95,'gps_sec':117.95,'has_epoch':True,
                'epoch_source_identity':'original_epoch','gnss_satellites':[{'excluded':False,'ecef':[0.,10.,0.],
                    'azimuth':0.,'elevation':0.}],
                'coordinates':{'required':True,'valid':True,'reason':'','map_frame':'map','body_frame':'imu',
                    'stamp':100.,'frame_id':'optimized','epoch_source_identity':'original_epoch',
                    'R_ecef_world':eye3,'R_ecef_enu':eye3,'R_map_enu':eye3,
                    'T_map_world':eye4,'T_world_imu':eye4,'T_lidar_imu':eye4,
                    'lever_arm_imu':[0.,0.,0.],'anchor_ecef':[0.,0.,0.]}}

    def test_current_fault_model_can_audit_coordinates_without_granting_uncertainty(self):
        result=audit(self.coordinate_input())
        self.assertTrue(result['valid']);self.assertEqual(result['directions_checked'],1)
        self.assertIn('uncertainty is not propagated',result['conditioning'])

    def test_old_or_unknown_models_cannot_acquire_current_coordinate_qualification(self):
        for key,value in [('recording_codec_version',7),('clock_model','unknown'),('gnss_fault_model','unknown')]:
            with self.subTest(key=key):
                meta=self.coordinate_input();meta[key]=value
                with self.assertRaisesRegex(ValueError,'production_coordinate_evidence_unavailable'):audit(meta)

    def test_current_codec_still_checks_original_epoch_time_and_direction(self):
        for key,value,reason in [('pose_stamp',100.1,'time_mismatch'),
                                ('epoch_source_identity','other_epoch','epoch_identity_mismatch')]:
            with self.subTest(key=key):
                meta=self.coordinate_input();meta[key]=value
                with self.assertRaisesRegex(ValueError,reason):audit(meta)
        meta=self.coordinate_input();meta['gnss_satellites'][0]['azimuth']=.1
        with self.assertRaisesRegex(ValueError,'direct_ecef_direction_mismatch'):audit(meta)

    def postopt_input(self):
        meta=self.coordinate_input();meta['estimation_frame_id']=meta['coordinates']['frame_id']=17
        meta['epoch_source_identity']=meta['coordinates']['epoch_source_identity']=101
        key=lambda symbol,index:(ord(symbol)<<56)|index
        means=np.eye(4).flatten().tolist()+[0.]*9+np.eye(3).flatten().tolist()+[0.]*7
        covariance=np.eye(25);covariance[21,23]=covariance[23,21]=.4
        meta['postopt_evidence']={'model':'postopt_X_V_B_R_E_active_clocks_joint_v1',
            'update_sequence':2,'frame_id':17,'state_stamp':100.,'gnss_stamp':99.95,
            'epoch_source_identity':101,'used_constellations':'CG','optimized_valid':True,
            'covariance_valid':True,'propagation':'NOT_PROPAGATED',
            'keys':[key(k,i) for k,i in [('x',17),('v',17),('b',17),('r',0),('e',0),('d',17),('c',17)]],
            'tangent_dimensions':[6,3,6,3,3,2,2],'mean_dimensions':[16,3,6,9,3,2,2],
            'optimized_means':means,'linearization_means':means,
            'joint_covariance_row_major':covariance.flatten().tolist()}
        return meta

    def test_postopt_covariance_uses_cross_terms_and_retains_acquisition_gap(self):
        result=audit_postopt(self.postopt_input())
        self.assertTrue(result['available']);self.assertEqual(result['joint_dimension'],25)
        self.assertAlmostEqual(result['clock_difference_joint_covariance']['C-G'][0][0],1.2)
        self.assertAlmostEqual(result['state_gnss_delta_s'],.05)
        self.assertFalse(result['meter_qualified']);self.assertFalse(result['time_propagation_qualified'])

    def test_mixed_owner_and_invalid_joint_evidence_do_not_gain_qualification(self):
        for field,value,reason in [('frame_id',18,'owner_mismatch'),('gnss_stamp',100.,'owner_mismatch'),
                                  ('keys',[1],'layout_mismatch'),('joint_covariance_row_major',[0.],'data_invalid')]:
            with self.subTest(field=field):
                meta=self.postopt_input();meta['postopt_evidence'][field]=value
                with self.assertRaisesRegex(ValueError,reason):audit_postopt(meta)
        meta=self.postopt_input();meta['postopt_evidence']['covariance_valid']=False
        meta['postopt_evidence']['failure_reason']='missing_current_covariance'
        result=audit_postopt(meta);self.assertFalse(result['available']);self.assertFalse(result['meter_qualified'])
        self.assertEqual(result['reason'],'missing_current_covariance')

    def test_misaligned_hypothesis_rows_are_not_zero_or_valid_counts(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'araim.csv'
            path.write_text('row_type,n_const,gnss_valid,sat_id\nepoch,1,0,\nworst_hyp,,,-1,,\n')
            value=araim_rows(path)
            self.assertEqual(value['epoch_rows'],1)
            self.assertEqual(value['gnss_rejected_epochs'],1)
            self.assertEqual(value['invalid_rows_by_type'],{'worst_hyp':1})
            self.assertIsNone(value['worst_hyp_sat_id_minus_one'])

    def test_valid_hypothesis_identity_uses_recorded_header(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'araim.csv'
            path.write_text('row_type,n_const,gnss_valid,sat_id\nepoch,1,0,\nworst_hyp,,,-1\n')
            self.assertEqual(araim_rows(path)['worst_hyp_sat_id_minus_one'],1)

    def test_repeated_trial_does_not_inflate_independent_count(self):
        with tempfile.TemporaryDirectory() as root:
            p=Path(root)
            with self.assertRaisesRegex(ValueError,'duplicate'):
                unique_trials([p,p])

    def test_changed_replay_artifact_cannot_be_real_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            run=Path(root);folder=run/'export/advisory/validation/a';folder.mkdir(parents=True)
            for name in ('input.bin','input.json','points.csv','matrices.jsonl','variant.json'):
                (folder/name).write_text('{}')
            manifest={'identity':'REAL_REPLAY','dirty':'','source_sha256':{'model':'hash'},'binary':{'libraries_sha256':{'lib':'hash'}},
                      'artifacts_sha256':{str(p.relative_to(run)):digest(p) for p in folder.iterdir()}}
            target=run/'metadata/manifests/advisory_replay_a.json';target.parent.mkdir(parents=True);target.write_text(json.dumps(manifest))
            (folder/'points.csv').write_text('changed')
            with self.assertRaisesRegex(ValueError,'checksum mismatch'):
                validated_replays(run)
    def test_production_matrix_object_and_missing_diagnostic(self):
        value={'row_major':[1,0,0,0,2,0,0,0,3],'eigenvalues':[1,2,3],'weak_direction':[1,0,0]}
        self.assertEqual(information_diagnostics(value)['lambda_min'],1)
        self.assertEqual(information_diagnostics(value)['weak_x'],1)
        self.assertIsNone(information_diagnostics(None)['lambda_min'])
    def test_saved_time_interpolation_without_extrapolation(self):
        actual=truth_at([10.,10.04],[[0,0,0],[4,2,0]],10.02)
        np.testing.assert_allclose(actual,[2,1,0])
        with self.assertRaisesRegex(ValueError,'unmatched'):
            truth_at([10.,10.04],[[0,0,0],[4,2,0]],9.99)

    def test_both_truth_endpoints_must_meet_fixed_tolerance(self):
        with self.assertRaisesRegex(ValueError,'gap'):
            truth_at([10.,10.1],[[0,0,0],[4,2,0]],10.01)

    def test_missing_pl_is_blank_and_failure_remains_in_table(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'raw.csv'
            csv_write(path,[{'valid':False,'pl':None,'reason':'input_missing'}],['valid','pl','reason'])
            self.assertEqual(path.read_text().splitlines()[1],'False,,input_missing')

    def test_fixture_or_dirty_run_cannot_be_counted_as_live_trial(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root);(path/'metadata').mkdir()
            manifest={'entrypoint':'iap_sim','scenario':'icra_dense_forest_four_fork_v2',
                      'lifecycle':'completed','source':{'git_worktree_clean':False}}
            (path/'metadata/run_manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError,'clean canonical'):
                analyze(path,lambda _:None)

if __name__=='__main__':unittest.main()
