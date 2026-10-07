import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from advisory_forest_report import truth_at, csv_write, analyze, information_diagnostics, unique_trials, validated_replays, digest, araim_rows


class ForestReportContract(unittest.TestCase):
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
