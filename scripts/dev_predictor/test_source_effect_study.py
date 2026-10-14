import copy
import csv
import gzip
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np
from g2_source_effect_study import pilot_protocol, fixed_reference, fixed_global_error, select_global_updates, phase_statistics, recovery_decomposition, REPO
from full_stack_runtime import load_advisory_trial, planner_parameters
import g2_source_effect_study as study


def global_row(sequence=1):
    origin, rotation = fixed_reference()
    row = dict(state_stamp_s='100', frame_id='1', update_sequence=str(sequence),
               pose_valid='1', datum_valid='1', fallback='0')
    for prefix, vector in (('p_world_imu', [3., 4., 2.]), ('anchor_ecef', origin)):
        row.update({f'{prefix}_{axis}': str(value) for axis, value in zip('xyz', vector)})
    row.update({f'R_ecef_world_{r}{c}': str(rotation[r, c]) for r in range(3) for c in range(3)})
    return row


class SourceEffectStudy(unittest.TestCase):
    def test_physical_refusal_without_matrix_stays_refused_not_finite(self):
        metadata=dict(K_H_adv=5.,K_V_adv=5.)
        for status in ('PHYSICAL_FILTERED','STALE','INPUT_UNAVAILABLE','MODEL_INVALID'):
            score=study.original_a0_diagnostic({'status':status}, {'joint_pose_information':None}, metadata)
            self.assertEqual(score['status'],status)
            self.assertFalse(score['valid']);self.assertIsNone(score['S'])
        with self.assertRaisesRegex(ValueError,'VALID_QUERY_WITHOUT'):
            study.original_a0_diagnostic({'status':'VALID'}, {'joint_pose_information':None}, metadata)
    def test_contrasts_retain_missing_near_floor_and_run_unit(self):
        def cell(local, global_error):
            def stats(value):
                return dict(count=8 if value is not None else 0, median=value,
                            last_minus_first_quartile_median=value,
                            phase_minimum_met=value is not None)
            return dict(local_phases={p:stats(local) for p in ('pre','absence','post')},
                global_phases={p:{f:stats(global_error) for f in ('horizontal_error_m','error_3d_m')}
                               for p in ('pre','absence','post')})
        cells = {'GG_LG':cell(.005,.1), 'GG_LB':cell(.0045,.15),
                 'GB_LG':cell(None,.09), 'GB_LB':cell(None,.2)}
        rows = study.paired_contrasts(cells)
        self.assertEqual(len(rows),36)
        near = next(r for r in rows if r['source']=='L' and r['good']=='GG_LG' and r['field']=='local_XY')
        self.assertFalse(near['above_detection_floor'])
        missing = next(r for r in rows if r['source']=='L' and r['good']=='GB_LG' and r['field']=='local_XY')
        self.assertIsNone(missing['bad_minus_good_m'])
        self.assertFalse(missing['phase_minimum_met'])
        self.assertTrue(all(r['independent_groups']==1 and not r['qualified_effect'] for r in rows))

    def test_four_cell_comparison_preserves_failed_response_requests_and_child_ownership(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); run = root/'output'; evaluations=[]
            selection = root/'selection.json'
            selection.write_text(json.dumps(dict(request_indices=[6,24,56],conditions=list(study.pilot_protocol()['conditions']))))
            stat=dict(count=0,median=None,last_minus_first_quartile_median=None,phase_minimum_met=False)
            for condition in study.pilot_protocol()['conditions']:
                value=dict(trial=dict(condition=condition,seed=9100),protocol=study.pilot_protocol(),
                    status='PILOT_DIAGNOSTIC_COMPLETE',source_lifecycle='completed',inputs_sha256={},
                    source_process=dict(required_input_failure=False,exit_codes=dict(capture=0,launch=0,record=0)),
                    source_run=str(root/condition),source_revision='UNIT_TEST_DOUBLE',producer_origin_stamp_s=100.,
                    local_rows=[dict(qualified=False,reason='MISSING') for _ in range(46)],global_rows=[],
                    actual_icp_support=[],physical_gnss_observations=[],actual_gnss_factors=[],recovery=None,
                    local_phases={p:stat for p in ('pre','absence','post')},
                    global_phases={p:{f:stat for f in ('horizontal_error_m','error_3d_m')} for p in ('pre','absence','post')},
                    requests=[dict(request=str(i),available='False') for i in (6,24,56)])
                path=root/condition/'export/analysis/unit/summary.json';path.parent.mkdir(parents=True)
                old_source=b'IMMUTABLE_EVALUATOR_BYTES_BEFORE_COMPARE_WAS_ADDED'
                snapshot=root/condition/'metadata/config/unit/source/g2_source_effect_study.py'
                snapshot.parent.mkdir(parents=True);snapshot.write_bytes(old_source)
                value['inputs_sha256']={str(Path(study.__file__).resolve()):hashlib.sha256(old_source).hexdigest()}
                path.write_text(json.dumps(value));evaluations.append(path)
            with mock.patch.object(study,'binary_identity',return_value={'unit_test_double':True}), \
                    mock.patch.object(study,'register_subordinate_manifest',side_effect=AssertionError('child changed owner')):
                result=study.compare(run,evaluations,selection,root/'decoder','compare',owner=False)
            self.assertEqual(len(result['responses']),12)
            self.assertTrue(all('ORIGINAL_REQUEST_UNAVAILABLE' in r['failure'] for r in result['responses']))
            self.assertTrue(all(c['local_requested']==46 and c['local_qualified']==0 for c in result['cells'].values()))
            self.assertFalse(result['model_qualified'])
            self.assertEqual(len(result['evaluation_source_snapshots']),4)
            snapshot.write_bytes(b'TAMPERED_HISTORICAL_SOURCE')
            with self.assertRaisesRegex(ValueError,'EVALUATOR_SNAPSHOT_BYTES_CHANGED'):
                study.compare(run,evaluations,selection,root/'decoder','changed_snapshot',owner=False)
            snapshot.write_bytes(old_source)
            # A native backend can emit evidence before returning nonzero.
            value=json.loads(evaluations[0].read_text())
            payload=root/'GG_LG/input.bin';payload.write_bytes(b'UNIT_TEST_DOUBLE')
            payload.with_name('record.json').write_text('{}')
            value['requests'][0].update(available='True',payload='input.bin')
            evaluations[0].write_text(json.dumps(value))
            def failed_backend(run, binary, arguments, name):
                target=run/'export/advisory/validation'/name;target.mkdir(parents=True)
                log=run/'runtime/ros'/('advisory_'+name+'.log');log.parent.mkdir(parents=True,exist_ok=True);log.write_text('UNIT_TEST_DOUBLE')
                if arguments[0]=='replay_header':
                    (target/'input.json').write_text(json.dumps(dict(frame_id='map',geometry_id='unit',generation=1,
                        current_motion_quality=1,reference_time_s=116.,epoch_source_identity=1,position=[0.,0.,0.])))
                else:
                    (target/'partial.csv').write_text('PARTIAL_FAILED_COMMAND_EVIDENCE')
                    raise subprocess.CalledProcessError(7,arguments)
            record=dict(frame_id='map',geometry_id='unit',generation=1,producer_binary={'libraries_sha256':{}})
            with mock.patch.object(study,'binary_identity',return_value={'libraries_sha256':{}}), \
                    mock.patch.object(study,'validate_record',return_value=record), \
                    mock.patch.object(study,'backend',side_effect=failed_backend):
                partial=study.compare(run,evaluations,selection,root/'decoder','failed_command',owner=False)
            response=partial['responses'][0]
            self.assertEqual(response['modes']['gnss']['exit_code'],7)
            failed_output=run/response['modes']['gnss']['retained_directory']/'partial.csv'
            self.assertEqual(partial['inputs_sha256'][str(failed_output.resolve())],study.sha(failed_output))
            self.assertEqual(len(partial['responses']),12)
            # Failed input identity cannot silently become another completed comparison.
            value=json.loads(evaluations[-1].read_text());value['source_process']['exit_codes']['record']=1
            evaluations[-1].write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'PILOT_PROCESS_HEALTH_FAILED'):
                study.compare(run,evaluations,selection,root/'decoder','failed_compare',owner=False)
            saved=json.loads((run/'export/analysis/failed_compare/summary.json').read_text())
            self.assertEqual(saved['status'],'COMPARISON_FAILED_PARTIAL_EVIDENCE_RETAINED')

    def test_half_pair_binding_and_missing_payload_do_not_rehabilitate_unqualified_queries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); run = root/'output'; source = root/'source'
            def write_json(path, value):
                path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value)); return path
            def write_csv(path, rows):
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('w') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
                return path
            protocol = write_json(source/'metadata/config/protocol.json', pilot_protocol())
            trial = dict(schema='iap_source_effect_trial_v1', condition='GG_LG', seed=9100,
                         protocol=str(protocol), protocol_sha256=study.sha(protocol))
            write_json(source/'metadata/run_manifest.json', dict(entrypoint='iap_sim',
                scenario='icra_dense_forest_four_fork_v2', lifecycle='failed',
                source={'git_worktree_clean':True, 'git_commit':'UNIT_TEST_DOUBLE'}, validation_trial=trial))
            recorder = source/'export/advisory/validation/recordings'
            requests = []
            for i in range(6):
                payload = recorder/f'unit_{i:04}'/'input.bin'; payload.parent.mkdir(parents=True, exist_ok=True)
                if i != 0: payload.write_bytes(b'UNIT_TEST_DOUBLE')
                write_json(payload.with_name('record.json'), {'unit':True})
                requests.append(dict(request=str(i), request_id=f'unit:{i}', available='False' if i==5 else 'True',
                                     payload=str(payload.relative_to(source))))
            ledger = write_csv(recorder/'unit_requests.csv', requests)
            write_json(recorder/'unit_requests_manifest.json', dict(requests_csv=str(ledger.relative_to(source)),
                requests_csv_sha256=study.sha(ledger), request_ids=[r['request_id'] for r in requests],
                sampling_plan={'count':92, 'interval_s':2., 'initial_delay_s':10.}))
            # A process-manifest failure must retain all46 pairs and the
            # available endpoint of the failed half-pair (requests4/5).
            with self.assertRaises(FileNotFoundError):
                study.evaluate(run, source, 'missing_process', root/'decoder', owner=False)
            partial = json.loads((run/'export/analysis/missing_process/summary.json').read_text())
            self.assertEqual(len(partial['local_rows']), 46)
            self.assertIn(str((recorder/'unit_0004/input.bin').resolve()), partial['inputs_sha256'])
            self.assertTrue(partial['request_bindings']['unit:0']['errors'])
            write_json(source/'metadata/manifests/forest_process_result.json', {'unit_test_double':True})
            poses = [dict(source='truth', stamp=str(t), frame='map', body='drone_0', x='3', y='4', z='2',
                          qx='0', qy='0', qz='0', qw='1') for t in (100.,101.)]
            write_csv(recorder/'unit_odometry.csv', poses)
            with gzip.open(recorder/'unit_native_reports.jsonl.gz', 'wt'): pass
            write_csv(source/'export/simulation/gnss_injected_noise.csv', [dict(stamp='100',scenario_time_s='0')])
            native = global_row(); native.update(pr_count='8', dop_count='8', gnss_epoch_stamp_s='100')
            write_csv(source/'export/glio/gnss_global_state.csv', [native])
            write_csv(source/'export/glio/iap_icp.csv', [dict(stamp='100',frame_id='1')])
            write_csv(source/'export/glio/iap_gnss_factor_debug.csv', [dict(stamp='100',frame_id='1')])
            decoded = dict(frame_id='map', geometry_id='unit', generation=1, current_motion_quality=0,
                           reference_time_s=100., estimation_frame_id=1, position=[3.,4.,2.],
                           coordinates={'T_map_world':np.eye(4).tolist()})
            def decode(_, binary, arguments, name):
                write_json(run/'export/advisory/validation'/name/'input.json', decoded)
            recorded = dict(frame_id='map', geometry_id='unit', generation=1, producer_binary={'libraries_sha256':{}})
            with mock.patch.object(study, 'binary_identity', return_value={'libraries_sha256':{}}), \
                    mock.patch.object(study, 'validate_record', return_value=recorded), \
                    mock.patch.object(study, 'backend', side_effect=decode), \
                    mock.patch.object(study, 'native_endpoint', side_effect=AssertionError('later report rehabilitated query')):
                result = study.evaluate(run, source, 'admission', root/'decoder', owner=False)
            self.assertIn('FileNotFoundError', result['local_rows'][0]['reason'])
            self.assertIn('ORIGINAL_QUERY_MOTION_QUALITY_UNQUALIFIED', result['local_rows'][1]['reason'])
            self.assertIn('ORIGINAL_PAIR_REQUEST_UNAVAILABLE', result['local_rows'][2]['reason'])
            self.assertEqual(len(result['local_rows']), 46)
            self.assertFalse(result['source_effects_qualified'])

    def test_phase_effect_keeps_run_as_unit_and_recovery_split_is_exact(self):
        rows = [dict(elapsed_s=i, horizontal_error_m=float(i), qualified=True) for i in range(11)]
        stats = phase_statistics(rows, 'horizontal_error_m', [0., 10.])
        self.assertEqual(stats['independent_sample_count'], 1)
        self.assertEqual(stats['median'], 5.)
        self.assertEqual(stats['last_minus_first_quartile_median'], 8.)
        a = fixed_global_error(global_row(), [3., 4., 2.])
        changed = global_row(); changed['p_world_imu_x'] = '4'
        origin, enu = fixed_reference()
        shifted = origin+enu @ [.2, .3, .4]
        changed.update({f'anchor_ecef_{axis}':str(value) for axis, value in zip('xyz', shifted)})
        b = fixed_global_error(changed, [3., 4., 2.])
        previous = dict(stamp_s=100., truth_position=[3.,4.,2.]); previous['global']=a
        current = dict(stamp_s=100.1, truth_position=[3.,4.,2.]); current['global']=b
        split = recovery_decomposition(previous, current)
        total = np.array(split['world_pose_change_fixed_enu_m'])+split['datum_change_fixed_enu_m']
        np.testing.assert_allclose(total, enu.T @ (np.array(b['estimate_ecef_m'])-a['estimate_ecef_m']), atol=1e-8)

    def test_evaluator_failure_preserves_frozen_sources_and_partial_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); run = root/'output'; source = root/'source'
            (source/'metadata').mkdir(parents=True)
            primary = source/'metadata/run_manifest.json'
            primary.write_text(json.dumps(dict(entrypoint='iap_sim', scenario='icra_dense_forest_four_fork_v2',
                source={'git_worktree_clean':False})))
            with self.assertRaisesRegex(ValueError, 'CLEAN_CANONICAL'):
                study.evaluate(run, source, 'failure', root/'unused_decoder', owner=False)
            saved = json.loads((run/'export/analysis/failure/summary.json').read_text())
            self.assertEqual(saved['status'], 'EVALUATION_FAILED_PARTIAL_EVIDENCE_RETAINED')
            self.assertFalse(saved['model_qualified'])
            self.assertIn(str(primary.resolve()), saved['inputs_sha256'])
            self.assertEqual((run/'metadata/config/failure/source/g2_source_effect_study.py').read_bytes(), Path(study.__file__).read_bytes())
    def test_one_analytic_global_reference_includes_datum_error(self):
        origin, enu = fixed_reference()
        np.testing.assert_allclose(fixed_reference((0., 0., 0.))[0], [6378137., 0., 0.])
        row = global_row()
        self.assertLess(fixed_global_error(row, [3., 4., 2.])['error_3d_m'], 1e-8)
        shifted = origin + enu @ [2., -1., .5]
        row.update({f'anchor_ecef_{axis}': str(value) for axis, value in zip('xyz', shifted)})
        np.testing.assert_allclose(fixed_global_error(row, [3., 4., 2.])['error_fixed_enu_m'],
                                   [2., -1., .5], atol=1e-8)

    def test_latest_native_update_selected_before_truth_and_invalid_not_reused(self):
        first, latest = global_row(), global_row(2)
        latest['datum_valid'] = '0'
        selected = select_global_updates([latest, first])
        self.assertEqual(selected, [latest])
        with self.assertRaisesRegex(ValueError, 'UNAVAILABLE'):
            fixed_global_error(selected[0], [3., 4., 2.])
        conflict = copy.deepcopy(first); conflict['anchor_ecef_x'] = '0'
        with self.assertRaisesRegex(ValueError, 'CONFLICTING'):
            select_global_updates([first, conflict])

    def test_four_cells_preserve_original_planner_and_reject_protocol_tampering(self):
        scenario = json.loads((REPO / 'config/scenarios/catalog.json').read_text())['icra_dense_forest_four_fork_v2']
        baseline = planner_parameters(scenario, advisory_guidance=False)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = root / 'protocol.json'; protocol.write_text(json.dumps(pilot_protocol()))
            for condition in pilot_protocol()['conditions']:
                trial = root / 'trial.json'
                value = dict(schema='iap_source_effect_trial_v1', scene='icra_dense_forest_four_fork_v2',
                             map_seed=41021, seed=9100, phase='pilot', condition=condition,
                             protocol=str(protocol), protocol_sha256=hashlib.sha256(protocol.read_bytes()).hexdigest())
                trial.write_text(json.dumps(value))
                loaded = load_advisory_trial(str(trial), scenario)
                self.assertEqual(loaded['observation'], pilot_protocol()['conditions'][condition])
                self.assertEqual(planner_parameters(scenario, advisory_guidance=False, advisory_trial=str(trial)), baseline)
                with self.assertRaisesRegex(ValueError, 'OFF'):
                    planner_parameters(scenario, advisory_guidance=True, advisory_trial=str(trial))
            value.update(phase='formal', seed=9101); trial.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'bounded'):
                load_advisory_trial(str(trial), scenario)
            value.update(phase='pilot', seed=9100); trial.write_text(json.dumps(value))
            modified = pilot_protocol(); modified['record_interval_s'] = 3.
            protocol.write_text(json.dumps(modified))
            with self.assertRaisesRegex(ValueError, 'checksum'):
                load_advisory_trial(str(trial), scenario)
            value['protocol_sha256'] = hashlib.sha256(protocol.read_bytes()).hexdigest(); trial.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'bounded'):
                load_advisory_trial(str(trial), scenario)

    def test_inherited_preparation_neither_allocates_nor_mutates_or_finalizes_owner(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {'IAP_RUN_ROOT':directory}):
            run = study.resolve_run_directory(entrypoint='g2_source_effect_preregistration')
            primary = run/'metadata/run_manifest.json'; original = primary.read_bytes()
            with mock.patch.dict(os.environ, {'IAP_RUN_DIR':str(run)}), \
                    mock.patch.object(sys, 'argv', ['study', 'prepare']), \
                    mock.patch.object(study, 'resolve_run_directory', side_effect=AssertionError('child allocated')), \
                    mock.patch.object(study, 'finalize_run', side_effect=AssertionError('child finalized')):
                study.main()
            self.assertEqual(primary.read_bytes(), original)
            self.assertTrue((run/'metadata/manifests/g2_source_effect_preregistration.json').is_file())
            study.finalize_run(run, lifecycle='completed')


if __name__ == '__main__': unittest.main()
