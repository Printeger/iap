#!/usr/bin/env python3
"""Production multi-goal replay must expose partial prediction coverage."""
import csv
import json
import os
from pathlib import Path
import subprocess
import unittest
import tempfile
import test_failure_map_tools as fixtures
from analyze_failure_map import _backend_path
from replay_on_search import replay_input, verify_planning_capture
from run_curve_channel_live import finalized_planning_inputs
from advisory_validation import sha


class OnSearchReplayTest(unittest.TestCase):
    def test_pending_unregistered_and_non_search_inputs_cannot_trigger_early_stop(self):
        with tempfile.TemporaryDirectory() as temporary:
            run=Path(temporary)
            manifests=run/'metadata/manifests';manifests.mkdir(parents=True)
            directory=run/'export/planner/failure_map/timeout.pending';directory.mkdir(parents=True)
            (directory/'planning_input.bin').write_bytes(b'payload')
            self.assertEqual(finalized_planning_inputs(run),[])
            directory.rename(directory.with_name('timeout'));directory=directory.with_name('timeout')
            (directory/'snapshot.json').write_text(json.dumps({'planning_goals_m':[], 'search_stage':None}))
            self.assertEqual(finalized_planning_inputs(run),[])
            (manifests/'planner_failure_map_timeout.json').write_text(json.dumps({
                'planning_input':'planner/failure_map/timeout/planning_input.bin'}))
            self.assertEqual(len(finalized_planning_inputs(run)),1)
            self.assertEqual(finalized_planning_inputs(run,require_search=True),[])

    def test_registered_capture_rejects_changed_risk_identity_and_payload(self):
        with tempfile.TemporaryDirectory() as temporary:
            run=Path(temporary);snapshot=run/'export/planner/failure_map/timeout';snapshot.mkdir(parents=True)
            manifests=run/'metadata/manifests';manifests.mkdir(parents=True)
            meta={'run_manifest':'../../../../metadata/run_manifest.json','planning_attempt_id':5,
                'planning_input_risk_version':24,'risk_version':24,'generation':23,'planning_time_s':10.}
            (snapshot/'snapshot.json').write_text(json.dumps(meta))
            payload=snapshot/'planning_input.bin';payload.write_bytes(b'payload')
            (run/'metadata/run_manifest.json').write_text(json.dumps({'source':{'git_worktree_clean':True,'git_commit':'revision'}}))
            entry={'payload':str(payload.relative_to(run)), 'snapshot':str((snapshot/'snapshot.json').relative_to(run)),
                'payload_sha256':sha(payload),'snapshot_sha256':sha(snapshot/'snapshot.json'),
                'planning_attempt_id':5,'risk_version':24,'generation':23,'reference_time_s':10.}
            (manifests/'planning_input_capture.json').write_text(json.dumps({'source':{'dirty':'','revision':'revision'},'inputs':[entry]}))
            self.assertEqual(verify_planning_capture(snapshot,payload,meta),entry)
            with self.assertRaisesRegex(ValueError,'identity mismatch'):
                verify_planning_capture(snapshot,payload,{**meta,'planning_input_risk_version':25})
            payload.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'identity mismatch'):
                verify_planning_capture(snapshot,payload,meta)

    def test_missing_pl_stays_unknown_and_both_runs_use_full_goal_set(self):
        fixture=fixtures.FailureMapToolsTest()
        fixture.setUp()
        try:
            fixture.meta.update(run_id='fixture',planning_attempt_id=1,
                real_start_p_m=fixture.meta['search_requested_start_m'],
                planning_goals_m=[fixture.meta['search_requested_end_m'],[-.5,.5,.25]],
                guide_fitting_reserve_m=0.,guide_reserve_taper_distance_m=.5,
                risk_reference_time_s=10.1,risk_valid_until_s=10.5)
            fixture.meta.update(virtual_ceiling_height_m=.5,inflation_radius_m=0.)
            fixture.write()
            params={'fsm/waypoint0_'+a:v for a,v in zip('xyz',[.5,0,.25])}
            params.update({'planning/'+k:v for k,v in zip(
                ('advisory_hpl_budget_m','advisory_vpl_budget_m','advisory_hpl_reserve_m',
                 'advisory_vpl_reserve_m','advisory_unknown_multiplier','advisory_stale_soft_s'),
                (.55,.6,.1,.1,1.5,1.))})
            risk=fixture.directory/'queried_risk.csv'
            risk.write_text('address,hpl_m,vpl_m,status,version,source_flags\n')
            reports={}
            for mode in ('off','sparse'):
                result=subprocess.run([str(_backend_path()),str(fixture.directory/'cells.bin'),
                    '--planning-search',mode,str(risk)],input=replay_input(fixture.meta,params,1.),
                    text=True,capture_output=True,timeout=5)
                self.assertEqual(result.returncode,0,result.stderr)
                reports[mode]=json.loads(result.stdout)
                self.assertTrue(reports[mode]['guide_found'])
                self.assertEqual(reports[mode]['predictor_calls'],0)
            self.assertGreater(reports['sparse']['missing_unique_voxels'],0)
            self.assertGreater(reports['sparse']['risk_cost_m'],0)
            self.assertEqual(reports['off']['risk_cost_m'],0)
        finally:
            fixture.tearDown()

if __name__=='__main__':unittest.main()
