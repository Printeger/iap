#!/usr/bin/env python3
"""Production multi-goal replay must expose partial prediction coverage."""
import csv
import json
import os
from pathlib import Path
import subprocess
import unittest
import test_failure_map_tools as fixtures
from analyze_failure_map import _backend_path
from replay_on_search import replay_input


class OnSearchReplayTest(unittest.TestCase):
    def test_missing_pl_stays_unknown_and_both_runs_use_full_goal_set(self):
        fixture=fixtures.FailureMapToolsTest()
        fixture.setUp()
        try:
            fixture.meta.update(run_id='fixture',planning_attempt_id=1,
                real_start_p_m=fixture.meta['search_requested_start_m'],
                planning_goals_m=[fixture.meta['search_requested_end_m'],[-.5,.5,.25]],
                guide_fitting_reserve_m=0.,guide_reserve_taper_distance_m=.5,
                risk_reference_time_s=10.1,risk_valid_until_s=10.5)
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
