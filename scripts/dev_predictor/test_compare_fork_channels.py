import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from advisory_validation import resolve_run_directory, finalize_run, backend
from compare_fork_channels import verify_libraries, route_risk_covered

parser=argparse.ArgumentParser()
parser.add_argument('--binary',required=True)
parser.add_argument('--fixture-binary',required=True)
args,extra=parser.parse_known_args()


class ForkReplayTest(unittest.TestCase):
    def test_library_identity_follows_name_across_install_prefixes(self):
        names=('libiap.so','libplan_env.so','libpath_searching.so')
        old={'/captured/'+name:'original' for name in names}
        replay={'/replay/'+name:'original' for name in names}
        verify_libraries(old,replay)
        replay['/replay/libiap.so']='changed'
        with self.assertRaisesRegex(ValueError,'mismatch'):
            verify_libraries(old,replay)
        del replay['/replay/libiap.so']
        with self.assertRaisesRegex(ValueError,'absent'):
            verify_libraries(old,replay)

    def test_unknown_integral_samples_cannot_grant_full_risk_coverage(self):
        route={'reference_executable':True,'model_unknown_vertices':0,
               'model_unknown_integral_samples':1}
        self.assertFalse(route_risk_covered({'left':route,'right':route}))
        route['model_unknown_integral_samples']=0
        self.assertTrue(route_risk_covered({'left':route,'right':route}))

    def test_complete_channels_deduplicate_and_unknown_is_never_zero_risk(self):
        with tempfile.TemporaryDirectory() as temporary, patch.dict(os.environ,{'IAP_RUN_ROOT':temporary}):
            run=resolve_run_directory(entrypoint='fork_replay_test')
            backend(run,Path(args.fixture_binary),['fixture','fixture'],'fork_fixture')
            policy='0 1 .4 .3 .4 2 21 1.05 .35 .1 .55 .5 .5 .55 .6 .1 .1 1.5 1 18 0 1.5\n'
            try:
                for name in ('S0','S5_physical_unobserved'):
                    source=run/'export/advisory/validation/fixture'/name/'input.bin'
                    result=subprocess.run([args.binary,str(source),name,'0'],input=policy,text=True,
                        capture_output=True,env={**os.environ,'IAP_RUN_DIR':str(run)},timeout=20)
                    self.assertEqual(result.returncode,0,result.stderr)
                    root=run/'export/advisory/forks'/name
                    meta=json.loads((root/'result.json').read_text())
                    self.assertFalse(meta['execution_authorized'])
                    self.assertTrue(meta['entrance_qualified'])
                    with (root/'samples.csv').open() as stream:rows=list(csv.DictReader(stream))
                    self.assertEqual(len(rows),len({(r['ix'],r['iy'],r['iz']) for r in rows}))
                    self.assertGreater(len(rows),10)
                    if name=='S0':
                        for route in meta['routes'].values():
                            self.assertTrue(route['reference_executable'])
                            self.assertIsNotNone(route['total_cost_m'])
                            self.assertEqual(route['model_unknown_integral_samples'],0)
                    else:
                        unknown=[r for r in rows if r['observed']=='0']
                        self.assertTrue(unknown)
                        for row in unknown:
                            self.assertEqual(row['queried'],'0')
                            self.assertEqual(row['hpl'],'')
                            self.assertEqual(row['vpl'],'')
                        for route in meta['routes'].values():
                            self.assertFalse(route['reference_executable'])
                            self.assertIsNone(route['total_cost_m'])
            finally:
                finalize_run(run,lifecycle='completed')


if __name__=='__main__':unittest.main(argv=[__file__]+extra)
