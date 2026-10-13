import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import g2_a0_rejection_check as check


class RejectionEvidence(unittest.TestCase):
    def test_adopted_manifest_is_immutable_and_owner_state_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run/'metadata/manifests').mkdir(parents=True)
            owner = run/'metadata/run_manifest.json'
            original = '{"lifecycle":"active","subordinate_manifests":[]}\n'
            owner.write_text(original)
            check.child_manifest(run, 'diagnostic', {'scope': 'test'}, owner=False)
            self.assertEqual(owner.read_text(), original)
            with self.assertRaises(FileExistsError):
                check.child_manifest(run, 'diagnostic', {'scope': 'overwrite'}, owner=False)
            self.assertEqual(owner.read_text(), original)

    def test_failure_preserves_partial_bindings_and_request_denominator(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            for directory in ['runtime', 'metadata/manifests', 'export/analysis']:
                (run/directory).mkdir(parents=True)
            owner = run/'metadata/run_manifest.json'
            owner.write_text('{"lifecycle":"active"}')
            def fail(run, label, evidence, owner):
                evidence.inputs['original.bin'] = 'original-sha'
                evidence.expected_rows = [{'seed': 7400, 'pair_index': 0},
                                          {'seed': 7400, 'pair_index': 1}]
                evidence.development_rows = [{'seed': 7400, 'pair_index': 0, 'qualified': False}]
                raise FileNotFoundError('required_input.bin')
            with patch.dict(check.os.environ, {'IAP_RUN_DIR': str(run)}), \
                 patch.object(check, 'adopt_run_directory', return_value=run), \
                 patch.object(check, 'evaluate', side_effect=fail), \
                 patch.object(check, 'source_identity', return_value={'revision': 'test'}), \
                 patch.object(check, 'finalize_run') as finalizer, \
                 patch.object(check.sys, 'argv', ['check', '--label', 'failure_test']):
                with self.assertRaises(FileNotFoundError):
                    check.main()
                finalizer.assert_not_called()
            result = json.loads((run/'export/analysis/failure_test/failure.json').read_text())
            self.assertEqual(result['inputs_sha256'], {'original.bin': 'original-sha'})
            self.assertEqual(len(result['expected_rows']), 2)
            self.assertEqual(len(result['development_rows']), 1)
            self.assertFalse(result['independent_qualification'])
            self.assertEqual(owner.read_text(), '{"lifecycle":"active"}')


if __name__ == '__main__':
    unittest.main()
