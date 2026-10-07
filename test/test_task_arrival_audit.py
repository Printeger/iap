import importlib.util
from pathlib import Path
import unittest
import json
import tempfile

path = Path(__file__).resolve().parents[1] / 'scripts/dev_planner/audit_task_arrival.py'
spec = importlib.util.spec_from_file_location('task_arrival_audit', path)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class TaskArrivalAudit(unittest.TestCase):
    def test_captured_preset_arrival_survives_reset_source_state(self):
        # Minimized real 2ff5c31d / 225155Z_510 lines 9862-9863.
        text = ('[ego_planner_node-10] [TRIG]: from EXEC_TRAJ to REPLAN_TRAJ\n'
                '[ego_planner_node-10] [FSM]: from REPLAN_TRAJ to WAIT_TARGET\n')
        found = audit.arrival_transitions(text)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]['source_state'], 'REPLAN_TRAJ')

    def test_startup_state_prints_and_foreign_transitions_are_not_arrival(self):
        text = ('[ego_planner_node-10] [FSM]: from INIT to WAIT_TARGET\n'
                '[ego_planner_node-10] [FSM]: state: WAIT_TARGET\n'
                '[ego_planner_node-10] [SAFETY]: from REPLAN_TRAJ to WAIT_TARGET\n'
                '[ego_planner_node-10] [FSM]: from EMERGENCY_STOP to WAIT_TARGET\n')
        self.assertEqual(audit.arrival_transitions(text), [])

    def test_manual_target_original_exec_transition_is_arrival(self):
        self.assertEqual(len(audit.arrival_transitions(
            '[ego_planner_node-10] [FSM]: from EXEC_TRAJ to WAIT_TARGET')), 1)

    def test_changed_or_unbound_original_log_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / 'runtime').mkdir()
            (run / 'metadata/manifests').mkdir(parents=True)
            (run / 'runtime/launch.log').write_text(
                '[ego_planner_node-10] [FSM]: from REPLAN_TRAJ to WAIT_TARGET\n')
            manifest = run / 'metadata/manifests/forest_process_result.json'
            for hashes in [{}, {'runtime/launch.log': '0' * 64}]:
                manifest.write_text(json.dumps(dict(revision='2ff5c31d', logs_sha256=hashes)))
                with self.subTest(hashes=hashes), self.assertRaisesRegex(ValueError, 'LOG_IDENTITY'):
                    audit.audit(run)


if __name__ == '__main__':
    unittest.main()
