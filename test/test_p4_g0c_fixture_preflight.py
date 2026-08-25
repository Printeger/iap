import importlib.util
import json
import stat
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts/dev_planner/preflight_p4_g0c_fixture.py"


class P4G0CFixturePreflightTest(unittest.TestCase):
    @staticmethod
    def _load_module():
        spec = importlib.util.spec_from_file_location(
            "preflight_p4_g0c_fixture_test", MODULE_PATH
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    @staticmethod
    def _scanner(path: Path, exit_code: int = 0) -> Path:
        path.write_text(f"#!/bin/sh\nexit {exit_code}\n")
        path.chmod(path.stat().st_mode | stat.S_IXUSR)
        return path

    def _protocol(self, root: Path, fixture: Path) -> Path:
        import hashlib

        payload = {
            "live_fixture": {
                "path": "config/icra27/p4_g0c_live_fixture_v2.json",
                "sha256": hashlib.sha256(fixture.read_bytes()).hexdigest(),
            },
            "schema_version": "p4_g0c_protocol_v5",
        }
        path = root / "protocol.json"
        path.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")
        return path

    def test_exact_source_protocol_launch_and_production_scanner_pass(self):
        module = self._load_module()
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v2.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = module.run_preflight(
                fixture,
                self._protocol(root, fixture),
                REPO / "launch/test_planner.launch.py",
                self._scanner(root / "scanner"),
            )
        self.assertEqual(result["typed_result"], "FIXTURE_ELIGIBILITY_PASS")
        self.assertTrue(result["eligible"])
        self.assertEqual(result["materialized_geometry"]["start_x_m"], -12.0)
        self.assertEqual(result["materialized_geometry"]["horizon_m"], 7.5)
        self.assertEqual(result["materialized_geometry"]["control_point_distance_m"], 0.4)
        self.assertEqual(result["materialized_geometry"]["obstacle_x_m"], [-9.0, -7.0])
        self.assertEqual(result["scanner_contract"]["r5_status"], "CLOSED_SEGMENTS")
        self.assertEqual(result["scanner_contract"]["r4_status"], "OPEN_ENDED_COLLISION")

    def test_protocol_fixture_hash_mismatch_fails_before_scanner(self):
        module = self._load_module()
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v2.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            protocol = self._protocol(root, fixture)
            payload = json.loads(protocol.read_text())
            payload["live_fixture"]["sha256"] = "0" * 64
            protocol.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")
            marker = root / "scanner-invoked"
            scanner = root / "scanner"
            scanner.write_text(f"#!/bin/sh\ntouch {marker}\nexit 0\n")
            scanner.chmod(scanner.stat().st_mode | stat.S_IXUSR)
            result = module.run_preflight(
                fixture, protocol, REPO / "launch/test_planner.launch.py", scanner
            )
        self.assertEqual(result["typed_result"], "FIXTURE_ELIGIBILITY_FAIL")
        self.assertEqual(result["failure_reason"], "PROTOCOL_FIXTURE_HASH_MISMATCH")
        self.assertFalse(marker.exists())

    def test_production_scanner_failure_is_fail_closed(self):
        module = self._load_module()
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v2.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = module.run_preflight(
                fixture,
                self._protocol(root, fixture),
                REPO / "launch/test_planner.launch.py",
                self._scanner(root / "scanner", 1),
            )
        self.assertEqual(result["typed_result"], "FIXTURE_ELIGIBILITY_FAIL")
        self.assertEqual(result["failure_reason"], "PRODUCTION_SCANNER_CONTRACT_FAILED")

    def test_effective_scenario_y_geometry_drift_fails_before_scanner(self):
        module = self._load_module()
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v2.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            launch = root / "test_planner.launch.py"
            launch.write_text(
                (REPO / "launch/test_planner.launch.py").read_text().replace(
                    '"p1_fixture_central_y_half_width_m": "0.65"',
                    '"p1_fixture_central_y_half_width_m": "0.66"',
                    1,
                )
            )
            marker = root / "scanner-invoked"
            scanner = root / "scanner"
            scanner.write_text(f"#!/bin/sh\ntouch {marker}\nexit 0\n")
            scanner.chmod(scanner.stat().st_mode | stat.S_IXUSR)
            result = module.run_preflight(
                fixture, self._protocol(root, fixture), launch, scanner
            )
        self.assertEqual(result["typed_result"], "FIXTURE_ELIGIBILITY_FAIL")
        self.assertEqual(result["failure_reason"], "EFFECTIVE_LAUNCH_GEOMETRY_MISMATCH")
        self.assertFalse(marker.exists())


if __name__ == "__main__":
    unittest.main()
