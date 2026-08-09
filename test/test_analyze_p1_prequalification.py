import importlib.util
import csv
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "dev_planner" / "analyze_p1_prequalification.py"
SPEC = importlib.util.spec_from_file_location("analyze_p1_prequalification", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def run(lane, mean, cvar, maximum, length=10.0):
    return {
        "passed": True,
        "selected_lane": lane,
        "mean": mean,
        "cvar": cvar,
        "max": maximum,
        "path_length_m": length,
        "localization_error_m": 0.1,
        "hard_gates": {"passed": True},
    }


class P1PrequalificationTest(unittest.TestCase):
    def test_occupancy_scan_indexes_only_the_selected_attempt(self):
        manifest = Path("/e/test_planner_manifest.json")
        common = {
            "schema_version": "p1_evidence_provenance_v4",
            "run_id": "run-1", "manifest_path": str(manifest),
            "candidate_id": "2", "snapshot_generation_id": "11",
            "query_base_time_s": "5.0", "sample_index": "3",
        }
        rows = [
            {**common, "planning_attempt_id": "7", "phase": "accepted"},
            {**common, "planning_attempt_id": "7", "phase": "final"},
            {**common, "planning_attempt_id": "8", "phase": "final"},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "occupancy.csv"
            with path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            accepted, final = MODULE._scan_occupancy_evidence(
                path, "run-1", manifest, rows[0])

        self.assertEqual(len(accepted), 1)
        self.assertEqual(sum(map(len, final.values())), 1)

    def test_candidate_rows_require_run_manifest_and_attempt_context_binding(self):
        manifest = Path("/e/test_planner_manifest.json")
        row = {
            "schema_version": "p1_evidence_provenance_v4",
            "run_id": "run-1", "manifest_path": str(manifest),
            "planning_attempt_id": "7", "candidate_id": "2",
            "snapshot_generation_id": "11", "query_base_time_s": "5.0",
        }
        MODULE._validate_provenance_rows([row], "run-1", manifest, "candidate")
        self.assertTrue(MODULE._same_attempt_context(row, dict(row)))
        changed = dict(row, snapshot_generation_id="12")
        self.assertFalse(MODULE._same_attempt_context(row, changed))
        with self.assertRaisesRegex(MODULE.PrequalificationError, "provenance"):
            MODULE._validate_provenance_rows(
                [dict(row, run_id="other")], "run-1", manifest, "candidate")

    def test_prequalification_requires_both_lanes_but_not_a_candidate_publish(self):
        rows = []
        for candidate_id, y in (("1", 2.0), ("2", -2.0)):
            for sample_index in range(200):
                rows.append({
                    "candidate_id": candidate_id,
                    "sample_index": str(sample_index),
                    "y": str(y), "c_pi": "1.0", "valid": "1",
                    "stale": "0", "collision_free": "1", "selected": "0",
                })
        result = MODULE.formal_metrics.candidate_route_precheck(
            rows, require_selected=False
        )
        self.assertTrue(result["passed"])
        self.assertIsNone(result["selected"])

    def test_enabled_admitted_candidates_use_recorded_collision_result(self):
        context = {
            "planning_attempt_id": "7", "snapshot_generation_id": "11",
            "query_base_time_s": "5.0",
        }
        candidate = {
            **context, "candidate_id": "2", "phase": "admitted",
            "sample_index": "0", "valid": "1", "stale": "0",
            "collision_free": "1", "selected": "0",
        }
        rows = MODULE._prequalification_candidate_rows([candidate], context)
        self.assertTrue(rows[0]["collision_free"])

    def test_both_arms_use_strictly_export_bound_prequalification_candidates(self):
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp).resolve()
            expected = export / "planner_p1_prequalification_candidate_profile.csv"
            manifest = {
                "p1.prequalification_candidate_profile_path": str(expected),
            }
            self.assertEqual(
                MODULE._artifact(
                    export, manifest,
                    "p1.prequalification_candidate_profile_path",
                    "planner_p1_prequalification_candidate_profile.csv",
                ),
                expected,
            )
            optimization, candidates = MODULE._candidate_evidence_paths(
                export, manifest
            )
            self.assertIsNone(optimization)
            self.assertEqual(candidates, expected)
            optimization, candidates = MODULE._candidate_evidence_paths(
                export, manifest
            )
            self.assertIsNone(optimization)
            self.assertEqual(candidates, expected)
            manifest["p1.prequalification_candidate_profile_path"] = str(
                export.parent / expected.name
            )
            with self.assertRaisesRegex(MODULE.PrequalificationError, "binding"):
                MODULE._artifact(
                    export, manifest,
                    "p1.prequalification_candidate_profile_path",
                    "planner_p1_prequalification_candidate_profile.csv",
                )

    def test_missing_metrics_fail_closed_instead_of_raising(self):
        incomplete = run("lower", 1.0, 1.0, 1.0)
        incomplete["mean"] = None
        result = MODULE.evaluate_pair("primary", incomplete, run("lower", .9, .9, .9))
        self.assertFalse(result["passed"])
        self.assertIn("metrics", " ".join(result["failures"]))

    def test_primary_and_mirror_direction_and_effectiveness(self):
        reference = run("upper", 1.0, 1.1, 1.2)
        primary = MODULE.evaluate_pair("primary", reference, run("lower", .98, 1.08, 1.19))
        mirror = MODULE.evaluate_pair("mirror", reference, run("upper", .98, 1.08, 1.19))
        self.assertTrue(primary["passed"])
        self.assertTrue(mirror["passed"])

    def test_null_uses_prefrozen_limits_and_path_growth(self):
        reference = run("lower", 1.0, 1.0, 1.0, 10.0)
        accepted = MODULE.evaluate_pair(
            "null", reference,
            run("upper", 1.005, 1.004, 1.002, 10.5),
        )
        rejected = MODULE.evaluate_pair(
            "null", reference,
            run("upper", 1.006, 1.004, 1.002, 10.5),
        )
        self.assertTrue(accepted["passed"])
        self.assertFalse(rejected["passed"])

    def test_soft_risk_requires_lower_route_and_nonregressing_max(self):
        reference = run("upper", 1.0, 1.0, 1.0)
        self.assertTrue(MODULE.evaluate_pair(
            "soft_risk", reference, run("lower", .99, .99, 1.0)
        )["passed"])
        self.assertFalse(MODULE.evaluate_pair(
            "soft_risk", reference, run("upper", .99, .99, 1.01)
        )["passed"])


if __name__ == "__main__":
    unittest.main()
