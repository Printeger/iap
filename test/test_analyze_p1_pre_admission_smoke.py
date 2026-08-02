#!/usr/bin/env python3
import csv
import json
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/dev_planner/analyze_p1_pre_admission_smoke.py"
SCHEMA = "p1_evidence_provenance_v3"


class P1PreAdmissionAnalyzerTest(unittest.TestCase):
    def test_explicit_bundle_generates_all_pre_admission_figures_on_entry_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            export = Path(directory) / "export"; export.mkdir()
            bag = Path(directory) / "bag"; bag.mkdir()
            run_id = "pre-admission-fixture"
            manifest = export / "test_planner_manifest.json"
            manifest.write_text(json.dumps({"artifact_provenance": {
                "schema_version": SCHEMA, "run_id": run_id, "bag_path": str(bag),
                "process_start_epoch_s": 1.0, "process_end_epoch_s": 2.0}}))
            common = {"schema_version": SCHEMA, "run_id": run_id, "manifest_path": str(manifest)}
            timeline = [{**common, "stage": "acquire", "outcome": "acquired", "reason": "ok", "stamp_s": "1"},
                        {**common, "stage": "p1_admission", "outcome": "base_fallback", "reason": "temporal_out_of_horizon", "stamp_s": "1.1"},
                        {**common, "stage": "base_optimizer_start", "outcome": "started", "reason": "ok", "stamp_s": "1.2"},
                        {**common, "stage": "base_optimizer_end", "outcome": "candidate_success", "reason": "ok", "stamp_s": "1.3"},
                        {**common, "stage": "publish", "outcome": "published", "reason": "ok", "stamp_s": "1.4"}]
            with (export / "planner_p1_planning_context_timeline.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=timeline[0]); writer.writeheader(); writer.writerows(timeline)
            attempt = {**common, "planning_attempt_id": "7", "candidate_id": "0", "snapshot_generation_id": "2", "query_base_time_s": "1",
                       "snapshot_time_max_s": "3.5", "initial_duration_s": "3", "initial_temporal_margin_s": "-0.5",
                       "expected_sample_count": "200", "matched_sample_count": "194", "occupied_miss_count": "6",
                       "base_duration_s": "2.1", "base_full_p1_support": "0", "p1_admission_verdict": "base_fallback", "p1_admission_reason": "temporal_out_of_horizon"}
            with (export / "planner_p1_pre_admission_attempt.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=attempt); writer.writeheader(); writer.writerow(attempt)
            profile = [{**common, "profile_seq": "1", "planning_attempt_id": "7", "sample_index": str(index), "x": str(index / 100), "y": "0", "reason": "occupied" if index < 6 else "ok", "base_collision_occupied": "1" if index < 6 else "0", "fallback_reason": "temporal_out_of_horizon"} for index in range(200)]
            with (export / "planner_p1_accepted_trajectory_risk_profile.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=profile[0]); writer.writeheader(); writer.writerows(profile)
            report = Path(directory) / "report.md"
            result = subprocess.run(["python3", str(SCRIPT), "--export-dir", str(export), "--bag-dir", str(bag),
                                     "--run-id", run_id, "--main-report", str(report)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            figures = list((export / "p1_pre_admission_diagnostic").glob("*.png"))
            self.assertEqual(len(figures), 8)
            self.assertIn("Observation:", report.read_text())


if __name__ == "__main__":
    unittest.main()
