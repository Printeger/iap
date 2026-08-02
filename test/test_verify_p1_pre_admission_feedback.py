#!/usr/bin/env python3
import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE = Path(__file__).parents[1] / "scripts/dev_planner/verify_p1_pre_admission_feedback.py"
SPEC = importlib.util.spec_from_file_location("pre_admission", MODULE)
pre_admission = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(pre_admission)


class P1PreAdmissionFeedbackTest(unittest.TestCase):
    def bundle(self, instrumented=False):
        root = tempfile.TemporaryDirectory(); self.addCleanup(root.cleanup)
        export = Path(root.name)
        run_id = "fixture-run"
        (export / "test_planner_manifest.json").write_text(json.dumps({
            "artifact_provenance": {"schema_version": pre_admission.SCHEMA,
                                    "run_id": run_id}}))
        rows = []
        for stage, outcome, reason in (
            ("p1_admission", "base_fallback", "temporal_out_of_horizon"),
            ("base_optimizer_start", "started", "ok"),
            ("base_optimizer_end", "candidate_success", "ok"),
            ("publish", "published", "ok")):
            rows.append({"schema_version": pre_admission.SCHEMA, "run_id": run_id,
                         "stage": stage, "outcome": outcome, "reason": reason})
        with (export / pre_admission.TIMELINE).open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
        if instrumented:
            fields = ["initial_duration_s", "initial_temporal_margin_s", "expected_sample_count",
                      "matched_sample_count", "occupied_miss_count", "base_duration_s", "base_full_p1_support"]
            with (export / pre_admission.PRE_ADMISSION).open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerow(dict.fromkeys(fields, "0"))
        return export, run_id

    def test_characterizes_base_publish_without_p1_candidate(self):
        export, run_id = self.bundle()
        result = pre_admission.validate(export, run_id, False, expect_zero_p1=True)
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(result["counts"]["p1_optimizer_start"], 0)
        self.assertEqual(result["counts"]["candidate_rows"], 0)

    def test_requires_new_feedback_contract(self):
        export, run_id = self.bundle()
        self.assertFalse(pre_admission.validate(export, run_id, True, expect_zero_p1=True)["passed"])
        export, run_id = self.bundle(instrumented=True)
        self.assertTrue(pre_admission.validate(export, run_id, True, expect_zero_p1=True)["passed"])


if __name__ == "__main__":
    unittest.main()
