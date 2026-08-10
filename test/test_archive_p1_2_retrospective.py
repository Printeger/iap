import csv
import gzip
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "dev_planner" / "archive_p1_2_retrospective.py"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "wt", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _profile_rows(run_id: str, mean: float, enabled: bool) -> list[dict[str, object]]:
    values = [mean - 0.01, mean + 0.01]
    return [
        {
            "run_id": run_id,
            "profile_seq": 1,
            "planning_attempt_id": 7,
            "candidate_id": 2 if enabled else 3,
            "snapshot_generation_id": 11,
            "query_base_time_s": 20.0,
            "sample_index": index,
            "arc_fraction": index,
            "x": index,
            "y": -0.2 if enabled else 0.2,
            "z": 1.5,
            "valid": 1,
            "stale": 0,
            "c_pi": value,
            "trace_available": 1,
            "grad_dot_displacement": -0.02 if enabled else 0.0,
            "delta_c_pi": -0.01 if enabled else 0.0,
            "objective_applied": 1 if enabled else 0,
        }
        for index, value in enumerate(values)
    ]


def _fixture(root: Path, *, omit_campaign: int | None = None) -> tuple[Path, Path]:
    compact = root / "compact"
    raw = root / "raw_c38"
    compact.mkdir()
    raw.mkdir()
    pairs_for_raw = []

    for campaign in range(31, 39):
        if campaign == omit_campaign:
            continue
        campaign_dir = compact / f"bundle-{campaign}"
        campaign_dir.mkdir()
        complete = campaign in {31, 32, 38}
        runs = []
        pairs = []
        summary_runs = []
        summary_pairs = []
        for pair_index, kind in enumerate(("primary", "primary", "mirror", "null", "soft_risk")):
            ref_id = f"c{campaign}-p{pair_index}-reference"
            enabled_id = f"c{campaign}-p{pair_index}-enabled"
            ref_mean = 1.2 + pair_index * 0.02
            enabled_mean = ref_mean - (0.003 if campaign == 38 else 0.001)
            for metrics_only, run_id, mean in (
                (True, ref_id, ref_mean), (False, enabled_id, enabled_mean)
            ):
                passed = complete or not (pair_index == 0 and not metrics_only)
                row = {
                    "run_id": run_id,
                    "scenario": kind,
                    "metrics_only": metrics_only,
                    "passed": passed,
                    "selected_lane": "upper" if metrics_only else "lower",
                    "mean": mean,
                    "cvar": mean + 0.02,
                    "max": mean + 0.04,
                    "path_length_m": 10.0,
                    "localization_error_m": 0.1,
                    "checkpoint_truth_dt_s": 0.0,
                    "export_dir": f"fixture/{run_id}",
                }
                runs.append(row)
                summary_runs.append({**row, "hard_gates": {"passed": passed}, "errors": []})
            pair = {
                "kind": kind,
                "reference_run_id": ref_id,
                "enabled_run_id": enabled_id,
                "passed": kind in {"null", "soft_risk"},
                "mean_improvement": ref_mean - enabled_mean,
                "cvar_improvement": ref_mean - enabled_mean,
                "max_regression": enabled_mean - ref_mean,
                "path_growth": 0.0,
                "localization_error_delta_m": 0.0,
            }
            pairs.append(pair)
            summary_pairs.append({**pair, "failures": [] if pair["passed"] else ["old threshold"]})

            if campaign == 38:
                ref_export = raw / "runs" / ref_id / "export"
                enabled_export = raw / "runs" / enabled_id / "export"
                ref_export.mkdir(parents=True)
                enabled_export.mkdir(parents=True)
                _write_csv(
                    ref_export / "planner_p1_accepted_trajectory_risk_profile.csv",
                    _profile_rows(ref_id, ref_mean, False),
                )
                _write_csv(
                    enabled_export / "planner_p1_accepted_trajectory_risk_profile.csv.gz",
                    _profile_rows(enabled_id, enabled_mean, True),
                )
                _write_csv(
                    enabled_export / "planner_p1_candidate_optimization.csv.gz",
                    [{
                        "run_id": enabled_id,
                        "planning_attempt_id": 7,
                        "candidate_id": 2,
                        "snapshot_generation_id": 11,
                        "query_base_time_s": 20.0,
                        "pre_raw_p1_cost": 1.1,
                        "post_raw_p1_cost": 1.0,
                        "pre_mean_c_pi": 1.2,
                        "post_mean_c_pi": 1.19,
                        "pre_max_c_pi": 1.3,
                        "post_max_c_pi": 1.3,
                        "grad_integrity_dot_displacement": -0.02,
                        "support_full_valid": 1,
                        "objective_applied": 1,
                        "optimization_success": 1,
                        "initial_control_points_hash": "initial",
                        "final_control_points_hash": "final",
                        "support_signature": "support",
                    }],
                )
                pairs_for_raw.append({
                    "kind": kind,
                    "reference_export": str(ref_export),
                    "enabled_export": str(enabled_export),
                })

        summary = {
            "schema_version": "p1_2_prequalification_summary_v1",
            "passed": False,
            "runs": summary_runs,
            "pairs": summary_pairs,
        }
        (campaign_dir / f"c{campaign}_campaign.json").write_text(
            json.dumps({"schema_version": "campaign_v1", "campaign": campaign}) + "\n"
        )
        (campaign_dir / f"c{campaign}_prequalification_summary.json").write_text(
            json.dumps(summary) + "\n"
        )
        suffix = ".csv" if campaign % 2 else ".csv.gz"
        _write_csv(campaign_dir / f"c{campaign}_prequalification_runs{suffix}", runs)
        _write_csv(campaign_dir / f"c{campaign}_prequalification_pairs{suffix}", pairs)

    (raw / "campaign.json").write_text(
        json.dumps({"schema_version": "campaign_v1", "campaign": 38}) + "\n"
    )
    (raw / "prequalification_runs.json").write_text(
        json.dumps({"schema_version": "p1_2_prequalification_runs_v1", "pairs": pairs_for_raw}) + "\n"
    )
    return compact, raw


def _invoke(compact: Path, raw: Path, output: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--compact-root", str(compact),
            "--c38-raw-campaign", str(raw),
            "--output-dir", str(output),
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
    )


def _hash_manifest(path: Path) -> dict[str, str]:
    entries = {}
    for line in path.read_text().splitlines():
        digest, name = line.split("  ", 1)
        entries[name] = digest
    return entries


class P12RetrospectiveArchiveTest(unittest.TestCase):
    def test_cli_archives_plain_and_gzip_inputs_with_historical_classification(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            compact, raw = _fixture(root)
            output = root / "archive"

            result = _invoke(compact, raw, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((output / "p1_2_retrospective_summary.json").read_text())
            self.assertEqual(summary["historical_verdict"], "BLOCKED")
            self.assertEqual(summary["formal_analyzer_invocation_count"], 0)
            classes = {row["campaign_id"]: row["classification"] for row in summary["campaigns"]}
            self.assertEqual(
                {key for key, value in classes.items() if value == "complete_comparable_failure"},
                {"c31", "c32", "c38"},
            )
            self.assertTrue(all(classes[f"c{i}"] == "incomplete_diagnostic" for i in range(33, 38)))
            with gzip.open(output / "p1_2_retrospective_mechanism.csv.gz", "rt", newline="") as handle:
                rows = list(csv.DictReader(handle))
            optimizations = [row for row in rows if row["record_type"] == "same_snapshot_optimization"]
            self.assertEqual(len(optimizations), 5)
            self.assertTrue(all(float(row["raw_p1_cost_delta"]) < 0 for row in optimizations))
            self.assertTrue(all(float(row["gradient_dot_displacement"]) < 0 for row in optimizations))
            for name in summary["figures"]:
                self.assertGreater((output / name).stat().st_size, 0)

    def test_missing_campaign_fails_closed_without_partial_archive(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            compact, raw = _fixture(root, omit_campaign=34)
            output = root / "archive"

            result = _invoke(compact, raw, output)

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("c34", result.stderr)
            self.assertFalse(output.exists())

    def test_missing_required_csv_column_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            compact, raw = _fixture(root)
            runs_path = next(compact.rglob("c31_prequalification_runs.csv"))
            with runs_path.open(newline="") as handle:
                rows = list(csv.DictReader(handle))
            for row in rows:
                row.pop("selected_lane")
            _write_csv(runs_path, rows)
            output = root / "archive"

            result = _invoke(compact, raw, output)

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("selected_lane", result.stderr)
            self.assertFalse(output.exists())

    def test_outputs_and_source_hashes_are_deterministic_and_verifiable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            compact, raw = _fixture(root)
            first = root / "archive-a"
            second = root / "archive-b"
            self.assertEqual(_invoke(compact, raw, first).returncode, 0)
            self.assertEqual(_invoke(compact, raw, second).returncode, 0)

            first_hashes = {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in first.iterdir()
                if path.name != "artifact_hashes.sha256"
            }
            second_hashes = {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in second.iterdir()
                if path.name != "artifact_hashes.sha256"
            }
            self.assertEqual(first_hashes, second_hashes)

            inventory = json.loads((first / "source_inventory.json").read_text())
            self.assertGreater(len(inventory["sources"]), 30)
            for item in inventory["sources"]:
                source = Path(item["path"])
                self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), item["sha256"])
            source_manifest = _hash_manifest(first / "source_hashes.sha256")
            self.assertEqual(len(source_manifest), len(inventory["sources"]))
            for name, digest in source_manifest.items():
                self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(), digest)

            artifact_manifest = _hash_manifest(first / "artifact_hashes.sha256")
            expected_artifacts = {path.name for path in first.iterdir()} - {"artifact_hashes.sha256"}
            self.assertEqual(set(artifact_manifest), expected_artifacts)
            for name, digest in artifact_manifest.items():
                self.assertEqual(hashlib.sha256((first / name).read_bytes()).hexdigest(), digest)


if __name__ == "__main__":
    unittest.main()
