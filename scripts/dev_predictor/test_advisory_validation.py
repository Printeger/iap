"""CPU contract tests; fixture evidence never becomes forest/live evidence."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import advisory_validation as validation
from compare_advisory_error import compare, errors, interpolate_truth, rigid

PARSER = argparse.ArgumentParser()
PARSER.add_argument("--binary", required=True)
ARGS, EXTRA = PARSER.parse_known_args()


class AdvisoryValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="iap_advisory_contract_")
        cls.root = Path(cls.temporary.name)
        with patch.dict(os.environ, {"IAP_RUN_ROOT": str(cls.root)}):
            cls.run_dir = validation.resolve_run_directory(entrypoint="advisory_validation")
        validation.write_subordinate_manifest(cls.run_dir, "full_stack", {
            "clock_contract": "system_clock_for_ros_and_simulated_sensor_stamps",
            "identity": "SYNTHETIC_TEST_DOUBLE"})
        validation.backend(cls.run_dir, Path(ARGS.binary), ["fixture", "fixture"], "fixture")
        cls.summary = validation.summary_for(cls.run_dir, "fixture")
        cls.tables = cls.summary["tables"]

    @classmethod
    def tearDownClass(cls):
        validation.finalize_run(cls.run_dir, lifecycle="completed", safety_outcome="not_applicable")
        cls.temporary.cleanup()

    def test_replay_equivalence_and_unique_voxel_centers(self):
        scan = self.tables["S0"]
        self.assertEqual(len(scan), 100)
        self.assertEqual(len({(r["ix"], r["iy"], r["iz"]) for r in scan}), 100)
        self.assertEqual({r["reference_time_s"] for r in scan}, {"100"})
        for table in self.tables.values():
            for row in table:
                self.assertEqual(row["codec_equal"], "1")
                if row["module_called"] == "1":
                    self.assertEqual(row["repeat_equal"], "1")
                    self.assertEqual(row["batch_equal"], "1")
                if row["wrapper_equal"]:
                    self.assertEqual(row["wrapper_equal"], "1")
                if row["valid"] != "1":
                    self.assertEqual(row["fused_hpl"], "")
                    self.assertEqual(row["fused_vpl"], "")

    def test_source_admission_and_no_prior_only_authorization(self):
        row = self.tables["S5_missing_gnss"][0]
        self.assertEqual(row["wrapper_called"], "1")
        self.assertEqual(row["module_valid"], "1")
        self.assertEqual(row["lidar_used"], "1")
        self.assertEqual(row["valid"], "1")
        self.assertEqual(self.tables["S5_no_observations"][0]["module_valid"], "0")
        self.assertEqual(self.tables["S5_both_missing"][0]["module_valid"], "0")

    def test_classification_preserves_failed_requests(self):
        expected = {"wrong_frame": "COORDINATE_ERROR", "out_of_map": "COORDINATE_ERROR",
                    "physical_occupied": "PHYSICAL_FILTERED", "physical_unobserved": "PHYSICAL_FILTERED",
                    "missing_physical": "INPUT_UNAVAILABLE", "missing_pose": "INPUT_UNAVAILABLE",
                    "invalid_current": "VALID", "preparation_budget": "BUDGET_EXCEEDED",
                    "no_observations": "MODEL_INVALID"}
        for name, status in expected.items():
            row = self.tables["S5_"+name][0]
            self.assertEqual(row["status"], status, name)
            self.assertTrue(row["reason"], name)
        for name in ("pose", "snapshot"):
            self.assertEqual(self.tables["S5_stale_"+name][0]["status"], "STALE")
        self.assertEqual(self.tables["S2_weak_normal_support"][0]["status"], "DIAGNOSTIC_ONLY")

    def test_information_matrices_and_degradation(self):
        for group in ("S1", "S2", "S3", "S4", "S3_no_prior"):
            self.assertEqual(self.summary["mechanism"][group]["status"], "PASS")
        for group in ("S1", "S2", "S3"):
            matrices = [json.loads((self.run_dir / "export/advisory/validation/fixture" / f"{group}_{i}/matrices.jsonl").read_text()) for i in range(3)]
            for key in ("gnss", "lidar"):
                if (group == "S1" and key == "lidar") or (group == "S2" and key == "gnss"):
                    np.testing.assert_array_equal(matrices[0][key]["row_major"], matrices[-1][key]["row_major"])
                else:
                    for a, b in zip(matrices, matrices[1:]):
                        eigen = np.linalg.eigvalsh(np.array(a[key]["row_major"]).reshape(3,3)-np.array(b[key]["row_major"]).reshape(3,3))
                        self.assertGreaterEqual(eigen.min(), -1e-10)
            info = np.array(matrices[0]["fused_information"]["row_major"]).reshape(3,3)
            cov = np.array(matrices[0]["covariance"]["row_major"]).reshape(3,3)
            np.testing.assert_allclose((info+np.eye(3)*1e-6) @ cov, np.eye(3), atol=1e-12)
            weak = np.array(matrices[0]["covariance"]["weak_direction"])
            self.assertAlmostEqual(float(weak @ cov @ weak), max(matrices[0]["covariance"]["eigenvalues"]))

    def test_report_has_raw_links_and_no_real_trial_claim(self):
        path = validation.report(self.run_dir, "fixture")
        report = (path / "report.md").read_text()
        self.assertIn("INCONCLUSIVE_INPUT_UNAVAILABLE", report)
        self.assertEqual(json.loads((path / "summary.json").read_text())["mechanism_identity"], "SYNTHETIC_MECHANISM")
        self.assertEqual(json.loads((path / "summary.json").read_text())["independent_live_runs"], 0)
        import re
        for target in re.findall(r"\]\(([^)]+)\)", report):
            self.assertTrue((path / target).resolve().is_file(), target)
        self.assertTrue((path / "actual_error_vs_pl.png").is_file())

    def test_recording_checksum_version_and_overwrite_guards(self):
        identity = validation.source_identity(); identity["dirty"] = ""
        source = self.run_dir / "export/advisory/validation/fixture/S0/input.bin"
        payload = validation.save_record(self.run_dir, "transport_test", source.read_bytes(), identity,
                                        {"generation": 1, "geometry_id": "fixture"})
        sidecar = payload.with_name("record.json")
        validation.validate_record(payload, sidecar)
        with patch.object(validation, "source_identity", return_value={**identity, "revision":"documentation-only-head"}):
            validation.validate_record(payload, sidecar)
        with patch.object(validation, "source_identity", return_value={**identity, "source_sha256":{}}):
            with self.assertRaisesRegex(ValueError, "cross-version"):
                validation.validate_record(payload, sidecar)
        with self.assertRaises(FileExistsError):
            validation.save_record(self.run_dir, "transport_test", source.read_bytes(), identity, {})
        with payload.open("ab") as stream: stream.write(b"corrupted")
        with self.assertRaisesRegex(ValueError, "checksum"):
            validation.validate_record(payload, sidecar)
        info = json.loads(sidecar.read_text()); info["payload_sha256"] = validation.sha(payload)
        info["revision"] = "wrong_version"; sidecar.write_text(json.dumps(info))
        with self.assertRaisesRegex(ValueError, "cross-version"):
            validation.validate_record(payload, sidecar)
        with self.assertRaises(ValueError): validation.artifact(self.run_dir, "export/../escape")
        with self.assertRaises(ValueError): validation.safe_label("../escape")

    def test_codec_rejects_corruption_at_backend(self):
        path = self.root / "broken.bin"; path.write_bytes(b"invalid")
        with self.assertRaises(subprocess.CalledProcessError):
            validation.backend(self.run_dir, Path(ARGS.binary), ["replay", "broken", str(path)], "broken")

    def test_record_service_preserves_rejection_and_full_payload(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from iap.srv import GetGridMapPredictionInput
        context = Context(); rclpy.init(args=[], context=context)
        server = rclpy.create_node("advisory_record_transport_fixture", context=context)
        executor = SingleThreadedExecutor(context=context); executor.add_node(server)
        payload = (self.run_dir / "export/advisory/validation/fixture/S0/input.bin").read_bytes()
        requests = []
        def respond(request, response):
            requests.append(request)
            response.available = len(requests) > 1
            if response.available:
                response.payload = payload; response.frame_id = "map"
                response.geometry_id = "fixture"; response.generation = 1
            else: response.reason = "physical epoch unavailable"
            return response
        service_name = f"/advisory_record_fixture_{os.getpid()}"
        service = server.create_service(GetGridMapPredictionInput, service_name, respond)
        thread = threading.Thread(target=executor.spin); thread.start()
        # This is a transport test in a cleaned temporary run. The guard's
        # synthetic manifest is never used as real forest evidence.
        primary = self.run_dir / "metadata/run_manifest.json"
        saved = primary.read_text(); info = json.loads(saved)
        identity = validation.source_identity(); identity["dirty"] = ""
        info.update(entrypoint="iap_sim", scenario=validation.SCENARIO,
                    source={"git_commit": identity["revision"], "git_worktree_clean": True})
        primary.write_text(json.dumps(info))
        args = SimpleNamespace(label="service_test", service=service_name, producer_binary=Path(ARGS.binary),
                               count=2, interval=.05, timeout=3., glio_topic="/fixture_glio", truth_topic="/fixture_truth")
        try:
            with patch.object(validation, "source_identity", return_value=identity), patch.dict(os.environ, {"ROS_LOG_DIR": str(self.run_dir / "runtime/ros")}):
                args.planning_input=False; args.planning_attempt_id=0
                validation.record(self.run_dir, args)
            table = validation.rows(self.run_dir / "export/advisory/validation/recordings/service_test_requests.csv")
            self.assertEqual(len(table), 2)
            self.assertEqual(table[0]["available"], "False")
            self.assertEqual(table[0]["reason"], "physical epoch unavailable")
            self.assertEqual((self.run_dir / table[1]["payload"]).read_bytes(), payload)
            recording=json.loads((self.run_dir/"export/advisory/validation/recordings/service_test_requests_manifest.json").read_text())
            self.assertEqual(recording["request_ids"],[r["request_id"] for r in table])
            compared=compare(self.run_dir)
            self.assertEqual(compared["requested"],2)
            failures=validation.rows(self.run_dir/"export/advisory/validation/error_requests.csv")
            self.assertEqual({r["request_id"] for r in failures},set(recording["request_ids"]))
            self.assertEqual(failures[0]["reason"],"physical epoch unavailable")
            self.assertEqual(failures[1]["reason"],"prediction_not_replayed")
            # Removal or loss never lowers the authoritative denominator.
            output=self.run_dir/"export/advisory/validation/error_requests.csv"
            summary=self.run_dir/"export/analysis/advisory_validation/error_summary.json"
            payload_path=self.run_dir/table[1]["payload"]
            evidence=self.run_dir/"metadata/manifests/advisory_errors.json"
            payload_path.unlink();output.unlink();summary.unlink();evidence.unlink()
            missing=compare(self.run_dir)
            self.assertEqual(missing["requested"],2)
            self.assertEqual(validation.rows(output)[1]["reason"],"recorded_payload_missing")
            recording_csv=self.run_dir/recording["requests_csv"]
            text=recording_csv.read_text().splitlines();recording_csv.write_text("\n".join(text[:-1])+"\n")
            output.unlink();summary.unlink();evidence.unlink()
            removed=compare(self.run_dir)
            self.assertEqual(removed["requested"],2)
            self.assertEqual({r["reason"] for r in validation.rows(output)},{"requests_csv_checksum_mismatch"})
        finally:
            primary.write_text(saved)
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
            server.destroy_service(service);server.destroy_node();rclpy.shutdown(context=context)

    def test_fixed_transform_and_time_alignment(self):
        samples = [{"stamp": t, "x": t, "y": 0., "z": 1., "qx": 0., "qy": 0., "qz": 0., "qw": 1.} for t in (10.,10.04)]
        p, rotation = interpolate_truth(samples, 10.02)
        np.testing.assert_allclose(p, [10.02, 0, 1])
        with self.assertRaisesRegex(ValueError, "unmatched"):
            interpolate_truth(samples, 9.)
        with self.assertRaisesRegex(ValueError, "gap"):
            interpolate_truth([samples[0], {**samples[1], "stamp": 11}], 10.5)
        world = np.eye(4); world[:3,3] = [1, 2, 3]
        body = np.eye(4); body[:3,3] = [.1, 0, 0]
        h, v = errors([0, 0, 0], [1.1, 2, 2.5], np.eye(3), {"T_truth_map": world, "T_truthbody_predictionbody": body})
        self.assertAlmostEqual(h, .2); self.assertAlmostEqual(v, .5)
        bad = np.eye(4);bad[0,0]=2
        with self.assertRaises(ValueError): rigid(bad)
        with self.assertRaisesRegex(ValueError, "invalid_truth_orientation"):
            interpolate_truth([{**samples[0], "qw": 0}], 10.)


if __name__ == "__main__":
    unittest.main(argv=[__file__, *EXTRA])
