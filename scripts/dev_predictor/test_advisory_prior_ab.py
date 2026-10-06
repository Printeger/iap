"""Executable same-observation and OFF-prior regression contracts."""
import argparse
import json
import os
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import advisory_prior_ab_validation as ab

parser=argparse.ArgumentParser()
parser.add_argument("--binary",required=True)
args,extra=parser.parse_known_args()


class AdvisoryPriorABTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary=tempfile.TemporaryDirectory(prefix="iap_prior_ab_")
        with patch.dict(os.environ,{"IAP_RUN_ROOT":cls.temporary.name}):
            cls.run_dir=ab.common.resolve_run_directory(entrypoint="advisory_prior_ab_validation")
        ab.common.backend(cls.run_dir,Path(args.binary),["fixture_ab","ab"],"ab")
        cls.root,cls.tables,cls.matrices,cls.inputs=ab.load(cls.run_dir,"ab")
        cls.summary=ab.analyze(cls.run_dir,"ab")

    @classmethod
    def tearDownClass(cls):
        ab.common.finalize_run(cls.run_dir,lifecycle="completed",safety_outcome="not_applicable")
        cls.temporary.cleanup()

    def test_complete_codec_pairs_and_cache_identity(self):
        self.assertEqual(len(self.tables["off"]["S0"]),100)
        self.assertEqual(len({(r["ix"],r["iy"],r["iz"]) for r in self.tables["off"]["S0"]}),100)
        for case,pair in self.summary["pairs"].items():
            if case!="S5_missing_physical":self.assertTrue(pair["observation_codec_equal"],case)
            self.assertFalse(self.inputs["off"][case]["has_lambda_base"],case)
        self.assertNotEqual(self.inputs["on"]["S0"]["prediction_input_identity"],self.inputs["off"]["S0"]["prediction_input_identity"])
        for mode in ("on","off"):
            self.assertEqual(self.inputs[mode]["S0"]["current_motion_quality"],1)
            self.assertEqual(self.inputs[mode]["S0"]["current_motion_error_proxy_m"],.012)
            self.assertEqual({r["reference_time_s"] for r in self.tables[mode]["S0"]},{"100"})

    def test_observation_sources_and_missing_stale_nonfinite_states(self):
        t=self.tables["off"]
        normal=t["S3_0"][0]
        self.assertEqual((normal["valid"],normal["gnss_used"],normal["lidar_used"]),("1","1","1"))
        for source in ("gnss","lidar"):
            r=t["source_"+source][0];self.assertEqual(r["valid"],"1");self.assertEqual(r[source+"_used"],"1")
        for case in ("S5_both_missing","S5_no_observations","S5_nonfinite_observations"):
            row=t[case][0];self.assertEqual(row["valid"],"0",case)
            self.assertEqual(row["fused_hpl"],"");self.assertTrue(row["reason"])
        for source in ("pose","snapshot"):
            self.assertEqual(t["S5_stale_"+source][0]["status"],"STALE")
        for source in ("current","cloud","gnss"):
            self.assertEqual(t["S5_stale_"+source][0]["valid"],"1")
        for table in t.values():
            for row in table:
                self.assertEqual(row["prior_used"],"0")
                if row["valid"]=="1":self.assertGreater(float(row["fused_hpl"]),0)

    def test_shared_admission_accepts_lidar_without_gnss(self):
        r=self.tables["off"]["S5_missing_gnss"][0]
        self.assertEqual((r["wrapper_called"],r["module_valid"],r["lidar_used"],r["valid"]),("1","1","1","1"))
        self.assertTrue(r["fused_hpl"])

    def test_dual_degradation_and_complementary_information(self):
        for mode in ("on","off"):
            for group in ("S1","S2","S3","S4"):
                for metric in ("hpl","vpl"):
                    self.assertEqual(self.summary["mechanism"][mode][group][metric]["monotonic_status"],"PASS")
        self.assertGreater(self.summary["mechanism"]["off"]["S3"]["hpl"]["values"][2],30)
        for case,m in self.matrices["off"].items():
            for point in m:
                if point["fused_information"] and point["module_valid"]:
                    a=np.asarray(point["gnss"]["row_major"]).reshape(3,3)
                    b=np.asarray(point["lidar"]["row_major"]).reshape(3,3)
                    total=np.asarray(point["fused_information"]["row_major"]).reshape(3,3)
                    np.testing.assert_allclose(total,a+b,atol=1e-9,rtol=1e-12)
                    covariance=np.asarray(point["covariance"]["row_major"]).reshape(3,3)
                    np.testing.assert_allclose((total+np.eye(3)*point["fusion_epsilon"])@covariance,np.eye(3),atol=1e-8)

    def test_weak_directions_and_regularization_are_exported(self):
        weak=self.matrices["off"]["weak_lidar_only"][0]
        self.assertLess(weak["fused_information"]["eigenvalues"][0],1.)
        self.assertGreater(float(self.tables["off"]["weak_lidar_only"][0]["module_hpl"]),8.)
        for case in ("weak_lidar_only","S3_regularization_limit"):
            m=self.matrices["off"][case][0]
            self.assertEqual(m["fusion_epsilon"],1e-6)
            self.assertTrue(m["epsilon_applied"])
            self.assertTrue(m["fused_information"])
        limit=self.matrices["off"]["S3_regularization_limit"][0]
        self.assertLess(limit["fused_information"]["eigenvalues"][0],1e-6)
        self.assertEqual(self.tables["off"]["S3_regularization_limit"][0]["module_hpl"],"")
        self.assertGreater(limit["regularization_fraction"],.01)
        self.assertGreater(limit["regularized_diagnostic_hpl"],1000)

    def test_report_links_and_synthetic_live_separation(self):
        output=ab.report(self.run_dir,"ab")
        report=(output/"report.md").read_text()
        self.assertIn("INCONCLUSIVE_INPUT_UNAVAILABLE",report)
        self.assertIn("SYNTHETIC_MECHANISM",report)
        self.assertEqual(self.summary["independent_live_runs"],0)
        for target in re.findall(r"\]\(([^)]+)\)",report):
            self.assertTrue((output/target).resolve().is_file(),target)
        self.assertTrue((output/"actual_error_vs_pl.png").is_file())
        with self.assertRaises(FileExistsError):ab.report(self.run_dir,"ab")
        # Altering an actual source/candidate is rejected, never accepted as an A/B.
        payload=self.root/"ab_off/S0/input.bin";payload.write_bytes(payload.read_bytes()+b"tamper")
        with self.assertRaisesRegex(ValueError,"codec differs"):ab.analyze(self.run_dir,"ab")


if __name__=="__main__":unittest.main(argv=[__file__,*extra])
