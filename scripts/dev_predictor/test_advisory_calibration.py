#!/usr/bin/env python3
"""Synthetic unit fixtures verify analysis contracts; these are not calibration evidence."""
import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest
import advisory_calibration as cal


def table(n=60):
    return [{"identity": "LIVE_MEASUREMENT", "valid": "true", "reference_time_s": str(i*5),
             "hpl": str(1+i*.01), "vpl": str(.5+i*.01),
             "error_h": str(.8+i*.008), "error_v": str(.4+i*.008),
             "gnss_residual_m": "2", "gnss_nominal_sigma_m": "1",
             "lidar_residual_m": "3", "lidar_nominal_sigma_m": "1"} for i in range(n)]


def data(phase):
    seeds=cal.CAL_SEEDS if phase=="calibration" else cal.VALIDATION_SEEDS
    return {"identity": "LIVE_MEASUREMENT", "route_sha256": "route", "coordinates_sha256": "frame",
            "degradation_schedule_sha256": "schedule", "map_seed": 41021, "posterior_prior": False,
            "guidance": False, "frame_verified": True, "source_revisions":["unit-fixture"],
            "trials": [{"phase": phase, "condition": c, "seed": s, "run_id": c+str(s), "rows": table()}
                       for c in cal.CONDITIONS for s in seeds]}


class CalibrationContract(unittest.TestCase):
    def test_protocol_separates_runs_and_keeps_route_pending(self):
        p=cal.protocol()
        self.assertEqual(len(p["calibration"]),9); self.assertEqual(len(p["validation"]),9)
        self.assertFalse(set(cal.CAL_SEEDS)&set(cal.VALIDATION_SEEDS))
        self.assertIsNone(p["reference_route"])

    def test_noise_uses_measurement_residuals_and_rejects_missing(self):
        d=data("calibration"); noise=cal.fit_noise(d)
        self.assertEqual(noise["risk/gnss_noise_scale"],2)
        self.assertEqual(noise["risk/lidar_noise_scale"],3)
        for r in d["trials"][0]["rows"]: r["gnss_residual_m"]=""
        with self.assertRaisesRegex(ValueError,"residuals missing"): cal.fit_noise(d)

    def test_conversion_requires_noise_replay_and_heldout_validation(self):
        d=data("calibration")
        noise=cal.frozen({"parameters":cal.fit_noise(d), "source_revisions":d["source_revisions"],"contract":cal.contract(d), "stage":"noise"})
        d["parameter_sha256"]=noise["sha256"]
        frozen=cal.frozen(cal.fit_conversion(d,noise)); v=data("validation")
        result=cal.evaluate(v,frozen)
        self.assertEqual(result["coverage_status"],"PASS")
        self.assertEqual(result["independent_runs"],9)
        self.assertFalse(result["future_error_guarantee"])
        changed=copy.deepcopy(v);changed["source_revisions"]=["other-model"]
        with self.assertRaisesRegex(ValueError,"model revision"):cal.evaluate(changed,frozen)
        v["trials"][0]["run_id"]=frozen["calibration_runs"][0]
        with self.assertRaisesRegex(ValueError,"held-out run reused"):cal.evaluate(v,frozen)
        d["parameter_sha256"]="another"
        with self.assertRaisesRegex(ValueError,"noise-scaled replay"):cal.fit_conversion(d,noise)

    def test_invalid_zero_and_missing_remain_coverage_denominator(self):
        d=data("calibration"); noise=cal.frozen({"parameters":cal.fit_noise(d),"source_revisions":d["source_revisions"],"contract":cal.contract(d)})
        d["parameter_sha256"]=noise["sha256"]; f=cal.frozen(cal.fit_conversion(d,noise))
        v=data("validation")
        for row in v["trials"][0]["rows"][:4]: row["hpl"]="0"
        out=cal.evaluate(v,f)["runs"][0]
        self.assertEqual(out["valid"],56); self.assertEqual(out["requests"],60)
        self.assertEqual(out["coverage_status"],"FAIL")
        f["parameters"]["risk/K_H_adv"]*=2
        with self.assertRaisesRegex(ValueError,"altered"):cal.evaluate(v,f)

    def test_low_variation_is_inconclusive(self):
        v=data("validation"); d=data("calibration")
        noise=cal.frozen({"parameters":cal.fit_noise(d),"source_revisions":d["source_revisions"],"contract":cal.contract(d)})
        d["parameter_sha256"]=noise["sha256"]; f=cal.frozen(cal.fit_conversion(d,noise))
        for t in v["trials"]:
            for r in t["rows"]: r.update(hpl="1",vpl="1",error_h=".5",error_v=".5")
        out=cal.evaluate(v,f)
        self.assertEqual(out["trend_status"],"INCONCLUSIVE")
        self.assertEqual(out["runs"][0]["trend_status"],"INCONCLUSIVE_LOW_VARIATION_OR_BLOCKS")

    def test_dataset_checksum_and_seed_admission(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); d=data("validation")
            for i,t in enumerate(d["trials"]):
                values=t.pop("rows"); p=root/(str(i)+".csv")
                for j,r in enumerate(values): r.update(run_id=t["run_id"],request_id=t["run_id"]+"/"+str(j))
                run=root/(str(i)+"_run.json")
                run.write_text(json.dumps({"schema_version":"iap_run_artifact_v1","run_id":t["run_id"],
                    "entrypoint":"iap_sim","scenario":"icra_dense_forest_four_fork_v2","lifecycle":"completed",
                    "source":{"git_worktree_clean":True,"git_commit":"unit-fixture"},
                    "validation_trial":{**cal.contract(d),"seed":t["seed"],"condition":t["condition"],"phase":"validation"}}))
                requests=root/(str(i)+"_requests.json")
                requests.write_text(json.dumps({"identity":"LIVE_MEASUREMENT","run_id":t["run_id"],
                    "request_ids":[r["request_id"] for r in values]}))
                t.update(run_manifest=run.name,run_manifest_sha256=cal.sha(run),
                         requests_manifest=requests.name,requests_manifest_sha256=cal.sha(requests))
                with p.open("w") as s:
                    w=csv.DictWriter(s,fieldnames=values[0]);w.writeheader();w.writerows(values)
                t.update(csv=p.name,sha256=cal.sha(p))
            p=root/"data.json";p.write_text(json.dumps(d))
            self.assertEqual(len(cal.checked_dataset(p,"validation")["trials"]),9)
            original=copy.deepcopy(d)
            d["trials"][1]["csv"]=d["trials"][0]["csv"]
            d["trials"][1]["sha256"]=d["trials"][0]["sha256"];p.write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,"duplicate trial evidence"):cal.checked_dataset(p,"validation")
            d=copy.deepcopy(original)
            first=root/d["trials"][0]["csv"]
            lines=first.read_text().splitlines();first.write_text("\n".join(lines[:-1])+"\n")
            d["trials"][0]["sha256"]=cal.sha(first);p.write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,"missing or duplicate"):cal.checked_dataset(p,"validation")
            d=copy.deepcopy(original)
            d["trials"][0]["seed"]=1101;p.write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,"independent predeclared"):cal.checked_dataset(p,"validation")


if __name__=="__main__":unittest.main()
