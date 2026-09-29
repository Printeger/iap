import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]


def load_analyzer():
    path = REPO / "tools" / "ana_log.py"
    spec = importlib.util.spec_from_file_location("iap_ana_log_contract", path)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AnalyzerContractTest(unittest.TestCase):
    def test_new_layout_is_preferred_and_analysis_stays_in_export(self):
        analyzer = load_analyzer()
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary) / "20260929T120000Z_000"
            for directory in (
                "runtime",
                "profiling",
                "export/glio",
                "export/current_integrity",
                "export/analysis",
                "metadata/config/iap",
            ):
                (run / directory).mkdir(parents=True, exist_ok=True)
            (run / "metadata/run_manifest.json").write_text(
                json.dumps(
                    {
                        "schema_version": "iap_run_artifact_v1",
                        "run_id": run.name,
                        "lifecycle": "completed",
                    }
                ),
                encoding="utf-8",
            )
            (run / "export/glio/iap_icp.csv").write_text(
                "stamp,rmse\n1,0.1\n", encoding="utf-8"
            )
            paths = analyzer.artifact_paths(run)
            self.assertEqual(paths["icp"][1], "export/glio/iap_icp.csv")
            self.assertEqual(paths["run_info"][1], "metadata/run_manifest.json")

            with mock.patch.object(
                sys,
                "argv",
                [
                    "ana_log.py",
                    "--run",
                    str(run),
                    "--no-plots",
                    "--skip-external-tools",
                ],
            ):
                self.assertEqual(analyzer.main(), 0)
            self.assertTrue((run / "export/analysis/report.json").is_file())
            self.assertFalse((run / "analysis").exists())
            external = Path(temporary) / "explicit-analysis"
            analyzer.register_external_analysis_export(run, external)
            manifest = json.loads(
                (run / "metadata/run_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["external_exports"][0]["path"], str(external))

    def test_legacy_flat_export_remains_read_only_compatible(self):
        analyzer = load_analyzer()
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / "export").mkdir()
            legacy = run / "export/iap_araim.csv"
            legacy.write_text("stamp,HPL\n1,2\n", encoding="utf-8")
            self.assertEqual(analyzer.artifact_paths(run)["araim"][0], legacy)


if __name__ == "__main__":
    unittest.main()
