#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/dev_planner/analyze_p1_candidate_diagnostic_smoke.py"
SPEC = importlib.util.spec_from_file_location("p1_candidate_diagnostic", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeAxis:
    def __init__(self):
        self.scatter_calls = []

    def scatter(self, x, y, **kwargs):
        self.scatter_calls.append((x, y, kwargs))

    def set_yticks(self, *args):
        self.yticks = args

    def set(self, **kwargs):
        self.settings = kwargs

    def axis(self, *args):
        self.axis_args = args

    def text(self, *args, **kwargs):
        self.text_args = (args, kwargs)


class P1CandidateDiagnosticSmokeTest(unittest.TestCase):
    def test_large_lifecycle_uses_one_vectorized_scatter_collection(self):
        timeline = [
            {"stage": f"stage-{index % 4}", "stamp_s": str(index / 1000.0)}
            for index in range(30_000)
        ]
        axis = FakeAxis()

        MODULE.plot_lifecycle(axis, timeline)

        self.assertEqual(len(axis.scatter_calls), 1)
        self.assertEqual(len(axis.scatter_calls[0][0]), 30_000)
        self.assertEqual(len(axis.scatter_calls[0][1]), 30_000)


if __name__ == "__main__":
    unittest.main()
