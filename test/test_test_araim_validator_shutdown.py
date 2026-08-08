import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "test_araim_validator.py"
SPEC = importlib.util.spec_from_file_location("test_araim_validator_shutdown", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class Node:
    def count_publishers(self, _topic):
        raise RuntimeError("context is invalid")


class AraimValidatorShutdownTest(unittest.TestCase):
    def test_uses_last_observed_count_after_context_shutdown(self):
        self.assertEqual(MODULE.safe_publisher_count(Node(), "/iap/integrity", [1, 1]), 1)
        self.assertEqual(MODULE.safe_publisher_count(Node(), "/iap/integrity", []), 0)


if __name__ == "__main__":
    unittest.main()
