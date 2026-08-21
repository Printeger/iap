import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "gate0_capture_p0_health.py"
)


class FakeNode:
    subscriptions = []

    def __init__(self, name):
        self.name = name

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((message_type, topic, callback, qos))
        return object()

    def create_timer(self, duration_s, callback):
        return (duration_s, callback)

    def destroy_node(self):
        return True


class FakeQoSProfile:
    def __init__(self, *, history, depth, reliability, durability):
        self.history = history
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


class Gate0CaptureTest(unittest.TestCase):
    def _load_module(self):
        fake_iap = types.ModuleType("iap")
        fake_iap.msg = types.SimpleNamespace(IntegrityReport=type(
            "IntegrityReport", (), {}
        ))
        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy_node = types.ModuleType("rclpy.node")
        fake_rclpy_node.Node = FakeNode
        fake_rclpy_qos = types.ModuleType("rclpy.qos")
        fake_rclpy_qos.QoSProfile = FakeQoSProfile
        fake_rclpy_qos.QoSHistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
        fake_rclpy_qos.QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
        fake_rclpy_qos.QoSDurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
        fake_std = types.ModuleType("std_msgs")
        fake_std.msg = types.SimpleNamespace(String=type("String", (), {}))
        modules = {
            "iap": fake_iap,
            "iap.msg": fake_iap.msg,
            "rclpy": fake_rclpy,
            "rclpy.node": fake_rclpy_node,
            "rclpy.qos": fake_rclpy_qos,
            "std_msgs": fake_std,
            "std_msgs.msg": fake_std.msg,
        }
        spec = importlib.util.spec_from_file_location(
            "gate0_capture_p0_health_tested", MODULE_PATH
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        with mock.patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        return module

    def test_topics_qos_and_readiness_match_actual_publishers(self):
        module = self._load_module()
        FakeNode.subscriptions = []
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ready = root / "capture_ready.json"
            node = module.Gate0P0HealthCapture(
                root / "health.jsonl", 1.0, root / "integrity.jsonl", ready
            )
            readiness = module.json.loads(ready.read_text())
            node.destroy_node()
        self.assertEqual(
            [item[1] for item in FakeNode.subscriptions],
            ["/planning/risk_grid_health", "/iap/integrity"],
        )
        for _, _, _, qos in FakeNode.subscriptions:
            self.assertEqual(qos.history, "keep_last")
            self.assertEqual(qos.depth, 100)
            self.assertEqual(qos.reliability, "reliable")
            self.assertEqual(qos.durability, "volatile")
        self.assertTrue(readiness["ready"])
        self.assertEqual(readiness["schema_version"], "gate0_capture_readiness_v1")


if __name__ == "__main__":
    unittest.main()
