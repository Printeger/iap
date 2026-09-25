import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    REPO / "scripts" / "dev_planner" / "verify_icra_rviz_runtime.py")
SPEC = importlib.util.spec_from_file_location(
    "verify_icra_rviz_runtime", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def point(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def complete_report():
    report = MODULE.empty_report()
    report["p0_ready"] = True
    report["risk_generation_max"] = 4
    report["p4_candidate_ready"] = True
    report["required_settle_s"] = 2.0
    report["settled_after_last_first_data_s"] = 2.0
    report["config"] = {
        "source_install_match": True,
        "loaded_install_config": True,
        "validation_failures": [],
    }
    report["visual_proof"] = {"captured": True}
    for topic, specification in MODULE.TARGET_TOPICS.items():
        report["topics"][topic] = {
            "publisher_endpoints": [{"qos": {}}],
            "rviz_subscriber_endpoints": [{"node": "/test_planner_rviz",
                                            "qos": {}}],
            "matched": True,
            "message_count": (
                1 if specification["payload_required"] else 0),
            "max_point_count": 10,
            "max_non_delete_markers": 2,
            "max_geometry_markers": 2,
            "max_path_markers": 2,
        }
    return report


class TestRvizRuntimeVerification(unittest.TestCase):
    def test_target_rviz_config_has_enabled_runtime_overlays(self):
        result = MODULE.validate_rviz_config(
            REPO / "config/sim_demo11/test_icra.rviz")

        self.assertEqual(result["fixed_frame"], "map")
        self.assertEqual(result["failures"], [])
        self.assertEqual(
            set(result["displays"]), set(MODULE.TARGET_TOPICS))

    def test_source_and_install_config_identity_must_match(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.rviz"
            installed = root / "installed.rviz"
            source.write_text("same\n")
            installed.write_text("same\n")

            identity = MODULE.config_identity(source, installed, installed)
            self.assertTrue(identity["source_install_match"])
            self.assertTrue(identity["loaded_install_config"])

            installed.write_text("different\n")
            identity = MODULE.config_identity(source, installed, installed)
            self.assertFalse(identity["source_install_match"])

    def test_symlink_install_keeps_loaded_install_path_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source/test_icra.rviz"
            installed = root / "install/test_icra.rviz"
            source.parent.mkdir()
            installed.parent.mkdir()
            source.write_text("same\n")
            installed.symlink_to(source)

            identity = MODULE.config_identity(source, installed, installed)

            self.assertTrue(identity["source_install_match"])
            self.assertTrue(identity["loaded_install_config"])
            self.assertEqual(identity["rviz_config_path"], str(installed))

    def test_best_effort_rviz_subscriber_matches_best_effort_publisher(self):
        self.assertTrue(MODULE.qos_compatible(
            {"reliability": "BEST_EFFORT", "durability": "VOLATILE"},
            {"reliability": "BEST_EFFORT", "durability": "VOLATILE"}))

    def test_reliable_rviz_subscriber_rejects_best_effort_publisher(self):
        self.assertFalse(MODULE.qos_compatible(
            {"reliability": "BEST_EFFORT", "durability": "VOLATILE"},
            {"reliability": "RELIABLE", "durability": "VOLATILE"}))

    def test_pointcloud_requires_nonempty_payload(self):
        nonempty = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), width=4, height=3)
        empty = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), width=0, height=1)

        self.assertEqual(
            MODULE.pointcloud_metrics(nonempty),
            {"frame_id": "map", "point_count": 12})
        self.assertEqual(MODULE.pointcloud_metrics(empty)["point_count"], 0)

    def test_marker_array_requires_non_delete_finite_geometry(self):
        delete_all = SimpleNamespace(
            action=3, type=4, points=[],
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(position=point(0.0, 0.0, 0.0)),
            scale=point(0.0, 0.0, 0.0))
        path = SimpleNamespace(
            action=0, type=4,
            points=[point(-1.0, -1.0, 1.5), point(1.0, -1.0, 1.5)],
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(position=point(0.0, 0.0, 0.0)),
            scale=point(0.12, 0.0, 0.0))

        metrics = MODULE.marker_array_metrics(
            SimpleNamespace(markers=[delete_all, path]))

        self.assertEqual(metrics["marker_count"], 2)
        self.assertEqual(metrics["non_delete_marker_count"], 1)
        self.assertEqual(metrics["geometry_marker_count"], 1)
        self.assertEqual(metrics["path_marker_count"], 1)
        self.assertEqual(metrics["frame_id"], "map")

    def test_startup_warning_does_not_fail_final_compatible_endpoint(self):
        report = complete_report()
        report["startup_qos_warning_count"] = 2
        for evidence in report["topics"].values():
            qos = {"reliability": "BEST_EFFORT",
                   "durability": "VOLATILE"}
            evidence["publisher_endpoints"] = [{"qos": qos}]
            evidence["rviz_subscriber_endpoints"] = [{
                "node": "/test_planner_rviz", "qos": qos}]

        self.assertEqual(MODULE.runtime_failures(report), [])

    def test_final_incompatible_endpoint_fails(self):
        report = complete_report()
        topic = "/iap/local_map/current_hits_map"
        report["topics"][topic]["matched"] = False

        self.assertIn(
            "qos_incompatible:/iap/local_map/current_hits_map",
            MODULE.runtime_failures(report))

    def test_topic_without_data_fails(self):
        report = complete_report()
        topic = "/iap/rviz/p4_topology_channels"
        report["topics"][topic]["message_count"] = 0

        self.assertIn(
            "no_messages:/iap/rviz/p4_topology_channels",
            MODULE.runtime_failures(report))


if __name__ == "__main__":
    unittest.main()
