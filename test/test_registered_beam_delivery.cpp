#include <gtest/gtest.h>
#include <iap/local_map/beam_evidence_binding.hpp>
#include <iap/msg/active_lidar_window_delta.hpp>
#include <iap/msg/registered_lidar_frame.hpp>
#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/preprocess/callbacks.hpp>
#include <iap/util/config.hpp>
#include <iap/util/extension_module_ros2.hpp>
#include <iap/util/run_log_manager.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
#include <unistd.h>

extern "C" glim::ExtensionModule* create_extension_module();

TEST(RegisteredBeamDelivery, LateEvidenceRefreshesCurrentAndActiveWithoutRollback) {
  const auto root = std::filesystem::temp_directory_path() /
      ("iap_beam_delivery_" + std::to_string(::getpid()));
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::filesystem::remove_all(path); }
  } cleanup{root};
  std::filesystem::create_directories(root / "config");
  std::filesystem::create_directories(root / "run/runtime/ros");
  setenv("IAP_RUN_DIR", (root / "run").c_str(), 1);
  setenv("ROS_LOG_DIR", (root / "run/runtime/ros").c_str(), 1);
  std::ofstream(root / "config/config.json") << R"({"global":{"config_path":"","config_ros":"config_ros.json"}})";
  std::ofstream(root / "config/config_ros.json") << R"({"glim_ros":{"planner_local_map":{
    "current_topic":"/beam_test/current","delta_topic":"/beam_test/delta",
    "beam_evidence_topic":"/beam_test/evidence","integrity_topic":"/beam_test/integrity",
    "recovery_service":"/beam_test/recovery","frame_contract_id":"beam-test",
    "planning_lattice_resolution_m":0.1,"planning_lattice_origin_m":[0,0,0],
    "planning_lattice_extent_m":[10,10,10],"window_rate_hz":20}}})";
  glim::GlobalConfig::instance((root / "config").string(), true);
  glim::RunLogManager::initialize("beam_delivery_test", (root / "config").string());
  rclcpp::init(0, nullptr);
  struct Shutdown { ~Shutdown() { rclcpp::shutdown(); } } shutdown;
  auto node = std::make_shared<rclcpp::Node>("registered_beam_delivery_test");
  std::unique_ptr<glim::ExtensionModule> extension(create_extension_module());
  auto* ros_extension = dynamic_cast<glim::ExtensionModuleROS2*>(extension.get());
  ASSERT_NE(ros_extension, nullptr);
  ros_extension->create_subscriptions(*node);
  std::vector<iap::msg::RegisteredLidarFrame> current;
  std::vector<iap::msg::ActiveLidarWindowDelta> deltas;
  auto current_sub = node->create_subscription<iap::msg::RegisteredLidarFrame>(
      "/beam_test/current", rclcpp::SensorDataQoS(),
      [&current](const iap::msg::RegisteredLidarFrame& value) { current.push_back(value); });
  auto delta_sub = node->create_subscription<iap::msg::ActiveLidarWindowDelta>(
      "/beam_test/delta", rclcpp::QoS(128).reliable(),
      [&deltas](const iap::msg::ActiveLidarWindowDelta& value) { deltas.push_back(value); });
  auto publisher = node->create_publisher<iap::msg::LidarBeamEvidence>(
      "/beam_test/evidence", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto until = [&](const auto& predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      executor.spin_some();
      if (predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  };
  ASSERT_TRUE(until([&] { return publisher->get_subscription_count() == 1U; }));
  const auto scan = [](int id, double stamp) {
    auto points = std::make_shared<glim::RawPoints>();
    points->stamp = stamp;
    points->points = {Eigen::Vector4d(3, 0, 0, 1)};
    points->times = {0};
    glim::PreprocessCallbacks::on_raw_points_received(points);
    auto raw = std::make_shared<glim::PreprocessedFrame>();
    raw->stamp = raw->scan_end_time = stamp;
    auto frame = std::make_shared<glim::EstimationFrame>();
    frame->id = id;
    frame->stamp = stamp;
    frame->raw_frame = raw;
    frame->T_world_lidar = Eigen::Isometry3d::Identity();
    frame->T_lidar_imu = Eigen::Isometry3d::Identity();
    glim::OdometryEstimationCallbacks::on_update_new_frame(frame);
    return frame;
  };
  const auto evidence = [](double stamp) {
    iap::msg::LidarBeamEvidence value;
    value.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    value.header.frame_id = "iap_lidar_reference";
    value.scan_end_stamp_s = stamp;
    value.sensor_model_id = "test-first-hit";
    value.horizontal_samples = 2;
    value.vertical_samples = 1;
    value.horizontal_fov_rad = 1;
    value.min_range_m = 0.1;
    value.max_range_m = 8;
    value.complete = true;
    value.outcomes = {1, 2};
    value.direction_x = {1, 0};
    value.direction_y = {0, 1};
    value.direction_z = {0, 0};
    value.ranges_m = {3, 8};
    value.content_hash = iap::local_map::beamEvidenceContentHash(value);
    return value;
  };
  auto frame89 = scan(89, 42.25);
  glim::OdometryEstimationCallbacks::on_update_keyframes(
      std::vector<glim::EstimationFrame::ConstPtr>{frame89});
  ASSERT_TRUE(until([&] { return !current.empty() && !deltas.empty(); }));
  EXPECT_FALSE(current.back().beam_evidence_complete);
  EXPECT_EQ(current.back().beam_binding_reason, "no_valid_received_evidence");
  ASSERT_EQ(deltas.back().added.size(), 1U);
  EXPECT_FALSE(deltas.back().added.front().beam_evidence_complete);
  publisher->publish(evidence(42.3));
  auto invalid = evidence(42.25);
  invalid.content_hash = "corrupt";
  publisher->publish(invalid);
  executor.spin_some();
  publisher->publish(evidence(42.25));
  ASSERT_TRUE(until([&] {
    return current.back().beam_evidence_complete && deltas.back().generation == 2U;
  }));
  EXPECT_EQ(current.back().frame_id, 89);
  EXPECT_EQ(current.back().beam_binding_reason, "matched_exact_scan");
  EXPECT_GE(current.back().beam_received_count, 3u);
  EXPECT_GE(current.back().beam_invalid_count, 1u);
  EXPECT_DOUBLE_EQ(current.back().beam_same_start_end_stamp_s, 42.25);
  EXPECT_TRUE(current.back().source_health_valid);
  EXPECT_EQ(deltas.back().removed_frame_ids, (std::vector<int64_t>{89}));
  ASSERT_EQ(deltas.back().added.size(), 1U);
  EXPECT_TRUE(deltas.back().added.front().beam_evidence_complete);
  EXPECT_TRUE(deltas.back().added.front().source_health_valid);
  EXPECT_DOUBLE_EQ(deltas.back().added.front().scan_end_stamp_s, 42.25);
  auto frame90 = scan(90, 42.4);
  glim::OdometryEstimationCallbacks::on_update_keyframes(
      std::vector<glim::EstimationFrame::ConstPtr>{frame89, frame90});
  ASSERT_TRUE(until([&] { return current.back().frame_id == 90 && deltas.back().generation == 3U; }));
  scan(91, 42.5);
  ASSERT_TRUE(until([&] { return current.back().frame_id == 91; }));
  publisher->publish(evidence(42.4));
  ASSERT_TRUE(until([&] { return deltas.back().generation == 4U; }));
  EXPECT_EQ(current.back().frame_id, 91);
  EXPECT_FALSE(current.back().beam_evidence_complete);
  EXPECT_EQ(deltas.back().removed_frame_ids, (std::vector<int64_t>{90}));
  ASSERT_EQ(deltas.back().added.size(), 1U);
  EXPECT_EQ(deltas.back().added.front().frame_id, 90);
  EXPECT_TRUE(deltas.back().added.front().beam_evidence_complete);
  executor.remove_node(node);
  extension.reset();
}
