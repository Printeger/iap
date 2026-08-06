#include <ego_planner/p0_risk_grid_runtime.h>

#include <cmath>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {

void ensure_rclcpp() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

class FakeProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr) {
      return false;
    }
    results->assign(queries.size(), iap::RiskPredictionResult{});
    for (auto& result : *results) {
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = 1.0;
      result.vpl_pred = 2.0;
      result.reason = "ok";
    }
    return true;
  }
};

class DelayedProvider final : public iap::RiskPredictionProvider {
 public:
  DelayedProvider(std::chrono::milliseconds delay,
                  std::shared_ptr<std::atomic<int>> calls)
      : delay_(delay), calls_(std::move(calls)) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    std::this_thread::sleep_for(delay_);
    calls_->fetch_add(1);
    results->assign(queries.size(), iap::RiskPredictionResult{});
    for (auto& result : *results) {
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = 1.0;
      result.vpl_pred = 1.0;
      result.reason = "ok";
    }
    return true;
  }

 private:
  std::chrono::milliseconds delay_;
  std::shared_ptr<std::atomic<int>> calls_;
};

class BlockingProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      entered_ = true;
    }
    condition_.notify_all();
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() { return released_; });
    results->assign(queries.size(), iap::RiskPredictionResult{});
    for (auto& result : *results) {
      result.available = true;
      result.valid = true;
      result.hpl_pred = 1.0;
      result.vpl_pred = 1.0;
      result.reason = "ok";
    }
    return true;
  }

  void waitUntilEntered() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() { return entered_; });
  }

  void release() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false;
  bool released_ = false;
};

ego_planner::P0RiskGridRuntime::Config enabledConfig() {
  ego_planner::P0RiskGridRuntime::Config config;
  config.enable_risk_grid = true;
  config.grid.resolution_m = 1.0;
  config.grid.size_x_m = 3.0;
  config.grid.size_y_m = 3.0;
  config.grid.size_z_m = 3.0;
  config.grid.horizons_s = {0.0, 1.0};
  config.grid.refresh_period_s = 1000.0;
  config.grid.stale_timeout_s = 100.0;
  config.debug_metrics_enable = false;
  return config;
}

sensor_msgs::msg::PointCloud2::SharedPtr makePointCloud(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>* normals = nullptr) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = "map";
  msg->header.stamp.sec = 123;
  msg->header.stamp.nanosec = 500000000u;
  sensor_msgs::PointCloud2Modifier modifier(*msg);
  if (normals != nullptr) {
    modifier.setPointCloud2Fields(
        6,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "normal_x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "normal_y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "normal_z", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    modifier.setPointCloud2Fields(
        3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  }
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  if (normals != nullptr) {
    sensor_msgs::PointCloud2Iterator<float> iter_nx(*msg, "normal_x");
    sensor_msgs::PointCloud2Iterator<float> iter_ny(*msg, "normal_y");
    sensor_msgs::PointCloud2Iterator<float> iter_nz(*msg, "normal_z");
    for (std::size_t i = 0; i < points.size();
         ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_nx, ++iter_ny, ++iter_nz) {
      *iter_x = static_cast<float>(points[i].x());
      *iter_y = static_cast<float>(points[i].y());
      *iter_z = static_cast<float>(points[i].z());
      const Eigen::Vector3d normal =
          i < normals->size() ? (*normals)[i] : Eigen::Vector3d::Zero();
      *iter_nx = static_cast<float>(normal.x());
      *iter_ny = static_cast<float>(normal.y());
      *iter_nz = static_cast<float>(normal.z());
    }
  } else {
    for (std::size_t i = 0; i < points.size();
         ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = static_cast<float>(points[i].x());
      *iter_y = static_cast<float>(points[i].y());
      *iter_z = static_cast<float>(points[i].z());
    }
  }
  return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr makeInvalidPointCloud() {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = "map";
  msg->header.stamp.sec = 123;
  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(
      2,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  return msg;
}

std::vector<Eigen::Vector3d> sixAxisPrimitivePoints() {
  return {
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, -1.0),
  };
}

std::vector<Eigen::Vector3d> sixAxisPrimitiveNormals() {
  return {
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(0.0, 0.0, -1.0),
  };
}

std::vector<Eigen::Vector3d> planePcaPoints() {
  std::vector<Eigen::Vector3d> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      points.emplace_back(0.35 * static_cast<double>(ix),
                          0.35 * static_cast<double>(iy),
                          0.0);
    }
  }
  return points;
}

}  // namespace

namespace ego_planner {

class P0RiskGridRuntimeStampTest : public ::testing::Test {
 protected:
  static double currentMessageStamp(const P0RiskGridRuntime& runtime) {
    return runtime.currentMessageStamp();
  }

  static double currentRefreshStamp(const P0RiskGridRuntime& runtime) {
    return runtime.currentRefreshStamp();
  }

  static void setOdomStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_odom_stamp_ = stamp;
  }

  static void setCurrentStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_current_.stamp = stamp;
  }

  static void seedValidInputs(P0RiskGridRuntime* runtime,
                              const double odom_stamp,
                              const double current_stamp) {
    runtime->latest_odom_stamp_ = odom_stamp;
    runtime->latest_odom_p_ = Eigen::Vector3d::Zero();
    runtime->latest_odom_q_ = Eigen::Quaterniond::Identity();
    runtime->latest_odom_pose_valid_ = true;
    runtime->latest_current_.stamp = current_stamp;
    runtime->latest_current_.valid = true;
    runtime->latest_current_.hpl = 1.0;
    runtime->latest_current_.vpl = 1.0;
    runtime->latest_current_.hal = 10.0;
    runtime->latest_current_.val = 10.0;
    runtime->latest_current_.im = 9.0;
    runtime->latest_current_valid_ = true;
  }

  static bool buildSnapshot(P0RiskGridRuntime* runtime,
                            const double now_s,
                            iap::IntegritySnapshot* snapshot) {
    return runtime->buildSnapshot(now_s, snapshot);
  }

  static bool refreshOnce(P0RiskGridRuntime* runtime) {
    return runtime->refreshOnceForTest();
  }

  static void publishHealthNow(P0RiskGridRuntime* runtime) {
    runtime->healthTimerCallback();
  }

  static void sendCloud(
      P0RiskGridRuntime* runtime,
      const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    runtime->cloudCallback(msg);
  }

  static std::shared_ptr<const std::vector<Eigen::Vector3d>> lidarMapPoints(
      const P0RiskGridRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.lidar_predictor_input_mutex_);
    return runtime.latest_lidar_map_points_;
  }

  static std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
  lidarFimPrimitives(const P0RiskGridRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.lidar_predictor_input_mutex_);
    return runtime.latest_lidar_fim_primitives_;
  }

  static std::string lidarFimFallbackReason(
      const P0RiskGridRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.lidar_predictor_input_mutex_);
    return runtime.latest_lidar_fim_fallback_reason_;
  }
};

}  // namespace ego_planner

TEST(P0RiskGridRuntimeTest, DisabledConfigCreatesNoRuntimeObject) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_disabled_runtime_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));

  auto runtime = ego_planner::P0RiskGridRuntime::createIfEnabled(node);
  EXPECT_EQ(runtime, nullptr);
  EXPECT_TRUE(node->has_parameter("p0.enable_risk_grid"));
  EXPECT_TRUE(node->has_parameter("p0.skip_occupied_voxels"));
}

TEST(P0RiskGridRuntimeTest, GnssEpochFreshnessDefaultIsTwoSeconds) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_gnss_freshness_default_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);

  EXPECT_DOUBLE_EQ(config.gnss_epoch_max_age_s, 2.0);
  EXPECT_EQ(config.predictor_source_mode, iap::PredictorSourceMode::Fusion);
  EXPECT_EQ(config.predictor_gnss_epoch_policy,
            iap::PredictorGnssEpochPolicy::Auto);
  EXPECT_TRUE(config.predictor_use_current_integrity_prior);
  EXPECT_FALSE(config.predictor_conservative_max_with_gnss);
  EXPECT_TRUE(config.predictor_lidar_legacy_observability);
  EXPECT_DOUBLE_EQ(config.predictor_lidar_fim_radius_m,
                   iap::LidarObservabilityFim::Params{}.fim_radius_m);
  EXPECT_FALSE(config.grid.p5_3_fixture.enabled);
  EXPECT_EQ(config.grid.p5_3_fixture.name, "future_high_risk_zone_v1");
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.x_min_m, -10.8);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.x_max_m, -8.7);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.y_min_m, -0.75);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.y_max_m, 0.75);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.z_min_m, 1.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.z_max_m, 1.35);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.tau_min_s, 1.2);
  EXPECT_DOUBLE_EQ(config.grid.p5_3_fixture.tau_max_s, 2.0);
  EXPECT_FALSE(config.grid.p5_4_fixture.enabled);
  EXPECT_EQ(config.grid.p5_4_fixture.name, "near_risk_zone_v1");
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.x_min_m, -11.7);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.x_max_m, -11.1);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.y_min_m, -0.75);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.y_max_m, 0.75);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.z_min_m, 1.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.z_max_m, 1.35);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.tau_min_s, 0.6);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.tau_max_s, 0.95);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.hpl_pred_m, 10.2);
  EXPECT_DOUBLE_EQ(config.grid.p5_4_fixture.vpl_pred_m, 10.2);
  EXPECT_FALSE(config.grid.p5_6_fixture.enabled);
  EXPECT_EQ(config.grid.p5_6_fixture.name, "future_unknown_zone_v1");
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.x_min_m, -1.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.x_max_m, 12.5);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.y_min_m, -15.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.y_max_m, 15.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.z_min_m, -3.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.z_max_m, 3.0);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.tau_min_s, 0.2);
  EXPECT_DOUBLE_EQ(config.grid.p5_6_fixture.tau_max_s, 2.0);
}

TEST(P0RiskGridRuntimeTest, GnssEpochFreshnessCanBeOverridden) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p0.gnss_epoch_max_age_s", 0.25),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p0_gnss_freshness_override_test", options);

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);

  EXPECT_DOUBLE_EQ(config.gnss_epoch_max_age_s, 0.25);
}

TEST(P0RiskGridRuntimeTest, PredictorParamsCanBeOverridden) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p0.predictor.source_mode", "lidar_only"),
      rclcpp::Parameter("p0.predictor.gnss_epoch_policy", "disabled"),
      rclcpp::Parameter("p0.predictor.use_current_integrity_prior", false),
      rclcpp::Parameter("p0.predictor.conservative_max_with_gnss", true),
      rclcpp::Parameter("p0.predictor.lidar_legacy_observability", false),
      rclcpp::Parameter("p0.predictor.lidar_fim_radius_m", 12.0),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p0_predictor_params_override_test", options);

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);

  EXPECT_EQ(config.predictor_source_mode,
            iap::PredictorSourceMode::LidarOnly);
  EXPECT_EQ(config.predictor_gnss_epoch_policy,
            iap::PredictorGnssEpochPolicy::Disabled);
  EXPECT_FALSE(config.predictor_use_current_integrity_prior);
  EXPECT_TRUE(config.predictor_conservative_max_with_gnss);
  EXPECT_FALSE(config.predictor_lidar_legacy_observability);
  EXPECT_DOUBLE_EQ(config.predictor_lidar_fim_radius_m, 12.0);
}

TEST(P0RiskGridRuntimeTest, P0_6FixtureParamsCanBeOverridden) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p0_6.fixture.enabled", true),
      rclcpp::Parameter("p0_6.fixture.name", "occupied_overlap_box_v1"),
      rclcpp::Parameter("p0_6.fixture.x_min", -1.5),
      rclcpp::Parameter("p0_6.fixture.x_max", 1.5),
      rclcpp::Parameter("p0_6.fixture.y_min", -0.75),
      rclcpp::Parameter("p0_6.fixture.y_max", 0.75),
      rclcpp::Parameter("p0_6.fixture.z_min", 1.0),
      rclcpp::Parameter("p0_6.fixture.z_max", 2.0),
      rclcpp::Parameter("p0_6.fixture.raw_hpl_m", 1.0),
      rclcpp::Parameter("p0_6.fixture.raw_vpl_m", 1.2),
      rclcpp::Parameter("p0_6.fixture.raw_c_pi", 1.2),
      rclcpp::Parameter("p0_6.fixture.low_raw_cost_threshold", 2.0),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p0_6_fixture_params_override_test", options);

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);

  EXPECT_TRUE(config.p0_6_fixture.enabled);
  EXPECT_EQ(config.p0_6_fixture.name, "occupied_overlap_box_v1");
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.x_min_m, -1.5);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.x_max_m, 1.5);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.y_min_m, -0.75);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.y_max_m, 0.75);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.z_min_m, 1.0);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.z_max_m, 2.0);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.raw_hpl_m, 1.0);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.raw_vpl_m, 1.2);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.raw_c_pi, 1.2);
  EXPECT_DOUBLE_EQ(config.p0_6_fixture.low_raw_cost_threshold, 2.0);
}

TEST(P0RiskGridRuntimeTest, P5_3FixtureParamsCanBeOverridden) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p5_3.fixture.enabled", true),
      rclcpp::Parameter("p5_3.fixture.name", "future_high_risk_zone_v1"),
      rclcpp::Parameter("p5_3.fixture.x_min", -14.0),
      rclcpp::Parameter("p5_3.fixture.x_max", 14.0),
      rclcpp::Parameter("p5_3.fixture.y_min", -1.5),
      rclcpp::Parameter("p5_3.fixture.y_max", 1.5),
      rclcpp::Parameter("p5_3.fixture.z_min", 0.5),
      rclcpp::Parameter("p5_3.fixture.z_max", 2.5),
      rclcpp::Parameter("p5_3.fixture.tau_min", 1.2),
      rclcpp::Parameter("p5_3.fixture.tau_max", 2.0),
      rclcpp::Parameter("p5_3.fixture.hpl_pred_m", 10.2),
      rclcpp::Parameter("p5_3.fixture.vpl_pred_m", 10.2),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p5_3_fixture_params_override_test", options);

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);
  const auto& fixture = config.grid.p5_3_fixture;

  EXPECT_TRUE(fixture.enabled);
  EXPECT_EQ(fixture.name, "future_high_risk_zone_v1");
  EXPECT_DOUBLE_EQ(fixture.x_min_m, -14.0);
  EXPECT_DOUBLE_EQ(fixture.x_max_m, 14.0);
  EXPECT_DOUBLE_EQ(fixture.y_min_m, -1.5);
  EXPECT_DOUBLE_EQ(fixture.y_max_m, 1.5);
  EXPECT_DOUBLE_EQ(fixture.z_min_m, 0.5);
  EXPECT_DOUBLE_EQ(fixture.z_max_m, 2.5);
  EXPECT_DOUBLE_EQ(fixture.tau_min_s, 1.2);
  EXPECT_DOUBLE_EQ(fixture.tau_max_s, 2.0);
  EXPECT_DOUBLE_EQ(fixture.hpl_pred_m, 10.2);
  EXPECT_DOUBLE_EQ(fixture.vpl_pred_m, 10.2);
}

TEST(P0RiskGridRuntimeTest, P5_4FixtureParamsCanBeOverridden) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p5_4.fixture.enabled", true),
      rclcpp::Parameter("p5_4.fixture.name", "near_risk_zone_v1"),
      rclcpp::Parameter("p5_4.fixture.x_min", -12.0),
      rclcpp::Parameter("p5_4.fixture.x_max", -11.0),
      rclcpp::Parameter("p5_4.fixture.y_min", -1.0),
      rclcpp::Parameter("p5_4.fixture.y_max", 1.0),
      rclcpp::Parameter("p5_4.fixture.z_min", 0.8),
      rclcpp::Parameter("p5_4.fixture.z_max", 1.6),
      rclcpp::Parameter("p5_4.fixture.tau_min", 0.55),
      rclcpp::Parameter("p5_4.fixture.tau_max", 0.98),
      rclcpp::Parameter("p5_4.fixture.hpl_pred_m", 10.3),
      rclcpp::Parameter("p5_4.fixture.vpl_pred_m", 10.4),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p5_4_fixture_params_override_test", options);

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);
  const auto& fixture = config.grid.p5_4_fixture;

  EXPECT_TRUE(fixture.enabled);
  EXPECT_EQ(fixture.name, "near_risk_zone_v1");
  EXPECT_DOUBLE_EQ(fixture.x_min_m, -12.0);
  EXPECT_DOUBLE_EQ(fixture.x_max_m, -11.0);
  EXPECT_DOUBLE_EQ(fixture.y_min_m, -1.0);
  EXPECT_DOUBLE_EQ(fixture.y_max_m, 1.0);
  EXPECT_DOUBLE_EQ(fixture.z_min_m, 0.8);
  EXPECT_DOUBLE_EQ(fixture.z_max_m, 1.6);
  EXPECT_DOUBLE_EQ(fixture.tau_min_s, 0.55);
  EXPECT_DOUBLE_EQ(fixture.tau_max_s, 0.98);
  EXPECT_DOUBLE_EQ(fixture.hpl_pred_m, 10.3);
  EXPECT_DOUBLE_EQ(fixture.vpl_pred_m, 10.4);
}

TEST(P0RiskGridRuntimeTest, P5_6FixtureParamsCanBeOverridden) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p5_6.fixture.enabled", true),
      rclcpp::Parameter("p5_6.fixture.name", "future_unknown_zone_v1"),
      rclcpp::Parameter("p5_6.fixture.x_min", -2.0),
      rclcpp::Parameter("p5_6.fixture.x_max", 13.0),
      rclcpp::Parameter("p5_6.fixture.y_min", -16.0),
      rclcpp::Parameter("p5_6.fixture.y_max", 16.0),
      rclcpp::Parameter("p5_6.fixture.z_min", -4.0),
      rclcpp::Parameter("p5_6.fixture.z_max", 4.0),
      rclcpp::Parameter("p5_6.fixture.tau_min", 0.3),
      rclcpp::Parameter("p5_6.fixture.tau_max", 1.7),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p5_6_fixture_params_override_test", options);

  const auto config = ego_planner::P0RiskGridRuntime::declareAndReadConfig(node);
  const auto& fixture = config.grid.p5_6_fixture;

  EXPECT_TRUE(fixture.enabled);
  EXPECT_EQ(fixture.name, "future_unknown_zone_v1");
  EXPECT_DOUBLE_EQ(fixture.x_min_m, -2.0);
  EXPECT_DOUBLE_EQ(fixture.x_max_m, 13.0);
  EXPECT_DOUBLE_EQ(fixture.y_min_m, -16.0);
  EXPECT_DOUBLE_EQ(fixture.y_max_m, 16.0);
  EXPECT_DOUBLE_EQ(fixture.z_min_m, -4.0);
  EXPECT_DOUBLE_EQ(fixture.z_max_m, 4.0);
  EXPECT_DOUBLE_EQ(fixture.tau_min_s, 0.3);
  EXPECT_DOUBLE_EQ(fixture.tau_max_s, 1.7);
}

TEST(P0RiskGridRuntimeTest, InvalidPredictorSourceModeThrows) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p0.predictor.source_mode", "camera_only"),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p0_predictor_invalid_source_mode_test", options);

  EXPECT_THROW(ego_planner::P0RiskGridRuntime::declareAndReadConfig(node),
               std::invalid_argument);
}

TEST(P0RiskGridRuntimeTest, InvalidPredictorGnssEpochPolicyThrows) {
  ensure_rclcpp();
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.parameter_overrides({
      rclcpp::Parameter("p0.predictor.gnss_epoch_policy", "sometimes"),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p0_predictor_invalid_gnss_policy_test", options);

  EXPECT_THROW(ego_planner::P0RiskGridRuntime::declareAndReadConfig(node),
               std::invalid_argument);
}

TEST(P0RiskGridRuntimeTest, EnabledRuntimeConstructsWithInjectedProvider) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_enabled_runtime_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));

  ego_planner::P0RiskGridRuntime runtime(
      node, enabledConfig(), std::make_unique<FakeProvider>());
  EXPECT_TRUE(runtime.enabled());
  EXPECT_FALSE(runtime.health().ready);

  EXPECT_FALSE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_EQ(health.reason, "message_stamp_unavailable");
}

TEST(P0RiskGridRuntimeTest, RawHealthTopicIsAbsoluteAndIndependentOfDebugFlag) {
  ensure_rclcpp();
  auto config = enabledConfig();
  config.debug_metrics_enable = false;
  EXPECT_EQ(config.health_topic, "/planning/risk_grid_health");
  auto node = std::make_shared<rclcpp::Node>(
      "p0_raw_health_contract_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  ego_planner::P0RiskGridRuntime runtime(
      node, config, std::make_unique<FakeProvider>());
  // Construction creates the publisher even when debug metrics are disabled;
  // this count is observable without depending on a RViz subscriber.
  EXPECT_EQ(node->count_publishers("/planning/risk_grid_health"), 1u);
}

TEST(SafetyRvizPublisherTest, RiskGridHealthMarkerUsesProvidedStamp) {
  ego_planner::SafetyRvizPublisher::Config config;
  config.frame_id = "map";
  iap::RiskGridHealth health;
  health.ready = true;
  health.stale = false;
  health.reason = "ok";

  const rclcpp::Time stamp(1657065614, 123000000, RCL_ROS_TIME);
  const auto markers =
      ego_planner::SafetyRvizPublisher::buildRiskGridHealthMarkers(
          health, config, stamp);

  ASSERT_FALSE(markers.markers.empty());
  EXPECT_EQ(markers.markers.front().header.stamp.sec, 1657065614);
  EXPECT_EQ(markers.markers.front().header.stamp.nanosec, 123000000u);
}

namespace ego_planner {

TEST_F(P0RiskGridRuntimeStampTest, P0_6FixtureProducesOccupiedSkipHealth) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_6_fixture_occupied_skip_runtime_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = true;
  config.p0_6_fixture.enabled = true;
  config.p0_6_fixture.name = "occupied_overlap_box_v1";
  config.p0_6_fixture.x_min_m = -1.5;
  config.p0_6_fixture.x_max_m = 1.5;
  config.p0_6_fixture.y_min_m = -0.75;
  config.p0_6_fixture.y_max_m = 0.75;
  config.p0_6_fixture.z_min_m = 1.0;
  config.p0_6_fixture.z_max_m = 2.0;
  P0RiskGridRuntime runtime(node, config, std::make_unique<FakeProvider>());

  seedValidInputs(&runtime, 10.0, 10.0);

  ASSERT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_GT(health.occupied_skip_count, 0u);
  EXPECT_EQ(health.dominant_unknown_reason, "occupied_skip");

  const auto snapshot = runtime.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  std::size_t occupied_skip_voxel_count = 0;
  const Eigen::Vector3i dims = snapshot->voxelNum();
  for (int h = 0; h < snapshot->horizonCount(); ++h) {
    for (int x = 0; x < dims.x(); ++x) {
      for (int y = 0; y < dims.y(); ++y) {
        for (int z = 0; z < dims.z(); ++z) {
          iap::RiskVoxel voxel;
          ASSERT_TRUE(snapshot->voxelAt(h, Eigen::Vector3i(x, y, z), &voxel));
          if ((voxel.source_flags & iap::RISK_GRID_SOURCE_OCCUPIED_SKIP) == 0u) {
            continue;
          }
          ++occupied_skip_voxel_count;
          EXPECT_FALSE(voxel.valid);
          EXPECT_TRUE(voxel.unknown);
          EXPECT_FALSE(voxel.stale);
        }
      }
    }
  }
  EXPECT_EQ(occupied_skip_voxel_count, health.occupied_skip_count);
}

TEST_F(P0RiskGridRuntimeStampTest,
       SlowRefreshKeepsDeadlineCadenceHealthAndInputProgress) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_slow_refresh_deadline_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.refresh_period_s = 0.05;
  config.grid.stale_timeout_s = 1.0;
  auto provider_calls = std::make_shared<std::atomic<int>>(0);
  P0RiskGridRuntime runtime(
      node, config,
      std::make_unique<DelayedProvider>(std::chrono::milliseconds(80),
                                        provider_calls));
  seedValidInputs(&runtime, 100.0, 100.0);

  std::atomic<int> health_callbacks{0};
  std::atomic<int> ready_non_stale_callbacks{0};
  std::mutex health_message_mutex;
  std::string last_health_message;
  auto health_sub = node->create_subscription<std_msgs::msg::String>(
      "/planning/risk_grid_health", 100,
      [&](const std_msgs::msg::String::ConstSharedPtr message) {
        health_callbacks.fetch_add(1);
        {
          std::lock_guard<std::mutex> lock(health_message_mutex);
          last_health_message = message->data;
        }
        if (message->data.find("\"ready\":true") != std::string::npos &&
            message->data.find("\"stale\":false") != std::string::npos) {
          ready_non_stale_callbacks.fetch_add(1);
        }
      });
  (void)health_sub;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  std::atomic<bool> update_inputs{true};
  std::thread input_thread([&]() {
    double stamp = 100.0;
    while (update_inputs.load()) {
      stamp += 0.01;
      seedValidInputs(&runtime, stamp, stamp);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  update_inputs.store(false);
  input_thread.join();
  executor.cancel();
  spin_thread.join();

  const auto health = runtime.health();
  EXPECT_GE(provider_calls->load(), 10);
  EXPECT_GE(health.generation_id, 10U);
  EXPECT_GE(health_callbacks.load(), 12);
  EXPECT_GT(ready_non_stale_callbacks.load(), 0);
  EXPECT_TRUE(health.ready);
  EXPECT_FALSE(health.stale);
  std::lock_guard<std::mutex> lock(health_message_mutex);
  EXPECT_NE(last_health_message.find("\"refresh_scheduled_steady_s\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find("\"refresh_queue_delay_ms\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find("\"provider_batch_duration_ms\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find("\"generation_interval_ms\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find("\"input_callback_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find("\"health_callback_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find("\"process_cpu_delta_ms\":"),
            std::string::npos);
}

TEST_F(P0RiskGridRuntimeStampTest,
       HealthJsonPreservesExactSnapshotStampForEvidenceBinding) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_health_stamp_precision_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  constexpr double kSnapshotStamp = 1657065621.4871123;
  seedValidInputs(&runtime, kSnapshotStamp, kSnapshotStamp);
  ASSERT_TRUE(refreshOnce(&runtime));

  std::string health_message;
  auto health_sub = node->create_subscription<std_msgs::msg::String>(
      "/planning/risk_grid_health", 10,
      [&](const std_msgs::msg::String::ConstSharedPtr message) {
        health_message = message->data;
      });
  (void)health_sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  publishHealthNow(&runtime);
  executor.spin_some();

  ASSERT_FALSE(health_message.empty());
  EXPECT_NE(health_message.find(
                "\"last_grid_stamp_s\":1657065621.4871123"),
            std::string::npos);
}

TEST_F(P0RiskGridRuntimeStampTest,
       BlockedRefreshDoesNotStarveHealthAndConsecutiveStaleReports) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_blocked_refresh_contention_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 1.0;
  auto provider = std::make_unique<BlockingProvider>();
  auto* provider_ptr = provider.get();
  P0RiskGridRuntime runtime(node, config, std::move(provider));
  seedValidInputs(&runtime, 100.0, 100.0);

  std::atomic<int> health_callbacks{0};
  std::atomic<int> stale_callbacks{0};
  auto health_sub = node->create_subscription<std_msgs::msg::String>(
      "/planning/risk_grid_health", 100,
      [&](const std_msgs::msg::String::ConstSharedPtr message) {
        health_callbacks.fetch_add(1);
        if (message->data.find("\"stale\":true") != std::string::npos)
          stale_callbacks.fetch_add(1);
      });
  (void)health_sub;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  std::thread refresh_thread([&runtime]() { refreshOnce(&runtime); });
  provider_ptr->waitUntilEntered();
  for (int index = 0; index < 5; ++index) {
    publishHealthNow(&runtime);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  EXPECT_GE(health_callbacks.load(), 4);
  provider_ptr->release();
  refresh_thread.join();
  EXPECT_TRUE(runtime.health().ready);

  setOdomStamp(&runtime, 102.0);
  for (int index = 0; index < 3; ++index) {
    publishHealthNow(&runtime);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  executor.cancel();
  spin_thread.join();
  EXPECT_GE(stale_callbacks.load(), 2);
  EXPECT_TRUE(runtime.health().stale);
}

TEST_F(P0RiskGridRuntimeStampTest, OdomStampIsPreferredForRefreshTime) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stamp_odom_preferred_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  setOdomStamp(&runtime, 123.5);
  setCurrentStamp(&runtime, 456.5);

  EXPECT_DOUBLE_EQ(currentMessageStamp(runtime), 123.5);
  EXPECT_DOUBLE_EQ(currentRefreshStamp(runtime), 123.5);
}

TEST_F(P0RiskGridRuntimeStampTest,
       OccupancyEpochFactoryFreezesOneGenerationPerRefresh) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_frozen_occupancy_epoch_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  uint64_t capture_count = 0;
  runtime.setOccupancyDiagnosticQueryFactory([&capture_count]() {
    const uint64_t captured_generation = ++capture_count;
    return [captured_generation](const Eigen::Vector3d& position) {
      iap::RiskOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.occupancy_generation = captured_generation;
      diagnostic.voxel_center = position;
      diagnostic.resolution_m = 1.0;
      diagnostic.frame_id = "map";
      diagnostic.cloud_stamp_s = 100.0 + captured_generation;
      diagnostic.source = "frozen_test_epoch";
      return diagnostic;
    };
  });
  seedValidInputs(&runtime, 100.0, 100.0);

  EXPECT_TRUE(runtime.refreshOnceForTest());
  ASSERT_EQ(capture_count, 1u);
  const auto first = runtime.acquireSnapshot();
  ASSERT_NE(first, nullptr);
  iap::RiskCostSample first_cost;
  iap::RiskCostQueryTrace first_trace;
  ASSERT_TRUE(first->queryCost(Eigen::Vector3d::Zero(), 100.0,
                               &first_cost, &first_trace));
  ASSERT_FALSE(first_trace.corners.empty());
  EXPECT_EQ(first_trace.corners.front().occupancy.occupancy_generation, 1u);

  seedValidInputs(&runtime, 100.5, 100.5);
  EXPECT_TRUE(runtime.refreshOnceForTest());
  EXPECT_EQ(capture_count, 2u);
  const auto second = runtime.acquireSnapshot();
  ASSERT_NE(second, nullptr);
  iap::RiskCostSample second_cost;
  iap::RiskCostQueryTrace second_trace;
  ASSERT_TRUE(second->queryCost(Eigen::Vector3d::Zero(), 100.5,
                                &second_cost, &second_trace));
  ASSERT_FALSE(second_trace.corners.empty());
  EXPECT_EQ(second_trace.corners.front().occupancy.occupancy_generation, 2u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       MissingOccupancyEpochFromConfiguredFactoryFailsClosed) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_missing_occupancy_epoch_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  runtime.setOccupancyDiagnosticQueryFactory(
      []() { return iap::RiskGridMap::OccupancyDiagnosticQuery{}; });
  seedValidInputs(&runtime, 100.0, 100.0);

  EXPECT_FALSE(runtime.refreshOnceForTest());
  EXPECT_FALSE(runtime.health().ready);
  EXPECT_EQ(runtime.health().reason, "occupancy_generation_changed");
}

TEST_F(P0RiskGridRuntimeStampTest, CurrentStampIsFallbackWhenOdomInvalid) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stamp_current_fallback_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  setOdomStamp(&runtime, std::numeric_limits<double>::quiet_NaN());
  setCurrentStamp(&runtime, 456.5);

  EXPECT_DOUBLE_EQ(currentMessageStamp(runtime), 456.5);
  EXPECT_DOUBLE_EQ(currentRefreshStamp(runtime), 456.5);
}

TEST_F(P0RiskGridRuntimeStampTest, MessageStampIsNaNWhenMessageStampsInvalid) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stamp_no_message_stamp_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  setOdomStamp(&runtime, std::numeric_limits<double>::quiet_NaN());
  setCurrentStamp(&runtime, std::numeric_limits<double>::quiet_NaN());

  const double message_stamp_s = currentMessageStamp(runtime);
  const double stamp_s = currentRefreshStamp(runtime);
  EXPECT_TRUE(std::isnan(message_stamp_s));
  EXPECT_TRUE(std::isnan(stamp_s));
}

TEST_F(P0RiskGridRuntimeStampTest, RefreshSnapshotUsesMessageStamp) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_refresh_uses_message_stamp_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  seedValidInputs(&runtime, 123.5, 456.5);

  ASSERT_TRUE(runtime.refreshOnceForTest());
  const auto snapshot = runtime.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_DOUBLE_EQ(snapshot->stamp_s(), 123.5);
}

TEST_F(P0RiskGridRuntimeStampTest,
       UseCurrentIntegrityPriorFalseOmitsLambdaBase) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_no_current_prior_snapshot_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_use_current_integrity_prior = false;
  P0RiskGridRuntime runtime(node, config, std::make_unique<FakeProvider>());

  seedValidInputs(&runtime, 123.5, 123.5);
  iap::IntegritySnapshot snapshot;
  ASSERT_TRUE(buildSnapshot(&runtime, 123.5, &snapshot));

  EXPECT_FALSE(snapshot.has_lambda_base);
}

TEST_F(P0RiskGridRuntimeStampTest, PointCloudXyzParsingStoresFiniteMapPoints) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_cloud_xyz_parse_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  const auto cloud = makePointCloud({
      Eigen::Vector3d(1.0, 2.0, 3.0),
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      Eigen::Vector3d(4.0, 5.0, 6.0),
  });

  sendCloud(&runtime, cloud);

  const auto points = lidarMapPoints(runtime);
  ASSERT_NE(points, nullptr);
  ASSERT_EQ(points->size(), 2u);
  EXPECT_TRUE((*points)[0].isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE((*points)[1].isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  const auto health = runtime.health();
  EXPECT_EQ(health.predictor_lidar_map_point_count, 2u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       PointCloudNormalFieldsGenerateFimPrimitives) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_cloud_normals_parse_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  const auto points = sixAxisPrimitivePoints();
  const auto normals = sixAxisPrimitiveNormals();

  sendCloud(&runtime, makePointCloud(points, &normals));

  const auto primitives = lidarFimPrimitives(runtime);
  ASSERT_NE(primitives, nullptr);
  ASSERT_EQ(primitives->size(), normals.size());
  EXPECT_GT(runtime.health().predictor_lidar_fim_valid_normal_count, 0u);
  EXPECT_EQ(runtime.health().predictor_lidar_fim_fallback_reason, "");
}

TEST_F(P0RiskGridRuntimeStampTest,
       PointCloudWithoutNormalsGeneratesPcaFimPrimitives) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_cloud_pca_primitives_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  sendCloud(&runtime, makePointCloud(planePcaPoints()));

  const auto health = runtime.health();
  EXPECT_GT(health.predictor_lidar_map_point_count, 0u);
  EXPECT_GT(health.predictor_lidar_fim_primitive_count, 0u);
  EXPECT_GT(health.predictor_lidar_fim_valid_normal_count, 0u);
  EXPECT_EQ(health.predictor_lidar_fim_fallback_reason, "");
}

TEST_F(P0RiskGridRuntimeStampTest,
       InvalidOrEmptyPointCloudClearsLidarPredictorInputs) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_cloud_clear_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  const auto points = sixAxisPrimitivePoints();
  const auto normals = sixAxisPrimitiveNormals();
  sendCloud(&runtime, makePointCloud(points, &normals));
  ASSERT_NE(lidarMapPoints(runtime), nullptr);

  sendCloud(&runtime, makeInvalidPointCloud());
  EXPECT_EQ(lidarMapPoints(runtime), nullptr);
  EXPECT_EQ(lidarFimPrimitives(runtime), nullptr);
  EXPECT_NE(lidarFimFallbackReason(runtime).find("invalid_lidar_pointcloud"),
            std::string::npos);

  sendCloud(&runtime, makePointCloud({}));
  EXPECT_EQ(lidarMapPoints(runtime), nullptr);
  EXPECT_EQ(lidarFimPrimitives(runtime), nullptr);
  EXPECT_EQ(lidarFimFallbackReason(runtime), "empty_lidar_pointcloud");
}

TEST_F(P0RiskGridRuntimeStampTest,
       LidarOnlyAutoMissingGnssEpochDoesNotMakeFrameStale) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_only_auto_missing_gnss_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Auto;
  P0RiskGridRuntime runtime(node, config);

  seedValidInputs(&runtime, 123.5, 123.5);
  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();

  EXPECT_TRUE(health.ready);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_GT(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_NE(health.reason, "stale_gnss_epoch");
}

TEST_F(P0RiskGridRuntimeStampTest,
       LidarOnlyDisabledWithFimPrimitivesUsesLidarAndNotGnss) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_only_fim_source_count_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  config.predictor_use_current_integrity_prior = false;
  config.predictor_lidar_legacy_observability = false;
  P0RiskGridRuntime runtime(node, config);
  const auto points = sixAxisPrimitivePoints();
  const auto normals = sixAxisPrimitiveNormals();

  sendCloud(&runtime, makePointCloud(points, &normals));
  seedValidInputs(&runtime, 123.5, 123.5);

  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_TRUE(health.ready);
  EXPECT_GT(health.valid_ratio, 0.0);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_GT(health.predictor_lidar_used_count, 0u);
  EXPECT_GT(health.predictor_lidar_fim_primitive_count, 0u);
  EXPECT_GT(health.predictor_lidar_fim_valid_normal_count, 0u);
  EXPECT_NE(health.reason, "stale_gnss_epoch");
}

TEST_F(P0RiskGridRuntimeStampTest,
       FusionAutoMissingGnssEpochUsesLidarAndNotStaleField) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_fusion_auto_missing_gnss_lidar_good_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::Fusion;
  config.predictor_gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Auto;
  config.predictor_lidar_legacy_observability = false;
  P0RiskGridRuntime runtime(node, config);
  const auto points = sixAxisPrimitivePoints();
  const auto normals = sixAxisPrimitiveNormals();

  sendCloud(&runtime, makePointCloud(points, &normals));
  seedValidInputs(&runtime, 123.5, 123.5);

  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_TRUE(health.ready);
  EXPECT_FALSE(health.stale);
  EXPECT_GT(health.valid_ratio, 0.0);
  EXPECT_LT(health.unknown_ratio, 1.0);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_GT(health.predictor_lidar_used_count, 0u);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_NE(health.reason, "stale_gnss_epoch");
  EXPECT_NE(health.dominant_unknown_reason, "stale_gnss_epoch");
}

TEST_F(P0RiskGridRuntimeStampTest,
       StaleCurrentPriorWithLidarGoodDoesNotMakeFieldStale) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stale_current_prior_lidar_good_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 0.5;
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  config.predictor_use_current_integrity_prior = true;
  config.predictor_lidar_legacy_observability = false;
  P0RiskGridRuntime runtime(node, config);
  const auto points = sixAxisPrimitivePoints();
  const auto normals = sixAxisPrimitiveNormals();

  sendCloud(&runtime, makePointCloud(points, &normals));
  seedValidInputs(&runtime, 100.0, 99.0);

  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_TRUE(health.ready);
  EXPECT_FALSE(health.stale);
  EXPECT_GT(health.valid_ratio, 0.0);
  EXPECT_LT(health.unknown_ratio, 1.0);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_GT(health.predictor_lidar_used_count, 0u);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_EQ(health.predictor_prior_used_count, 0u);
  EXPECT_GT(health.predictor_stale_current_prior_count, 0u);
  EXPECT_NE(health.reason, "stale_integrity");
  EXPECT_NE(health.dominant_unknown_reason, "stale_integrity");
}

TEST_F(P0RiskGridRuntimeStampTest,
       StaleCurrentWithNoUsablePredictorSourceStaysFullUnknown) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stale_current_no_predictor_source_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 0.5;
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  config.predictor_lidar_legacy_observability = false;
  P0RiskGridRuntime runtime(node, config);

  seedValidInputs(&runtime, 100.0, 99.0);

  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_TRUE(health.ready);
  EXPECT_DOUBLE_EQ(health.valid_ratio, 0.0);
  EXPECT_DOUBLE_EQ(health.unknown_ratio, 1.0);
  EXPECT_GT(health.provider_stale_count, 0u);
  EXPECT_EQ(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.predictor_lidar_used_count, 0u);
  EXPECT_EQ(health.predictor_stale_current_prior_count, 0u);
  EXPECT_EQ(health.reason, "stale_integrity");
  EXPECT_EQ(health.dominant_unknown_reason, "stale_integrity");
}

TEST_F(P0RiskGridRuntimeStampTest,
       LidarOnlyDisabledMissingGnssEpochDoesNotMakeFrameStale) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_only_disabled_missing_gnss_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  P0RiskGridRuntime runtime(node, config);

  seedValidInputs(&runtime, 123.5, 123.5);
  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();

  EXPECT_TRUE(health.ready);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_GT(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_NE(health.reason, "stale_gnss_epoch");
}

}  // namespace ego_planner
