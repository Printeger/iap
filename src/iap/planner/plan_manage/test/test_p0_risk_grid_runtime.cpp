#include <ego_planner/p0_risk_grid_runtime.h>

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

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
