#include <ego_planner/p0_risk_grid_runtime.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

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

}  // namespace ego_planner
