#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <unistd.h>

#include <iap/odometry/initial_state_estimation.hpp>
#include <iap/util/config.hpp>

namespace {

TEST(NaiveInitialStateEstimationTest, PreservesLatestImuTimestamp) {
  const auto root = std::filesystem::temp_directory_path() /
      ("iap_initial_state_test_" + std::to_string(::getpid()) + "_" +
       std::to_string(std::chrono::steady_clock::now()
                          .time_since_epoch().count()));
  std::filesystem::create_directories(root);
  {
    std::ofstream config(root / "config.json");
    config << "{\"global\":{\"config_odometry\":\"config_odometry.json\"}}\n";
  }
  {
    std::ofstream config(root / "config_odometry.json");
    config << "{\"odometry_estimation\":{\"initialization_window_size\":1.0}}\n";
  }
  glim::GlobalConfig::instance(root.string(), true);

  glim::NaiveInitialStateEstimation estimator(
      Eigen::Isometry3d::Identity(), Eigen::Matrix<double, 6, 1>::Zero());
  estimator.set_init_state(
      Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
  estimator.insert_imu(
      42.5, Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());

  const auto state = estimator.initial_pose();
  ASSERT_NE(state, nullptr);
  EXPECT_DOUBLE_EQ(state->stamp, 42.5);
  EXPECT_TRUE(state->T_world_imu.matrix().allFinite());
  EXPECT_TRUE(state->T_world_lidar.matrix().allFinite());
  EXPECT_TRUE(state->v_world_imu.allFinite());
  EXPECT_TRUE(state->imu_bias.allFinite());

  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
  EXPECT_FALSE(cleanup_error);
}

}  // namespace
