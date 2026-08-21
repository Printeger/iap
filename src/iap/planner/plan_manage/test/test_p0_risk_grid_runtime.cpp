#include <ego_planner/p0_risk_grid_runtime.h>

#include <algorithm>
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

class CallbackProvider final : public iap::RiskPredictionProvider {
 public:
  explicit CallbackProvider(std::function<void()> callback)
      : callback_(std::move(callback)) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr) {
      return false;
    }
    if (callback_) {
      callback_();
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

 private:
  std::function<void()> callback_;
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
  config.predictor_sigma_grow_m_sqrt_s = 0.1;
  config.debug_metrics_enable = false;
  return config;
}

iap::GnssEpoch makeGnssEpoch(const int satellite_count,
                             const double stamp_s) {
  constexpr double kPi = 3.14159265358979323846;
  iap::GnssEpoch epoch;
  epoch.stamp = stamp_s;
  epoch.gps_sec = 2100000.0;
  for (int index = 0; index < satellite_count; ++index) {
    iap::SatObs satellite;
    satellite.sat_id = 300 + index;
    satellite.constellation = 'G';
    satellite.elevation = 0.45 + 0.08 * static_cast<double>(index % 4);
    satellite.azimuth = 2.0 * kPi * static_cast<double>(index) /
                        static_cast<double>(std::max(1, satellite_count));
    satellite.pr_sigma = 3.0 + static_cast<double>(index % 2);
    epoch.sats.push_back(satellite);
  }
  return epoch;
}

ego_planner::P0OccupancyEpochCapture makeOccupancyEpochCapture(
    const std::shared_ptr<std::atomic<uint64_t>>& live_generation,
    const uint64_t captured_generation,
    const double stamp_s,
    const std::string& frame_id,
    const std::vector<Eigen::Vector3d>& occupied_centers = {}) {
  iap::LocalOccupancyGrid::Params params;
  params.voxel_size = 1.0;
  params.lattice_origin = Eigen::Vector3d(-1.5, -1.5, -1.5);
  params.max_voxels = static_cast<int>(
      std::max<std::size_t>(1u, occupied_centers.size()));
  params.enable_eviction = false;
  auto owner = std::make_shared<iap::LocalOccupancyGrid>(params);
  owner->insert_points(occupied_centers);

  ego_planner::P0OccupancyEpoch epoch;
  epoch.generation = captured_generation;
  epoch.cloud_stamp_s = stamp_s;
  epoch.frame_id = frame_id;
  epoch.los_owner = owner;
  epoch.live_generation = [live_generation]() {
    return live_generation->load();
  };
  epoch.diagnostic_query =
      [owner, captured_generation, stamp_s, frame_id](
          const Eigen::Vector3d& position) {
        iap::RiskOccupancyDiagnostic diagnostic;
        diagnostic.available = true;
        diagnostic.raw_occupied = owner->occupied_at(position);
        diagnostic.inflated_occupied = diagnostic.raw_occupied;
        diagnostic.voxel_center = position;
        diagnostic.resolution_m = owner->params().voxel_size;
        diagnostic.inflation_m = 0.0;
        diagnostic.frame_id = frame_id;
        diagnostic.cloud_stamp_s = stamp_s;
        diagnostic.occupancy_generation = captured_generation;
        diagnostic.source = "frozen_test_epoch";
        return diagnostic;
      };
  return {ego_planner::P0OccupancyEpochCaptureStatus::VALID,
          std::move(epoch)};
}

std::vector<Eigen::Vector3d> riskGridCenters() {
  std::vector<Eigen::Vector3d> centers;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        centers.emplace_back(static_cast<double>(x),
                             static_cast<double>(y),
                             static_cast<double>(z));
      }
    }
  }
  return centers;
}

std::vector<Eigen::Vector3d> gnssLosBlockers(const iap::GnssEpoch& epoch) {
  std::vector<Eigen::Vector3d> points = riskGridCenters();
  for (const auto& origin : riskGridCenters()) {
    for (const auto& satellite : epoch.sats) {
      const double cos_elevation = std::cos(satellite.elevation);
      const Eigen::Vector3d direction(
          cos_elevation * std::sin(satellite.azimuth),
          cos_elevation * std::cos(satellite.azimuth),
          std::sin(satellite.elevation));
      for (double distance = 1.0; distance <= 6.0; distance += 0.5) {
        points.push_back(origin + distance * direction);
      }
    }
  }
  return points;
}

void expectEquivalentDouble(const double lhs, const double rhs) {
  if (std::isfinite(lhs) || std::isfinite(rhs)) {
    ASSERT_TRUE(std::isfinite(lhs));
    ASSERT_TRUE(std::isfinite(rhs));
    EXPECT_NEAR(lhs, rhs, 1.0e-12);
    return;
  }
  EXPECT_EQ(std::isnan(lhs), std::isnan(rhs));
  if (std::isinf(lhs) || std::isinf(rhs)) {
    EXPECT_EQ(lhs, rhs);
  }
}

void expectSnapshotsScientificallyEquivalent(
    const std::shared_ptr<const iap::RiskGridSnapshot>& lhs,
    const std::shared_ptr<const iap::RiskGridSnapshot>& rhs,
    const bool require_same_generation = true) {
  ASSERT_NE(lhs, nullptr);
  ASSERT_NE(rhs, nullptr);
  if (require_same_generation) {
    EXPECT_EQ(lhs->generation_id(), rhs->generation_id());
  }
  EXPECT_EQ(lhs->horizonCount(), rhs->horizonCount());
  EXPECT_EQ(lhs->layerVoxelCount(), rhs->layerVoxelCount());
  EXPECT_EQ(lhs->voxelNum(), rhs->voxelNum());
  expectEquivalentDouble(lhs->stamp_s(), rhs->stamp_s());
  ASSERT_EQ(lhs->params().horizons_s.size(), rhs->params().horizons_s.size());
  for (std::size_t index = 0; index < lhs->params().horizons_s.size(); ++index) {
    expectEquivalentDouble(lhs->params().horizons_s[index],
                           rhs->params().horizons_s[index]);
  }
  const Eigen::Vector3i dims = lhs->voxelNum();
  for (int horizon = 0; horizon < lhs->horizonCount(); ++horizon) {
    for (int x = 0; x < dims.x(); ++x) {
      for (int y = 0; y < dims.y(); ++y) {
        for (int z = 0; z < dims.z(); ++z) {
          iap::RiskVoxel lhs_voxel;
          iap::RiskVoxel rhs_voxel;
          const Eigen::Vector3i index(x, y, z);
          ASSERT_TRUE(lhs->voxelAt(horizon, index, &lhs_voxel));
          ASSERT_TRUE(rhs->voxelAt(horizon, index, &rhs_voxel));
          EXPECT_EQ(lhs_voxel.valid, rhs_voxel.valid);
          EXPECT_EQ(lhs_voxel.stale, rhs_voxel.stale);
          EXPECT_EQ(lhs_voxel.unknown, rhs_voxel.unknown);
          EXPECT_EQ(lhs_voxel.source_flags, rhs_voxel.source_flags);
          EXPECT_EQ(lhs_voxel.reason, rhs_voxel.reason);
          expectEquivalentDouble(lhs_voxel.c_pi, rhs_voxel.c_pi);
          expectEquivalentDouble(lhs_voxel.hpl_pred, rhs_voxel.hpl_pred);
          expectEquivalentDouble(lhs_voxel.vpl_pred, rhs_voxel.vpl_pred);
          expectEquivalentDouble(lhs_voxel.stamp_s, rhs_voxel.stamp_s);
          ASSERT_EQ(lhs_voxel.occupancy == nullptr,
                    rhs_voxel.occupancy == nullptr);
          if (lhs_voxel.occupancy && rhs_voxel.occupancy) {
            EXPECT_EQ(lhs_voxel.occupancy->available,
                      rhs_voxel.occupancy->available);
            EXPECT_EQ(lhs_voxel.occupancy->raw_occupied,
                      rhs_voxel.occupancy->raw_occupied);
            EXPECT_EQ(lhs_voxel.occupancy->inflated_occupied,
                      rhs_voxel.occupancy->inflated_occupied);
            EXPECT_EQ(lhs_voxel.occupancy->occupancy_generation,
                      rhs_voxel.occupancy->occupancy_generation);
            EXPECT_EQ(lhs_voxel.occupancy->frame_id,
                      rhs_voxel.occupancy->frame_id);
            EXPECT_EQ(lhs_voxel.occupancy->source,
                      rhs_voxel.occupancy->source);
            expectEquivalentDouble(lhs_voxel.occupancy->cloud_stamp_s,
                                   rhs_voxel.occupancy->cloud_stamp_s);
          }
        }
      }
    }
  }
}

void expectSameActiveGeneration(
    const std::shared_ptr<const iap::RiskGridSnapshot>& accepted,
    const std::shared_ptr<const iap::RiskGridSnapshot>& retained) {
  ASSERT_NE(accepted, nullptr);
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(&accepted->params(), &retained->params());
  expectSnapshotsScientificallyEquivalent(accepted, retained);
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

  static P0RiskGridRuntime::InputReadiness inputReadiness(
      const P0RiskGridRuntime& runtime, const double now_s) {
    return runtime.inputReadiness(now_s);
  }

  static std::string snapshotFailureReason(
      const P0RiskGridRuntime& runtime, const double now_s) {
    return runtime.snapshotFailureReason(now_s);
  }

  static void setOdomStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_odom_stamp_ = stamp;
  }

  static void setCurrentStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_current_.stamp = stamp;
  }

  static void setOdomSeen(P0RiskGridRuntime* runtime, const bool seen) {
    runtime->odom_seen_ = seen;
  }

  static void setOdomPoseValid(P0RiskGridRuntime* runtime, const bool valid) {
    runtime->latest_odom_pose_valid_ = valid;
  }

  static void setOdomStampExact(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_odom_stamp_ = stamp;
  }

  static void setCurrentSeen(P0RiskGridRuntime* runtime, const bool seen) {
    runtime->current_integrity_seen_ = seen;
  }

  static void setCurrentValid(P0RiskGridRuntime* runtime, const bool valid) {
    runtime->latest_current_valid_ = valid;
  }

  static void setOriginSeen(P0RiskGridRuntime* runtime, const bool seen) {
    runtime->origin_seen_ = seen;
  }

  static void setOriginValid(P0RiskGridRuntime* runtime, const bool valid) {
    runtime->origin_set_ = valid;
  }

  static void setOriginStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_origin_stamp_ = stamp;
  }

  static void setGnssEpochSeen(P0RiskGridRuntime* runtime, const bool seen) {
    runtime->gnss_epoch_seen_ = seen;
  }

  static void setGnssEpochStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_gnss_epoch_stamp_ = stamp;
  }

  static void setGnssEpochSatelliteCount(
      P0RiskGridRuntime* runtime, const uint64_t count) {
    runtime->latest_gnss_epoch_satellite_count_ = count;
  }

  static void setLatestGnssEpoch(
      P0RiskGridRuntime* runtime, const iap::GnssEpoch& epoch) {
    runtime->latest_epoch_ = epoch;
  }

  static void seedGnssEpoch(P0RiskGridRuntime* runtime,
                            const double stamp_s,
                            const int satellite_count = 8) {
    runtime->latest_epoch_ = makeGnssEpoch(satellite_count, stamp_s);
    runtime->gnss_epoch_seen_ = true;
    runtime->latest_gnss_epoch_stamp_ = stamp_s;
    runtime->latest_gnss_epoch_satellite_count_ =
        static_cast<uint64_t>(satellite_count);
  }

  static std::shared_ptr<std::atomic<uint64_t>> installOccupancyEpoch(
      P0RiskGridRuntime* runtime,
      const double stamp_s,
      const std::vector<Eigen::Vector3d>& occupied_centers = {},
      const std::string& frame_id = "map",
      const uint64_t generation = 2u) {
    auto live_generation =
        std::make_shared<std::atomic<uint64_t>>(generation);
    runtime->setOccupancyEpochFactory(
        [live_generation, generation, stamp_s, frame_id,
         occupied_centers]() {
          return makeOccupancyEpochCapture(live_generation, generation,
                                           stamp_s, frame_id,
                                           occupied_centers);
        });
    return live_generation;
  }

  static bool gnssEpochSeen(const P0RiskGridRuntime& runtime) {
    return runtime.gnss_epoch_seen_;
  }

  static uint64_t gnssEpochSatelliteCount(const P0RiskGridRuntime& runtime) {
    return runtime.latest_gnss_epoch_satellite_count_;
  }

  static void setMapSeen(P0RiskGridRuntime* runtime, const bool seen) {
    runtime->map_seen_ = seen;
  }

  static void setMapStamp(P0RiskGridRuntime* runtime, const double stamp) {
    runtime->latest_map_stamp_ = stamp;
  }

  static void setMapPointCount(P0RiskGridRuntime* runtime, const std::size_t count) {
    runtime->latest_lidar_map_point_count_ = count;
  }

  static void seedValidInputs(P0RiskGridRuntime* runtime,
                              const double odom_stamp,
                              const double current_stamp) {
    runtime->latest_odom_stamp_ = odom_stamp;
    runtime->latest_odom_p_ = Eigen::Vector3d::Zero();
    runtime->latest_odom_q_ = Eigen::Quaterniond::Identity();
    runtime->latest_odom_pose_valid_ = true;
    runtime->odom_seen_ = true;
    runtime->latest_current_.stamp = current_stamp;
    runtime->latest_current_.valid = true;
    runtime->latest_current_.hpl = 1.0;
    runtime->latest_current_.vpl = 1.0;
    runtime->latest_current_.hal = 10.0;
    runtime->latest_current_.val = 10.0;
    runtime->latest_current_.im = 9.0;
    runtime->latest_current_valid_ = true;
    runtime->current_integrity_seen_ = true;
    if (runtime->latest_current_generation_ == 0u) {
      runtime->latest_current_generation_ = 1u;
    }
  }

  static void advancePriorGeneration(P0RiskGridRuntime* runtime) {
    std::lock_guard<std::mutex> lock(runtime->health_state_mutex_);
    ++runtime->latest_current_generation_;
    if (runtime->latest_current_generation_ == 0u) {
      ++runtime->latest_current_generation_;
    }
  }

  static void setPriorGeneration(P0RiskGridRuntime* runtime,
                                 const uint64_t generation) {
    runtime->latest_current_generation_ = generation;
  }

  static void setCurrentProtectionLevels(P0RiskGridRuntime* runtime,
                                         const double hpl,
                                         const double vpl) {
    runtime->latest_current_.hpl = hpl;
    runtime->latest_current_.vpl = vpl;
  }

  static void setGrowthPriorEnabled(P0RiskGridRuntime* runtime,
                                    const bool enabled) {
    runtime->config_.predictor_use_current_integrity_prior = enabled;
  }

  static void setGrowthSigma(P0RiskGridRuntime* runtime,
                             const double sigma) {
    runtime->config_.predictor_sigma_grow_m_sqrt_s = sigma;
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

  static void sendRange(
      P0RiskGridRuntime* runtime,
      const gnss_comm::msg::GnssMeasMsg::SharedPtr& msg) {
    runtime->rangeCallback(msg);
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
  EXPECT_TRUE(std::isnan(config.predictor_sigma_grow_m_sqrt_s));
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
      rclcpp::Parameter("p0.predictor.sigma_grow_m_sqrt_s", 0.08),
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
  EXPECT_DOUBLE_EQ(config.predictor_sigma_grow_m_sqrt_s, 0.08);
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

TEST(SafetyRvizPublisherTest, AcceptedSnapshotCloudBypassesPeriodicThrottle) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "accepted_snapshot_cloud_force_publish_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  ego_planner::SafetyRvizPublisher::Config config;
  config.publish_rate_hz = 1.0;
  ego_planner::SafetyRvizPublisher publisher(node, config);

  std::atomic<int> received{0};
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.predicted_pl_cloud_topic, rclcpp::QoS(10).best_effort(),
      [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
        received.fetch_add(1);
      });
  (void)subscription;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  publisher.publishPredictedPLCloud(nullptr, 1.0, 100.0);
  executor.spin_some();
  publisher.publishPredictedPLCloud(nullptr, 1.0, 100.1);
  executor.spin_some();
  EXPECT_EQ(received.load(), 1);

  publisher.publishPredictedPLCloud(nullptr, 1.0, 100.1, true);
  executor.spin_some();
  EXPECT_EQ(received.load(), 2);
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

TEST_F(P0RiskGridRuntimeStampTest, SnapshotFailureReasonDistinguishesSourceStates) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_snapshot_failure_reason_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  EXPECT_EQ(snapshotFailureReason(runtime, 10.0), "odom_missing");

  setOdomSeen(&runtime, true);
  setOdomPoseValid(&runtime, false);
  EXPECT_EQ(snapshotFailureReason(runtime, 10.0), "odom_invalid");

  setOdomPoseValid(&runtime, true);
  setOdomStampExact(&runtime, 9.5);
  EXPECT_EQ(snapshotFailureReason(runtime, 10.0),
            "current_integrity_missing");

  setCurrentSeen(&runtime, true);
  setCurrentValid(&runtime, false);
  setCurrentStamp(&runtime, 9.5);
  EXPECT_EQ(snapshotFailureReason(runtime, 10.0),
            "current_integrity_invalid");
}

TEST_F(P0RiskGridRuntimeStampTest, InputReadinessReportsSeenValidAndFreshSources) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_input_readiness_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 1.0;
  P0RiskGridRuntime runtime(node, config, std::make_unique<FakeProvider>());

  seedValidInputs(&runtime, 100.0, 100.0);
  setOdomSeen(&runtime, true);
  setCurrentSeen(&runtime, true);
  setOriginSeen(&runtime, true);
  setOriginValid(&runtime, true);
  setOriginStamp(&runtime, 99.0);
  setMapSeen(&runtime, true);
  setMapStamp(&runtime, 99.5);
  setMapPointCount(&runtime, 10);

  const auto readiness = inputReadiness(runtime, 100.0);
  EXPECT_TRUE(readiness.odom_seen);
  EXPECT_TRUE(readiness.odom_valid);
  EXPECT_TRUE(readiness.odom_fresh);
  EXPECT_TRUE(readiness.current_integrity_seen);
  EXPECT_TRUE(readiness.current_integrity_valid);
  EXPECT_TRUE(readiness.current_integrity_fresh);
  EXPECT_TRUE(readiness.origin_seen);
  EXPECT_TRUE(readiness.origin_valid);
  EXPECT_TRUE(readiness.origin_fresh);
  EXPECT_DOUBLE_EQ(readiness.origin_stamp_s, 99.0);
  EXPECT_TRUE(readiness.map_seen);
  EXPECT_TRUE(readiness.map_valid);
  EXPECT_TRUE(readiness.map_fresh);
  EXPECT_EQ(readiness.map_point_count, 10u);
  EXPECT_EQ(snapshotFailureReason(runtime, 100.0), "none");
}

TEST_F(P0RiskGridRuntimeStampTest, StaleOdomOrCurrentPreventsSnapshot) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stale_snapshot_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 1.0;
  P0RiskGridRuntime runtime(node, config, std::make_unique<FakeProvider>());

  seedValidInputs(&runtime, 98.0, 98.0);
  setOdomSeen(&runtime, true);
  setCurrentSeen(&runtime, true);
  iap::IntegritySnapshot snapshot;
  EXPECT_FALSE(buildSnapshot(&runtime, 100.0, &snapshot));

  seedValidInputs(&runtime, 100.0, 100.0);
  setOdomSeen(&runtime, true);
  setCurrentSeen(&runtime, true);
  EXPECT_TRUE(buildSnapshot(&runtime, 100.0, &snapshot));
}

TEST_F(P0RiskGridRuntimeStampTest, GnssRangeCallbackCompletesWithoutRecursiveLock) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_range_callback_no_recursive_lock_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  auto msg = std::make_shared<gnss_comm::msg::GnssMeasMsg>();
  sendRange(&runtime, msg);
  EXPECT_TRUE(gnssEpochSeen(runtime));
  EXPECT_EQ(gnssEpochSatelliteCount(runtime), 0u);
}

TEST_F(P0RiskGridRuntimeStampTest, GnssOriginMapValidityMatrixIsTruthful) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_source_validity_matrix_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 1.0;
  P0RiskGridRuntime runtime(node, config, std::make_unique<FakeProvider>());

  setGnssEpochSeen(&runtime, true);
  setGnssEpochStamp(&runtime, 99.0);
  setGnssEpochSatelliteCount(&runtime, 0);
  setOriginSeen(&runtime, true);
  setOriginValid(&runtime, false);
  setOriginStamp(&runtime, 99.0);
  setMapSeen(&runtime, true);
  setMapStamp(&runtime, 99.0);
  setMapPointCount(&runtime, 0);

  const auto readiness = inputReadiness(runtime, 100.0);
  EXPECT_TRUE(readiness.gnss_epoch_seen);
  EXPECT_FALSE(readiness.gnss_epoch_valid);
  EXPECT_EQ(readiness.gnss_epoch_satellite_count, 0u);
  EXPECT_TRUE(readiness.origin_seen);
  EXPECT_FALSE(readiness.origin_valid);
  EXPECT_TRUE(readiness.map_seen);
  EXPECT_FALSE(readiness.map_valid);
}

TEST_F(P0RiskGridRuntimeStampTest, RefreshSnapshotUsesMessageStamp) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_refresh_uses_message_stamp_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());

  seedValidInputs(&runtime, 123.5, 123.5);

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
  installOccupancyEpoch(&runtime, 123.5);
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
  config.predictor_use_current_integrity_prior = true;
  config.predictor_lidar_legacy_observability = false;
  P0RiskGridRuntime runtime(node, config);
  const auto points = sixAxisPrimitivePoints();
  const auto normals = sixAxisPrimitiveNormals();

  sendCloud(&runtime, makePointCloud(points, &normals));
  seedValidInputs(&runtime, 123.5, 123.5);
  installOccupancyEpoch(&runtime, 123.5);

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
  installOccupancyEpoch(&runtime, 123.5);

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
       StaleCurrentPriorWithLidarGoodPreventsSnapshot) {
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
  installOccupancyEpoch(&runtime, 100.0);

  EXPECT_FALSE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_FALSE(health.ready);
  EXPECT_EQ(health.reason, "stale_covariance_growth_prior");
}

TEST_F(P0RiskGridRuntimeStampTest,
       StaleCurrentWithNoUsablePredictorSourcePreventsSnapshot) {
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
  installOccupancyEpoch(&runtime, 100.0);

  EXPECT_FALSE(runtime.refreshOnceForTest());
  const auto health = runtime.health();
  EXPECT_FALSE(health.ready);
  EXPECT_EQ(health.reason, "stale_covariance_growth_prior");
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
  installOccupancyEpoch(&runtime, 123.5);
  EXPECT_TRUE(runtime.refreshOnceForTest());
  const auto health = runtime.health();

  EXPECT_TRUE(health.ready);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_GT(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_NE(health.reason, "stale_gnss_epoch");
}

TEST_F(P0RiskGridRuntimeStampTest,
       ProductionProviderBindsVersionedImmutableGnssOccupancyEpoch) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_production_immutable_occupancy_epoch_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = false;
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);

  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  installOccupancyEpoch(&runtime, 100.0, {}, "map", 2u);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto open_snapshot = runtime.acquireSnapshot();
  ASSERT_NE(open_snapshot, nullptr);
  const auto open_health = runtime.health();
  EXPECT_GT(open_health.predictor_gnss_used_count, 0u);

  advancePriorGeneration(&runtime);
  seedValidInputs(&runtime, 100.5, 100.5);
  seedGnssEpoch(&runtime, 100.5);
  const auto blocked_live = installOccupancyEpoch(
      &runtime, 100.5, gnssLosBlockers(makeGnssEpoch(8, 100.5)),
      "map", 4u);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto blocked_snapshot = runtime.acquireSnapshot();
  ASSERT_NE(blocked_snapshot, nullptr);
  EXPECT_GT(blocked_snapshot->generation_id(), open_snapshot->generation_id());
  EXPECT_EQ(runtime.health().predictor_gnss_used_count,
            open_health.predictor_gnss_used_count);

  iap::RiskVoxel blocked_voxel;
  ASSERT_TRUE(blocked_snapshot->voxelAt(
      0, Eigen::Vector3i(1, 1, 1), &blocked_voxel));
  ASSERT_NE(blocked_voxel.occupancy, nullptr);
  EXPECT_TRUE(blocked_voxel.occupancy->raw_occupied);
  EXPECT_EQ(blocked_voxel.occupancy->occupancy_generation, 4u);
  EXPECT_DOUBLE_EQ(blocked_voxel.occupancy->cloud_stamp_s, 100.5);
  EXPECT_EQ(blocked_voxel.occupancy->frame_id, "map");
  iap::RiskVoxel open_voxel;
  ASSERT_TRUE(open_snapshot->voxelAt(
      0, Eigen::Vector3i(1, 1, 1), &open_voxel));
  EXPECT_GT(blocked_voxel.hpl_pred, open_voxel.hpl_pred);
  EXPECT_GT(blocked_voxel.vpl_pred, open_voxel.vpl_pred);

  blocked_live->store(6u);
  iap::RiskVoxel still_frozen;
  ASSERT_TRUE(blocked_snapshot->voxelAt(
      0, Eigen::Vector3i(1, 1, 1), &still_frozen));
  ASSERT_NE(still_frozen.occupancy, nullptr);
  EXPECT_EQ(still_frozen.occupancy->occupancy_generation, 4u);
  EXPECT_EQ(still_frozen.occupancy->raw_occupied,
            blocked_voxel.occupancy->raw_occupied);
}

TEST_F(P0RiskGridRuntimeStampTest,
       OccupancyGenerationChangeDuringProviderBatchKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_occupancy_generation_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  std::atomic<bool> mutate{false};
  auto provider = std::make_unique<CallbackProvider>([&]() {
    if (mutate.load()) {
      live_generation->store(4u);
    }
  });
  P0RiskGridRuntime runtime(node, enabledConfig(), std::move(provider));
  runtime.setOccupancyEpochFactory([live_generation]() {
    return makeOccupancyEpochCapture(live_generation, 2u, 100.0, "map");
  });
  seedValidInputs(&runtime, 100.0, 100.0);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);
  mutate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_generation_changed");
  const auto retained = runtime.acquireSnapshot();
  EXPECT_EQ(retained->generation_id(), accepted->generation_id());
  expectSameActiveGeneration(accepted, retained);
}

TEST_F(P0RiskGridRuntimeStampTest,
       PriorGenerationChangeDuringProviderBatchKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_prior_generation_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  std::atomic<bool> mutate{false};
  P0RiskGridRuntime* runtime_ptr = nullptr;
  auto provider = std::make_unique<CallbackProvider>([&]() {
    if (mutate.load()) {
      advancePriorGeneration(runtime_ptr);
    }
  });
  P0RiskGridRuntime runtime(node, enabledConfig(), std::move(provider));
  runtime_ptr = &runtime;
  installOccupancyEpoch(&runtime, 100.0);
  seedValidInputs(&runtime, 100.0, 100.0);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);
  mutate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "prior_generation_changed");
  const auto retained = runtime.acquireSnapshot();
  EXPECT_EQ(retained->generation_id(), accepted->generation_id());
  expectSameActiveGeneration(accepted, retained);
}

TEST_F(P0RiskGridRuntimeStampTest,
       MissingStaleOrWrongFrameOccupancyEpochKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_invalid_occupancy_epoch_retention_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 0.5;
  P0RiskGridRuntime runtime(node, config, std::make_unique<FakeProvider>());
  seedValidInputs(&runtime, 100.0, 100.0);
  installOccupancyEpoch(&runtime, 100.0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);

  runtime.setOccupancyEpochFactory([]() {
    return P0OccupancyEpochCapture{
        P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE, std::nullopt};
  });
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_snapshot_unavailable");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  installOccupancyEpoch(&runtime, 99.0);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_stale");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  installOccupancyEpoch(&runtime, 100.0, {}, "odom");
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_frame_mismatch");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
}

TEST_F(P0RiskGridRuntimeStampTest,
       MissingStaleOrInvalidGrowthPriorKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_invalid_growth_prior_retention_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 0.5;
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  installOccupancyEpoch(&runtime, 100.0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);

  setPriorGeneration(&runtime, 0u);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "missing_covariance_growth_prior");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  setPriorGeneration(&runtime, 1u);
  seedValidInputs(&runtime, 100.0, 99.0);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "stale_covariance_growth_prior");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  seedValidInputs(&runtime, 100.0, 100.0);
  setCurrentProtectionLevels(&runtime, 0.0, 1.0);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "invalid_covariance_growth_prior");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
}

TEST_F(P0RiskGridRuntimeStampTest,
       MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent) {
  ensure_rclcpp();
  const std::vector<int> worker_counts = {1, 2, 4};
  std::vector<std::shared_ptr<const iap::RiskGridSnapshot>> snapshots;
  std::vector<iap::RiskGridHealth> health_states;
  for (const int worker_count : worker_counts) {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_worker_equivalence_" + std::to_string(worker_count),
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    auto config = enabledConfig();
    config.grid.skip_occupied_voxels = false;
    config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
    config.predictor_gnss_epoch_policy =
        iap::PredictorGnssEpochPolicy::Required;
    config.predictor_requested_worker_count = worker_count;
    config.predictor_effective_worker_count = worker_count;
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0,
                          {Eigen::Vector3d(0.0, 0.0, 0.0)});

    ASSERT_TRUE(refreshOnce(&runtime));
    snapshots.push_back(runtime.acquireSnapshot());
    health_states.push_back(runtime.health());
  }

  ASSERT_EQ(snapshots.size(), 3u);
  expectSnapshotsScientificallyEquivalent(snapshots[0], snapshots[1]);
  expectSnapshotsScientificallyEquivalent(snapshots[0], snapshots[2]);
  for (std::size_t index = 1; index < health_states.size(); ++index) {
    EXPECT_EQ(health_states[index].provider_query_count,
              health_states[0].provider_query_count);
    EXPECT_EQ(health_states[index].predictor_gnss_used_count,
              health_states[0].predictor_gnss_used_count);
    EXPECT_EQ(health_states[index].predictor_lidar_used_count,
              health_states[0].predictor_lidar_used_count);
    EXPECT_EQ(health_states[index].predictor_prior_used_count,
              health_states[0].predictor_prior_used_count);
  }
}

}  // namespace ego_planner
