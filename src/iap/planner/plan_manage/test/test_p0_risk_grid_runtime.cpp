#include <ego_planner/p0_risk_grid_runtime.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
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

struct RuntimeOccupancyDiagnostic {
  bool available = false;
  bool raw_occupied = false;
  bool inflated_occupied = false;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Zero();
  double resolution_m = 1.0;
  double inflation_m = 0.0;
  std::string frame_id;
  double cloud_stamp_s = 0.0;
  uint64_t generation = 0;
  std::string source;
};

struct RuntimeFrozenOccupancyEpoch {
  std::function<RuntimeOccupancyDiagnostic(const Eigen::Vector3d&)>
      diagnostic_query;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      raw_occupied_voxel_centers;
  Eigen::Vector3d lattice_origin = Eigen::Vector3d::Zero();
  double resolution_m = 1.0;
  std::string frame_id;
  double cloud_stamp_s = 0.0;
  uint64_t generation = 0;
};

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
    const std::vector<Eigen::Vector3d>& occupied_centers = {},
    ego_planner::P0OccupancyEpoch::SourceOwner source_owner = {},
    ego_planner::P0OccupancyEpoch::LiveSourceOwner live_source_owner = {},
    const double resolution_m = 1.0,
    const Eigen::Vector3d& lattice_origin =
        Eigen::Vector3d(-1.5, -1.5, -1.5)) {
  iap::LocalOccupancyGrid::Params params;
  params.voxel_size = resolution_m;
  params.lattice_origin = lattice_origin;
  std::vector<Eigen::Vector3d> normalized_centers;
  normalized_centers.reserve(occupied_centers.size());
  std::set<std::tuple<int, int, int>> normalized_keys;
  for (const auto& point : occupied_centers) {
    const Eigen::Vector3d scaled =
        (point - params.lattice_origin) / params.voxel_size;
    const Eigen::Vector3i key = scaled.array().floor().cast<int>();
    if (normalized_keys.emplace(key.x(), key.y(), key.z()).second) {
      normalized_centers.push_back(
          params.lattice_origin +
          (key.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
              params.voxel_size);
    }
  }
  params.max_voxels = static_cast<int>(
      std::max<std::size_t>(1u, normalized_centers.size()));
  params.enable_eviction = false;
  auto owner = std::make_shared<iap::LocalOccupancyGrid>(params);
  owner->insert_points(normalized_centers);

  if (!source_owner) {
    source_owner = live_generation;
  }
  if (!live_source_owner) {
    live_source_owner = [source_owner]() { return source_owner; };
  }
  RuntimeFrozenOccupancyEpoch frozen;
  frozen.raw_occupied_voxel_centers =
      std::make_shared<const std::vector<Eigen::Vector3d>>(
          std::move(normalized_centers));
  frozen.lattice_origin = params.lattice_origin;
  frozen.resolution_m = params.voxel_size;
  frozen.generation = captured_generation;
  frozen.cloud_stamp_s = stamp_s;
  frozen.frame_id = frame_id;
  frozen.diagnostic_query =
      [owner, captured_generation, stamp_s, frame_id](
          const Eigen::Vector3d& position) {
        RuntimeOccupancyDiagnostic diagnostic;
        diagnostic.available = true;
        diagnostic.raw_occupied = owner->occupied_at(position);
        diagnostic.inflated_occupied = diagnostic.raw_occupied;
        diagnostic.voxel_center = position;
        diagnostic.resolution_m = owner->params().voxel_size;
        diagnostic.inflation_m = 0.0;
        diagnostic.frame_id = frame_id;
        diagnostic.cloud_stamp_s = stamp_s;
        diagnostic.generation = captured_generation;
        diagnostic.source = "frozen_test_epoch";
        return diagnostic;
      };
  auto adapted = ego_planner::P0OccupancyEpochAdapter::adapt(
      frozen, source_owner, std::move(live_source_owner),
      [live_generation]() { return live_generation->load(); });
  return {adapted ? ego_planner::P0OccupancyEpochCaptureStatus::VALID
                  : ego_planner::P0OccupancyEpochCaptureStatus::ADAPTER_INVALID,
          std::move(adapted)};
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

using ProfileClock = std::chrono::steady_clock;

constexpr int kProfileGridX = 40;
constexpr int kProfileGridY = 40;
constexpr int kProfileGridZ = 8;
constexpr double kProfileResolutionM = 0.75;
constexpr std::array<double, 6> kProfileHorizons{
    {0.0, 0.5, 1.0, 1.5, 2.0, 2.5}};
constexpr std::size_t kProfileLogicalPositions = 12800u;
constexpr std::size_t kProfileLogicalQueries = 76800u;
constexpr std::size_t kProfileWarmups = 2u;
constexpr std::size_t kProfileMeasuredSamples = 10u;
constexpr std::uint64_t kProfileFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kProfileFnvPrime = 1099511628211ULL;
constexpr char kProfileFilter[] =
    "P0RiskGridRuntimeStampTest."
    "DISABLED_ICRA020_ProductionRuntimeWorkerScalingProfile";
constexpr char kProfileTestBinaryPath[] =
    "results/icra27/icra020/build_ego/test_p0_risk_grid_runtime";
constexpr char kProfileLibiapPath[] =
    "results/icra27/icra020/install/lib/libiap.so";
constexpr char kProfileOutputPath[] =
    "results/icra27/icra020/p0_rolling_worker_profile.json";

enum class ProfileScenario {
  ColdFullRebuild,
  StationaryEmptyDelta,
  ShiftPlusOneXEmptyDelta,
  StationaryNonemptyDelta,
};

const char* profileScenarioName(const ProfileScenario scenario) {
  switch (scenario) {
    case ProfileScenario::ColdFullRebuild:
      return "cold_full_rebuild";
    case ProfileScenario::StationaryEmptyDelta:
      return "stationary_empty_delta";
    case ProfileScenario::ShiftPlusOneXEmptyDelta:
      return "shift_plus_one_x_empty_delta";
    case ProfileScenario::StationaryNonemptyDelta:
      return "stationary_nonempty_delta";
  }
  return "invalid";
}

struct ProfileSample {
  std::size_t sample_index = 0;
  bool refresh_succeeded = false;
  bool scientifically_equal_to_fresh = false;
  double wall_ms = std::numeric_limits<double>::quiet_NaN();
  double refresh_elapsed_ms = std::numeric_limits<double>::quiet_NaN();
  double provider_batch_duration_ms =
      std::numeric_limits<double>::quiet_NaN();
  std::size_t logical_query_count = 0;
  std::size_t provider_query_count = 0;
  std::size_t spatial_recompute_count = 0;
  std::size_t spatial_reuse_count = 0;
  std::size_t retained_position_count = 0;
  std::size_t entered_position_count = 0;
  std::size_t evicted_position_count = 0;
  std::size_t gnss_advisory_invocation_count = 0;
  std::size_t lidar_advisory_invocation_count = 0;
  std::size_t horizon_fusion_count = 0;
  bool full_rebuild = false;
  std::size_t full_invalidation_count = 0;
  std::string invalidation_reason;
  std::uint64_t gnss_generation = 0;
  std::uint64_t lidar_generation = 0;
  std::uint64_t prior_generation = 0;
  std::uint64_t occupancy_generation = 0;
  std::uint64_t occupancy_content_identity = 0;
  std::uint64_t snapshot_occupancy_generation = 0;
  double snapshot_occupancy_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  std::string snapshot_scientific_hash;
  std::string fresh_scientific_hash;
};

struct ProfileTimingSummary {
  double p50_ms = std::numeric_limits<double>::quiet_NaN();
  double p95_ms = std::numeric_limits<double>::quiet_NaN();
  double max_ms = std::numeric_limits<double>::quiet_NaN();
};

struct ProfileScenarioRow {
  ProfileScenario scenario = ProfileScenario::ColdFullRebuild;
  std::vector<ProfileSample> samples;
  ProfileTimingSummary wall;
  ProfileTimingSummary refresh;
  ProfileTimingSummary provider;
  double wall_speedup = std::numeric_limits<double>::quiet_NaN();
  double refresh_speedup = std::numeric_limits<double>::quiet_NaN();
  double provider_speedup = std::numeric_limits<double>::quiet_NaN();
};

struct ProfileWorkerRow {
  int requested_worker_count = 0;
  int effective_worker_count = 0;
  std::vector<ProfileScenarioRow> scenarios;
};

void profileHashBytes(std::uint64_t* hash, const void* data,
                      const std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= kProfileFnvPrime;
  }
}

template <typename T>
void profileHashScalar(std::uint64_t* hash, const T& value) {
  profileHashBytes(hash, &value, sizeof(value));
}

void profileHashBool(std::uint64_t* hash, const bool value) {
  const std::uint8_t normalized = value ? 1u : 0u;
  profileHashScalar(hash, normalized);
}

void profileHashString(std::uint64_t* hash, const std::string& value) {
  profileHashScalar(hash, value.size());
  profileHashBytes(hash, value.data(), value.size());
}

void profileHashVector(std::uint64_t* hash, const Eigen::Vector3d& value) {
  for (int axis = 0; axis < 3; ++axis) profileHashScalar(hash, value(axis));
}

void profileHashVector(std::uint64_t* hash, const Eigen::Vector3i& value) {
  for (int axis = 0; axis < 3; ++axis) profileHashScalar(hash, value(axis));
}

std::string profileHexHash(const std::uint64_t value) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << value;
  return output.str();
}

std::string profileSnapshotScientificHash(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot) {
  if (!snapshot) return {};
  std::uint64_t hash = kProfileFnvOffset;
  profileHashVector(&hash, snapshot->origin());
  profileHashVector(&hash, snapshot->voxelNum());
  profileHashScalar(&hash, snapshot->params().resolution_m);
  profileHashString(&hash, snapshot->params().frame_id);
  profileHashScalar(&hash, snapshot->params().horizons_s.size());
  for (const double horizon : snapshot->params().horizons_s) {
    profileHashScalar(&hash, horizon);
  }
  const Eigen::Vector3i dimensions = snapshot->voxelNum();
  for (int horizon = 0; horizon < snapshot->horizonCount(); ++horizon) {
    for (int x = 0; x < dimensions.x(); ++x) {
      for (int y = 0; y < dimensions.y(); ++y) {
        for (int z = 0; z < dimensions.z(); ++z) {
          iap::RiskVoxel voxel;
          if (!snapshot->voxelAt(horizon, Eigen::Vector3i(x, y, z), &voxel)) {
            return {};
          }
          profileHashScalar(&hash, voxel.c_pi);
          profileHashScalar(&hash, voxel.hpl_pred);
          profileHashScalar(&hash, voxel.vpl_pred);
          profileHashBool(&hash, voxel.valid);
          profileHashBool(&hash, voxel.stale);
          profileHashBool(&hash, voxel.unknown);
          profileHashScalar(&hash, voxel.source_flags);
          profileHashString(&hash, voxel.reason);
          profileHashBool(&hash, static_cast<bool>(voxel.occupancy));
          if (voxel.occupancy) {
            profileHashBool(&hash, voxel.occupancy->available);
            profileHashBool(&hash, voxel.occupancy->raw_occupied);
            profileHashBool(&hash, voxel.occupancy->inflated_occupied);
            profileHashVector(&hash, voxel.occupancy->voxel_index);
            profileHashVector(&hash, voxel.occupancy->voxel_center);
            profileHashScalar(&hash, voxel.occupancy->resolution_m);
            profileHashScalar(&hash, voxel.occupancy->inflation_m);
            profileHashString(&hash, voxel.occupancy->frame_id);
            profileHashString(&hash, voxel.occupancy->source);
          }
        }
      }
    }
  }
  return profileHexHash(hash);
}

std::vector<Eigen::Vector3d> profileOccupancyCenters() {
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(704u);
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 11; ++y) {
      for (int z = 0; z < 4; ++z) {
        const Eigen::Vector3i key(-19 + 2 * x, -19 + 3 * y, -3 + 2 * z);
        centers.push_back((key.cast<double>() +
                           Eigen::Vector3d::Constant(0.5)) *
                          kProfileResolutionM);
      }
    }
  }
  return centers;
}

std::vector<Eigen::Vector3d> profileChangedOccupancyCenters() {
  auto centers = profileOccupancyCenters();
  centers.back() = Eigen::Vector3d(15.375, 14.625, 2.625);
  return centers;
}

std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
profileLidarPrimitives() {
  auto primitives = std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  primitives->reserve(704u);
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 11; ++y) {
      for (int z = 0; z < 4; ++z) {
        iap::LidarFimPrimitive primitive;
        primitive.center_w = Eigen::Vector3d(-14.0 + 1.8 * x,
                                             -14.0 + 2.7 * y,
                                             -2.0 + 1.3 * z);
        const int axis = (x + y + z) % 3;
        primitive.normal_w = axis == 0 ? Eigen::Vector3d::UnitX()
                             : axis == 1 ? Eigen::Vector3d::UnitY()
                                         : Eigen::Vector3d::UnitZ();
        primitives->push_back(primitive);
      }
    }
  }
  return primitives;
}

std::shared_ptr<const std::vector<Eigen::Vector3d>> profileLidarMapPoints() {
  auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
  points->reserve(23309u);
  for (std::size_t index = 0; index < 23309u; ++index) {
    const int x = static_cast<int>(index % 47u);
    const int y = static_cast<int>((index / 47u) % 47u);
    const int z = static_cast<int>((index / (47u * 47u)) % 11u);
    points->emplace_back(-14.5 + 0.63 * x, -14.5 + 0.63 * y,
                         -2.5 + 0.55 * z);
  }
  return points;
}

iap::GnssEpoch profileGnssEpoch(const double stamp_s) {
  constexpr double kPi = 3.14159265358979323846;
  iap::GnssEpoch epoch;
  epoch.stamp = stamp_s;
  epoch.gps_sec = 2100000.0;
  for (int index = 0; index < 31; ++index) {
    iap::SatObs satellite;
    satellite.sat_id = 100 + index;
    satellite.constellation = 'G';
    satellite.azimuth = 2.0 * kPi * static_cast<double>(index) / 31.0;
    satellite.elevation = 0.25 + 0.06 * static_cast<double>(index % 10);
    satellite.pr_sigma = 3.0 + 0.25 * static_cast<double>(index % 4);
    epoch.sats.push_back(satellite);
  }
  return epoch;
}

double profilePercentileR7(std::vector<double> values,
                           const double probability) {
  std::sort(values.begin(), values.end());
  const double position =
      static_cast<double>(values.size() - 1u) * probability;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const double fraction = position - static_cast<double>(lower);
  if (lower + 1u >= values.size()) return values[lower];
  return values[lower] + fraction * (values[lower + 1u] - values[lower]);
}

ProfileTimingSummary profileTimingSummary(std::vector<double> values) {
  return {profilePercentileR7(values, 0.50),
          profilePercentileR7(values, 0.95),
          *std::max_element(values.begin(), values.end())};
}

std::string profileJsonString(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20u) {
          output << "\\u00" << std::hex << std::setw(2)
                 << std::setfill('0') << static_cast<int>(character)
                 << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

std::string profileRequiredEnvironment(const char* name) {
  const char* value = std::getenv(name);
  if (!value || !*value) {
    throw std::runtime_error(std::string("missing environment variable: ") +
                             name);
  }
  return value;
}

std::string profileCommandOutput(const std::string& command) {
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    throw std::runtime_error("unable to start provenance command");
  }
  std::string output;
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) output += buffer;
  const int status = pclose(pipe);
  if (status != 0) {
    throw std::runtime_error("provenance command failed: " + command);
  }
  while (!output.empty() &&
         (output.back() == '\n' || output.back() == '\r' ||
          output.back() == ' ' || output.back() == '\t')) {
    output.pop_back();
  }
  return output;
}

bool profileIsLowerHex(const std::string& value, const std::size_t size) {
  return value.size() == size &&
      std::all_of(value.begin(), value.end(), [](const char character) {
        return (character >= '0' && character <= '9') ||
               (character >= 'a' && character <= 'f');
      });
}

std::string profileFileSha256(const std::string& repository_relative_path) {
  const std::string output = profileCommandOutput(
      "/usr/bin/sha256sum -- " + repository_relative_path);
  const auto separator = output.find_first_of(" \t");
  const std::string digest = output.substr(0, separator);
  if (!profileIsLowerHex(digest, 64u)) {
    throw std::runtime_error("invalid sha256sum output for " +
                             repository_relative_path);
  }
  return digest;
}

std::string profileCpuModel() {
  std::ifstream cpuinfo("/proc/cpuinfo");
  std::string line;
  while (std::getline(cpuinfo, line)) {
    constexpr char kPrefix[] = "model name";
    if (line.rfind(kPrefix, 0) == 0) {
      const auto separator = line.find(':');
      if (separator != std::string::npos) {
        const auto first = line.find_first_not_of(" \t", separator + 1u);
        if (first != std::string::npos) return line.substr(first);
      }
    }
  }
  return {};
}

struct ProfileProvenance {
  std::string implementation_sha;
  std::string build_type;
  std::string exact_command;
  std::string test_binary_path;
  std::string test_binary_sha256;
  std::string libiap_path;
  std::string libiap_sha256;
  std::string cpu_model;
  unsigned int logical_core_count = 0;
};

std::string profileCanonicalCommand(const ProfileProvenance& provenance) {
  return std::string("env IAP_ICRA020_PROFILE_OUTPUT=") +
      kProfileOutputPath + " IAP_ICRA020_IMPLEMENTATION_SHA=" +
      provenance.implementation_sha + " IAP_ICRA020_BUILD_TYPE=" +
      provenance.build_type + " IAP_ICRA020_TEST_BINARY_PATH=" +
      provenance.test_binary_path + " IAP_ICRA020_TEST_BINARY_SHA256=" +
      provenance.test_binary_sha256 + " IAP_ICRA020_LIBIAP_PATH=" +
      provenance.libiap_path + " IAP_ICRA020_LIBIAP_SHA256=" +
      provenance.libiap_sha256 + " " + provenance.test_binary_path +
      " --gtest_also_run_disabled_tests --gtest_filter=" + kProfileFilter;
}

void writeProfileTimingSummary(std::ostream& output,
                               const ProfileTimingSummary& summary) {
  output << "{\"percentile_method\":\"R-7 linear interpolation\","
         << "\"p50_ms\":" << summary.p50_ms << ","
         << "\"p95_ms\":" << summary.p95_ms << ","
         << "\"max_ms\":" << summary.max_ms << "}";
}

std::string profileArtifactJson(const std::vector<ProfileWorkerRow>& workers,
                                const ProfileProvenance& provenance) {
  std::ostringstream output;
  output << std::setprecision(17);
  output << "{\n"
         << "  \"schema_version\": \"p0_rolling_stage5_profile_v1\",\n"
         << "  \"diagnostic_execution_status\": \"PASS\",\n"
         << "  \"latency_status\": \"COST_RANKING_DIAGNOSTIC\",\n"
         << "  \"gate0b_qualification_status\": \"NOT_RUN\",\n"
         << "  \"production_worker_selection_status\": \"NOT_SELECTED\",\n"
         << "  \"reverse_ray_decision_status\": \"SUPERVISOR_REVIEW_PENDING\",\n"
         << "  \"gpu_status\": \"NOT_EVALUATED\",\n"
         << "  \"percentile_method\": \"R-7 linear interpolation\",\n"
         << "  \"warmup_samples_per_scenario\": " << kProfileWarmups
         << ",\n"
         << "  \"measured_samples_per_scenario\": "
         << kProfileMeasuredSamples << ",\n"
         << "  \"provenance\": {\n"
         << "    \"implementation_sha\": "
         << profileJsonString(provenance.implementation_sha) << ",\n"
         << "    \"compiler\": " << profileJsonString(__VERSION__)
         << ",\n"
         << "    \"build_type\": "
         << profileJsonString(provenance.build_type) << ",\n"
         << "    \"clock\": \"std::chrono::steady_clock\",\n"
         << "    \"cpu_model\": "
         << profileJsonString(provenance.cpu_model) << ",\n"
         << "    \"logical_core_count\": "
         << provenance.logical_core_count << ",\n"
         << "    \"exact_command\": "
         << profileJsonString(provenance.exact_command) << ",\n"
         << "    \"test_binary\": {\"repository_relative_path\": "
         << profileJsonString(provenance.test_binary_path)
         << ", \"sha256\": "
         << profileJsonString(provenance.test_binary_sha256) << "},\n"
         << "    \"libiap\": {\"repository_relative_path\": "
         << profileJsonString(provenance.libiap_path)
         << ", \"sha256\": "
         << profileJsonString(provenance.libiap_sha256) << "}\n"
         << "  },\n"
         << "  \"workload\": {\n"
         << "    \"profile_path\": \"production P0RiskGridRuntime::refreshOnceForTest\",\n"
         << "    \"grid_shape\": [40, 40, 8],\n"
         << "    \"resolution_m\": 0.75,\n"
         << "    \"fixed_map_lattice_anchor\": [0.0, 0.0, 0.0],\n"
         << "    \"horizons_s\": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5],\n"
         << "    \"logical_position_count\": 12800,\n"
         << "    \"logical_risk_voxel_count\": 76800,\n"
         << "    \"predictor_source_mode\": \"Fusion\",\n"
         << "    \"gnss_epoch_policy\": \"Required\",\n"
         << "    \"satellite_count\": 31,\n"
         << "    \"map_los_occupied_voxel_count\": 704,\n"
         << "    \"lidar_fim_primitive_count\": 704,\n"
         << "    \"lidar_map_point_count\": 23309,\n"
         << "    \"sigma_grow_m_sqrt_s\": 0.15,\n"
         << "    \"skip_occupied_voxels\": false,\n"
         << "    \"requested_effective_worker_pairs\": [[1, 1], [2, 2], [4, 4]],\n"
         << "    \"ttl_watchdog_policy\": \"DISABLED_TEST_ONLY\",\n"
         << "    \"affinity_scheduler_clock_manipulation\": \"NONE\",\n"
         << "    \"fresh_runtime_per_sample\": true,\n"
         << "    \"untimed_base_per_non_cold_sample\": true,\n"
         << "    \"fresh_scientific_validation_outside_measured_interval\": true,\n"
         << "    \"fixture_sources\": {\n"
         << "      \"gnss\": \"ICRA-011 deterministic 31-satellite dimensions and formula\",\n"
         << "      \"occupancy\": \"704 unique centers aligned to the 0.75 m fixed map lattice\",\n"
         << "      \"lidar_fim\": \"ICRA-011 deterministic 704-primitive dimensions and formula\",\n"
         << "      \"lidar_map\": \"ICRA-011 deterministic 23309-point dimensions and formula\"\n"
         << "    }\n"
         << "  },\n"
         << "  \"workers\": [\n";
  for (std::size_t worker_index = 0; worker_index < workers.size();
       ++worker_index) {
    const auto& worker = workers[worker_index];
    output << "    {\n"
           << "      \"requested_worker_count\": "
           << worker.requested_worker_count << ",\n"
           << "      \"effective_worker_count\": "
           << worker.effective_worker_count << ",\n"
           << "      \"scenarios\": [\n";
    for (std::size_t scenario_index = 0;
         scenario_index < worker.scenarios.size(); ++scenario_index) {
      const auto& scenario = worker.scenarios[scenario_index];
      output << "        {\n"
             << "          \"scenario\": "
             << profileJsonString(profileScenarioName(scenario.scenario))
             << ",\n"
             << "          \"samples\": [\n";
      for (std::size_t sample_index = 0;
           sample_index < scenario.samples.size(); ++sample_index) {
        const auto& sample = scenario.samples[sample_index];
        output << "            {"
               << "\"sample_index\":" << sample.sample_index << ","
               << "\"refresh_succeeded\":"
               << (sample.refresh_succeeded ? "true" : "false") << ","
               << "\"scientifically_equal_to_fresh\":"
               << (sample.scientifically_equal_to_fresh ? "true" : "false")
               << ","
               << "\"wall_ms\":" << sample.wall_ms << ","
               << "\"refresh_elapsed_ms\":" << sample.refresh_elapsed_ms
               << ","
               << "\"provider_batch_duration_ms\":"
               << sample.provider_batch_duration_ms << ","
               << "\"logical_query_count\":"
               << sample.logical_query_count << ","
               << "\"provider_query_count\":"
               << sample.provider_query_count << ","
               << "\"spatial_recompute_count\":"
               << sample.spatial_recompute_count << ","
               << "\"spatial_reuse_count\":"
               << sample.spatial_reuse_count << ","
               << "\"retained_position_count\":"
               << sample.retained_position_count << ","
               << "\"entered_position_count\":"
               << sample.entered_position_count << ","
               << "\"evicted_position_count\":"
               << sample.evicted_position_count << ","
               << "\"gnss_advisory_invocation_count\":"
               << sample.gnss_advisory_invocation_count << ","
               << "\"lidar_advisory_invocation_count\":"
               << sample.lidar_advisory_invocation_count << ","
               << "\"horizon_fusion_count\":"
               << sample.horizon_fusion_count << ","
               << "\"full_rebuild\":"
               << (sample.full_rebuild ? "true" : "false") << ","
               << "\"full_invalidation_count\":"
               << sample.full_invalidation_count << ","
               << "\"invalidation_reason\":"
               << profileJsonString(sample.invalidation_reason) << ","
               << "\"gnss_generation\":" << sample.gnss_generation << ","
               << "\"lidar_generation\":" << sample.lidar_generation
               << ","
               << "\"prior_generation\":" << sample.prior_generation
               << ","
               << "\"occupancy_generation\":"
               << sample.occupancy_generation << ","
               << "\"occupancy_content_identity\":"
               << sample.occupancy_content_identity << ","
               << "\"snapshot_occupancy_generation\":"
               << sample.snapshot_occupancy_generation << ","
               << "\"snapshot_occupancy_stamp_s\":"
               << sample.snapshot_occupancy_stamp_s << ","
               << "\"snapshot_scientific_hash_fnv1a64\":"
               << profileJsonString(sample.snapshot_scientific_hash) << ","
               << "\"fresh_scientific_hash_fnv1a64\":"
               << profileJsonString(sample.fresh_scientific_hash) << "}";
        if (sample_index + 1u != scenario.samples.size()) output << ',';
        output << '\n';
      }
      output << "          ],\n"
             << "          \"summary_ms\": {\n"
             << "            \"wall_ms\": ";
      writeProfileTimingSummary(output, scenario.wall);
      output << ",\n            \"refresh_elapsed_ms\": ";
      writeProfileTimingSummary(output, scenario.refresh);
      output << ",\n            \"provider_batch_duration_ms\": ";
      writeProfileTimingSummary(output, scenario.provider);
      output << "\n          },\n"
             << "          \"speedup_vs_worker_1_p50\": {"
             << "\"wall_ms\":" << scenario.wall_speedup << ","
             << "\"refresh_elapsed_ms\":" << scenario.refresh_speedup
             << ","
             << "\"provider_batch_duration_ms\":"
             << scenario.provider_speedup << "}\n"
             << "        }";
      if (scenario_index + 1u != worker.scenarios.size()) output << ',';
      output << '\n';
    }
    output << "      ]\n    }";
    if (worker_index + 1u != workers.size()) output << ',';
    output << '\n';
  }
  output << "  ],\n"
         << "  \"validation\": {\n"
         << "    \"complete_worker_scenario_matrix\": true,\n"
         << "    \"all_refreshes_succeeded\": true,\n"
         << "    \"all_timings_finite\": true,\n"
         << "    \"exact_work_contracts\": true,\n"
         << "    \"scientific_equivalence_to_fresh\": true,\n"
         << "    \"scientific_hashes_stable_across_samples_and_workers\": true,\n"
         << "    \"current_source_provenance_recorded\": true\n"
         << "  }\n"
         << "}\n";
  return output.str();
}

}  // namespace

namespace ego_planner {

class P0RiskGridRuntimeStampTest : public ::testing::Test {
 protected:
  struct PredictorDiagnosticCounts {
    std::size_t legacy_unique_positions = 0;
    std::size_t legacy_lidar_evaluations = 0;
    std::size_t legacy_lidar_cache_hits = 0;
    std::size_t spatial_recompute = 0;
    std::size_t spatial_reuse = 0;
    std::size_t gnss_invocations = 0;
    std::size_t lidar_invocations = 0;
    std::size_t horizon_fusions = 0;
    std::size_t retained_positions = 0;
    std::size_t entered_positions = 0;
    std::size_t evicted_positions = 0;
    std::size_t full_invalidations = 0;
    std::size_t exact_retained_positions = 0;
    std::size_t ttl_retained_positions = 0;
    std::size_t gnss_ttl_expired_positions = 0;
    std::size_t legacy_current_ttl_expired_positions = 0;
    std::size_t watchdog_forced_full_rebuilds = 0;
    std::size_t invalid_source_provenance = 0;
    std::string invalidation_reason = "none";
  };

  static PredictorDiagnosticCounts predictorDiagnosticCounts(
      const P0RiskGridRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.health_state_mutex_);
    const auto state = runtime.healthPublicationStateSnapshot();
    return {state.predictor_unique_positions,
            state.predictor_lidar_evaluations,
            state.predictor_lidar_cache_hits,
            state.predictor_spatial_advisory_recompute_count,
            state.predictor_spatial_advisory_reuse_count,
            state.predictor_gnss_advisory_invocation_count,
            state.predictor_lidar_advisory_invocation_count,
            state.predictor_horizon_fusion_count,
            state.predictor_spatial_retained_position_count,
            state.predictor_spatial_entered_position_count,
            state.predictor_spatial_evicted_position_count,
            state.predictor_spatial_full_invalidation_count,
            state.predictor_spatial_exact_retained_position_count,
            state.predictor_spatial_ttl_retained_position_count,
            state.predictor_spatial_gnss_ttl_expired_position_count,
            state.predictor_spatial_legacy_current_ttl_expired_position_count,
            state.predictor_spatial_watchdog_forced_full_rebuild_count,
            state.predictor_spatial_invalid_source_provenance_count,
            state.predictor_spatial_invalidation_reason};
  }

  struct ProfileRuntimeObservation {
    P0RiskGridRuntime::HealthPublicationState state;
    iap::RiskGridHealth health;
  };

  static ProfileRuntimeObservation profileRuntimeObservation(
      const P0RiskGridRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.health_state_mutex_);
    return {runtime.healthPublicationStateSnapshot(), runtime.risk_grid_.health()};
  }

  static P0RiskGridRuntime::Config profileConfig(const int worker_count) {
    auto config = enabledConfig();
    config.grid.frame_id = "map";
    config.grid.lattice_anchor_w = Eigen::Vector3d::Zero();
    config.grid.resolution_m = kProfileResolutionM;
    config.grid.size_x_m = 30.0;
    config.grid.size_y_m = 30.0;
    config.grid.size_z_m = 6.0;
    config.grid.horizons_s.assign(kProfileHorizons.begin(),
                                 kProfileHorizons.end());
    config.grid.skip_occupied_voxels = false;
    config.grid.use_predictor_batch_query = true;
    config.grid.stale_timeout_s = 100.0;
    config.predictor_source_mode = iap::PredictorSourceMode::Fusion;
    config.predictor_gnss_epoch_policy =
        iap::PredictorGnssEpochPolicy::Required;
    config.predictor_use_current_integrity_prior = true;
    config.predictor_conservative_max_with_gnss = false;
    config.predictor_lidar_legacy_observability = false;
    config.predictor_lidar_fim_radius_m = 12.0;
    config.predictor_sigma_grow_m_sqrt_s = 0.15;
    config.predictor_gnss_spatial_ttl_s =
        std::numeric_limits<double>::quiet_NaN();
    config.predictor_legacy_current_spatial_ttl_s =
        std::numeric_limits<double>::quiet_NaN();
    config.predictor_full_refresh_watchdog_s =
        std::numeric_limits<double>::quiet_NaN();
    config.predictor_requested_worker_count = worker_count;
    config.predictor_effective_worker_count = worker_count;
    return config;
  }

  static void seedProfileCurrentInputs(P0RiskGridRuntime* runtime,
                                       const double stamp_s,
                                       const std::uint64_t prior_generation,
                                       const Eigen::Vector3d& position) {
    seedValidInputs(runtime, stamp_s, stamp_s);
    runtime->latest_odom_p_ = position;
    runtime->latest_current_.hpl = 4.0;
    runtime->latest_current_.vpl = 5.0;
    runtime->latest_current_.hal = 30.0;
    runtime->latest_current_.val = 20.0;
    runtime->latest_current_.im = 15.0;
    runtime->latest_current_.n_sv_used = 31;
    runtime->latest_current_.pdop = 2.0;
    runtime->latest_current_.n_hypotheses = 31;
    runtime->latest_current_.tdop = 2.0;
    runtime->latest_current_.n_trunks_observed = 4;
    runtime->latest_current_generation_ = prior_generation;
  }

  static void seedProfileGnss(P0RiskGridRuntime* runtime) {
    runtime->latest_epoch_ = profileGnssEpoch(100.0);
    runtime->latest_gnss_epoch_generation_ = 1u;
    runtime->gnss_epoch_seen_ = true;
    runtime->latest_gnss_epoch_stamp_ = 100.0;
    runtime->latest_gnss_epoch_satellite_count_ = 31u;
  }

  static void seedProfileLidar(P0RiskGridRuntime* runtime) {
    const auto points = profileLidarMapPoints();
    const auto primitives = profileLidarPrimitives();
    {
      std::lock_guard<std::mutex> lock(runtime->health_state_mutex_);
      runtime->map_seen_ = true;
      runtime->latest_map_stamp_ = 100.0;
    }
    {
      std::lock_guard<std::mutex> lock(
          runtime->lidar_predictor_input_mutex_);
      runtime->latest_lidar_map_points_ = points;
      runtime->latest_lidar_fim_primitives_ = primitives;
      runtime->latest_lidar_fim_diagnostics_.valid = true;
      runtime->latest_lidar_fim_diagnostics_.fallback_reason = "ok";
      runtime->latest_lidar_fim_diagnostics_.lidar_pca_primitives_total = 704;
      runtime->latest_lidar_fim_diagnostics_.lidar_pca_valid_normals = 704;
      runtime->latest_lidar_fim_diagnostics_.lidar_pca_invalid_normals = 0;
      runtime->latest_lidar_fim_diagnostics_.lidar_pca_support_mean = 0.0;
      runtime->latest_lidar_fim_diagnostics_.lidar_pca_support_min = 0;
      runtime->latest_lidar_map_point_count_ = points->size();
      runtime->latest_lidar_fim_primitive_count_ = primitives->size();
      runtime->latest_lidar_fim_valid_normal_count_ = primitives->size();
      runtime->latest_lidar_fim_fallback_reason_ = "ok";
      runtime->latest_lidar_generation_ = 1u;
      runtime->latest_lidar_stamp_ = 100.0;
    }
  }

  static void installProfileOccupancyEpoch(
      P0RiskGridRuntime* runtime,
      const std::shared_ptr<std::atomic<std::uint64_t>>& live_generation,
      const P0OccupancyEpoch::SourceOwner& source_owner,
      const std::uint64_t generation, const double stamp_s,
      const std::vector<Eigen::Vector3d>& occupied_centers) {
    live_generation->store(generation);
    runtime->setOccupancyEpochFactory(
        [live_generation, source_owner, generation, stamp_s,
         occupied_centers]() {
          return makeOccupancyEpochCapture(
              live_generation, generation, stamp_s, "map", occupied_centers,
              source_owner, [source_owner]() { return source_owner; },
              kProfileResolutionM, Eigen::Vector3d::Zero());
        });
  }

  static void validateProfileWorkContract(const ProfileScenario scenario,
                                          const ProfileSample& sample) {
    const bool cold = scenario == ProfileScenario::ColdFullRebuild;
    const bool stationary = scenario == ProfileScenario::StationaryEmptyDelta;
    const bool shifted =
        scenario == ProfileScenario::ShiftPlusOneXEmptyDelta;
    const bool changed =
        scenario == ProfileScenario::StationaryNonemptyDelta;
    const auto require = [scenario](const bool condition,
                                    const std::string& field) {
      if (!condition) {
        throw std::runtime_error(std::string(profileScenarioName(scenario)) +
                                 ": " + field);
      }
    };
    require(sample.refresh_succeeded, "refresh failed");
    require(sample.scientifically_equal_to_fresh,
            "fresh scientific hash mismatch");
    require(std::isfinite(sample.wall_ms), "nonfinite wall_ms");
    require(std::isfinite(sample.refresh_elapsed_ms),
            "nonfinite refresh_elapsed_ms");
    require(std::isfinite(sample.provider_batch_duration_ms),
            "nonfinite provider_batch_duration_ms");
    require(sample.logical_query_count == kProfileLogicalQueries,
            "logical_query_count");
    require(sample.provider_query_count == kProfileLogicalQueries,
            "provider_query_count");
    require(sample.spatial_recompute_count ==
                (shifted ? 320u : stationary ? 0u : 12800u),
            "spatial_recompute_count");
    require(sample.spatial_reuse_count ==
                (shifted ? 76480u : stationary ? 76800u : 64000u),
            "spatial_reuse_count");
    require(sample.retained_position_count ==
                (shifted ? 12480u : stationary ? 12800u : 0u),
            "retained_position_count");
    require(sample.entered_position_count ==
                (shifted ? 320u : stationary ? 0u : 12800u),
            "entered_position_count");
    require(sample.evicted_position_count ==
                (changed ? 12800u : shifted ? 320u : 0u),
            "evicted_position_count");
    require(sample.gnss_advisory_invocation_count ==
                (shifted ? 320u : stationary ? 0u : 12800u),
            "gnss_advisory_invocation_count");
    require(sample.lidar_advisory_invocation_count ==
                (shifted ? 320u : stationary ? 0u : 12800u),
            "lidar_advisory_invocation_count");
    require(sample.horizon_fusion_count == kProfileLogicalQueries,
            "horizon_fusion_count");
    require(sample.full_rebuild == (cold || changed), "full_rebuild");
    require(sample.full_invalidation_count == (changed ? 1u : 0u),
            "full_invalidation_count");
    require(sample.invalidation_reason ==
                (cold ? "uninitialized"
                      : changed ? "occupancy_source_changed" : "none"),
            "invalidation_reason");
    require(sample.gnss_generation == 1u, "gnss_generation");
    require(sample.lidar_generation == 1u, "lidar_generation");
    require(sample.prior_generation == (cold ? 1u : 2u),
            "prior_generation");
    require(sample.occupancy_generation == (cold ? 2u : 3u),
            "occupancy_generation");
    require(sample.occupancy_content_identity == (changed ? 2u : 1u),
            "occupancy_content_identity");
    require(sample.snapshot_occupancy_generation ==
                sample.occupancy_generation,
            "current occupancy diagnostic generation");
    require(sample.snapshot_occupancy_stamp_s ==
                (cold ? 100.0 : 100.5),
            "current occupancy diagnostic stamp");
    require(!sample.snapshot_scientific_hash.empty(),
            "empty scientific hash");
  }

  static ProfileSample runProfileSample(
      const int worker_count, const ProfileScenario scenario,
      const std::size_t sample_index, const bool warmup) {
    const auto config = profileConfig(worker_count);
    const std::string suffix = std::to_string(worker_count) + "_" +
        profileScenarioName(scenario) + "_" +
        (warmup ? "warmup_" : "sample_") + std::to_string(sample_index);
    auto node = std::make_shared<rclcpp::Node>(
        "icra020_measured_" + suffix,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedProfileCurrentInputs(&runtime, 100.0, 1u,
                             Eigen::Vector3d::Zero());
    seedProfileGnss(&runtime);
    seedProfileLidar(&runtime);
    const auto source_owner = std::make_shared<const int>(worker_count);
    const auto live_generation =
        std::make_shared<std::atomic<std::uint64_t>>(2u);
    const auto base_occupancy = profileOccupancyCenters();
    installProfileOccupancyEpoch(&runtime, live_generation, source_owner,
                                 2u, 100.0, base_occupancy);

    if (scenario != ProfileScenario::ColdFullRebuild) {
      if (!refreshOnce(&runtime)) {
        throw std::runtime_error(suffix + ": accepted base refresh failed");
      }
      const Eigen::Vector3d target_position =
          scenario == ProfileScenario::ShiftPlusOneXEmptyDelta
              ? Eigen::Vector3d(kProfileResolutionM, 0.0, 0.0)
              : Eigen::Vector3d::Zero();
      seedProfileCurrentInputs(&runtime, 100.5, 2u, target_position);
      installProfileOccupancyEpoch(
          &runtime, live_generation, source_owner, 3u, 100.5,
          scenario == ProfileScenario::StationaryNonemptyDelta
              ? profileChangedOccupancyCenters()
              : base_occupancy);
    }

    const auto wall_begin = ProfileClock::now();
    const bool succeeded = refreshOnce(&runtime);
    const double wall_ms =
        std::chrono::duration<double, std::milli>(ProfileClock::now() -
                                                  wall_begin)
            .count();
    if (!succeeded) {
      throw std::runtime_error(suffix + ": measured refresh failed");
    }
    const auto snapshot = runtime.acquireSnapshot();
    if (!snapshot) {
      throw std::runtime_error(suffix + ": missing measured snapshot");
    }
    const auto observation = profileRuntimeObservation(runtime);
    const auto counts = predictorDiagnosticCounts(runtime);

    const bool cold = scenario == ProfileScenario::ColdFullRebuild;
    const Eigen::Vector3d target_position =
        scenario == ProfileScenario::ShiftPlusOneXEmptyDelta
            ? Eigen::Vector3d(kProfileResolutionM, 0.0, 0.0)
            : Eigen::Vector3d::Zero();
    const auto target_occupancy =
        scenario == ProfileScenario::StationaryNonemptyDelta
            ? profileChangedOccupancyCenters()
            : base_occupancy;
    auto fresh_node = std::make_shared<rclcpp::Node>(
        "icra020_fresh_" + suffix,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime fresh(fresh_node, config);
    seedProfileCurrentInputs(&fresh, cold ? 100.0 : 100.5,
                             cold ? 1u : 2u, target_position);
    seedProfileGnss(&fresh);
    seedProfileLidar(&fresh);
    const auto fresh_owner = std::make_shared<const int>(100 + worker_count);
    const auto fresh_live = std::make_shared<std::atomic<std::uint64_t>>(
        cold ? 2u : 3u);
    installProfileOccupancyEpoch(&fresh, fresh_live, fresh_owner,
                                 cold ? 2u : 3u,
                                 cold ? 100.0 : 100.5,
                                 target_occupancy);
    if (!refreshOnce(&fresh)) {
      throw std::runtime_error(suffix + ": fresh refresh failed");
    }
    const auto fresh_snapshot = fresh.acquireSnapshot();
    const std::string measured_hash =
        profileSnapshotScientificHash(snapshot);
    const std::string fresh_hash =
        profileSnapshotScientificHash(fresh_snapshot);

    iap::RiskVoxel diagnostic_voxel;
    if (!snapshot->voxelAt(0, Eigen::Vector3i::Zero(), &diagnostic_voxel) ||
        !diagnostic_voxel.occupancy) {
      throw std::runtime_error(suffix + ": missing occupancy diagnostic");
    }

    ProfileSample sample;
    sample.sample_index = sample_index;
    sample.refresh_succeeded = succeeded;
    sample.scientifically_equal_to_fresh =
        !measured_hash.empty() && measured_hash == fresh_hash;
    sample.wall_ms = wall_ms;
    sample.refresh_elapsed_ms = observation.state.refresh_elapsed_ms;
    sample.provider_batch_duration_ms =
        observation.state.provider_batch_duration_ms;
    sample.logical_query_count = observation.state.refresh_query_count;
    sample.provider_query_count = observation.health.provider_query_count;
    sample.spatial_recompute_count = counts.spatial_recompute;
    sample.spatial_reuse_count = counts.spatial_reuse;
    sample.retained_position_count = counts.retained_positions;
    sample.entered_position_count = counts.entered_positions;
    sample.evicted_position_count = counts.evicted_positions;
    sample.gnss_advisory_invocation_count = counts.gnss_invocations;
    sample.lidar_advisory_invocation_count = counts.lidar_invocations;
    sample.horizon_fusion_count = counts.horizon_fusions;
    sample.full_rebuild = counts.spatial_recompute == kProfileLogicalPositions &&
                          counts.retained_positions == 0u;
    sample.full_invalidation_count = counts.full_invalidations;
    sample.invalidation_reason = counts.invalidation_reason;
    sample.gnss_generation = runtime.latest_gnss_epoch_generation_;
    {
      std::lock_guard<std::mutex> lock(runtime.lidar_predictor_input_mutex_);
      sample.lidar_generation = runtime.latest_lidar_generation_;
    }
    sample.prior_generation = runtime.latest_current_generation_;
    sample.occupancy_generation = rollingOccupancyGeneration(runtime);
    sample.occupancy_content_identity =
        rollingOccupancyContentIdentity(runtime);
    sample.snapshot_occupancy_generation =
        diagnostic_voxel.occupancy->occupancy_generation;
    sample.snapshot_occupancy_stamp_s =
        diagnostic_voxel.occupancy->cloud_stamp_s;
    sample.snapshot_scientific_hash = measured_hash;
    sample.fresh_scientific_hash = fresh_hash;
    validateProfileWorkContract(scenario, sample);
    return sample;
  }

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

  static void setOdomPosition(P0RiskGridRuntime* runtime,
                              const Eigen::Vector3d& position) {
    runtime->latest_odom_p_ = position;
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
    ++runtime->latest_gnss_epoch_generation_;
    if (runtime->latest_gnss_epoch_generation_ == 0u) {
      ++runtime->latest_gnss_epoch_generation_;
    }
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

  static void installStableOccupancyEpoch(
      P0RiskGridRuntime* runtime,
      const std::shared_ptr<std::atomic<uint64_t>>& live_generation,
      const P0OccupancyEpoch::SourceOwner& source_owner,
      const uint64_t generation, const double stamp_s,
      const std::vector<Eigen::Vector3d>& occupied_centers = {},
      std::function<void()> first_query_hook = {}) {
    live_generation->store(generation);
    runtime->setOccupancyEpochFactory(
        [live_generation, source_owner, generation, stamp_s,
         occupied_centers,
         first_query_hook = std::move(first_query_hook)]() {
          auto capture = makeOccupancyEpochCapture(
              live_generation, generation, stamp_s, "map",
              occupied_centers, source_owner,
              [source_owner]() { return source_owner; });
          if (capture.epoch && first_query_hook) {
            auto original = capture.epoch->diagnostic_query;
            auto invoked = std::make_shared<std::atomic<bool>>(false);
            capture.epoch->diagnostic_query =
                [original, invoked, first_query_hook](
                    const Eigen::Vector3d& position) {
                  if (!invoked->exchange(true)) first_query_hook();
                  return original(position);
                };
          }
          return capture;
        });
  }

  static void installOccupancyEpochWithFirstQueryHook(
      P0RiskGridRuntime* runtime, std::function<void()> hook) {
    auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
    runtime->setOccupancyEpochFactory(
        [live_generation, hook = std::move(hook)]() {
          auto capture = makeOccupancyEpochCapture(
              live_generation, 2u, 100.0, "map");
          auto original = capture.epoch->diagnostic_query;
          auto invoked = std::make_shared<std::atomic<bool>>(false);
          capture.epoch->diagnostic_query =
              [original, invoked, hook](const Eigen::Vector3d& position) {
                if (!invoked->exchange(true)) {
                  hook();
                }
                return original(position);
              };
          return capture;
        });
  }

  static bool gnssEpochSeen(const P0RiskGridRuntime& runtime) {
    return runtime.gnss_epoch_seen_;
  }

  static uint64_t gnssEpochSatelliteCount(const P0RiskGridRuntime& runtime) {
    return runtime.latest_gnss_epoch_satellite_count_;
  }

  static uint64_t gnssEpochGeneration(const P0RiskGridRuntime& runtime) {
    return runtime.latest_gnss_epoch_generation_;
  }

  static bool hasLatestGnssEpoch(const P0RiskGridRuntime& runtime) {
    return runtime.latest_epoch_.has_value();
  }

  static std::shared_ptr<const iap::LocalOccupancyGrid>
  rollingOccupancyOwner(const P0RiskGridRuntime& runtime) {
    return runtime.rolling_occupancy_owner_;
  }

  static uint64_t rollingOccupancyGeneration(
      const P0RiskGridRuntime& runtime) {
    return runtime.rolling_occupancy_generation_;
  }

  static uint64_t rollingOccupancyContentIdentity(
      const P0RiskGridRuntime& runtime) {
    return runtime.rolling_occupancy_content_identity_;
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

  static void seedEmptyLidarOwners(P0RiskGridRuntime* runtime) {
    std::lock_guard<std::mutex> lock(
        runtime->lidar_predictor_input_mutex_);
    runtime->latest_lidar_map_points_ =
        std::make_shared<const std::vector<Eigen::Vector3d>>();
    runtime->latest_lidar_fim_primitives_ =
        std::make_shared<const std::vector<iap::LidarFimPrimitive>>();
    runtime->latest_lidar_generation_ = 1u;
    runtime->latest_lidar_stamp_ = 100.0;
  }

  static void clearLidarOwners(P0RiskGridRuntime* runtime) {
    std::lock_guard<std::mutex> lock(
        runtime->lidar_predictor_input_mutex_);
    runtime->latest_lidar_map_points_.reset();
    runtime->latest_lidar_fim_primitives_.reset();
  }

  static void advanceGnssEpochGeneration(P0RiskGridRuntime* runtime) {
    ++runtime->latest_gnss_epoch_generation_;
  }

  static void setGnssEpochGeneration(P0RiskGridRuntime* runtime,
                                     const uint64_t generation) {
    runtime->latest_gnss_epoch_generation_ = generation;
  }

  static void setGnssSatelliteElevation(P0RiskGridRuntime* runtime,
                                        const double elevation) {
    ASSERT_TRUE(runtime->latest_epoch_.has_value());
    ASSERT_FALSE(runtime->latest_epoch_->sats.empty());
    runtime->latest_epoch_->sats.front().elevation = elevation;
  }

  static void replaceLidarMapOwner(P0RiskGridRuntime* runtime) {
    std::lock_guard<std::mutex> lock(
        runtime->lidar_predictor_input_mutex_);
    runtime->latest_lidar_map_points_ =
        std::make_shared<const std::vector<Eigen::Vector3d>>();
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
    runtime->latest_current_.n_trunks_observed = 0;
    runtime->latest_current_.tdop = 20.0;
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

  static void setLegacyCurrentSpatial(P0RiskGridRuntime* runtime,
                                      const double tdop,
                                      const int n_trunks_observed) {
    std::lock_guard<std::mutex> lock(runtime->health_state_mutex_);
    runtime->latest_current_.tdop = tdop;
    runtime->latest_current_.n_trunks_observed = n_trunks_observed;
    ++runtime->latest_current_generation_;
    if (runtime->latest_current_generation_ == 0u) {
      ++runtime->latest_current_generation_;
    }
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

  static gnss_comm::msg::GnssMeasMsg::SharedPtr makeValidGloRange(
      P0RiskGridRuntime* runtime, const uint32_t week,
      const double tow) {
    const uint32_t sat = gnss_comm::sat_no(SYS_GLO, 1);
    auto ephemeris = std::make_shared<gnss_comm::GloEphem>();
    ephemeris->sat = sat;
    ephemeris->toe = gnss_comm::gpst2time(week, tow);
    ephemeris->pos[0] = 2.02e7;
    ephemeris->pos[1] = 1.4e7;
    ephemeris->pos[2] = 1.0e7;
    ephemeris->vel[0] = 0.0;
    ephemeris->vel[1] = 2500.0;
    ephemeris->vel[2] = 500.0;
    for (int axis = 0; axis < 3; ++axis) {
      ephemeris->acc[axis] = 0.0;
    }
    {
      std::lock_guard<std::mutex> lock(runtime->health_state_mutex_);
      runtime->origin_set_ = true;
      runtime->origin_seen_ = true;
      runtime->origin_ecef_ = Eigen::Vector3d(6.378e6, 0.0, 0.0);
      runtime->glo_ephem_cache_[sat] = ephemeris;
    }

    auto msg = std::make_shared<gnss_comm::msg::GnssMeasMsg>();
    msg->meas.emplace_back();
    auto& observation = msg->meas.back();
    observation.time.week = week;
    observation.time.tow = tow;
    observation.sat = sat;
    observation.freqs = {FREQ1_GLO};
    observation.psr = {2.1e7};
    observation.psr_std = {3.0};
    return msg;
  }

  struct ExplicitAbsentGnssRaceScenario {
    std::string node_name;
    iap::PredictorSourceMode mode = iap::PredictorSourceMode::GnssOnly;
    iap::PredictorGnssEpochPolicy policy =
        iap::PredictorGnssEpochPolicy::Optional;
    bool needs_lidar = false;
    bool callback_produces_valid_epoch = false;
  };

  static void expectExplicitAbsentGnssRaceRollback(
      const ExplicitAbsentGnssRaceScenario& scenario) {
    SCOPED_TRACE(scenario.node_name);
    ensure_rclcpp();
    auto node = std::make_shared<rclcpp::Node>(
        scenario.node_name,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    auto config = enabledConfig();
    config.predictor_source_mode = scenario.mode;
    config.predictor_gnss_epoch_policy = scenario.policy;
    config.predictor_full_refresh_watchdog_s = 5.0;
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    if (scenario.needs_lidar) {
      seedEmptyLidarOwners(&runtime);
    }
    sendRange(&runtime, std::make_shared<gnss_comm::msg::GnssMeasMsg>());
    ASSERT_FALSE(hasLatestGnssEpoch(runtime));
    const uint64_t absent_generation = gnssEpochGeneration(runtime);
    ASSERT_NE(absent_generation, 0u);

    std::atomic<bool> publish_callback{false};
    installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
      if (!publish_callback.exchange(false)) {
        return;
      }
      if (scenario.callback_produces_valid_epoch) {
        sendRange(&runtime, makeValidGloRange(&runtime, 2200u, 100.0));
      } else {
        sendRange(&runtime,
                  std::make_shared<gnss_comm::msg::GnssMeasMsg>());
      }
    });
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto accepted = runtime.acquireSnapshot();
    ASSERT_NE(accepted, nullptr);

    seedValidInputs(&runtime, 105.0, 105.0);
    publish_callback.store(true);
    EXPECT_FALSE(refreshOnce(&runtime));
    EXPECT_EQ(runtime.health().reason, "predictor_spatial_source_changed");
    EXPECT_EQ(gnssEpochGeneration(runtime), absent_generation + 1u);
    EXPECT_EQ(hasLatestGnssEpoch(runtime),
              scenario.callback_produces_valid_epoch);
    expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
    const auto aborted = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(aborted.retained_positions, 0u);
    EXPECT_EQ(aborted.exact_retained_positions, 0u);
    EXPECT_EQ(aborted.ttl_retained_positions, 0u);
    EXPECT_EQ(aborted.entered_positions, 0u);
    EXPECT_EQ(aborted.evicted_positions, 0u);
    EXPECT_EQ(aborted.watchdog_forced_full_rebuilds, 0u);

    // Reconstruct the captured explicit-absent source version only to
    // observe the committed rolling slots and watchdog epoch after abort.
    if (scenario.callback_produces_valid_epoch) {
      sendRange(&runtime,
                std::make_shared<gnss_comm::msg::GnssMeasMsg>());
    }
    setGnssEpochGeneration(&runtime, absent_generation);
    ASSERT_FALSE(hasLatestGnssEpoch(runtime));
    seedValidInputs(&runtime, 100.0, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto rolling_retry = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(rolling_retry.spatial_recompute, 0u);
    EXPECT_EQ(rolling_retry.spatial_reuse, 54u);
    EXPECT_EQ(rolling_retry.retained_positions, 27u);
    EXPECT_EQ(rolling_retry.exact_retained_positions, 27u);
    EXPECT_EQ(rolling_retry.horizon_fusions, 54u);
    EXPECT_EQ(rolling_retry.watchdog_forced_full_rebuilds, 0u);

    seedValidInputs(&runtime, 105.0, 105.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto watchdog_retry = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(watchdog_retry.spatial_recompute, 27u);
    EXPECT_EQ(watchdog_retry.retained_positions, 0u);
    EXPECT_EQ(watchdog_retry.watchdog_forced_full_rebuilds, 1u);
    EXPECT_EQ(watchdog_retry.invalidation_reason, "watchdog_forced");
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

  struct LidarProvenanceView {
    uint64_t generation = 0;
    double stamp = std::numeric_limits<double>::quiet_NaN();
    bool has_map_owner = false;
    bool has_fim_owner = false;
  };

  static LidarProvenanceView lidarProvenance(
      const P0RiskGridRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.lidar_predictor_input_mutex_);
    return {runtime.latest_lidar_generation_, runtime.latest_lidar_stamp_,
            static_cast<bool>(runtime.latest_lidar_map_points_),
            static_cast<bool>(runtime.latest_lidar_fim_primitives_)};
  }

  static void advanceLidarGeneration(P0RiskGridRuntime* runtime) {
    std::lock_guard<std::mutex> lock(
        runtime->lidar_predictor_input_mutex_);
    ++runtime->latest_lidar_generation_;
    if (runtime->latest_lidar_generation_ == 0u) {
      ++runtime->latest_lidar_generation_;
    }
  }

  static void setLidarGeneration(P0RiskGridRuntime* runtime,
                                 const uint64_t generation) {
    std::lock_guard<std::mutex> lock(
        runtime->lidar_predictor_input_mutex_);
    runtime->latest_lidar_generation_ = generation;
  }

  static void setLidarStamp(P0RiskGridRuntime* runtime,
                            const double stamp) {
    std::lock_guard<std::mutex> lock(
        runtime->lidar_predictor_input_mutex_);
    runtime->latest_lidar_stamp_ = stamp;
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
  EXPECT_TRUE(std::isnan(config.predictor_gnss_spatial_ttl_s));
  EXPECT_TRUE(
      std::isnan(config.predictor_legacy_current_spatial_ttl_s));
  EXPECT_TRUE(std::isnan(config.predictor_full_refresh_watchdog_s));
  EXPECT_FALSE(node->has_parameter("p0.predictor.gnss_spatial_ttl_s"));
  EXPECT_FALSE(
      node->has_parameter("p0.predictor.legacy_current_spatial_ttl_s"));
  EXPECT_FALSE(node->has_parameter("p0.predictor.full_refresh_watchdog_s"));
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
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_advisory_recompute_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_advisory_reuse_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_exact_retained_position_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_ttl_retained_position_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_gnss_ttl_expired_position_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_legacy_current_ttl_expired_position_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_watchdog_forced_full_rebuild_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_spatial_invalid_source_provenance_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_gnss_advisory_invocation_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_lidar_advisory_invocation_count\":"),
            std::string::npos);
  EXPECT_NE(last_health_message.find(
                "\"predictor_horizon_fusion_count\":"),
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

TEST_F(P0RiskGridRuntimeStampTest,
       NonNullInvalidGnssCallbackPublishesOneAbsentGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_invalid_range_callback_generation_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, enabledConfig(),
                            std::make_unique<FakeProvider>());
  seedGnssEpoch(&runtime, 100.0);
  const uint64_t accepted_generation = gnssEpochGeneration(runtime);
  ASSERT_TRUE(hasLatestGnssEpoch(runtime));

  const auto expect_absent_update = [&](const auto& msg,
                                        const uint64_t before) {
    sendRange(&runtime, msg);
    EXPECT_EQ(gnssEpochGeneration(runtime), before + 1u);
    EXPECT_TRUE(gnssEpochSeen(runtime));
    EXPECT_EQ(gnssEpochSatelliteCount(runtime), 0u);
    EXPECT_FALSE(hasLatestGnssEpoch(runtime));
    EXPECT_TRUE(
        std::isnan(inputReadiness(runtime, 100.0).gnss_epoch_stamp_s));
  };

  expect_absent_update(std::make_shared<gnss_comm::msg::GnssMeasMsg>(),
                       accepted_generation);

  seedGnssEpoch(&runtime, 100.0);
  setOriginValid(&runtime, true);
  uint64_t before = gnssEpochGeneration(runtime);
  expect_absent_update(std::make_shared<gnss_comm::msg::GnssMeasMsg>(),
                       before);

  seedGnssEpoch(&runtime, 100.0);
  auto filtered = std::make_shared<gnss_comm::msg::GnssMeasMsg>();
  filtered->meas.emplace_back();
  filtered->meas.back().time.week = 2200u;
  filtered->meas.back().time.tow = 100.0;
  filtered->meas.back().sat = 1u;
  before = gnssEpochGeneration(runtime);
  expect_absent_update(filtered, before);

  seedGnssEpoch(&runtime, 100.0);
  auto missing_ephemeris =
      std::make_shared<gnss_comm::msg::GnssMeasMsg>();
  missing_ephemeris->meas.emplace_back();
  missing_ephemeris->meas.back().time.week = 2200u;
  missing_ephemeris->meas.back().time.tow = 100.0;
  missing_ephemeris->meas.back().sat = 1u;
  missing_ephemeris->meas.back().freqs = {FREQ1};
  missing_ephemeris->meas.back().psr = {2.1e7};
  missing_ephemeris->meas.back().psr_std = {3.0};
  before = gnssEpochGeneration(runtime);
  expect_absent_update(missing_ephemeris, before);

  sendRange(&runtime, {});
  EXPECT_EQ(gnssEpochGeneration(runtime), before + 1u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       InvalidGnssCallbackDuringProviderWorkAbortsAndLaterEpochRecovers) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_invalid_range_callback_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  std::atomic<bool> invalidate{false};
  installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
    if (invalidate.exchange(false)) {
      sendRange(&runtime,
                std::make_shared<gnss_comm::msg::GnssMeasMsg>());
    }
  });
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);
  const uint64_t accepted_epoch_generation = gnssEpochGeneration(runtime);

  invalidate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "predictor_spatial_source_changed");
  EXPECT_EQ(gnssEpochGeneration(runtime), accepted_epoch_generation + 1u);
  EXPECT_FALSE(hasLatestGnssEpoch(runtime));
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason,
            "missing_required_gnss_epoch_identity");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  const uint32_t recovery_week = 2200u;
  const double recovery_tow = 100.0;
  const auto recovery_gpst =
      gnss_comm::gpst2time(recovery_week, recovery_tow);
  const auto recovery_utc = gnss_comm::gpst2utc(recovery_gpst);
  const double recovery_stamp =
      static_cast<double>(recovery_utc.time) + recovery_utc.sec;
  sendRange(&runtime,
            makeValidGloRange(&runtime, recovery_week, recovery_tow));
  EXPECT_TRUE(hasLatestGnssEpoch(runtime));
  EXPECT_EQ(gnssEpochSatelliteCount(runtime), 1u);
  seedValidInputs(&runtime, recovery_stamp, recovery_stamp);
  installOccupancyEpoch(&runtime, recovery_stamp);
  EXPECT_TRUE(refreshOnce(&runtime));
  EXPECT_GT(runtime.acquireSnapshot()->generation_id(),
            accepted->generation_id());
}

TEST_F(P0RiskGridRuntimeStampTest,
       OptionalExplicitAbsentToValidCallbackDuringProviderWorkRollsBack) {
  expectExplicitAbsentGnssRaceRollback(
      {"p0_optional_absent_to_valid_gnss_race_test",
       iap::PredictorSourceMode::GnssOnly,
       iap::PredictorGnssEpochPolicy::Optional,
       false, true});
}

TEST_F(P0RiskGridRuntimeStampTest,
       AutoExplicitAbsentToAbsentCallbackRollsBackRollingState) {
  expectExplicitAbsentGnssRaceRollback(
      {"p0_auto_absent_to_absent_gnss_race_test",
       iap::PredictorSourceMode::Fusion,
       iap::PredictorGnssEpochPolicy::Auto,
       true, false});
}

TEST_F(P0RiskGridRuntimeStampTest,
       NeverSeenOptionalAndAutoAbortOnFirstCallbackDuringProviderWork) {
  ensure_rclcpp();
  const std::array<iap::PredictorGnssEpochPolicy, 2> policies = {
      iap::PredictorGnssEpochPolicy::Optional,
      iap::PredictorGnssEpochPolicy::Auto};
  for (std::size_t index = 0; index < policies.size(); ++index) {
    const bool auto_policy =
        policies[index] == iap::PredictorGnssEpochPolicy::Auto;
    auto node = std::make_shared<rclcpp::Node>(
        "p0_never_seen_gnss_race_test_" + std::to_string(index),
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    auto config = enabledConfig();
    config.predictor_source_mode =
        auto_policy ? iap::PredictorSourceMode::Fusion
                    : iap::PredictorSourceMode::GnssOnly;
    config.predictor_gnss_epoch_policy = policies[index];
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    if (auto_policy) {
      seedEmptyLidarOwners(&runtime);
    }
    ASSERT_EQ(gnssEpochGeneration(runtime), 0u);
    ASSERT_FALSE(hasLatestGnssEpoch(runtime));

    std::atomic<bool> publish_first_absent{false};
    installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
      if (publish_first_absent.exchange(false)) {
        sendRange(&runtime,
                  std::make_shared<gnss_comm::msg::GnssMeasMsg>());
      }
    });
    ASSERT_TRUE(refreshOnce(&runtime)) << index;
    ASSERT_EQ(gnssEpochGeneration(runtime), 0u) << index;
    const auto accepted = runtime.acquireSnapshot();
    ASSERT_NE(accepted, nullptr);

    publish_first_absent.store(true);
    EXPECT_FALSE(refreshOnce(&runtime)) << index;
    EXPECT_EQ(runtime.health().reason, "predictor_spatial_source_changed")
        << index;
    EXPECT_EQ(gnssEpochGeneration(runtime), 1u) << index;
    EXPECT_FALSE(hasLatestGnssEpoch(runtime)) << index;
    expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
    const auto aborted = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(aborted.retained_positions, 0u) << index;
    EXPECT_EQ(aborted.exact_retained_positions, 0u) << index;
    EXPECT_EQ(aborted.ttl_retained_positions, 0u) << index;
    EXPECT_EQ(aborted.watchdog_forced_full_rebuilds, 0u) << index;
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       InactiveGnssCallbacksDoNotAbortLidarOnlyOrDisabledRefresh) {
  ensure_rclcpp();
  struct Scenario {
    iap::PredictorSourceMode mode;
    iap::PredictorGnssEpochPolicy policy;
    bool publish_valid;
  };
  const std::array<Scenario, 2> scenarios = {{
      {iap::PredictorSourceMode::LidarOnly,
       iap::PredictorGnssEpochPolicy::Auto, true},
      {iap::PredictorSourceMode::Fusion,
       iap::PredictorGnssEpochPolicy::Disabled, false},
  }};
  for (std::size_t index = 0; index < scenarios.size(); ++index) {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_inactive_gnss_race_test_" + std::to_string(index),
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    auto config = enabledConfig();
    config.predictor_source_mode = scenarios[index].mode;
    config.predictor_gnss_epoch_policy = scenarios[index].policy;
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedEmptyLidarOwners(&runtime);

    std::atomic<bool> publish_callback{false};
    installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
      if (!publish_callback.exchange(false)) {
        return;
      }
      if (scenarios[index].publish_valid) {
        sendRange(&runtime, makeValidGloRange(&runtime, 2200u, 100.0));
      } else {
        sendRange(&runtime,
                  std::make_shared<gnss_comm::msg::GnssMeasMsg>());
      }
    });
    ASSERT_TRUE(refreshOnce(&runtime)) << index;
    const auto accepted = runtime.acquireSnapshot();
    ASSERT_NE(accepted, nullptr);
    ASSERT_EQ(gnssEpochGeneration(runtime), 0u);

    publish_callback.store(true);
    EXPECT_TRUE(refreshOnce(&runtime)) << index;
    EXPECT_EQ(gnssEpochGeneration(runtime), 1u) << index;
    EXPECT_NE(runtime.health().reason, "predictor_spatial_source_changed")
        << index;
    EXPECT_GT(runtime.acquireSnapshot()->generation_id(),
              accepted->generation_id())
        << index;
    const auto counts = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(counts.spatial_recompute, 0u) << index;
    EXPECT_EQ(counts.spatial_reuse, 54u) << index;
    EXPECT_EQ(counts.retained_positions, 27u) << index;
    EXPECT_EQ(counts.horizon_fusions, 54u) << index;
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       OptionalAndAutoNeverReuseEpochClearedByInvalidCallback) {
  ensure_rclcpp();
  const std::array<iap::PredictorGnssEpochPolicy, 2> policies = {
      iap::PredictorGnssEpochPolicy::Optional,
      iap::PredictorGnssEpochPolicy::Auto};
  for (std::size_t index = 0; index < policies.size(); ++index) {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_absent_gnss_policy_test_" + std::to_string(index),
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    auto config = enabledConfig();
    config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
    config.predictor_gnss_epoch_policy = policies[index];
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    ASSERT_GT(predictorDiagnosticCounts(runtime).gnss_invocations, 0u);

    sendRange(&runtime, std::make_shared<gnss_comm::msg::GnssMeasMsg>());
    ASSERT_FALSE(hasLatestGnssEpoch(runtime));
    (void)refreshOnce(&runtime);
    EXPECT_EQ(predictorDiagnosticCounts(runtime).gnss_invocations, 0u);
  }
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
  const auto accepted = lidarProvenance(runtime);
  EXPECT_EQ(accepted.generation, 1u);
  EXPECT_DOUBLE_EQ(accepted.stamp, 123.5);
  EXPECT_TRUE(accepted.has_map_owner);
  EXPECT_TRUE(accepted.has_fim_owner);

  sendCloud(&runtime, makeInvalidPointCloud());
  EXPECT_EQ(lidarMapPoints(runtime), nullptr);
  EXPECT_EQ(lidarFimPrimitives(runtime), nullptr);
  EXPECT_NE(lidarFimFallbackReason(runtime).find("invalid_lidar_pointcloud"),
            std::string::npos);
  const auto invalid = lidarProvenance(runtime);
  EXPECT_EQ(invalid.generation, 2u);
  EXPECT_DOUBLE_EQ(invalid.stamp, 123.0);
  EXPECT_FALSE(invalid.has_map_owner);
  EXPECT_FALSE(invalid.has_fim_owner);

  sendCloud(&runtime, makePointCloud({}));
  EXPECT_EQ(lidarMapPoints(runtime), nullptr);
  EXPECT_EQ(lidarFimPrimitives(runtime), nullptr);
  EXPECT_EQ(lidarFimFallbackReason(runtime), "empty_lidar_pointcloud");
  const auto empty = lidarProvenance(runtime);
  EXPECT_EQ(empty.generation, 3u);
  EXPECT_DOUBLE_EQ(empty.stamp, 123.5);
  EXPECT_FALSE(empty.has_map_owner);
  EXPECT_FALSE(empty.has_fim_owner);

  setLidarGeneration(&runtime, std::numeric_limits<uint64_t>::max());
  sendCloud(&runtime, makePointCloud(points, &normals));
  const auto wrapped = lidarProvenance(runtime);
  EXPECT_EQ(wrapped.generation, 1u);
  EXPECT_DOUBLE_EQ(wrapped.stamp, 123.5);
  EXPECT_TRUE(wrapped.has_map_owner);
  EXPECT_TRUE(wrapped.has_fim_owner);
}

TEST_F(P0RiskGridRuntimeStampTest,
       LidarOnlyInvalidGnssCallbackStillRebuildsCurrentHorizons) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_only_invalid_gnss_callback_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Auto;
  P0RiskGridRuntime runtime(node, config);

  seedValidInputs(&runtime, 123.5, 123.5);
  seedGnssEpoch(&runtime, 123.5);
  seedEmptyLidarOwners(&runtime);
  installOccupancyEpoch(&runtime, 123.5);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);
  const uint64_t accepted_epoch_generation = gnssEpochGeneration(runtime);

  sendRange(&runtime, std::make_shared<gnss_comm::msg::GnssMeasMsg>());
  EXPECT_EQ(gnssEpochGeneration(runtime), accepted_epoch_generation + 1u);
  EXPECT_FALSE(hasLatestGnssEpoch(runtime));

  advancePriorGeneration(&runtime);
  setCurrentProtectionLevels(&runtime, 1.5, 1.25);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto rebuilt = runtime.acquireSnapshot();
  ASSERT_NE(rebuilt, nullptr);
  EXPECT_GT(rebuilt->generation_id(), accepted->generation_id());
  EXPECT_EQ(rebuilt->horizonCount(), 2);
  const auto counts = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(counts.gnss_invocations, 0u);
  EXPECT_EQ(counts.horizon_fusions, 54u);
  const auto health = runtime.health();

  EXPECT_TRUE(health.ready);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_GT(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.predictor_gnss_used_count, 0u);
  EXPECT_NE(health.reason, "stale_gnss_epoch");

  auto fresh_node = std::make_shared<rclcpp::Node>(
      "p0_lidar_only_invalid_gnss_callback_fresh_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime fresh(fresh_node, config);
  seedValidInputs(&fresh, 123.5, 123.5);
  setPriorGeneration(&fresh, 2u);
  setCurrentProtectionLevels(&fresh, 1.5, 1.25);
  seedEmptyLidarOwners(&fresh);
  installOccupancyEpoch(&fresh, 123.5);
  ASSERT_TRUE(refreshOnce(&fresh));
  expectSnapshotsScientificallyEquivalent(
      rebuilt, fresh.acquireSnapshot(), false);
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
  seedEmptyLidarOwners(&runtime);
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
       PositiveHorizonEarlyFailureKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_positive_horizon_early_failure_retention_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  config.gnss_epoch_max_age_s = 0.25;
  P0RiskGridRuntime runtime(node, config);

  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  installOccupancyEpoch(&runtime, 100.0, {}, "map", 2u);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);

  seedValidInputs(&runtime, 100.5, 100.5);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason,
            "missing_required_gnss_epoch_identity");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
}

TEST_F(P0RiskGridRuntimeStampTest,
       WithinRefreshSpatialDedupReportsExactProductionCounts) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_within_refresh_spatial_dedup_count_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = false;
  config.predictor_source_mode = iap::PredictorSourceMode::Fusion;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  seedEmptyLidarOwners(&runtime);
  installOccupancyEpoch(&runtime, 100.0, {}, "map", 2u);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto health = runtime.health();
  const auto counts = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(health.provider_query_count, 54u);
  EXPECT_EQ(health.predictor_gnss_used_count, 54u);
  EXPECT_EQ(counts.spatial_recompute, 27u);
  EXPECT_EQ(counts.spatial_reuse, 27u);
  EXPECT_EQ(counts.legacy_unique_positions, 27u);
  EXPECT_EQ(counts.legacy_lidar_evaluations, 27u);
  EXPECT_EQ(counts.legacy_lidar_cache_hits, 27u);
  EXPECT_EQ(counts.gnss_invocations, 27u);
  EXPECT_EQ(counts.lidar_invocations, 27u);
  EXPECT_EQ(counts.horizon_fusions, 54u);
  EXPECT_NE(health.predictor_gnss_used_count, counts.gnss_invocations);

  setOdomStamp(&runtime, std::numeric_limits<double>::quiet_NaN());
  setCurrentStamp(&runtime, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(refreshOnce(&runtime));
  const auto reset_counts = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(reset_counts.spatial_recompute, 0u);
  EXPECT_EQ(reset_counts.spatial_reuse, 0u);
  EXPECT_EQ(reset_counts.gnss_invocations, 0u);
  EXPECT_EQ(reset_counts.lidar_invocations, 0u);
  EXPECT_EQ(reset_counts.horizon_fusions, 0u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       ProductionProviderReusesSpatialAdviceAcrossPriorOnlyRefresh) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_cross_refresh_spatial_reuse_count_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = false;
  config.predictor_source_mode = iap::PredictorSourceMode::Fusion;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  seedEmptyLidarOwners(&runtime);
  installOccupancyEpoch(&runtime, 100.0, {}, "map", 2u);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto first = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(first.spatial_recompute, 27u);
  EXPECT_EQ(first.entered_positions, 27u);
  EXPECT_EQ(first.retained_positions, 0u);

  seedValidInputs(&runtime, 100.5, 100.5);
  advancePriorGeneration(&runtime);
  setCurrentProtectionLevels(&runtime, 1.5, 1.25);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto rolling_snapshot = runtime.acquireSnapshot();
  const auto stationary = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(stationary.spatial_recompute, 0u);
  EXPECT_EQ(stationary.spatial_reuse, 54u);
  EXPECT_EQ(stationary.legacy_unique_positions, 0u);
  EXPECT_EQ(stationary.legacy_lidar_evaluations, 0u);
  EXPECT_EQ(stationary.legacy_lidar_cache_hits, 0u);
  EXPECT_EQ(stationary.gnss_invocations, 0u);
  EXPECT_EQ(stationary.lidar_invocations, 0u);
  EXPECT_EQ(stationary.horizon_fusions, 54u);
  EXPECT_EQ(stationary.retained_positions, 27u);
  EXPECT_EQ(stationary.entered_positions, 0u);
  EXPECT_EQ(stationary.evicted_positions, 0u);
  EXPECT_EQ(stationary.full_invalidations, 0u);
  EXPECT_EQ(stationary.invalidation_reason, "none");

  auto fresh_node = std::make_shared<rclcpp::Node>(
      "p0_cross_refresh_spatial_reuse_fresh_full_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime fresh(fresh_node, config);
  seedValidInputs(&fresh, 100.5, 100.5);
  setPriorGeneration(&fresh, 2u);
  setCurrentProtectionLevels(&fresh, 1.5, 1.25);
  seedGnssEpoch(&fresh, 100.0);
  seedEmptyLidarOwners(&fresh);
  installOccupancyEpoch(&fresh, 100.0, {}, "map", 2u);
  ASSERT_TRUE(refreshOnce(&fresh));
  expectSnapshotsScientificallyEquivalent(
      rolling_snapshot, fresh.acquireSnapshot(), false);
}

TEST_F(P0RiskGridRuntimeStampTest,
       NewerIdenticalRawOccupancyRetainsAllLosAndUsesCurrentDiagnostics) {
  ensure_rclcpp();
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = false;
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  auto node = std::make_shared<rclcpp::Node>(
      "p0_empty_occupancy_delta_reuse",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, config);
  const auto source_owner = std::make_shared<const int>(19);
  const auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  const std::vector<Eigen::Vector3d> occupied = {
      Eigen::Vector3d(-1.0, 0.0, 0.0)};
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                              2u, 100.0, occupied);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto initial_content_identity =
      rollingOccupancyContentIdentity(runtime);
  ASSERT_NE(initial_content_identity, 0u);

  seedValidInputs(&runtime, 100.5, 100.5);
  advancePriorGeneration(&runtime);
  installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                              9u, 100.5, occupied);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto retained_snapshot = runtime.acquireSnapshot();
  const auto retained = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(retained.spatial_recompute, 0u);
  EXPECT_EQ(retained.spatial_reuse, 54u);
  EXPECT_EQ(retained.gnss_invocations, 0u);
  EXPECT_EQ(retained.horizon_fusions, 54u);
  EXPECT_EQ(retained.retained_positions, 27u);
  EXPECT_EQ(retained.entered_positions, 0u);
  EXPECT_EQ(retained.full_invalidations, 0u);
  EXPECT_EQ(retained.invalidation_reason, "none");
  EXPECT_EQ(rollingOccupancyGeneration(runtime), 9u);
  EXPECT_EQ(rollingOccupancyContentIdentity(runtime),
            initial_content_identity);
  iap::RiskVoxel current_diagnostic;
  ASSERT_TRUE(retained_snapshot->voxelAt(
      0, Eigen::Vector3i(1, 1, 1), &current_diagnostic));
  ASSERT_NE(current_diagnostic.occupancy, nullptr);
  EXPECT_EQ(current_diagnostic.occupancy->occupancy_generation, 9u);
  EXPECT_DOUBLE_EQ(current_diagnostic.occupancy->cloud_stamp_s, 100.5);

  auto fresh_node = std::make_shared<rclcpp::Node>(
      "p0_empty_occupancy_delta_fresh",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime fresh(fresh_node, config);
  seedValidInputs(&fresh, 100.5, 100.5);
  setPriorGeneration(&fresh, 2u);
  seedGnssEpoch(&fresh, 100.0);
  const auto fresh_owner = std::make_shared<const int>(20);
  const auto fresh_live = std::make_shared<std::atomic<uint64_t>>(9u);
  installStableOccupancyEpoch(&fresh, fresh_live, fresh_owner,
                              9u, 100.5, occupied);
  ASSERT_TRUE(refreshOnce(&fresh));
  expectSnapshotsScientificallyEquivalent(
      retained_snapshot, fresh.acquireSnapshot(), false);
}

TEST_F(P0RiskGridRuntimeStampTest,
       AddedRemovedAndMixedRawDeltaEachForceCompleteFreshEquivalentRebuild) {
  ensure_rclcpp();
  struct Scenario {
    const char* name;
    std::vector<Eigen::Vector3d> base;
    std::vector<Eigen::Vector3d> target;
  };
  const std::vector<Scenario> scenarios = {
      {"added", {Eigen::Vector3d(-1.0, 0.0, 0.0)},
       {Eigen::Vector3d(-1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.0)}},
      {"removed", {Eigen::Vector3d(-1.0, 0.0, 0.0),
                    Eigen::Vector3d(0.0, 0.0, 0.0)},
       {Eigen::Vector3d(-1.0, 0.0, 0.0)}},
      {"mixed", {Eigen::Vector3d(-1.0, 0.0, 0.0),
                  Eigen::Vector3d(0.0, 0.0, 0.0)},
       {Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0)}},
  };
  for (std::size_t index = 0; index < scenarios.size(); ++index) {
    const auto& scenario = scenarios[index];
    auto config = enabledConfig();
    config.grid.skip_occupied_voxels = false;
    config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
    config.predictor_gnss_epoch_policy =
        iap::PredictorGnssEpochPolicy::Required;
    auto node = std::make_shared<rclcpp::Node>(
        std::string("p0_nonempty_delta_") + scenario.name,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    const auto source_owner = std::make_shared<const int>(30 + index);
    const auto live_generation =
        std::make_shared<std::atomic<uint64_t>>(2u);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                                2u, 100.0, scenario.base);
    ASSERT_TRUE(refreshOnce(&runtime)) << scenario.name;
    const uint64_t base_content = rollingOccupancyContentIdentity(runtime);

    seedValidInputs(&runtime, 100.5, 100.5);
    advancePriorGeneration(&runtime);
    installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                                7u, 100.5, scenario.target);
    ASSERT_TRUE(refreshOnce(&runtime)) << scenario.name;
    const auto rebuilt_snapshot = runtime.acquireSnapshot();
    const auto rebuilt = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(rebuilt.spatial_recompute, 27u) << scenario.name;
    EXPECT_EQ(rebuilt.gnss_invocations, 27u) << scenario.name;
    EXPECT_EQ(rebuilt.horizon_fusions, 54u) << scenario.name;
    EXPECT_EQ(rebuilt.retained_positions, 0u) << scenario.name;
    EXPECT_EQ(rebuilt.entered_positions, 27u) << scenario.name;
    EXPECT_EQ(rebuilt.full_invalidations, 1u) << scenario.name;
    EXPECT_EQ(rebuilt.invalidation_reason, "occupancy_source_changed")
        << scenario.name;
    EXPECT_GT(rollingOccupancyContentIdentity(runtime), base_content)
        << scenario.name;

    auto fresh_node = std::make_shared<rclcpp::Node>(
        std::string("p0_nonempty_delta_fresh_") + scenario.name,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime fresh(fresh_node, config);
    seedValidInputs(&fresh, 100.5, 100.5);
    setPriorGeneration(&fresh, 2u);
    seedGnssEpoch(&fresh, 100.0);
    const auto fresh_owner = std::make_shared<const int>(40 + index);
    const auto fresh_live =
        std::make_shared<std::atomic<uint64_t>>(7u);
    installStableOccupancyEpoch(&fresh, fresh_live, fresh_owner,
                                7u, 100.5, scenario.target);
    ASSERT_TRUE(refreshOnce(&fresh)) << scenario.name;
    expectSnapshotsScientificallyEquivalent(
        rebuilt_snapshot, fresh.acquireSnapshot(), false);
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       ContradictorySameVersionAndDeltaRacePreserveLastCommittedBase) {
  ensure_rclcpp();
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = false;
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  auto node = std::make_shared<rclcpp::Node>(
      "p0_delta_transaction_rollback",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, config);
  const auto source_owner = std::make_shared<const int>(50);
  const auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  const std::vector<Eigen::Vector3d> base = {
      Eigen::Vector3d(-1.0, 0.0, 0.0)};
  const std::vector<Eigen::Vector3d> changed = {
      Eigen::Vector3d(1.0, 0.0, 0.0)};
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                              2u, 100.0, base);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  const uint64_t accepted_content = rollingOccupancyContentIdentity(runtime);

  installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                              2u, 100.0, changed);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_los_adapter_invalid");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
  EXPECT_EQ(rollingOccupancyGeneration(runtime), 2u);
  EXPECT_EQ(rollingOccupancyContentIdentity(runtime), accepted_content);

  seedValidInputs(&runtime, 100.5, 100.5);
  advancePriorGeneration(&runtime);
  installStableOccupancyEpoch(
      &runtime, live_generation, source_owner, 4u, 100.5, base,
      [live_generation]() { live_generation->store(5u); });
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_generation_changed");
  EXPECT_EQ(rollingOccupancyGeneration(runtime), 2u);
  EXPECT_EQ(rollingOccupancyContentIdentity(runtime), accepted_content);

  installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                              8u, 100.5, base);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto retry = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(retry.spatial_recompute, 0u);
  EXPECT_EQ(retry.spatial_reuse, 54u);
  EXPECT_EQ(retry.retained_positions, 27u);
  EXPECT_EQ(retry.full_invalidations, 0u);
  EXPECT_EQ(rollingOccupancyGeneration(runtime), 8u);
  EXPECT_EQ(rollingOccupancyContentIdentity(runtime), accepted_content);
}

TEST_F(P0RiskGridRuntimeStampTest,
       PriorGnssAndLidarRacesAfterEmptyDeltaAbortWithoutAdvancingBase) {
  ensure_rclcpp();
  enum class Race { Prior, Gnss, Lidar };
  const std::vector<std::pair<const char*, Race>> races = {
      {"prior", Race::Prior}, {"gnss", Race::Gnss},
      {"lidar", Race::Lidar}};
  for (std::size_t index = 0; index < races.size(); ++index) {
    const auto& race = races[index];
    auto config = enabledConfig();
    config.grid.skip_occupied_voxels = false;
    config.predictor_source_mode = iap::PredictorSourceMode::Fusion;
    config.predictor_gnss_epoch_policy =
        iap::PredictorGnssEpochPolicy::Required;
    auto node = std::make_shared<rclcpp::Node>(
        std::string("p0_post_delta_") + race.first + "_race",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    const auto source_owner = std::make_shared<const int>(70 + index);
    const auto live_generation =
        std::make_shared<std::atomic<uint64_t>>(2u);
    const std::vector<Eigen::Vector3d> content = {
        Eigen::Vector3d(-1.0, 0.0, 0.0)};
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    seedEmptyLidarOwners(&runtime);
    installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                                2u, 100.0, content);
    ASSERT_TRUE(refreshOnce(&runtime)) << race.first;
    const auto accepted = runtime.acquireSnapshot();
    const uint64_t accepted_content =
        rollingOccupancyContentIdentity(runtime);

    seedValidInputs(&runtime, 100.5, 100.5);
    advancePriorGeneration(&runtime);
    const uint64_t captured_prior = 2u;
    const uint64_t captured_gnss = gnssEpochGeneration(runtime);
    const uint64_t captured_lidar = 1u;
    std::function<void()> mutate;
    if (race.second == Race::Prior) {
      mutate = [&runtime]() { advancePriorGeneration(&runtime); };
    } else if (race.second == Race::Gnss) {
      mutate = [&runtime]() { advanceGnssEpochGeneration(&runtime); };
    } else {
      mutate = [&runtime]() { advanceLidarGeneration(&runtime); };
    }
    installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                                5u, 100.5, content, std::move(mutate));
    EXPECT_FALSE(refreshOnce(&runtime)) << race.first;
    EXPECT_EQ(rollingOccupancyGeneration(runtime), 2u) << race.first;
    EXPECT_EQ(rollingOccupancyContentIdentity(runtime), accepted_content)
        << race.first;
    expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

    setPriorGeneration(&runtime, captured_prior);
    setGnssEpochGeneration(&runtime, captured_gnss);
    setLidarGeneration(&runtime, captured_lidar);
    installStableOccupancyEpoch(&runtime, live_generation, source_owner,
                                9u, 100.5, content);
    ASSERT_TRUE(refreshOnce(&runtime)) << race.first;
    const auto retry = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(retry.spatial_recompute, 0u) << race.first;
    EXPECT_EQ(retry.spatial_reuse, 54u) << race.first;
    EXPECT_EQ(retry.retained_positions, 27u) << race.first;
    EXPECT_EQ(retry.full_invalidations, 0u) << race.first;
    EXPECT_EQ(rollingOccupancyGeneration(runtime), 9u) << race.first;
    EXPECT_EQ(rollingOccupancyContentIdentity(runtime), accepted_content)
        << race.first;
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       ChangedProducerWithCoincidentGenerationCannotReuseLosContent) {
  ensure_rclcpp();
  auto config = enabledConfig();
  config.grid.skip_occupied_voxels = false;
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  auto node = std::make_shared<rclcpp::Node>(
      "p0_changed_occupancy_producer",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime runtime(node, config);
  const std::vector<Eigen::Vector3d> content = {
      Eigen::Vector3d(-1.0, 0.0, 0.0)};
  auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  auto first_owner = std::make_shared<const int>(60);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  installStableOccupancyEpoch(&runtime, live_generation, first_owner,
                              2u, 100.0, content);
  ASSERT_TRUE(refreshOnce(&runtime));
  const uint64_t first_content = rollingOccupancyContentIdentity(runtime);

  auto second_owner = std::make_shared<const int>(61);
  installStableOccupancyEpoch(&runtime, live_generation, second_owner,
                              2u, 100.0, content);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto changed_owner = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(changed_owner.spatial_recompute, 27u);
  EXPECT_EQ(changed_owner.retained_positions, 0u);
  EXPECT_EQ(changed_owner.full_invalidations, 1u);
  EXPECT_EQ(changed_owner.invalidation_reason,
            "source_provenance_invalid");
  EXPECT_GT(rollingOccupancyContentIdentity(runtime), first_content);
}

TEST_F(P0RiskGridRuntimeStampTest,
       DisabledProductionSourcesDoNotEraseStationarySpatialReuse) {
  ensure_rclcpp();
  for (const auto mode : {iap::PredictorSourceMode::GnssOnly,
                          iap::PredictorSourceMode::LidarOnly}) {
    const bool gnss_only = mode == iap::PredictorSourceMode::GnssOnly;
    const std::string mode_name = gnss_only ? "gnss" : "lidar";
    auto config = enabledConfig();
    config.grid.skip_occupied_voxels = false;
    config.predictor_source_mode = mode;
    config.predictor_gnss_epoch_policy =
        gnss_only ? iap::PredictorGnssEpochPolicy::Required
                  : iap::PredictorGnssEpochPolicy::Disabled;

    auto node = std::make_shared<rclcpp::Node>(
        "p0_disabled_source_reuse_" + mode_name,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    seedEmptyLidarOwners(&runtime);
    installOccupancyEpoch(&runtime, 100.0, {}, "map", 2u);
    ASSERT_TRUE(refreshOnce(&runtime));

    seedValidInputs(&runtime, 100.5, 100.5);
    advancePriorGeneration(&runtime);
    if (gnss_only) {
      seedEmptyLidarOwners(&runtime);
    } else {
      seedGnssEpoch(&runtime, 100.5);
      installOccupancyEpoch(&runtime, 100.5, {}, "map", 4u);
    }
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto rolling_snapshot = runtime.acquireSnapshot();
    const auto counts = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(counts.spatial_recompute, 0u) << mode_name;
    EXPECT_EQ(counts.spatial_reuse, 54u) << mode_name;
    EXPECT_EQ(counts.retained_positions, 27u) << mode_name;
    EXPECT_EQ(counts.entered_positions, 0u) << mode_name;
    EXPECT_EQ(counts.evicted_positions, 0u) << mode_name;
    EXPECT_EQ(counts.horizon_fusions, 54u) << mode_name;
    EXPECT_EQ(counts.full_invalidations, 0u) << mode_name;
    EXPECT_EQ(counts.invalidation_reason, "none") << mode_name;
    if (gnss_only) {
      EXPECT_EQ(counts.legacy_unique_positions, 0u);
      EXPECT_EQ(counts.legacy_lidar_evaluations, 0u);
      EXPECT_EQ(counts.legacy_lidar_cache_hits, 0u);
    }

    auto fresh_node = std::make_shared<rclcpp::Node>(
        "p0_disabled_source_reuse_fresh_" + mode_name,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime fresh(fresh_node, config);
    seedValidInputs(&fresh, 100.5, 100.5);
    setPriorGeneration(&fresh, 2u);
    seedGnssEpoch(&fresh, gnss_only ? 100.0 : 100.5);
    seedEmptyLidarOwners(&fresh);
    installOccupancyEpoch(&fresh, gnss_only ? 100.0 : 100.5, {}, "map",
                          gnss_only ? 2u : 4u);
    ASSERT_TRUE(refreshOnce(&fresh));
    expectSnapshotsScientificallyEquivalent(
        rolling_snapshot, fresh.acquireSnapshot(), false);
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       StationarySubVoxelAndOneVoxelRollMatchFreshFullForWorkersOneTwoFour) {
  ensure_rclcpp();
  const auto fresh_snapshot = [&](const std::string& node_name,
                                  const P0RiskGridRuntime::Config& config,
                                  const uint64_t prior_generation,
                                  const Eigen::Vector3d& position) {
    auto node = std::make_shared<rclcpp::Node>(
        node_name,
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime fresh(node, config);
    seedValidInputs(&fresh, 100.0, 100.0);
    setPriorGeneration(&fresh, prior_generation);
    setOdomPosition(&fresh, position);
    seedGnssEpoch(&fresh, 100.0);
    installOccupancyEpoch(&fresh, 100.0, {}, "map", 2u);
    EXPECT_TRUE(refreshOnce(&fresh));
    return fresh.acquireSnapshot();
  };
  std::vector<std::shared_ptr<const iap::RiskGridSnapshot>> shifted_snapshots;
  for (const int worker_count : {1, 2, 4}) {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_rolling_worker_" + std::to_string(worker_count),
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
    installOccupancyEpoch(&runtime, 100.0, {}, "map", 2u);
    ASSERT_TRUE(refreshOnce(&runtime));
    expectSnapshotsScientificallyEquivalent(
        runtime.acquireSnapshot(),
        fresh_snapshot("p0_rolling_fresh_first_" +
                           std::to_string(worker_count),
                       config, 1u, Eigen::Vector3d::Zero()),
        false);

    advancePriorGeneration(&runtime);
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto stationary = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(stationary.spatial_recompute, 0u);
    EXPECT_EQ(stationary.retained_positions, 27u);
    EXPECT_EQ(stationary.entered_positions, 0u);
    EXPECT_EQ(stationary.evicted_positions, 0u);
    EXPECT_EQ(stationary.horizon_fusions, 54u);
    expectSnapshotsScientificallyEquivalent(
        runtime.acquireSnapshot(),
        fresh_snapshot("p0_rolling_fresh_stationary_" +
                           std::to_string(worker_count),
                       config, 2u, Eigen::Vector3d::Zero()),
        false);

    advancePriorGeneration(&runtime);
    setOdomPosition(&runtime, Eigen::Vector3d(0.4, 0.0, 0.0));
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto sub_voxel = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(sub_voxel.spatial_recompute, 0u);
    EXPECT_EQ(sub_voxel.retained_positions, 27u);
    EXPECT_EQ(sub_voxel.horizon_fusions, 54u);
    expectSnapshotsScientificallyEquivalent(
        runtime.acquireSnapshot(),
        fresh_snapshot("p0_rolling_fresh_subvoxel_" +
                           std::to_string(worker_count),
                       config, 3u, Eigen::Vector3d(0.4, 0.0, 0.0)),
        false);

    advancePriorGeneration(&runtime);
    setOdomPosition(&runtime, Eigen::Vector3d(1.1, 0.0, 0.0));
    ASSERT_TRUE(refreshOnce(&runtime));
    const auto shifted = predictorDiagnosticCounts(runtime);
    EXPECT_EQ(shifted.spatial_recompute, 9u);
    EXPECT_EQ(shifted.retained_positions, 18u);
    EXPECT_EQ(shifted.entered_positions, 9u);
    EXPECT_EQ(shifted.evicted_positions, 9u);
    EXPECT_EQ(shifted.gnss_invocations, 9u);
    EXPECT_EQ(shifted.lidar_invocations, 0u);
    EXPECT_EQ(shifted.horizon_fusions, 54u);
    shifted_snapshots.push_back(runtime.acquireSnapshot());
    expectSnapshotsScientificallyEquivalent(
        runtime.acquireSnapshot(),
        fresh_snapshot("p0_rolling_fresh_shift_" +
                           std::to_string(worker_count),
                       config, 4u, Eigen::Vector3d(1.1, 0.0, 0.0)),
        false);
  }
  ASSERT_EQ(shifted_snapshots.size(), 3u);
  expectSnapshotsScientificallyEquivalent(shifted_snapshots[0],
                                          shifted_snapshots[1]);
  expectSnapshotsScientificallyEquivalent(shifted_snapshots[0],
                                          shifted_snapshots[2]);

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
       RematerializedLosWithStableSourceVersionUsesOneCapturePerRefresh) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_stable_occupancy_source_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  const auto source_owner = std::make_shared<const int>(1);
  auto factory_calls = std::make_shared<std::atomic<std::size_t>>(0u);
  auto live_owner_calls = std::make_shared<std::atomic<std::size_t>>(0u);
  auto live_generation_calls =
      std::make_shared<std::atomic<std::size_t>>(0u);
  runtime.setOccupancyEpochFactory(
      [live_generation, source_owner, factory_calls, live_owner_calls,
       live_generation_calls]() {
        factory_calls->fetch_add(1u);
        auto capture = makeOccupancyEpochCapture(
            live_generation, 2u, 100.0, "map", {}, source_owner,
            [source_owner, live_owner_calls]() {
              live_owner_calls->fetch_add(1u);
              return source_owner;
            });
        capture.epoch->live_generation =
            [live_generation, live_generation_calls]() {
              live_generation_calls->fetch_add(1u);
              return live_generation->load();
            };
        return capture;
      });

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto canonical_owner = rollingOccupancyOwner(runtime);
  ASSERT_NE(canonical_owner, nullptr);
  EXPECT_EQ(factory_calls->load(), 1u);
  EXPECT_EQ(live_owner_calls->load(), 2u);
  EXPECT_EQ(live_generation_calls->load(), 2u);

  ASSERT_TRUE(refreshOnce(&runtime));
  EXPECT_EQ(factory_calls->load(), 2u);
  EXPECT_EQ(live_owner_calls->load(), 4u);
  EXPECT_EQ(live_generation_calls->load(), 4u);
  EXPECT_EQ(rollingOccupancyOwner(runtime), canonical_owner);
}

TEST_F(P0RiskGridRuntimeStampTest,
       MissingActiveLidarFailsBeforeBatchWithTypedAttemptDiagnostic) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_missing_active_lidar_typed_failure_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedEmptyLidarOwners(&runtime);
  installOccupancyEpoch(&runtime, 100.0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);

  clearLidarOwners(&runtime);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "invalid_lidar_provenance");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
  const auto diagnostics = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(diagnostics.invalid_source_provenance, 1u);
  EXPECT_EQ(diagnostics.invalidation_reason, "source_provenance_invalid");
  EXPECT_EQ(diagnostics.spatial_recompute, 0u);
  EXPECT_EQ(diagnostics.spatial_reuse, 0u);
  EXPECT_EQ(diagnostics.retained_positions, 0u);
  EXPECT_EQ(diagnostics.exact_retained_positions, 0u);
  EXPECT_EQ(diagnostics.ttl_retained_positions, 0u);
  EXPECT_EQ(diagnostics.entered_positions, 0u);
  EXPECT_EQ(diagnostics.evicted_positions, 0u);
  EXPECT_EQ(diagnostics.gnss_ttl_expired_positions, 0u);
  EXPECT_EQ(diagnostics.legacy_current_ttl_expired_positions, 0u);
  EXPECT_EQ(diagnostics.watchdog_forced_full_rebuilds, 0u);

  seedEmptyLidarOwners(&runtime);
  setLidarGeneration(&runtime, 0u);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "invalid_lidar_provenance");
  EXPECT_EQ(predictorDiagnosticCounts(runtime).invalid_source_provenance,
            1u);
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());

  seedEmptyLidarOwners(&runtime);
  setLidarStamp(&runtime, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "invalid_lidar_provenance");
  EXPECT_EQ(predictorDiagnosticCounts(runtime).invalid_source_provenance,
            1u);
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
}

TEST_F(P0RiskGridRuntimeStampTest,
       RequiredGnssAndZeroCurrentProvenancePublishTypedAttemptFailure) {
  ensure_rclcpp();
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  const auto expect_typed_failure = [&](P0RiskGridRuntime* runtime,
                                        const std::string& detail,
                                        uint64_t accepted_generation) {
    EXPECT_FALSE(refreshOnce(runtime));
    EXPECT_EQ(runtime->health().reason, detail);
    ASSERT_NE(runtime->acquireSnapshot(), nullptr);
    EXPECT_EQ(runtime->acquireSnapshot()->generation_id(),
              accepted_generation);
    const auto diagnostics = predictorDiagnosticCounts(*runtime);
    EXPECT_EQ(diagnostics.invalid_source_provenance, 1u);
    EXPECT_EQ(diagnostics.invalidation_reason,
              "source_provenance_invalid");
    EXPECT_EQ(diagnostics.spatial_recompute, 0u);
    EXPECT_EQ(diagnostics.spatial_reuse, 0u);
    EXPECT_EQ(diagnostics.retained_positions, 0u);
    EXPECT_EQ(diagnostics.entered_positions, 0u);
    EXPECT_EQ(diagnostics.evicted_positions, 0u);
  };

  {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_required_absent_gnss_typed_failure_test",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const uint64_t accepted_generation =
        runtime.acquireSnapshot()->generation_id();
    sendRange(&runtime, std::make_shared<gnss_comm::msg::GnssMeasMsg>());
    expect_typed_failure(&runtime,
                         "missing_required_gnss_epoch_identity",
                         accepted_generation);
  }

  {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_zero_current_provenance_typed_failure_test",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const uint64_t accepted_generation =
        runtime.acquireSnapshot()->generation_id();
    setPriorGeneration(&runtime, 0u);
    expect_typed_failure(&runtime, "invalid_current_provenance",
                         accepted_generation);
  }

  {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_zero_gnss_provenance_typed_failure_test",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const uint64_t accepted_generation =
        runtime.acquireSnapshot()->generation_id();
    setGnssEpochGeneration(&runtime, 0u);
    expect_typed_failure(&runtime, "invalid_gnss_epoch_identity",
                         accepted_generation);
  }

  {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_nonfinite_current_provenance_typed_failure_test",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const uint64_t accepted_generation =
        runtime.acquireSnapshot()->generation_id();
    setCurrentStamp(&runtime,
                    std::numeric_limits<double>::quiet_NaN());
    expect_typed_failure(&runtime, "invalid_current_provenance",
                         accepted_generation);
  }

  {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_invalid_gnss_satellite_typed_failure_test",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const uint64_t accepted_generation =
        runtime.acquireSnapshot()->generation_id();
    setGnssSatelliteElevation(
        &runtime, std::numeric_limits<double>::quiet_NaN());
    expect_typed_failure(&runtime, "invalid_gnss_satellite_identity",
                         accepted_generation);
  }

  {
    auto node = std::make_shared<rclcpp::Node>(
        "p0_nonfinite_gnss_provenance_typed_failure_test",
        rclcpp::NodeOptions().allow_undeclared_parameters(false));
    P0RiskGridRuntime runtime(node, config);
    seedValidInputs(&runtime, 100.0, 100.0);
    seedGnssEpoch(&runtime, 100.0);
    installOccupancyEpoch(&runtime, 100.0);
    ASSERT_TRUE(refreshOnce(&runtime));
    const uint64_t accepted_generation =
        runtime.acquireSnapshot()->generation_id();
    auto invalid_epoch = makeGnssEpoch(8, 100.0);
    invalid_epoch.stamp = std::numeric_limits<double>::quiet_NaN();
    setLatestGnssEpoch(&runtime, invalid_epoch);
    expect_typed_failure(&runtime, "invalid_gnss_epoch_identity",
                         accepted_generation);
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       ExpiredStableOccupancyOwnerDuringBatchFailsClosed) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_occupancy_owner_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  const auto captured_source_owner = std::make_shared<const int>(1);
  auto live_source_owner =
      std::make_shared<ego_planner::P0OccupancyEpoch::SourceOwner>(
          captured_source_owner);
  std::atomic<bool> replace_owner{false};
  auto provider = std::make_unique<CallbackProvider>([&]() {
    if (replace_owner.load()) {
      live_source_owner->reset();
    }
  });
  P0RiskGridRuntime runtime(node, enabledConfig(), std::move(provider));
  runtime.setOccupancyEpochFactory(
      [live_generation, captured_source_owner, live_source_owner]() {
        return makeOccupancyEpochCapture(
            live_generation, 2u, 100.0, "map", {},
            captured_source_owner,
            [live_source_owner]() { return *live_source_owner; });
      });
  seedValidInputs(&runtime, 100.0, 100.0);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);

  replace_owner.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_generation_changed");
  const auto retained = runtime.acquireSnapshot();
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(retained->generation_id(), accepted->generation_id());
  expectSameActiveGeneration(accepted, retained);
}

TEST_F(P0RiskGridRuntimeStampTest,
       ProductionSameGenerationStableOccupancyOwnerReplacementFailsClosed) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_production_occupancy_owner_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  auto live_generation = std::make_shared<std::atomic<uint64_t>>(2u);
  const auto captured_source_owner = std::make_shared<const int>(1);
  auto live_source_owner =
      std::make_shared<ego_planner::P0OccupancyEpoch::SourceOwner>(
          captured_source_owner);
  auto replace_on_query = std::make_shared<std::atomic<bool>>(false);
  runtime.setOccupancyEpochFactory(
      [live_generation, captured_source_owner, live_source_owner,
       replace_on_query]() {
        auto capture = makeOccupancyEpochCapture(
            live_generation, 2u, 100.0, "map", {},
            captured_source_owner,
            [live_source_owner]() { return *live_source_owner; });
        auto query = capture.epoch->diagnostic_query;
        capture.epoch->diagnostic_query =
            [query, live_source_owner, replace_on_query](
                const Eigen::Vector3d& position) {
              if (replace_on_query->exchange(false)) {
                *live_source_owner = std::make_shared<const int>(2);
              }
              return query(position);
            };
        return capture;
      });
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);

  replace_on_query->store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "occupancy_generation_changed");
  const auto retained = runtime.acquireSnapshot();
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(retained->generation_id(), accepted->generation_id());
  expectSameActiveGeneration(accepted, retained);
}

TEST_F(P0RiskGridRuntimeStampTest,
       ProductionGnssTtlRetainsThenExpiresOriginalSpatialEpoch) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_gnss_spatial_ttl_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  config.gnss_epoch_max_age_s = 10.0;
  config.predictor_gnss_spatial_ttl_s = 5.0;
  P0RiskGridRuntime runtime(node, config);
  installOccupancyEpoch(&runtime, 100.0);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);

  ASSERT_TRUE(refreshOnce(&runtime));
  EXPECT_EQ(predictorDiagnosticCounts(runtime).spatial_recompute, 27u);

  seedValidInputs(&runtime, 102.0, 102.0);
  seedGnssEpoch(&runtime, 102.0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto retained = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(retained.spatial_recompute, 0u);
  EXPECT_EQ(retained.ttl_retained_positions, 27u);
  EXPECT_EQ(retained.exact_retained_positions, 0u);

  seedValidInputs(&runtime, 106.0, 106.0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto expired = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(expired.spatial_recompute, 27u);
  EXPECT_EQ(expired.gnss_ttl_expired_positions, 27u);
  EXPECT_EQ(expired.ttl_retained_positions, 0u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       ProductionWatchdogAdvancesOnlyAfterSuccessfulFullPublication) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_full_refresh_watchdog_rollback_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  config.gnss_epoch_max_age_s = 10.0;
  config.predictor_full_refresh_watchdog_s = 5.0;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  std::atomic<bool> mutate{false};
  installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
    if (mutate.exchange(false)) {
      advancePriorGeneration(&runtime);
    }
  });

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  seedValidInputs(&runtime, 105.0, 105.0);
  mutate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
  const auto aborted = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(aborted.watchdog_forced_full_rebuilds, 0u);
  EXPECT_EQ(aborted.spatial_recompute, 27u);
  EXPECT_EQ(aborted.retained_positions, 0u);

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto retried = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(retried.watchdog_forced_full_rebuilds, 1u);
  EXPECT_EQ(retried.spatial_recompute, 27u);
  EXPECT_EQ(retried.invalidation_reason, "watchdog_forced");

  seedValidInputs(&runtime, 106.0, 106.0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto after_commit = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(after_commit.spatial_recompute, 0u);
  EXPECT_EQ(after_commit.exact_retained_positions, 27u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       ProductionLegacyCurrentTtlRetainsTdopThenExpiresAndRejectsDiscreteChange) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_legacy_current_spatial_ttl_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  config.predictor_legacy_current_spatial_ttl_s = 5.0;
  P0RiskGridRuntime runtime(node, config);
  installOccupancyEpoch(&runtime, 100.0);
  seedEmptyLidarOwners(&runtime);
  seedValidInputs(&runtime, 100.0, 100.0);

  ASSERT_TRUE(refreshOnce(&runtime));
  EXPECT_EQ(predictorDiagnosticCounts(runtime).spatial_recompute, 27u);

  seedValidInputs(&runtime, 102.0, 102.0);
  setLegacyCurrentSpatial(&runtime, 21.0, 0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto retained = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(retained.spatial_recompute, 0u);
  EXPECT_EQ(retained.ttl_retained_positions, 27u);

  seedValidInputs(&runtime, 106.0, 102.0);
  setLegacyCurrentSpatial(&runtime, 21.0, 0);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto expired = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(expired.spatial_recompute, 27u);
  EXPECT_EQ(expired.legacy_current_ttl_expired_positions, 27u);

  seedValidInputs(&runtime, 107.0, 107.0);
  setLegacyCurrentSpatial(&runtime, 21.0, 1);
  ASSERT_TRUE(refreshOnce(&runtime));
  const auto discrete = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(discrete.spatial_recompute, 27u);
  EXPECT_EQ(discrete.full_invalidations, 1u);
  EXPECT_EQ(discrete.invalidation_reason, "current_integrity_changed");
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
       GnssEpochChangeDuringProductionRefreshKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_gnss_epoch_generation_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::GnssOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Required;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedGnssEpoch(&runtime, 100.0);
  std::atomic<bool> mutate{false};
  installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
    if (mutate.load()) {
      advanceGnssEpochGeneration(&runtime);
    }
  });

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  mutate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "predictor_spatial_source_changed");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
}

TEST_F(P0RiskGridRuntimeStampTest,
       LidarOwnerChangeDuringProductionRefreshKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_owner_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedEmptyLidarOwners(&runtime);
  std::atomic<bool> mutate{false};
  installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
    if (mutate.load()) {
      replaceLidarMapOwner(&runtime);
    }
  });

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  mutate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "predictor_spatial_source_changed");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
}

TEST_F(P0RiskGridRuntimeStampTest,
       LidarGenerationChangeDuringProductionRefreshKeepsPreviousGeneration) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_lidar_generation_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedEmptyLidarOwners(&runtime);
  std::atomic<bool> mutate{false};
  installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
    if (mutate.exchange(false)) {
      advanceLidarGeneration(&runtime);
    }
  });

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  mutate.store(true);
  EXPECT_FALSE(refreshOnce(&runtime));
  EXPECT_EQ(runtime.health().reason, "predictor_spatial_source_changed");
  expectSameActiveGeneration(accepted, runtime.acquireSnapshot());
  const auto diagnostics = predictorDiagnosticCounts(runtime);
  EXPECT_EQ(diagnostics.retained_positions, 0u);
  EXPECT_EQ(diagnostics.ttl_retained_positions, 0u);
  EXPECT_EQ(diagnostics.watchdog_forced_full_rebuilds, 0u);
}

TEST_F(P0RiskGridRuntimeStampTest,
       DisabledLegacyMapOwnerChangeDoesNotAbortProductionRefresh) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p0_disabled_legacy_map_owner_race_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = enabledConfig();
  config.predictor_source_mode = iap::PredictorSourceMode::LidarOnly;
  config.predictor_gnss_epoch_policy =
      iap::PredictorGnssEpochPolicy::Disabled;
  config.predictor_lidar_legacy_observability = false;
  P0RiskGridRuntime runtime(node, config);
  seedValidInputs(&runtime, 100.0, 100.0);
  seedEmptyLidarOwners(&runtime);
  std::atomic<bool> mutate{false};
  installOccupancyEpochWithFirstQueryHook(&runtime, [&]() {
    if (mutate.load()) {
      replaceLidarMapOwner(&runtime);
    }
  });

  ASSERT_TRUE(refreshOnce(&runtime));
  const auto accepted = runtime.acquireSnapshot();
  ASSERT_NE(accepted, nullptr);
  mutate.store(true);
  EXPECT_TRUE(refreshOnce(&runtime));
  EXPECT_GT(runtime.acquireSnapshot()->generation_id(),
            accepted->generation_id());
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
       OccupancyFreshnessUsesMessageClockAndRejectsFutureOrStaleEpochs) {
  ensure_rclcpp();
  auto config = enabledConfig();
  config.grid.stale_timeout_s = 0.5;

  auto fresh_node = std::make_shared<rclcpp::Node>(
      "p0_message_clock_fresh_occupancy_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime fresh(fresh_node, config,
                          std::make_unique<FakeProvider>());
  seedValidInputs(&fresh, 100.0, 100.0);
  installOccupancyEpoch(&fresh, 100.0);
  EXPECT_TRUE(refreshOnce(&fresh));
  EXPECT_EQ(fresh.health().reason, "ok");

  auto future_node = std::make_shared<rclcpp::Node>(
      "p0_message_clock_future_occupancy_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime future(future_node, config,
                           std::make_unique<FakeProvider>());
  seedValidInputs(&future, 100.0, 100.0);
  installOccupancyEpoch(&future, 100.001);
  EXPECT_FALSE(refreshOnce(&future));
  EXPECT_EQ(future.health().reason, "occupancy_stale");

  auto stale_node = std::make_shared<rclcpp::Node>(
      "p0_message_clock_stale_occupancy_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  P0RiskGridRuntime stale(stale_node, config,
                          std::make_unique<FakeProvider>());
  seedValidInputs(&stale, 100.0, 100.0);
  installOccupancyEpoch(&stale, 99.499);
  EXPECT_FALSE(refreshOnce(&stale));
  EXPECT_EQ(stale.health().reason, "occupancy_stale");
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
  EXPECT_EQ(runtime.health().reason, "invalid_current_provenance");
  EXPECT_EQ(predictorDiagnosticCounts(runtime).invalid_source_provenance, 1u);
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
  std::vector<PredictorDiagnosticCounts> diagnostic_counts;
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
    diagnostic_counts.push_back(predictorDiagnosticCounts(runtime));
  }

  ASSERT_EQ(snapshots.size(), 3u);
  expectSnapshotsScientificallyEquivalent(snapshots[0], snapshots[1]);
  expectSnapshotsScientificallyEquivalent(snapshots[0], snapshots[2]);
  for (const auto& counts : diagnostic_counts) {
    EXPECT_EQ(counts.legacy_unique_positions, 0u);
    EXPECT_EQ(counts.legacy_lidar_evaluations, 0u);
    EXPECT_EQ(counts.legacy_lidar_cache_hits, 0u);
    EXPECT_GT(counts.spatial_recompute, 0u);
    EXPECT_GT(counts.spatial_reuse, 0u);
    EXPECT_GT(counts.gnss_invocations, 0u);
    EXPECT_EQ(counts.lidar_invocations, 0u);
    EXPECT_GT(counts.horizon_fusions, 0u);
  }
  for (std::size_t index = 1; index < health_states.size(); ++index) {
    EXPECT_EQ(health_states[index].provider_query_count,
              health_states[0].provider_query_count);
    EXPECT_EQ(health_states[index].predictor_gnss_used_count,
              health_states[0].predictor_gnss_used_count);
    EXPECT_EQ(health_states[index].predictor_lidar_used_count,
              health_states[0].predictor_lidar_used_count);
    EXPECT_EQ(health_states[index].predictor_prior_used_count,
              health_states[0].predictor_prior_used_count);
    EXPECT_EQ(diagnostic_counts[index].spatial_recompute,
              diagnostic_counts[0].spatial_recompute);
    EXPECT_EQ(diagnostic_counts[index].spatial_reuse,
              diagnostic_counts[0].spatial_reuse);
    EXPECT_EQ(diagnostic_counts[index].gnss_invocations,
              diagnostic_counts[0].gnss_invocations);
    EXPECT_EQ(diagnostic_counts[index].lidar_invocations,
              diagnostic_counts[0].lidar_invocations);
    EXPECT_EQ(diagnostic_counts[index].horizon_fusions,
              diagnostic_counts[0].horizon_fusions);
  }
}

TEST_F(P0RiskGridRuntimeStampTest,
       DISABLED_ICRA020_ProductionRuntimeWorkerScalingProfile) {
  ensure_rclcpp();
  ASSERT_EQ(GTEST_FLAG_GET(filter), kProfileFilter)
      << "ICRA-020 requires the exact gtest filter";

  ProfileProvenance provenance;
  std::filesystem::path output_path;
  try {
    output_path = profileRequiredEnvironment("IAP_ICRA020_PROFILE_OUTPUT");
    provenance.implementation_sha =
        profileRequiredEnvironment("IAP_ICRA020_IMPLEMENTATION_SHA");
    provenance.build_type =
        profileRequiredEnvironment("IAP_ICRA020_BUILD_TYPE");
    provenance.exact_command =
        profileRequiredEnvironment("IAP_ICRA020_EXACT_COMMAND");
    provenance.test_binary_path =
        profileRequiredEnvironment("IAP_ICRA020_TEST_BINARY_PATH");
    provenance.test_binary_sha256 =
        profileRequiredEnvironment("IAP_ICRA020_TEST_BINARY_SHA256");
    provenance.libiap_path =
        profileRequiredEnvironment("IAP_ICRA020_LIBIAP_PATH");
    provenance.libiap_sha256 =
        profileRequiredEnvironment("IAP_ICRA020_LIBIAP_SHA256");
  } catch (const std::exception& error) {
    FAIL() << error.what();
  }
  provenance.cpu_model = profileCpuModel();
  provenance.logical_core_count = std::thread::hardware_concurrency();

  const auto repository_root = std::filesystem::current_path();
  const auto expected_output =
      repository_root / "results" / "icra27" / "icra020" /
      "p0_rolling_worker_profile.json";
  ASSERT_TRUE(std::filesystem::exists(repository_root / ".git"))
      << "profile must run from the repository root";
  ASSERT_EQ(std::filesystem::absolute(output_path).lexically_normal(),
            expected_output.lexically_normal())
      << "profile output must be the one authorized repository-local path";
  ASSERT_TRUE(std::filesystem::is_directory(expected_output.parent_path()));
  ASSERT_FALSE(std::filesystem::exists(expected_output))
      << "refusing to overwrite an existing canonical profile";
  ASSERT_TRUE(profileIsLowerHex(provenance.implementation_sha, 40u));
  ASSERT_TRUE(profileIsLowerHex(provenance.test_binary_sha256, 64u));
  ASSERT_TRUE(profileIsLowerHex(provenance.libiap_sha256, 64u));
  ASSERT_EQ(provenance.build_type, "RelWithDebInfo");
  ASSERT_FALSE(provenance.cpu_model.empty());
  ASSERT_GT(provenance.logical_core_count, 0u);
  for (const auto& path : {provenance.test_binary_path,
                           provenance.libiap_path}) {
    const std::filesystem::path relative(path);
    ASSERT_TRUE(relative.is_relative());
    ASSERT_EQ(relative.lexically_normal().string(), path);
    ASSERT_EQ(path.find(".."), std::string::npos);
    ASSERT_TRUE(std::filesystem::is_regular_file(repository_root / relative));
  }
  ASSERT_EQ(provenance.test_binary_path, kProfileTestBinaryPath);
  ASSERT_EQ(provenance.libiap_path, kProfileLibiapPath);
  ASSERT_EQ(provenance.exact_command, profileCanonicalCommand(provenance));
  try {
    ASSERT_EQ(profileCommandOutput("git rev-parse HEAD"),
              provenance.implementation_sha);
    ASSERT_EQ(profileCommandOutput(
                  "git diff --quiet HEAD -- && git diff --cached --quiet "
                  "HEAD -- && printf clean"),
              "clean");
    ASSERT_EQ(profileFileSha256(provenance.test_binary_path),
              provenance.test_binary_sha256);
    ASSERT_EQ(profileFileSha256(provenance.libiap_path),
              provenance.libiap_sha256);
  } catch (const std::exception& error) {
    FAIL() << error.what();
  }

  const std::array<ProfileScenario, 4> scenarios{
      {ProfileScenario::ColdFullRebuild,
       ProfileScenario::StationaryEmptyDelta,
       ProfileScenario::ShiftPlusOneXEmptyDelta,
       ProfileScenario::StationaryNonemptyDelta}};
  std::vector<ProfileWorkerRow> workers;
  try {
    for (const int worker_count : {1, 2, 4}) {
      ProfileWorkerRow worker;
      worker.requested_worker_count = worker_count;
      worker.effective_worker_count = worker_count;
      for (const ProfileScenario scenario : scenarios) {
        for (std::size_t warmup = 0; warmup < kProfileWarmups; ++warmup) {
          static_cast<void>(
              runProfileSample(worker_count, scenario, warmup, true));
        }
        ProfileScenarioRow row;
        row.scenario = scenario;
        std::vector<double> wall_samples;
        std::vector<double> refresh_samples;
        std::vector<double> provider_samples;
        for (std::size_t sample_index = 0;
             sample_index < kProfileMeasuredSamples; ++sample_index) {
          row.samples.push_back(runProfileSample(
              worker_count, scenario, sample_index, false));
          wall_samples.push_back(row.samples.back().wall_ms);
          refresh_samples.push_back(row.samples.back().refresh_elapsed_ms);
          provider_samples.push_back(
              row.samples.back().provider_batch_duration_ms);
        }
        row.wall = profileTimingSummary(std::move(wall_samples));
        row.refresh = profileTimingSummary(std::move(refresh_samples));
        row.provider = profileTimingSummary(std::move(provider_samples));
        worker.scenarios.push_back(std::move(row));
      }
      workers.push_back(std::move(worker));
    }
  } catch (const std::exception& error) {
    FAIL() << error.what();
  }

  ASSERT_EQ(workers.size(), 3u);
  for (std::size_t scenario_index = 0; scenario_index < scenarios.size();
       ++scenario_index) {
    const auto& baseline = workers.front().scenarios[scenario_index];
    std::set<std::string> stable_hashes;
    for (auto& worker : workers) {
      auto& scenario = worker.scenarios[scenario_index];
      ASSERT_EQ(scenario.samples.size(), kProfileMeasuredSamples);
      scenario.wall_speedup = baseline.wall.p50_ms / scenario.wall.p50_ms;
      scenario.refresh_speedup =
          baseline.refresh.p50_ms / scenario.refresh.p50_ms;
      scenario.provider_speedup =
          baseline.provider.p50_ms / scenario.provider.p50_ms;
      ASSERT_TRUE(std::isfinite(scenario.wall_speedup));
      ASSERT_TRUE(std::isfinite(scenario.refresh_speedup));
      ASSERT_TRUE(std::isfinite(scenario.provider_speedup));
      ASSERT_GT(scenario.wall_speedup, 0.0);
      ASSERT_GT(scenario.refresh_speedup, 0.0);
      ASSERT_GT(scenario.provider_speedup, 0.0);
      for (const auto& sample : scenario.samples) {
        stable_hashes.insert(sample.snapshot_scientific_hash);
      }
    }
    ASSERT_EQ(stable_hashes.size(), 1u)
        << profileScenarioName(scenarios[scenario_index]);
  }

  const std::string artifact = profileArtifactJson(workers, provenance);
  ASSERT_NE(artifact.find("\"diagnostic_execution_status\": \"PASS\""),
            std::string::npos);
  const auto temporary_output = expected_output.string() + ".tmp";
  {
    std::ofstream stream(temporary_output, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.good());
    stream << artifact;
    stream.flush();
    ASSERT_TRUE(stream.good());
  }
  std::filesystem::rename(temporary_output, expected_output);
  ASSERT_TRUE(std::filesystem::is_regular_file(expected_output));
}

}  // namespace ego_planner
