#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <iap/map/local_occupancy.hpp>
#include <iap/predictor/predictor_module.hpp>

namespace iap {

enum class RollingSpatialInvalidationReason {
  None = 0,
  Uninitialized,
  GeometryChanged,
  PredictorParametersChanged,
  GnssEpochChanged,
  OccupancySourceChanged,
  LidarSourceChanged,
  CurrentIntegrityChanged,
  SourcePolicyChanged,
  WindowDisjoint,
  SourceProvenanceInvalid,
  WatchdogForced,
};

const char* rollingSpatialInvalidationReasonName(
    RollingSpatialInvalidationReason reason);

struct RollingSpatialWindowGeometry {
  std::string frame_id = "map";
  Eigen::Vector3d lattice_anchor_w = Eigen::Vector3d::Zero();
  double resolution_m = 1.0;
  Eigen::Vector3i shape = Eigen::Vector3i::Ones();
};

struct RollingSpatialRetentionPolicy {
  double gnss_spatial_ttl_s = std::numeric_limits<double>::quiet_NaN();
  double legacy_current_spatial_ttl_s =
      std::numeric_limits<double>::quiet_NaN();
  double full_refresh_watchdog_s =
      std::numeric_limits<double>::quiet_NaN();
};

struct RollingSpatialSourceProvenance {
  std::uint64_t gnss_epoch_generation = 0;
  double gnss_epoch_stamp = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t occupancy_generation = 0;
  double occupancy_stamp = std::numeric_limits<double>::quiet_NaN();
  // Stable identity of raw LOS content, independent of authoritative source
  // generation. Zero means that content equality was not proven.
  std::uint64_t occupancy_content_identity = 0;
  std::uint64_t lidar_generation = 0;
  double lidar_stamp = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t current_generation = 0;
  double current_stamp = std::numeric_limits<double>::quiet_NaN();
  double refresh_reference_time_s =
      std::numeric_limits<double>::quiet_NaN();
};

struct RollingSpatialRefreshInput {
  RollingSpatialWindowGeometry geometry;
  PredictorModule module;
  IntegritySnapshot snapshot;
  RollingSpatialRetentionPolicy policy;
  RollingSpatialSourceProvenance provenance;
  std::shared_ptr<const LocalOccupancyGrid> occupancy_owner;
  std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points_owner;
  std::shared_ptr<const std::vector<LidarFimPrimitive>>
      lidar_fim_primitives_owner;
};

struct RollingSpatialRefreshDiagnostics {
  std::size_t retained_position_count = 0;
  std::size_t entered_position_count = 0;
  std::size_t evicted_position_count = 0;
  std::size_t full_invalidation_count = 0;
  std::size_t exact_retained_position_count = 0;
  std::size_t ttl_retained_position_count = 0;
  std::size_t gnss_ttl_expired_position_count = 0;
  std::size_t legacy_current_ttl_expired_position_count = 0;
  std::size_t watchdog_forced_full_rebuild_count = 0;
  std::size_t invalid_source_provenance_count = 0;
  RollingSpatialInvalidationReason invalidation_reason =
      RollingSpatialInvalidationReason::None;
};

// Production-facing transactional spatial-advisory reuse seam. The cached
// payload remains private to PredictorModule and cannot be forged by callers.
class RollingSpatialAdvisoryWindow {
 public:
  RollingSpatialAdvisoryWindow();
  ~RollingSpatialAdvisoryWindow();
  RollingSpatialAdvisoryWindow(RollingSpatialAdvisoryWindow&&) noexcept;
  RollingSpatialAdvisoryWindow& operator=(
      RollingSpatialAdvisoryWindow&&) noexcept;
  RollingSpatialAdvisoryWindow(const RollingSpatialAdvisoryWindow&) = delete;
  RollingSpatialAdvisoryWindow& operator=(
      const RollingSpatialAdvisoryWindow&) = delete;

  bool beginRefresh(RollingSpatialRefreshInput input,
                    std::string* reason = nullptr);
  std::vector<PredictorQueryResult> queryPositionHorizons(
      const std::vector<PredictorQueryInput>& inputs,
      PredictorBatchDiagnostics* diagnostics = nullptr);
  void commitRefresh();
  void abortRefresh();

  RollingSpatialRefreshDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace iap
