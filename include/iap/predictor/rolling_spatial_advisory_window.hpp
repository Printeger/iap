#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
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
};

const char* rollingSpatialInvalidationReasonName(
    RollingSpatialInvalidationReason reason);

struct RollingSpatialWindowGeometry {
  std::string frame_id = "map";
  Eigen::Vector3d lattice_anchor_w = Eigen::Vector3d::Zero();
  double resolution_m = 1.0;
  Eigen::Vector3i shape = Eigen::Vector3i::Ones();
};

struct RollingSpatialRefreshInput {
  RollingSpatialWindowGeometry geometry;
  PredictorModule module;
  IntegritySnapshot snapshot;
  std::shared_ptr<const LocalOccupancyGrid> occupancy_owner;
  std::uint64_t occupancy_generation = 0;
  std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points_owner;
  std::shared_ptr<const std::vector<LidarFimPrimitive>>
      lidar_fim_primitives_owner;
};

struct RollingSpatialRefreshDiagnostics {
  std::size_t retained_position_count = 0;
  std::size_t entered_position_count = 0;
  std::size_t evicted_position_count = 0;
  std::size_t full_invalidation_count = 0;
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
