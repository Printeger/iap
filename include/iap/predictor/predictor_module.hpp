#pragma once
// Public entry point for the independent advisory Predictor module.

#include <cstdint>
#include <memory>
#include <vector>

#include <iap/map/local_occupancy.hpp>
#include <iap/predictor/fusion_advisory_predictor.hpp>
#include <iap/predictor/gnss_advisory_predictor.hpp>
#include <iap/predictor/lidar_advisory_predictor.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace iap {

struct PredictorBatchDiagnostics {
  bool collect_component_timing = false;
  std::size_t query_count = 0;
  std::size_t unique_positions = 0;
  std::size_t lidar_evaluations = 0;
  std::size_t lidar_cache_hits = 0;
  std::size_t spatial_advisory_recompute_count = 0;
  std::size_t spatial_advisory_reuse_count = 0;
  std::size_t gnss_advisory_invocations = 0;
  std::size_t lidar_advisory_invocations = 0;
  std::size_t fusion_advisory_invocations = 0;
  std::uint64_t gnss_advisory_duration_ns = 0;
  std::uint64_t lidar_advisory_duration_ns = 0;
  std::uint64_t fusion_advisory_duration_ns = 0;
};

class PredictorModule {
 public:
  PredictorModule();
  explicit PredictorModule(const PredictorParams& params);

  void set_params(const PredictorParams& params);
  void set_local_occupancy(const LocalOccupancyGrid* occupancy);
  void set_lidar_fim_primitives(
      std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives);
  void set_lidar_map_points(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> points);

  PredictorQueryResult query(const PredictorQueryInput& input) const;
  std::vector<PredictorQueryResult> queryBatch(
      const std::vector<PredictorQueryInput>& inputs,
      PredictorBatchDiagnostics* diagnostics = nullptr) const;

  const PredictorParams& params() const { return params_; }

 private:
  struct SpatialAdvisory;

  PredictorParams params_;
  GnssAdvisoryPredictor gnss_;
  LidarAdvisoryPredictor lidar_;
  FusionAdvisoryPredictor fusion_;
  PredictorQueryResult queryWithSpatialAdvisory(
      const PredictorQueryInput& input,
      const SpatialAdvisory* cached_spatial_advisory,
      SpatialAdvisory* evaluated_spatial_advisory,
      PredictorBatchDiagnostics* diagnostics) const;
};

}  // namespace iap
