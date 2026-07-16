#pragma once
// Public entry point for the independent advisory Predictor module.

#include <memory>
#include <vector>

#include <iap/map/local_occupancy.hpp>
#include <iap/predictor/fusion_advisory_predictor.hpp>
#include <iap/predictor/gnss_advisory_predictor.hpp>
#include <iap/predictor/lidar_advisory_predictor.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace iap {

struct PredictorBatchDiagnostics {
  std::size_t query_count = 0;
  std::size_t unique_positions = 0;
  std::size_t lidar_evaluations = 0;
  std::size_t lidar_cache_hits = 0;
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
  PredictorParams params_;
  GnssAdvisoryPredictor gnss_;
  LidarAdvisoryPredictor lidar_;
  FusionAdvisoryPredictor fusion_;
  PredictorQueryResult queryWithLidar(
      const PredictorQueryInput& input,
      const LidarAdvisoryResult* cached_lidar) const;
};

}  // namespace iap
