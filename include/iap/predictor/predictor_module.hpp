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

  const PredictorParams& params() const { return params_; }

 private:
  PredictorParams params_;
  GnssAdvisoryPredictor gnss_;
  LidarAdvisoryPredictor lidar_;
  FusionAdvisoryPredictor fusion_;
};

}  // namespace iap
