#pragma once
// LiDAR advisory predictor for the independent Predictor module.

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <iap/predictor/predictor_types.hpp>

namespace iap {

class LidarAdvisoryPredictor {
 public:
  LidarAdvisoryPredictor();
  explicit LidarAdvisoryPredictor(const LidarAdvisoryPredictorParams& params);

  void set_params(const LidarAdvisoryPredictorParams& params);
  void set_lidar_fim_primitives(
      std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives);
  void set_lidar_map_points(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> points);

  LidarAdvisoryResult query(const Eigen::Vector3d& query_position,
                            const IntegritySnapshot& snapshot) const;

  const LidarAdvisoryPredictorParams& params() const { return params_; }

 private:
  void rebuild_lidar_fim_index();

  LidarAdvisoryPredictorParams params_;
  std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives_;
  std::shared_ptr<const LidarFimPrimitiveIndex> primitive_index_;
  std::shared_ptr<const std::vector<Eigen::Vector3d>> map_points_;
};

}  // namespace iap
