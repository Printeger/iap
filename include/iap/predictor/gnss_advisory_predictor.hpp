#pragma once
// GNSS advisory predictor for the independent Predictor module.

#include <Eigen/Core>

#include <iap/map/local_occupancy.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace iap {

class GnssAdvisoryPredictor {
 public:
  GnssAdvisoryPredictor();
  explicit GnssAdvisoryPredictor(const GnssAdvisoryPredictorParams& params);

  void set_params(const GnssAdvisoryPredictorParams& params);
  void set_local_occupancy(const LocalOccupancyGrid* occupancy);

  GnssAdvisoryResult query(const Eigen::Vector3d& query_position,
                           const IntegritySnapshot& snapshot) const;

  const GnssAdvisoryPredictorParams& params() const { return params_; }

 private:
  GnssAdvisoryResult fallback(const std::string& reason) const;
  GnssAdvisoryResult compute_advisory_fim(
      const Eigen::Vector3d& query_position,
      const GnssEpoch& epoch,
      const VisibilityResult& visibility,
      const GnssAdvisoryResult& base) const;

  GnssAdvisoryPredictorParams params_;
  GnssGeometryPlPredictor geometry_predictor_;
  VisibilityPredictor visibility_predictor_;
};

}  // namespace iap
