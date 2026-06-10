#pragma once
// Fusion advisory predictor for the independent Predictor module.

#include <iap/predictor/predictor_types.hpp>

namespace iap {

class FusionAdvisoryPredictor {
 public:
  FusionAdvisoryPredictor();
  explicit FusionAdvisoryPredictor(const FusionAdvisoryPredictorParams& params);

  void set_params(const FusionAdvisoryPredictorParams& params);

  FusionAdvisoryResult query(const IntegritySnapshot& snapshot,
                             const GnssAdvisoryResult& gnss,
                             const LidarAdvisoryResult& lidar) const;

  const FusionAdvisoryPredictorParams& params() const { return params_; }

 private:
  FusionAdvisoryPredictorParams params_;
};

}  // namespace iap
