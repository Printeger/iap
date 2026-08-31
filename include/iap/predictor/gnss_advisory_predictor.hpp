#pragma once
// GNSS advisory predictor for the independent Predictor module.

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <mutex>

#include <iap/map/local_occupancy.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace iap {

class GnssAdvisoryPredictor {
 public:
  GnssAdvisoryPredictor();
  explicit GnssAdvisoryPredictor(const GnssAdvisoryPredictorParams& params);

  void set_params(const GnssAdvisoryPredictorParams& params);
  void set_local_occupancy(const LocalOccupancyGrid* occupancy);
  void set_observation_predicate(
      VisibilityPredictor::ObservationPredicate predicate);

  GnssAdvisoryResult query(const Eigen::Vector3d& query_position,
                           const IntegritySnapshot& snapshot) const;
  // Current receiver measurement support is an anchor-only operation.  It is
  // intentionally separate from query() so standalone advisory diagnostics
  // remain raw and planning code cannot extend measured support spatially.
  GnssAdvisoryResult query_receiver_measured(
      const IntegritySnapshot& snapshot) const;
  VisibilityResult visibility_evidence(
      const Eigen::Vector3d& query_position,
      const IntegritySnapshot& snapshot) const;

  const GnssAdvisoryPredictorParams& params() const { return params_; }

 private:
  struct ReceiverAnchorCache;
  struct VisibilityEvidenceCache;
  GnssAdvisoryResult fallback(const std::string& reason) const;
  GnssAdvisoryResult query_unanchored(
      const Eigen::Vector3d& query_position,
      const IntegritySnapshot& snapshot,
      bool force_measured_epoch_support) const;
  GnssAdvisoryResult receiver_anchor_advisory(
      const IntegritySnapshot& snapshot) const;
  VisibilityResult cached_visibility_evidence(
      const Eigen::Vector3d& query_position,
      const GnssEpoch& epoch,
      bool measured_epoch_support) const;
  GnssAdvisoryResult compute_advisory_fim(
      const Eigen::Vector3d& query_position,
      const GnssEpoch& epoch,
      const VisibilityResult& visibility,
      const GnssAdvisoryResult& base) const;

  GnssAdvisoryPredictorParams params_;
  GnssGeometryPlPredictor geometry_predictor_;
  VisibilityPredictor visibility_predictor_;
  VisibilityPredictor::ObservationPredicate observation_predicate_;
  std::shared_ptr<ReceiverAnchorCache> receiver_anchor_cache_;
  std::shared_ptr<VisibilityEvidenceCache> visibility_evidence_cache_;
};

}  // namespace iap
