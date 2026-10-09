#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace iap {
inline constexpr const char* kGnssPostoptModel = "postopt_X_V_B_R_E_active_clocks_joint_v1";

// One smoother finish, not a propagated state. Means are row-major manifold
// representations: X=4x4, V=3, B=accel/gyro 6, R=3x3, E=3, clock=bias/drift 2.
// Joint covariance belongs to the separately captured linearization means:
// X=right-local rotation/translation 6, V=world 3, B=6, R=local rotation 3,
// E=ECEF 3, then the clocks of actually used constellations. Cross terms remain.
// Availability is diagnostic evidence, never source or motion authorization.
struct GnssPostoptEvidence {
  std::string model = kGnssPostoptModel;
  std::uint64_t update_sequence = 0;
  std::int64_t frame_id = -1;
  double state_stamp = 0.;
  double gnss_stamp = 0.;
  std::uint64_t epoch_source_identity = 0;
  std::string used_constellations;
  bool optimized_valid = false;
  bool covariance_valid = false;
  std::string failure_reason = "postopt_evidence_unavailable";
  std::string propagation = "NOT_PROPAGATED";
  std::vector<std::uint64_t> keys;
  std::vector<std::uint32_t> tangent_dimensions;
  std::vector<std::uint32_t> mean_dimensions;
  std::vector<double> optimized_means;
  std::vector<double> linearization_means;
  std::vector<double> joint_covariance_row_major;
  // Original state/epoch stamps and means above never change. These are the
  // separately propagated state and covariance at the acquisition epoch.
  std::vector<double> propagated_optimized_means;
  std::vector<double> propagated_linearization_means;
  std::vector<double> propagated_joint_covariance;
  std::vector<double> propagation_transition;
  std::vector<double> propagation_noise;
  std::vector<double> imu_measurements;
  std::vector<double> imu_noise;
  std::vector<double> imu_bias_hat;
};
// ROS transport and frozen snapshots copy every member, including unavailable
// results and original times. No conversion grants freshness or qualification.
template<class Target, class Source> Target copy_gnss_postopt_evidence(const Source& source) {
  Target target;
  target.model=source.model;
  target.update_sequence=source.update_sequence;
  target.frame_id=source.frame_id;
  target.state_stamp=source.state_stamp;
  target.gnss_stamp=source.gnss_stamp;
  target.epoch_source_identity=source.epoch_source_identity;
  target.used_constellations=source.used_constellations;
  target.optimized_valid=source.optimized_valid;
  target.covariance_valid=source.covariance_valid;
  target.failure_reason=source.failure_reason;
  target.propagation=source.propagation;
  target.keys.assign(source.keys.begin(),source.keys.end());
  target.tangent_dimensions.assign(source.tangent_dimensions.begin(),source.tangent_dimensions.end());
  target.mean_dimensions.assign(source.mean_dimensions.begin(),source.mean_dimensions.end());
  target.optimized_means.assign(source.optimized_means.begin(),source.optimized_means.end());
  target.linearization_means.assign(source.linearization_means.begin(),source.linearization_means.end());
  target.joint_covariance_row_major.assign(source.joint_covariance_row_major.begin(),source.joint_covariance_row_major.end());
  target.propagated_optimized_means.assign(source.propagated_optimized_means.begin(),source.propagated_optimized_means.end());
  target.propagated_linearization_means.assign(source.propagated_linearization_means.begin(),source.propagated_linearization_means.end());
  target.propagated_joint_covariance.assign(source.propagated_joint_covariance.begin(),source.propagated_joint_covariance.end());
  target.propagation_transition.assign(source.propagation_transition.begin(),source.propagation_transition.end());
  target.propagation_noise.assign(source.propagation_noise.begin(),source.propagation_noise.end());
  target.imu_measurements.assign(source.imu_measurements.begin(),source.imu_measurements.end());
  target.imu_noise.assign(source.imu_noise.begin(),source.imu_noise.end());
  target.imu_bias_hat.assign(source.imu_bias_hat.begin(),source.imu_bias_hat.end());
  return target;
}
}
