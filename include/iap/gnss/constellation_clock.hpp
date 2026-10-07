#pragma once

#include <gtsam/inference/Symbol.h>
#include <iap/util/relinearization_policy.hpp>
#include <Eigen/Core>
#include <stdexcept>
#include <vector>

namespace gtsam { class GaussianFactorGraph; }

namespace iap {

// GPS retains the existing odometry/GNSS c(i) ownership contract. The other
// receiver bias/drift Vector2 states belong exclusively to the GNSS extension.
// e/r remain the shared ECEF anchor/rotation; b/v/x remain IMU bias/velocity/pose.
inline char gnss_clock_symbol(char constellation) {
  switch (constellation) {
    case 'G': return 'c';
    case 'C': return 'd';
    case 'E': return 'h';
    case 'R': return 'j';
    default: throw std::invalid_argument("unsupported GNSS clock constellation");
  }
}

inline gtsam::Key gnss_clock_key(char constellation, std::uint64_t frame_id) {
  return gtsam::Symbol(gnss_clock_symbol(constellation), frame_id);
}

inline void register_gnss_clock_relinearization(
    glim::RelinearizationPolicyRegistry& registry, const gtsam::Vector2& threshold) {
  for (const char system : {'G', 'C', 'E', 'R'}) {
    registry.register_policy(gnss_clock_symbol(system), 2, threshold);
  }
}

// Marginalize the actual linearized graph jointly, preserving the cross terms
// before taking system-reference. Adding two separate marginals is incorrect.
Eigen::Matrix2d gnss_clock_difference_covariance(
    const gtsam::GaussianFactorGraph& linear_graph,
    gtsam::Key reference, gtsam::Key system);

// At most X/V/B/R/E + four Vector2 clocks (29 tangent dimensions). No
// regularization or independent-marginal reconstruction is permitted.
Eigen::MatrixXd gnss_postopt_joint_covariance(
    const gtsam::GaussianFactorGraph& linear_graph,
    const gtsam::KeyVector& keys, const std::vector<std::uint32_t>& dimensions);

}  // namespace iap
