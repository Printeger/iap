#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact backend interface for the planned hybrid CT architecture.
// Mainline use: compact backend only.
// Local frontend must not attach GNSS factors directly into its dense LiDAR graph.
// This boundary consumes only compact frontend summary state while keeping
// GNSS/shared-state/mapping ownership on the backend side.

#include <iap/gnss/gnss_types.hpp>
#include <iap/odometry/ct_backend_summary.hpp>

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>

// Forward declarations to avoid pulling NonlinearFactorGraph into every TU.
namespace gtsam {
class NonlinearFactorGraph;
class Values;
}  // namespace gtsam

namespace iap {

class CTCompactBackend {
 public:
  struct DebugStats {
    std::size_t raw_lidar_factor_count{0};
    std::size_t summary_pose_count{0};
    std::size_t gnss_factor_count{0};
  };

  // IAP-RQ-300 / IAP-RQ-410: Input carries GNSS epochs and anchor state for backend assembly.
  // Backend never holds raw LiDAR bucket factors; GNSS is the only sensor assembled here.
  struct Input {
    // GNSS epochs to assemble into the backend graph (empty = skip GNSS)
    std::vector<iap::GnssEpoch> gnss_epochs;
    // ECEF anchor state (needed for pseudorange/Doppler factors)
    bool gnss_anchor_initialized{false};
    gtsam::Vector3 ecef_origin{0.0, 0.0, 0.0};
    gtsam::Rot3 ecef_rot = gtsam::Rot3::Identity();
    // GNSS lever arm (antenna offset from IMU in IMU frame)
    Eigen::Vector3d gnss_lever_arm = Eigen::Vector3d::Zero();
    // Noise parameters
    double gnss_pr_noise_base{1.0};
    double gnss_dop_noise_base{0.1};
    double gnss_min_elevation{0.0};
    double gnss_elev_noise_exp{2.0};
  };

  struct LayerSegmentInput {
    double stamp{0.0};
    std::size_t auxiliary_index{0};
    std::vector<iap::GnssEpoch> gnss_epochs;
  };

  struct LayerInput {
    BSplineUnifiedGraphContext graph_context;
    std::vector<LayerSegmentInput> segments;
    bool gnss_anchor_initialized{false};
    gtsam::Vector3 ecef_origin{0.0, 0.0, 0.0};
    gtsam::Rot3 ecef_rot = gtsam::Rot3::Identity();
    Eigen::Vector3d gnss_lever_arm = Eigen::Vector3d::Zero();
    double gnss_pr_noise_base{1.0};
    double gnss_dop_noise_base{0.1};
    double gnss_min_elevation{0.0};
    double gnss_elev_noise_exp{2.0};
  };

  // IAP-RQ-300 / IAP-RQ-410: Assemble GNSS pseudorange and Doppler factors into graph/values
  // using the layout from local_result. Never adds raw LiDAR or IMU factors.
  BSplineNavigationLayerContribution assemble_navigation_layer(
    const LayerInput& input,
    gtsam::Values* values);

  void update(
    const CTLocalFrontendResult& local_result,
    const Input& input,
    gtsam::NonlinearFactorGraph* graph,
    gtsam::Values* values);

  DebugStats debug_stats(const CTBackendSummary& summary) const;

 private:
  std::size_t last_gnss_factor_count_{0};
};

}  // namespace iap
