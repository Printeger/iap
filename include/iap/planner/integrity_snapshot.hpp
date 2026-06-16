#pragma once
// Phase C: frozen current-monitor inputs for advisory future PL prediction.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <iap/gnss/gnss_types.hpp>
#include <iap/integrity/lidar_araim.hpp>

namespace iap {

struct CurrentIntegrityState {
  double stamp = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;

  int integrity_state = -1;

  // Current certified monitor outputs copied from /iap/integrity.
  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double pl_e = std::numeric_limits<double>::quiet_NaN();
  double pl_n = std::numeric_limits<double>::quiet_NaN();
  double pl_u = std::numeric_limits<double>::quiet_NaN();
  double pl = std::numeric_limits<double>::quiet_NaN();

  double hal = std::numeric_limits<double>::quiet_NaN();
  double val = std::numeric_limits<double>::quiet_NaN();
  double im = std::numeric_limits<double>::quiet_NaN();  ///< monitor IM

  double pl_ff = std::numeric_limits<double>::quiet_NaN();
  double pl_ff_v = std::numeric_limits<double>::quiet_NaN();
  double k_ff_used = std::numeric_limits<double>::quiet_NaN();
  double k_fa_used = std::numeric_limits<double>::quiet_NaN();

  int n_sv_used = 0;
  int n_constellations = 0;
  double pdop = std::numeric_limits<double>::quiet_NaN();
  double sigma_h = std::numeric_limits<double>::quiet_NaN();

  int n_hypotheses = 0;
  int n_detected = 0;
  std::vector<int> excluded_prns;
  std::vector<int> excluded_trunk_ids;

  int n_trunks_observed = 0;
  double tdop = std::numeric_limits<double>::quiet_NaN();

  double monitor_fused_hpl() const { return hpl; }
  double monitor_fused_vpl() const { return vpl; }
  double monitor_fused_pl() const { return pl; }
  double monitor_fused_pl_e() const { return pl_e; }
  double monitor_fused_pl_n() const { return pl_n; }
  double monitor_fused_pl_u() const { return pl_u; }
  double monitor_integrity_margin() const { return im; }
};

struct IntegritySnapshot {
  double stamp = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;

  bool has_pose = false;
  double pose_stamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d p_wb =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();

  CurrentIntegrityState current;  ///< current certified monitor snapshot

  bool has_epoch = false;
  GnssEpoch gnss_epoch;

  // Optional prior information for advisory future prediction only.
  bool has_lambda_base = false;
  Eigen::Matrix3d lambda_base_pos = Eigen::Matrix3d::Zero();

  bool has_lidar_snapshot = false;
  bool lidar_snapshot_valid = false;
  int lidar_block_count = 0;
  double lidar_alpha = std::numeric_limits<double>::quiet_NaN();

  bool has_lidar_araim_result = false;
  bool lidar_araim_valid = false;
  int lidar_araim_n_hypotheses = 0;
  int lidar_araim_n_detected = 0;
};

struct IntegritySnapshotBuilderInput {
  double stamp = std::numeric_limits<double>::quiet_NaN();

  bool has_pose = false;
  double pose_stamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d p_wb =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();

  CurrentIntegrityState current;

  const GnssEpoch* gnss_epoch = nullptr;
  const Eigen::Matrix3d* lambda_base_pos = nullptr;
  const LidarAraimSnapshot* lidar_snapshot = nullptr;
  const LidarAraimResult* lidar_araim_result = nullptr;
};

class IntegritySnapshotBuilder {
 public:
  IntegritySnapshot build_from_latest(
      const IntegritySnapshotBuilderInput& input) const;
};

inline double current_pl_scalar(const double hpl, const double vpl) {
  return std::isfinite(hpl) && std::isfinite(vpl)
             ? std::max(hpl, vpl)
             : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace iap
