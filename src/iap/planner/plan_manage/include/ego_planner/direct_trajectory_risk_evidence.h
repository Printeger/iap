#ifndef EGO_PLANNER_DIRECT_TRAJECTORY_RISK_EVIDENCE_H_
#define EGO_PLANNER_DIRECT_TRAJECTORY_RISK_EVIDENCE_H_

#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace ego_planner {

struct P0ExecutionRiskSnapshot;

enum class P4ActualCurveCertificationStatus {
  NOT_EVALUATED = 0,
  SAFE,
  UNSAFE_SPATIAL_DOMINANT,
  UNSAFE_TEMPORAL_DOMINANT,
  INCOMPLETE,
};

inline const char* p4ActualCurveCertificationStatusName(
    const P4ActualCurveCertificationStatus status) {
  switch (status) {
    case P4ActualCurveCertificationStatus::NOT_EVALUATED:
      return "NOT_EVALUATED";
    case P4ActualCurveCertificationStatus::SAFE:
      return "SAFE";
    case P4ActualCurveCertificationStatus::UNSAFE_SPATIAL_DOMINANT:
      return "UNSAFE_SPATIAL_DOMINANT";
    case P4ActualCurveCertificationStatus::UNSAFE_TEMPORAL_DOMINANT:
      return "UNSAFE_TEMPORAL_DOMINANT";
    case P4ActualCurveCertificationStatus::INCOMPLETE:
      return "INCOMPLETE";
  }
  return "UNKNOWN";
}

inline std::string p4IdentityHash(const std::string& canonical) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : canonical) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

inline std::string p4ControlPointHash(const Eigen::MatrixXd& points) {
  std::ostringstream canonical;
  canonical << points.rows() << ';' << points.cols() << ';';
  for (int column = 0; column < points.cols(); ++column) {
    for (int row = 0; row < points.rows(); ++row) {
      canonical << std::hexfloat << points(row, column) << ';';
    }
  }
  return p4IdentityHash(canonical.str());
}

inline std::string p4KnotVectorHash(const Eigen::VectorXd& knots) {
  std::ostringstream canonical;
  canonical << knots.size() << ';';
  for (int index = 0; index < knots.size(); ++index) {
    canonical << std::hexfloat << knots(index) << ';';
  }
  return p4IdentityHash(canonical.str());
}

inline std::string p4RiskQueryLatticeHash(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<double>& relative_times) {
  if (points.size() != relative_times.size() || points.empty()) {
    return {};
  }
  std::ostringstream canonical;
  canonical << std::hexfloat << "p4_risk_query_lattice_v1;";
  for (std::size_t index = 0; index < points.size(); ++index) {
    canonical << relative_times[index] << ';' << points[index].x() << ';'
              << points[index].y() << ';' << points[index].z() << ';';
  }
  return p4IdentityHash(canonical.str());
}

struct P4DirectTrajectoryRiskEvidence {
  bool complete = false;
  bool certified_safe = false;
  P4ActualCurveCertificationStatus certification_status =
      P4ActualCurveCertificationStatus::NOT_EVALUATED;
  std::size_t first_failure_index = std::numeric_limits<std::size_t>::max();
  int trajectory_id = 0;
  std::int64_t trajectory_start_ns = 0;
  std::string control_points_hash;
  std::string knot_vector_hash;
  std::string sample_lattice_hash;
  std::string request_identity;
  std::uint64_t risk_generation = 0;
  std::uint64_t execution_snapshot_id = 0;
  std::uint64_t occupancy_generation = 0;
  std::uint64_t gnss_epoch_identity = 0;
  double evaluation_time_s = std::numeric_limits<double>::quiet_NaN();
  double compute_duration_ms = std::numeric_limits<double>::quiet_NaN();
  std::vector<int> common_satellite_ids;
  // Keeps the exact immutable risk snapshot used by the direct batch alive.
  // P5 consumes this pointer so a newly published generation cannot create a
  // split-snapshot race between the P4 check and P5 admission.
  std::shared_ptr<const iap::RiskGridSnapshot> risk_snapshot;
  std::shared_ptr<const P0ExecutionRiskSnapshot> execution_snapshot;
  std::vector<Eigen::Vector3d> positions;
  std::vector<double> relative_times;
  std::vector<iap::ForwardRiskPointResult> points;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_DIRECT_TRAJECTORY_RISK_EVIDENCE_H_
