#ifndef EGO_PLANNER_P4_EXECUTION_RISK_WINDOW_H_
#define EGO_PLANNER_P4_EXECUTION_RISK_WINDOW_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ego_planner {

struct P4ExecutionRiskSample {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  // Time on the committed parent B-spline. A braking sample uses
  // anchor_time_s + time_on_brake.
  double relative_time_s = std::numeric_limits<double>::quiet_NaN();
};

struct P4BrakingRiskCurveSamples {
  double anchor_time_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<P4ExecutionRiskSample> samples;
};

struct P4ExecutionRiskWindowParams {
  double reaction_time_s = 1.2;
  double transition_overlap_s = 0.4;
  double maximum_anchor_gap_s = 0.2;
};

struct P4ExecutionRiskWindow {
  std::uint64_t window_id = 0;
  double nominal_start_time_s = 0.0;
  double nominal_end_time_s = 0.0;
  double certified_start_time_s = 0.0;
  double certified_end_time_s = 0.0;
  std::vector<std::size_t> request_row_indices;
};

struct P4ExecutionRiskWindowQueryRow {
  P4ExecutionRiskSample sample;
  std::uint64_t evidence_point_id = 0;
  std::uint64_t satellite_window_id = 0;
  bool nominal = false;
  bool braking = false;
  bool transition_overlap = false;
  std::size_t braking_curve_index = std::numeric_limits<std::size_t>::max();
};

struct P4ExecutionRiskWindowLayout {
  bool valid = false;
  std::string reason;
  std::string identity_hash;
  std::size_t unique_evidence_point_count = 0;
  std::size_t transition_count = 0;
  std::vector<P4ExecutionRiskWindow> windows;
  std::vector<P4ExecutionRiskWindowQueryRow> rows;
};

// Builds the deterministic execution-commitment windows used by direct
// ForwardRisk. It performs no GNSS calculation and never chooses a satellite
// set from PL values. Each output row is one membership of a physical evidence
// point in a braking window; overlap rows deliberately repeat an evidence ID.
P4ExecutionRiskWindowLayout buildP4ExecutionRiskWindowLayout(
    const std::vector<P4ExecutionRiskSample>& nominal_samples,
    const std::vector<P4BrakingRiskCurveSamples>& braking_curves,
    const P4ExecutionRiskWindowParams& params);

}  // namespace ego_planner

#endif  // EGO_PLANNER_P4_EXECUTION_RISK_WINDOW_H_
