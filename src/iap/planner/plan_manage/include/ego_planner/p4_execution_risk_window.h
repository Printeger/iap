#ifndef EGO_PLANNER_P4_EXECUTION_RISK_WINDOW_H_
#define EGO_PLANNER_P4_EXECUTION_RISK_WINDOW_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <iap/predictor/predictor_types.hpp>

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
  // A physical sample may be shared by multiple certified braking curves.
  // Keep every responsibility; the scalar is the first index for legacy
  // diagnostics only and must not drive reachability decisions.
  std::vector<std::size_t> braking_curve_indices;
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

// Immutable safety-responsibility plan created with the exact trajectory that
// is committed for execution.  Runtime checks may select a subset of rows, but
// must never rebuild or renumber this layout.
struct P4CommittedRiskWindowPlan {
  bool valid = false;
  std::string reason;
  int trajectory_id = 0;
  std::int64_t trajectory_start_ns = 0;
  std::string control_points_hash;
  std::string knot_vector_hash;
  P4ExecutionRiskWindowLayout layout;
  std::vector<P4BrakingRiskCurveSamples> braking_curves;
};

struct P4CommittedRiskWindowSelection {
  bool valid = false;
  std::string reason;
  // This remains the full committed layout identity.  It is deliberately not
  // rehashed from the selected rows.
  std::string window_layout_hash;
  std::uint64_t current_window_id = 0;
  std::uint64_t next_window_id = 0;
  double next_window_boundary_time_s =
      std::numeric_limits<double>::infinity();
  std::vector<P4ExecutionRiskWindow> windows;
  std::vector<P4ExecutionRiskWindowQueryRow> rows;
  std::vector<std::size_t> source_row_indices;
};

struct P4RuntimeWindowWorstPoint {
  std::uint64_t satellite_window_id = 0;
  std::size_t result_index = std::numeric_limits<std::size_t>::max();
  double safety_ratio = std::numeric_limits<double>::quiet_NaN();
};

// Immutable result captured before the execution state machine consumes a
// direct-risk answer.  It intentionally owns the rejected/partial result too,
// so a fail-closed return cannot erase the physical reason for the decision.
struct P4RuntimeWindowEvidence {
  std::uint64_t sequence_id = 0;
  bool complete = false;
  std::string reason = "not_evaluated";
  int trajectory_id = 0;
  std::int64_t trajectory_start_ns = 0;
  std::string control_points_hash;
  std::string knot_vector_hash;
  std::string window_layout_hash;
  std::uint64_t execution_snapshot_id = 0;
  std::uint64_t occupancy_generation = 0;
  std::uint64_t support_generation = 0;
  std::uint64_t gnss_epoch_identity = 0;
  std::uint64_t integrity_generation = 0;
  double evaluation_time_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t current_window_id = 0;
  std::uint64_t next_window_id = 0;
  std::vector<P4ExecutionRiskWindow> active_windows;
  std::vector<P4ExecutionRiskWindowQueryRow> rows;
  std::vector<iap::ForwardRiskWindowResult> windows;
  std::vector<iap::ForwardRiskPointResult> points;
  iap::ForwardRiskBatchTiming timing;
  std::size_t global_worst_nominal_index =
      std::numeric_limits<std::size_t>::max();
  std::vector<P4RuntimeWindowWorstPoint> per_window_worst;
};

// Builds the deterministic execution-commitment windows used by direct
// ForwardRisk. It performs no GNSS calculation and never chooses a satellite
// set from PL values. Each output row is one membership of a physical evidence
// point in a braking window; overlap rows deliberately repeat an evidence ID.
P4ExecutionRiskWindowLayout buildP4ExecutionRiskWindowLayout(
    const std::vector<P4ExecutionRiskSample>& nominal_samples,
    const std::vector<P4BrakingRiskCurveSamples>& braking_curves,
    const P4ExecutionRiskWindowParams& params);

P4CommittedRiskWindowPlan buildP4CommittedRiskWindowPlan(
    int trajectory_id, std::int64_t trajectory_start_ns,
    const std::string& control_points_hash,
    const std::string& knot_vector_hash,
    const std::vector<P4ExecutionRiskSample>& nominal_samples,
    const std::vector<P4BrakingRiskCurveSamples>& braking_curves,
    const P4ExecutionRiskWindowParams& params);

// Selects the current and next pre-existing responsibility windows.  It drops
// only fixed rows that can no longer be reached; it never creates a new point,
// window, evidence ID, or layout identity.
P4CommittedRiskWindowSelection selectP4CommittedRiskWindowRows(
    const P4CommittedRiskWindowPlan& plan, double current_time_s);

std::vector<std::size_t> reachableP4CommittedBrakingCurveIndices(
    const P4CommittedRiskWindowPlan& plan,
    const P4CommittedRiskWindowSelection& selection,
    double current_time_s);

P4RuntimeWindowEvidence buildP4RuntimeWindowEvidence(
    std::uint64_t sequence_id,
    const P4CommittedRiskWindowPlan& plan,
    const P4CommittedRiskWindowSelection& selection,
    std::uint64_t execution_snapshot_id,
    std::uint64_t occupancy_generation,
    std::uint64_t support_generation,
    std::uint64_t gnss_epoch_identity,
    std::uint64_t integrity_generation,
    double evaluation_time_s,
    const iap::ForwardRiskBatchResult& result);

}  // namespace ego_planner

#endif  // EGO_PLANNER_P4_EXECUTION_RISK_WINDOW_H_
