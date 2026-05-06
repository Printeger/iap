#include <iap/planner/integrity_snapshot.hpp>

#include <algorithm>
#include <cmath>

namespace iap {

namespace {

bool finite_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  return p.allFinite() && std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z());
}

bool finite_current_integrity(const CurrentIntegrityState& state) {
  return state.valid && std::isfinite(state.hpl) && std::isfinite(state.vpl) &&
         std::isfinite(state.hal) && std::isfinite(state.val) &&
         std::isfinite(state.im);
}

}  // namespace

IntegritySnapshot IntegritySnapshotBuilder::build_from_latest(
    const IntegritySnapshotBuilderInput& input) const {
  IntegritySnapshot out;
  out.stamp = std::isfinite(input.stamp) ? input.stamp : input.current.stamp;

  out.has_pose = input.has_pose && finite_pose(input.p_wb, input.q_wb);
  if (out.has_pose) {
    out.p_wb = input.p_wb;
    out.q_wb = input.q_wb.normalized();
  }

  out.current = input.current;
  if (!std::isfinite(out.current.pl)) {
    out.current.pl = current_pl_scalar(out.current.hpl, out.current.vpl);
  }

  if (input.gnss_epoch != nullptr) {
    out.has_epoch = true;
    out.gnss_epoch = *input.gnss_epoch;
  }

  if (input.lambda_base_pos != nullptr && input.lambda_base_pos->allFinite()) {
    out.has_lambda_base = true;
    out.lambda_base_pos = *input.lambda_base_pos;
  }

  if (input.lidar_snapshot != nullptr) {
    out.has_lidar_snapshot = true;
    out.lidar_snapshot_valid = input.lidar_snapshot->valid;
    out.lidar_block_count =
        static_cast<int>(input.lidar_snapshot->blocks.size());
    out.lidar_alpha =
        input.lidar_snapshot->current_icp_quality.gamma_lidar;
  }

  if (input.lidar_araim_result != nullptr) {
    out.has_lidar_araim_result = true;
    out.lidar_araim_valid = input.lidar_araim_result->valid;
    out.lidar_araim_n_hypotheses =
        input.lidar_araim_result->n_hypotheses;
    out.lidar_araim_n_detected = input.lidar_araim_result->n_detected;
  }

  out.valid = out.has_pose && finite_current_integrity(out.current);
  return out;
}

}  // namespace iap
