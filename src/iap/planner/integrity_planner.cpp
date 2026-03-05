// IAP-RQ-400: Integrity-aware planning objective
// IAP-RQ-410: Receding horizon loop
// IAP-RQ-331/421/422: ARAIM-predicted PL + per-waypoint AL

#include <iap/planner/integrity_planner.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace iap {

// ----------------------------------------------------------------------------
IntegrityPlanner::IntegrityPlanner()
: params_(),
  generator_(),
  predictor_() {}

IntegrityPlanner::IntegrityPlanner(const Params& p,
                                   const TrajectoryGenerator::Params& gen_p,
                                   const PredictedIntegrityComputer::Params& pic_p)
: params_(p),
  generator_(gen_p),
  predictor_(pic_p),
  araim_predictor_(p.araim_pred_params) {}

// ----------------------------------------------------------------------------
void IntegrityPlanner::set_occupancy(const LocalOccupancyGrid* grid) {
  predictor_.set_occupancy(grid);
  araim_predictor_.set_occupancy(grid);
}

void IntegrityPlanner::set_epoch(const GnssEpoch* epoch) {
  predictor_.set_epoch(epoch);
  araim_predictor_.set_epoch(epoch);
}

void IntegrityPlanner::set_al_fn(std::function<double(const Eigen::Vector3d&)> fn) {
  al_fn_ = std::move(fn);
}

// ----------------------------------------------------------------------------
void IntegrityPlanner::evaluate(CandidateTrajectory& traj,
                                const Eigen::Vector3d& goal,
                                double AL,
                                double w_integrity) const {
  // --- J_integrity: sum of hinge(PL_pred_k - AL_k)^2 ---
  // Use per-waypoint AL_pred when available (IAP-RQ-422); fall back to scalar AL
  double J_int = 0.0;
  for (std::size_t k = 0; k < traj.PL_pred.size(); ++k) {
    const double al_k = (k < traj.AL_pred.size()) ? traj.AL_pred[k] : AL;
    const double h = std::max(0.0, traj.PL_pred[k] - al_k);
    J_int += h * h;
  }

  // --- J_goal: Euclidean distance from last point to goal ---
  double J_goal = 0.0;
  if (!traj.points.empty()) {
    J_goal = (traj.points.back().pos - goal).norm();
  }

  // --- J_effort: mean velocity change magnitude (smoothness proxy) ---
  double J_effort = 0.0;
  if (traj.points.size() >= 2) {
    for (std::size_t i = 1; i < traj.points.size(); ++i) {
      J_effort += (traj.points[i].vel - traj.points[i - 1].vel).norm();
    }
    J_effort /= static_cast<double>(traj.points.size() - 1);
  }

  traj.J_integrity = w_integrity * J_int;
  traj.J_goal      = params_.w_mission * J_goal;
  traj.J_effort    = params_.w_smooth  * J_effort;
  traj.J_total     = traj.J_integrity + traj.J_goal + traj.J_effort;
}

// ----------------------------------------------------------------------------
CandidateTrajectory IntegrityPlanner::plan(const Eigen::Vector3d& pos0,
                                           const Eigen::Vector3d& vel0,
                                           double yaw0,
                                           const Eigen::Vector3d& goal,
                                           double sigma0,
                                           const IntegrityReport* report) const {
  // Determine current AL and effective integrity weight
  double AL = params_.al_default;
  double w_int = params_.w_integrity;

  if (report) {
    AL = report->AL;
    if (report->mode == IntegrityMode::SEARCH) {
      w_int *= params_.search_weight_multiplier;
    }
  }

  // Generate candidates (IAP-RQ-300)
  auto candidates = generator_.generate(pos0, vel0, yaw0);
  if (candidates.empty()) {
    spdlog::warn("[IntegrityPlanner] No candidates generated.");
    return {};
  }

  // Predict PL_pred for all candidates (IAP-RQ-320)
  predictor_.predict_all(candidates, sigma0);

  // Phase-4: fill AL_pred per waypoint (IAP-RQ-421) and optionally
  // replace PL_pred with ARAIM-predicted PL (IAP-RQ-331)
  for (auto& traj : candidates) {
    const int K = static_cast<int>(traj.points.size());
    traj.AL_pred.resize(K, AL);

    for (int k = 0; k < K; ++k) {
      const Eigen::Vector3d& wpt_pos = traj.points[k].pos;

      // Per-waypoint AL via user callback (IAP-RQ-421)
      if (al_fn_) {
        traj.AL_pred[k] = al_fn_(wpt_pos);
      }

      // Replace PL_pred with ARAIM prediction (IAP-RQ-331)
      if (params_.use_araim_pl) {
        traj.PL_pred[k] = araim_predictor_.predict_araim_pl(wpt_pos);
      }
    }
 }

  // Evaluate cost and find best (IAP-RQ-400)
  double best_cost = std::numeric_limits<double>::infinity();
  int    best_idx  = 0;

  for (auto& traj : candidates) {
    evaluate(traj, goal, AL, w_int);
    if (traj.J_total < best_cost) {
      best_cost = traj.J_total;
      best_idx  = traj.id;
    }
  }

  spdlog::trace(
      "[IntegrityPlanner] plan: {} candidates; best_id={} J_total={:.3f} "
      "(J_int={:.3f} J_goal={:.3f} J_eff={:.3f}) AL={:.3f} sigma0={:.4f}",
      candidates.size(), best_idx, best_cost,
      candidates[static_cast<std::size_t>(best_idx)].J_integrity,
      candidates[static_cast<std::size_t>(best_idx)].J_goal,
      candidates[static_cast<std::size_t>(best_idx)].J_effort,
      AL, sigma0);

  return candidates[static_cast<std::size_t>(best_idx)];
}

// ----------------------------------------------------------------------------
TrajectoryPoint IntegrityPlanner::execution_target(
    const CandidateTrajectory& chosen) const {
  if (chosen.points.empty()) {
    return {};
  }
  // Find first point with stamp >= dt_execute
  for (const auto& pt : chosen.points) {
    if (pt.stamp >= params_.dt_execute) {
      return pt;
    }
  }
  // Fallback: return last point
  return chosen.points.back();
}

}  // namespace iap
