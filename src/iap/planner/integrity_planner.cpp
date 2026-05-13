// IAP-RQ-400: Integrity-aware planning objective
// IAP-RQ-410: Receding horizon loop
// IAP-RQ-331/421/422: advisory predicted PL proxy + per-waypoint AL
// §5.2: Cost = HPL/AL ratio hinge + D_turn + dist_to_goal + effort + infeasibility

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

void IntegrityPlanner::on_state_change(IntegrityState /*new_state*/) {
  // Could adjust internal state; currently a no-op.
}

// ----------------------------------------------------------------------------
// §5.2: Updated cost function
//   J(τ) = w_integrity · Σ_k hinge(HPL_k / AL_k − 1)²
//        + w_turn      · D_turn(τ)
//        + w_mission   · dist_to_goal
//        + w_smooth    · effort
//        + w_infeasible · I_{infeasible}
// ----------------------------------------------------------------------------
void IntegrityPlanner::evaluate(CandidateTrajectory& traj,
                                const Eigen::Vector3d& goal,
                                double AL,
                                double w_integrity) const {
  // --- J_integrity: HPL/AL ratio hinge ---
  double J_int = 0.0;
  bool   infeasible = false;
  for (std::size_t k = 0; k < traj.PL_pred.size(); ++k) {
    const double al_k = (k < traj.AL_pred.size()) ? traj.AL_pred[k] : AL;
    if (al_k <= 0.0) { infeasible = true; continue; }
    const double ratio = traj.PL_pred[k] / al_k;
    if (ratio >= 1.0) {
      infeasible = true;
      const double h = ratio - 1.0;
      J_int += h * h;
    } else if (ratio > 0.8) {
      // Soft penalty in caution zone [0.8, 1.0)
      const double h = ratio - 0.8;
      J_int += h * h;
    }
  }

  // --- J_turn: D_turn = cumulative absolute heading change (§5.2) ---
  double J_turn = 0.0;
  if (traj.points.size() >= 2) {
    for (std::size_t i = 1; i < traj.points.size(); ++i) {
      double dyaw = traj.points[i].yaw - traj.points[i - 1].yaw;
      // Normalize to [-π, π]
      while (dyaw > M_PI)  dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      J_turn += std::abs(dyaw);
    }
  }

  // --- J_goal: Euclidean distance from last point to goal ---
  double J_goal = 0.0;
  if (!traj.points.empty()) {
    J_goal = (traj.points.back().pos - goal).norm();
  }

  // --- J_effort: mean velocity change magnitude ---
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
  traj.J_total     = traj.J_integrity + traj.J_goal + traj.J_effort
                     + params_.w_turn * J_turn;

  // Infeasibility penalty: large constant added if any waypoint has PL > AL
  if (infeasible) {
    traj.J_total += params_.w_infeasible;
  }
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
    // Boost integrity weight in degraded state
    if (report->state == IntegrityState::SAFE_EXCLUDED ||
        report->state == IntegrityState::UNSAFE) {
      w_int *= params_.search_weight_multiplier;
    }
  }

  // Generate candidates (IAP-RQ-300)
  auto candidates = generator_.generate(pos0, vel0, yaw0);
  if (candidates.empty()) {
    spdlog::warn("[IntegrityPlanner] No candidates generated.");
    return {};
  }

  // Predict advisory PL_pred proxies for all candidates (IAP-RQ-320).
  predictor_.predict_all(candidates, sigma0);

  // Fill AL_pred per waypoint and optionally replace PL_pred with the
  // geometry-only GNSS advisory proxy.
  for (auto& traj : candidates) {
    const int K = static_cast<int>(traj.points.size());
    traj.AL_pred.resize(K, AL);

    for (int k = 0; k < K; ++k) {
      const Eigen::Vector3d& wpt_pos = traj.points[k].pos;

      if (al_fn_) {
        traj.AL_pred[k] = al_fn_(wpt_pos);
      }

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
  for (const auto& pt : chosen.points) {
    if (pt.stamp >= params_.dt_execute) {
      return pt;
    }
  }
  return chosen.points.back();
}

}  // namespace iap
