// IAP-RQ-320: Covariance propagation + advisory PL_pred proxy
// IAP-RQ-321: Trajectory-dependent advisory PL_pred via visibility predictor

#include <iap/planner/predicted_integrity.hpp>
#include <algorithm>
#include <cmath>

namespace iap {

PredictedIntegrityComputer::PredictedIntegrityComputer()
: params_(Params{}), vis_predictor_(params_.vis_params) {}

PredictedIntegrityComputer::PredictedIntegrityComputer(const Params& p)
: params_(p), vis_predictor_(p.vis_params) {}

void PredictedIntegrityComputer::set_occupancy(const LocalOccupancyGrid* grid) {
  grid_ = grid;
  vis_predictor_.set_occupancy(grid_);
}

void PredictedIntegrityComputer::set_epoch(const GnssEpoch* epoch) {
  epoch_ = epoch;
}

// ---------------------------------------------------------------------------
double PredictedIntegrityComputer::sigma_grow_at(const Eigen::Vector3d& pos) const {
  // Baseline: constant sigma_grow (no visibility info)
  if (grid_ == nullptr || epoch_ == nullptr) {
    return params_.sigma_grow;
  }

  // IAP-RQ-321: scale sigma_grow by f(n_vis, mean_kappa)
  const VisibilityResult vis = vis_predictor_.predict(pos, *epoch_);
  const double n_vis_nom = static_cast<double>(params_.n_vis_nominal);
  const double deficit = std::max(0.0, n_vis_nom - static_cast<double>(vis.n_vis)) / n_vis_nom;
  const double f = 1.0 + params_.beta_vis * deficit + params_.gamma_kappa * vis.mean_kappa;
  return params_.sigma_grow * std::max(1.0, f);
}

// ---------------------------------------------------------------------------
void PredictedIntegrityComputer::predict(CandidateTrajectory& traj,
                                          double sigma0) const {
  const int N = static_cast<int>(traj.points.size());
  traj.PL_pred.resize(N);
  traj.sigma_pred.resize(N);

  double sigma = sigma0;

  for (int k = 0; k < N; ++k) {
    traj.sigma_pred[k] = sigma;
    traj.PL_pred[k]    = params_.K_pl * sigma;

    if (k + 1 < N) {
      const double dt = traj.points[k + 1].stamp - traj.points[k].stamp;
      const double sg = sigma_grow_at(traj.points[k].pos);
      const double new_var = sigma * sigma + sg * sg * dt;
      sigma = (new_var > 0.0) ? std::sqrt(new_var) : 0.0;
    }
  }
}

// ---------------------------------------------------------------------------
void PredictedIntegrityComputer::predict_all(
    std::vector<CandidateTrajectory>& trajs, double sigma0) const {
  for (auto& t : trajs) {
    predict(t, sigma0);
  }
}

}  // namespace iap
