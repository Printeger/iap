// IAP-RQ-320: Covariance propagation + PL_pred
// IAP-RQ-310: Visibility proxy (placeholder)

#include <iap/planner/predicted_integrity.hpp>
#include <cmath>

namespace iap {

PredictedIntegrityComputer::PredictedIntegrityComputer() : params_(Params{}) {}
PredictedIntegrityComputer::PredictedIntegrityComputer(const Params& p) : params_(p) {}

void PredictedIntegrityComputer::predict(CandidateTrajectory& traj, double sigma0) const {
  const int N = static_cast<int>(traj.points.size());
  traj.PL_pred.resize(N);
  traj.sigma_pred.resize(N);

  double sigma = sigma0;

  for (int k = 0; k < N; ++k) {
    traj.sigma_pred[k] = sigma;
    traj.PL_pred[k]    = params_.K_pl * sigma;

    // Propagate: sigma^2 grows by sigma_grow^2 * dt
    if (k + 1 < N) {
      const double dt = traj.points[k + 1].stamp - traj.points[k].stamp;
      const double new_var = sigma * sigma + params_.sigma_grow * params_.sigma_grow * dt;
      sigma = (new_var > 0.0) ? std::sqrt(new_var) : 0.0;
    }
  }
}

void PredictedIntegrityComputer::predict_all(
    std::vector<CandidateTrajectory>& trajs, double sigma0) const {
  for (auto& t : trajs) {
    predict(t, sigma0);
  }
}

}  // namespace iap
