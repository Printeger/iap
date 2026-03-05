// IAP-RQ-300: Candidate trajectory generator

#include <iap/planner/trajectory_generator.hpp>
#include <cmath>

namespace iap {

TrajectoryGenerator::TrajectoryGenerator() : params_(Params{}) {
  // Default grid if not set
  if (params_.speeds.empty())    params_.speeds    = {0.5, 1.0, 1.5};
  if (params_.yaw_rates.empty()) params_.yaw_rates = {-0.3, 0.0, 0.3};
  if (params_.alt_rates.empty()) params_.alt_rates = {-0.2, 0.0, 0.2};
}

TrajectoryGenerator::TrajectoryGenerator(const Params& p) : params_(p) {
  if (params_.speeds.empty())    params_.speeds    = {0.5, 1.0, 1.5};
  if (params_.yaw_rates.empty()) params_.yaw_rates = {-0.3, 0.0, 0.3};
  if (params_.alt_rates.empty()) params_.alt_rates = {-0.2, 0.0, 0.2};
}

std::vector<CandidateTrajectory> TrajectoryGenerator::generate(
    const Eigen::Vector3d& pos0,
    const Eigen::Vector3d& vel0,
    double yaw0) const {

  std::vector<CandidateTrajectory> candidates;
  int id = 0;

  const int steps = static_cast<int>(std::ceil(params_.horizon / params_.dt));

  for (double speed     : params_.speeds) {
  for (double yaw_rate  : params_.yaw_rates) {
  for (double alt_rate  : params_.alt_rates) {
    CandidateTrajectory traj;
    traj.id = id++;

    Eigen::Vector3d pos = pos0;
    double yaw = yaw0;

    TrajectoryPoint tp0;
    tp0.stamp = 0.0;
    tp0.pos   = pos;
    tp0.vel   = vel0;
    tp0.yaw   = yaw;
    traj.points.push_back(tp0);

    for (int k = 1; k <= steps; ++k) {
      const double t  = k * params_.dt;
      yaw            += yaw_rate * params_.dt;
      // Forward velocity in heading direction
      Eigen::Vector3d v(speed * std::cos(yaw), speed * std::sin(yaw), alt_rate);
      pos += v * params_.dt;

      TrajectoryPoint tp;
      tp.stamp = t;
      tp.pos   = pos;
      tp.vel   = v;
      tp.yaw   = yaw;
      traj.points.push_back(tp);
    }

    candidates.push_back(std::move(traj));
  }}}

  return candidates;
}

}  // namespace iap
