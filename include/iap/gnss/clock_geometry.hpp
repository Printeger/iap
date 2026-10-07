#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace iap {

inline constexpr char kGnssClockGeometryModel[] = "per_constellation_pseudorange_bias_v1";

// Measurement constellation is authoritative; wire satellite numbering is not
// a second system classifier. These IDs also index the existing fault priors.
inline int gnss_constellation_id(char system) {
  switch (system) {
    case 'G': return 0;
    case 'E': return 1;
    case 'C': return 2;
    case 'R': return 3;
    default: return -1;
  }
}

// Construct ENU/query position rows plus one bias column per actually used
// constellation, in ascending ID order. Calling this on each fault subset
// drops absent clocks structurally; no epsilon/prior repairs an empty column.
inline Eigen::MatrixXd gnss_clock_design(
    const Eigen::MatrixXd& position_rows, const std::vector<int>& systems) {
  if (position_rows.cols() != 3 || position_rows.rows() != static_cast<int>(systems.size())) return {};
  std::vector<int> active = systems;
  if (std::any_of(active.begin(), active.end(), [](int id) { return id < 0 || id > 3; })) return {};
  std::sort(active.begin(), active.end());
  active.erase(std::unique(active.begin(), active.end()), active.end());
  Eigen::MatrixXd design = Eigen::MatrixXd::Zero(position_rows.rows(), 3 + active.size());
  design.leftCols(3) = position_rows;
  for (int row = 0; row < position_rows.rows(); ++row) {
    const auto col = std::lower_bound(active.begin(), active.end(), systems[row]) - active.begin();
    design(row, 3 + col) = 1.0;
  }
  return design;
}

inline bool gnss_eliminate_active_clocks(
    const Eigen::MatrixXd& normal, double clock_epsilon,
    Eigen::Matrix3d* position_information) {
  if (!position_information || !std::isfinite(clock_epsilon) || clock_epsilon <= 0 || normal.rows() != normal.cols() ||
      normal.cols() < 4 || !normal.allFinite()) return false;
  Eigen::Matrix3d result = normal.topLeftCorner<3,3>();
  for (int col = 3; col < normal.cols(); ++col) {
    const double information = normal(col,col);
    if (information <= clock_epsilon) return false;
    // Indicator columns are disjoint. Off-diagonal clock information would
    // represent a different model and cannot use this independent-bias solve.
    for (int other = 3; other < normal.cols(); ++other) {
      if (other != col && normal(col,other) != 0) return false;
    }
    const Eigen::Vector3d cross = normal.block<3,1>(0,col);
    result -= cross * cross.transpose() / information;
  }
  *position_information = 0.5 * (result + result.transpose());
  return position_information->allFinite();
}

}  // namespace iap
