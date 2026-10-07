#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace iap {

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

}  // namespace iap
