#pragma once
// Phase D: local 3-D advisory PL grid for fast future integrity queries.

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#include <iap/planner/future_pl_query_result.hpp>

namespace iap {

struct PLGridCell {
  FuturePLQueryResult value;
};

class PLGrid {
 public:
  bool reset(const Eigen::Vector3d& center,
             double sx,
             double sy,
             double sz,
             double res);

  bool contains(const Eigen::Vector3d& p) const;
  bool cell_index(const Eigen::Vector3d& p, int* ix, int* iy, int* iz) const;

  PLGridCell& at(int ix, int iy, int iz);
  const PLGridCell& at(int ix, int iy, int iz) const;

  Eigen::Vector3d position(int ix, int iy, int iz) const;
  FuturePLQueryResult interpolate(const Eigen::Vector3d& p) const;
  void compute_gradients();

  bool valid() const { return valid_; }
  const Eigen::Vector3d& center() const { return center_; }
  const Eigen::Vector3d& min_corner() const { return min_corner_; }
  double resolution() const { return resolution_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  int generation() const { return generation_; }
  void set_generation(int generation) { generation_ = generation; }
  double build_time_ms() const { return build_time_ms_; }
  void set_build_time_ms(double value) { build_time_ms_ = value; }
  double stamp_s() const { return stamp_s_; }
  void set_stamp_s(double value) { stamp_s_ = value; }

 private:
  std::size_t flat_index(int ix, int iy, int iz) const;
  bool in_bounds(int ix, int iy, int iz) const;

  bool valid_ = false;
  Eigen::Vector3d center_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d min_corner_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d size_ = Eigen::Vector3d::Zero();
  double resolution_ = 1.0;
  int nx_ = 0;
  int ny_ = 0;
  int nz_ = 0;
  int generation_ = -1;
  double stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double build_time_ms_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<PLGridCell> cells_;
};

}  // namespace iap
