#pragma once
#include <ego_planner/prediction_input.h>
#include <array>
#include <deque>
namespace ego_planner {
struct SlicePoint { Eigen::Vector3d center; GridRiskVoxel risk; };
struct RiskDisplayFrame {
  std::vector<SlicePoint> samples;
  std::array<int,100> lattice;
  std::array<bool,81> surfaces{};
  Eigen::Vector3d vehicle=Eigen::Vector3d::Zero();
  std::string frame_id, geometry_id, reason;
  uint64_t generation=0;
  double reference_time_s=0, valid_until_s=0, expires_at_s=0;
  double preparation_s=0, query_s=0, budget_s=.02, overrun_s=0, preparation_overrun_s=0, scalar_overrun_s=0;
  uint64_t predictor_calls=0;
  bool incomplete=false;
  int id=0;
  RiskDisplayFrame() { lattice.fill(-1); }
  bool current(double now, uint64_t latest_generation) const {
    return generation==latest_generation && now>=reference_time_s && now<=valid_until_s;
  }
};
inline double timingP95(const std::deque<double>& values) {
  if (values.empty()) return 0;
  std::vector<double> sorted(values.begin(),values.end()); std::sort(sorted.begin(),sorted.end());
  return sorted[static_cast<size_t>(std::ceil(.95*sorted.size()))-1];
}
inline double visualizationBudget(const std::deque<double>& preparation,
                                  const std::deque<double>& points) {
  return std::clamp(timingP95(preparation)+100*timingP95(points),.02,.2);
}
// Every voxel touched by the horizontal quad must have native observation
// evidence and no raw/inflate mark. This covers holes between the four samples.
inline bool observedSurfaceRectangle(const FrozenOccupancyEpoch& epoch,
                                     const Eigen::Vector3d& a, const Eigen::Vector3d& d) {
  const auto first=GridMap::queryFrozenOccupancy(epoch,a);
  const auto last=GridMap::queryFrozenOccupancy(epoch,d);
  if (!first.available || !last.available || first.voxel_index.z()!=last.voxel_index.z()) return false;
  for (int x=std::min(first.voxel_index.x(),last.voxel_index.x());x<=std::max(first.voxel_index.x(),last.voxel_index.x());++x)
    for (int y=std::min(first.voxel_index.y(),last.voxel_index.y());y<=std::max(first.voxel_index.y(),last.voxel_index.y());++y) {
      const int address=(x*epoch.voxel_dimensions.y()+y)*epoch.voxel_dimensions.z()+first.voxel_index.z();
      const auto flags=epoch.cells->at(address);
      if (!(flags&4) || (flags&3)) return false;
    }
  return true;
}
uint64_t predictionInputIdentity(const PredictionInput& input);
}
