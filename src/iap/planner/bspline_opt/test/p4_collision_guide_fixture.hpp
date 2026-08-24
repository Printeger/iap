#ifndef BSPLINE_OPT__TEST__P4_COLLISION_GUIDE_FIXTURE_HPP_
#define BSPLINE_OPT__TEST__P4_COLLISION_GUIDE_FIXTURE_HPP_

#include <Eigen/Core>

#include <string_view>
#include <vector>

namespace p4_collision_guide_fixture
{

inline constexpr std::string_view kName = "p4_collision_guide_v1";
inline constexpr double kObstacleXMin = -1.0;
inline constexpr double kObstacleXMax = 1.0;
inline constexpr double kObstacleYMin = -1.0;
inline constexpr double kObstacleYMax = 1.0;
inline constexpr double kHighCorridorCost = 20.0;
inline constexpr double kLowCorridorCost = 1.0;

inline std::vector<Eigen::Vector3d> originalGuide()
{
  return {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(-2.0, -2.0, 0.0),
    Eigen::Vector3d(2.0, -2.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
}

inline std::vector<Eigen::Vector3d> riskGuide()
{
  return {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(-2.0, 2.0, 0.0),
    Eigen::Vector3d(2.0, 2.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
}

}  // namespace p4_collision_guide_fixture

#endif  // BSPLINE_OPT__TEST__P4_COLLISION_GUIDE_FIXTURE_HPP_
