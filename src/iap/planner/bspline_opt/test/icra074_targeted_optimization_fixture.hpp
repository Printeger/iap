#ifndef BSPLINE_OPT__TEST__ICRA074_TARGETED_OPTIMIZATION_FIXTURE_HPP_
#define BSPLINE_OPT__TEST__ICRA074_TARGETED_OPTIMIZATION_FIXTURE_HPP_

#include <Eigen/Core>

#include <string_view>
#include <vector>

namespace icra074_targeted_optimization_fixture
{

inline constexpr std::string_view kName =
  "icra074_offline_two_homotopy_v1";
inline constexpr double kResolutionM = 0.5;
inline constexpr int kXCells = 20;
inline constexpr int kYCells = 16;
inline constexpr int kZCells = 4;
inline constexpr double kObstacleXMin = -1.0;
inline constexpr double kObstacleXMax = 1.0;
inline constexpr double kObstacleYMin = -0.75;
inline constexpr double kObstacleYMax = 1.25;
inline constexpr double kRiskyCost = 4.0;
inline constexpr double kSafeCost = 1.0;
inline constexpr double kFlatCost = 1.0;

enum class ProviderTruth
{
  ORDERED = 0,
  FLAT_NULL,
  INCOMPLETE,
  STALE,
  NON_FINITE,
};

inline Eigen::Vector3d start()
{
  return Eigen::Vector3d(-4.0, 0.0, 0.0);
}

inline Eigen::Vector3d goal()
{
  return Eigen::Vector3d(4.0, 0.0, 0.0);
}

inline std::vector<Eigen::Vector3d> shorterRiskyGuide()
{
  return {
    start(), Eigen::Vector3d(-2.0, -1.25, 0.0),
    Eigen::Vector3d(2.0, -1.25, 0.0), goal()};
}

inline std::vector<Eigen::Vector3d> longerSafeGuide()
{
  return {
    start(), Eigen::Vector3d(-2.0, 1.75, 0.0),
    Eigen::Vector3d(2.0, 1.75, 0.0), goal()};
}

inline std::vector<Eigen::Vector3d> symmetricSafeGuide()
{
  return {
    start(), Eigen::Vector3d(-2.0, 1.25, 0.0),
    Eigen::Vector3d(2.0, 1.25, 0.0), goal()};
}

}  // namespace icra074_targeted_optimization_fixture

#endif  // BSPLINE_OPT__TEST__ICRA074_TARGETED_OPTIMIZATION_FIXTURE_HPP_
