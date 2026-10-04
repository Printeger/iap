#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>

// The route test uses the real A* lattice while supplying a deterministic
// planning query. Predictor and sensor behavior are covered at their seams.
struct GridMapTestAccess {
  static void configure(GridMap& map) {
    map.mp_.map_origin_ = Eigen::Vector3d(-5, -5, -1);
    map.mp_.map_size_ = Eigen::Vector3d(10, 10, 5);
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    map.mp_.map_voxel_num_ = Eigen::Vector3i(100, 100, 50);
    map.mp_.resolution_ = 0.1;
    map.mp_.resolution_inv_ = 10.0;
  }
};

namespace {
GridPlanningCell cellFor(const Eigen::Vector3d& p) {
  GridPlanningCell cell;
  cell.execution_reason = GridExecutionReason::OK;
  cell.advisory.classification =
      std::abs(p.x()) < 0.4 && std::abs(p.y()) < 2.0
          ? GridAdvisoryClass::AVOID : GridAdvisoryClass::VALID;
  cell.advisory.cost_multiplier = 1.0;
  return cell;
}
}  // namespace

TEST(AdvisoryAStar, TakesLongerRouteAroundPredictedBand) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(80, 80, 10));
  search.setPlanningQuery(cellFor);
  ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(-2, 0, 1),
                                Eigen::Vector3d(2, 0, 1)));
  const auto route = search.getPath();
  ASSERT_GT(route.size(), 2u);
  double length = 0.0;
  for (size_t i = 1; i < route.size(); ++i) {
    length += (route[i] - route[i - 1]).norm();
    EXPECT_NE(cellFor(route[i]).advisory.classification,
              GridAdvisoryClass::AVOID);
  }
  EXPECT_GT(length, 5.2);  // > 1.30 times the 4 m direct route.
}

TEST(AdvisoryAStar, StartInsideWarningUsesSameHighCostSearch) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(80, 80, 10));
  search.setPlanningQuery(cellFor, true);
  ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(0, 0, 1),
                                Eigen::Vector3d(2, 0, 1)));
  EXPECT_FALSE(search.getPath().empty());
}
