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
  static void nextGeneration(GridMap& map) {
    map.occupancy_update_sequence_.fetch_add(2);
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

TEST(AdvisoryAStar, UnobservedEndpointDoesNotWalkOutOfPool) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(100, 100, 20));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = p.x() > 1.0
        ? GridExecutionReason::ENVIRONMENT_UNOBSERVED
        : GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                  Eigen::Vector3d(2, 0, 1)));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::END_UNOBSERVED);
  EXPECT_EQ(search.lastResult().expanded, 0u);
}

TEST(AdvisoryAStar, PhysicalEndpointCannotBeSilentlyMoved) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(100, 100, 20));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = p.x() > 0.8
        ? GridExecutionReason::INSUFFICIENT_CLEARANCE
        : GridExecutionReason::OK;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                  Eigen::Vector3d(2, 0, 1)));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::END_BLOCKED);
  EXPECT_EQ(search.lastResult().expanded, 0u);
}

TEST(AdvisoryAStar, OutOfMapTargetKeepsDistinctReason) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(100, 100, 20));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = p.x() > 1.0
        ? GridExecutionReason::OUT_OF_MAP : GridExecutionReason::OK;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                  Eigen::Vector3d(2, 0, 1)));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::END_OUT_OF_MAP);
  EXPECT_EQ(search.lastResult().expanded, 0u);
}

TEST(AdvisoryAStar, RoundedEndpointNeedsAnExecutableConnector) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(100, 100, 20));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = p.x() > 0.99 && p.x() < 1.03
        ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1.0, 0, 1),
                                  Eigen::Vector3d(1.04, 0, 1)));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::END_BLOCKED);
}

TEST(AdvisoryAStar, MidpointRejectionBendsOtherwiseFreeEdge) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(20, 20, 10));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = std::abs(p.x() - 0.15) < 1e-6 &&
        std::abs(p.y()) < 1e-6
        ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(0, 0, 1),
                                 Eigen::Vector3d(0.4, 0, 1)));
  const auto route = search.getPath();
  ASSERT_GE(route.size(), 3u);
  double length = 0.0;
  for (size_t i = 1; i < route.size(); ++i)
    length += (route[i] - route[i - 1]).norm();
  EXPECT_GT(length, 0.4);
}

TEST(AdvisoryAStar, UnobservedBarrierHasItsOwnFailureReason) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(30, 30, 10));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = std::abs(p.x()) < 0.11
        ? GridExecutionReason::ENVIRONMENT_UNOBSERVED
        : GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                  Eigen::Vector3d(1, 0, 1)));
  EXPECT_EQ(search.lastResult().failure,
            AStar::Failure::NO_PATH_WITH_UNOBSERVED);
  EXPECT_GT(search.lastResult().rejected_execution[
      static_cast<size_t>(GridExecutionReason::ENVIRONMENT_UNOBSERVED)], 0u);
}

TEST(AdvisoryAStar, ReusesPhysicalAndRiskQueriesWithinOneSearch) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(80, 80, 10));
  size_t calls = 0;
  search.setPlanningQuery([&calls](const Eigen::Vector3d&) {
    ++calls;
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.classification = GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                Eigen::Vector3d(1, 0, 1)));
  EXPECT_GT(search.lastResult().cache_hits, 0u);
  EXPECT_LT(calls, search.lastResult().query_calls +
                   search.lastResult().cache_hits + 3u);
}

TEST(AdvisoryAStar, MapChangeEndsSearchBeforeUsingPreviousCachedCells) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(80, 80, 10));
  size_t calls = 0;
  search.setPlanningQuery([&](const Eigen::Vector3d&) {
    if (++calls == 12) GridMapTestAccess::nextGeneration(*map);
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                  Eigen::Vector3d(1, 0, 1)));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::MAP_STALE);
  EXPECT_TRUE(search.lastResult().map_changed);
}

TEST(AdvisoryAStar, FrozenSearchCompletesWhileLiveMapAdvances) {
  auto live = std::make_shared<GridMap>();
  auto frozen = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*live);
  GridMapTestAccess::configure(*frozen);
  AStar search;
  search.initGridMap(frozen, Eigen::Vector3i(80, 80, 20));
  search.setLiveGenerationProvider([live] {
    return live->occupancyGeneration();
  });
  int calls = 0;
  search.setPlanningQuery([&](const Eigen::Vector3d&) {
    if (++calls == 12) GridMapTestAccess::nextGeneration(*live);
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  EXPECT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                 Eigen::Vector3d(1, 0, 1)));
  EXPECT_GT(calls, 12);
  EXPECT_NE(live->occupancyGeneration(), frozen->occupancyGeneration());
  EXPECT_TRUE(search.lastResult().map_changed);
  EXPECT_EQ(search.lastResult().live_generation_at_finish, live->occupancyGeneration());
}

TEST(AdvisoryAStar, TimeoutIsNotReportedAsNoPath) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(100, 100, 20));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-2, 0, 1),
                                  Eigen::Vector3d(2, 0, 1), 0.0));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::TIME_BUDGET);
}

TEST(AdvisoryAStar, LiveUpdateDoesNotOverwriteFrozenSearchTimeout) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(100, 100, 20));
  search.setLiveGenerationProvider([map] {
    return map->occupancyGeneration() + 1;
  });
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-2, 0, 1),
                                  Eigen::Vector3d(2, 0, 1), 0.0));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::TIME_BUDGET);
  EXPECT_TRUE(search.lastResult().map_changed);
  EXPECT_EQ(search.lastResult().live_generation_at_finish,
            search.lastResult().occupancy_generation + 1);
}
