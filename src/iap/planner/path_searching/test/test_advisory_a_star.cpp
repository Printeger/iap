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

TEST(AdvisoryAStar, OneSearchReachesAnotherGoalWhenPreferredGoalIsDisconnected) {
  auto map=std::make_shared<GridMap>(); GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map,Eigen::Vector3i(60,60,10));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason=std::abs(p.x())<.15 ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
    cell.advisory.classification=GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier=1;
    return cell;
  });
  const std::vector<Eigen::Vector3d> goals={Eigen::Vector3d(1,0,1),Eigen::Vector3d(-.5,1,1)};
  ASSERT_TRUE(search.AstarSearchGoals(.1,Eigen::Vector3d(-1,0,1),goals));
  EXPECT_EQ(search.lastResult().selected_goal,1u);
  EXPECT_TRUE(search.getPath().back().isApprox(goals[1],1e-9));
  for(const auto& point:search.getPath()) EXPECT_LT(point.x(),-.15);
}

TEST(AdvisoryAStar, TimeoutWithAdvisoryRejectionsIsNotExhaustion) {
  auto map=std::make_shared<GridMap>(); GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map,Eigen::Vector3i(40,40,10));
  search.setPlanningQuery(cellFor);
  EXPECT_FALSE(search.AstarSearchGoals(.1,Eigen::Vector3d(-1,0,1),{Eigen::Vector3d(1,0,1)},0));
  EXPECT_EQ(search.lastResult().failure,AStar::Failure::TIME_BUDGET);
  EXPECT_FALSE(search.lastResult().exhausted);
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

TEST(AdvisoryAStar, RejectedIncomingEdgeDoesNotDiscoverItsNeighbor) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(20, 20, 10));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.advisory.cost_multiplier = 1.0;
    cell.advisory.classification = GridAdvisoryClass::VALID;
    const bool lattice = std::abs(p.z() * 10 - std::round(p.z() * 10)) < 1e-8 &&
        std::abs(p.x() * 10 - std::round(p.x() * 10)) < 1e-8 &&
        std::abs(p.y() * 10 - std::round(p.y() * 10)) < 1e-8;
    const bool allowed_node =
        p.isApprox(Eigen::Vector3d(0, 0, 1), 1e-8) ||
        p.isApprox(Eigen::Vector3d(0.1, 0.1, 1), 1e-8) ||
        p.isApprox(Eigen::Vector3d(0.1, 0, 1), 1e-8) ||
        p.isApprox(Eigen::Vector3d(0.2, 0, 1), 1e-8);
    const bool blocked_midpoint =
        p.isApprox(Eigen::Vector3d(0.05, 0, 1), 1e-8) ||
        p.isApprox(Eigen::Vector3d(0.15, 0.05, 1), 1e-8);
    cell.execution_reason = (lattice && !allowed_node) || blocked_midpoint
        ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
    return cell;
  });
  // A->B is blocked at its midpoint. A->C->B->goal is executable. Neither B
  // nor goal can inherit old scores/state when their first incoming edge fails.
  for (int attempt = 0; attempt < 3; ++attempt) {
    ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0.2, 0, 1), 1.0, Eigen::Vector3d(0, 0, 1)));
    const auto route = search.getPath();
    EXPECT_GE(route.size(), 4u);
    EXPECT_EQ(search.lastResult().failure, AStar::Failure::NONE);
  }
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

TEST(AdvisoryAStar, NegativePoolCoordinatesUseNearestLattice) {
  auto map=std::make_shared<GridMap>(); GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map,Eigen::Vector3i(100,100,40));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell; cell.execution_reason=GridExecutionReason::OK; return cell;
  });
  const Eigen::Vector3d start(-1.06,-1.06,1.06);
  ASSERT_TRUE(search.AstarSearch(0.1,start,Eigen::Vector3d(1,1,1),1.0,Eigen::Vector3d(0,0,1)));
  const auto path=search.getPath(); ASSERT_GE(path.size(),3u);
  EXPECT_TRUE(path.front().isApprox(start));
  EXPECT_TRUE(path[1].isApprox(Eigen::Vector3d(-1.1,-1.1,1.1),1e-9));
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

TEST(AdvisoryAStar, InteriorVoxelsOnLongDiagonalCannotBeSkipped) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(20, 20, 10));
  bool examined_interior = false;
  search.setPlanningQuery([&](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    // This narrow interior voxel is neither an endpoint nor the midpoint of
    // the direct 0.4 m diagonal search edge.
    const bool obstacle = p.x() > 0.10 && p.x() < 0.20 &&
        p.y() > 0.10 && p.y() < 0.20;
    examined_interior = examined_interior || obstacle;
    cell.execution_reason = obstacle ? GridExecutionReason::PHYSICAL_OBSTACLE
                                     : GridExecutionReason::OK;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  ASSERT_TRUE(search.AstarSearch(0.4, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0.8, 0.8, 1), 1.0, Eigen::Vector3d(0, 0, 1)));
  EXPECT_TRUE(examined_interior);
  const auto route = search.getPath();
  double length = 0;
  for (size_t i = 1; i < route.size(); ++i) length += (route[i]-route[i-1]).norm();
  EXPECT_GT(length, std::sqrt(1.28));
}

TEST(AdvisoryAStar, DiagnosticsToggleAndRoundCacheKeepIdenticalRouteAndCost) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(80, 80, 10));
  search.setPlanningQuery(cellFor);
  std::vector<Eigen::Vector3d> reference;
  double cost = 0;
  for (int round = 0; round < 3; ++round) {
    search.setPerformanceDiagnostics(round == 1);
    ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(-2, 0, 1),
                                  Eigen::Vector3d(2, 0, 1)));
    const auto route = search.getPath();
    if (round == 0) { reference = route; cost = search.lastResult().path_cost; }
    else {
      ASSERT_EQ(route.size(), reference.size());
      for (size_t i = 0; i < route.size(); ++i) EXPECT_EQ(route[i], reference[i]);
      EXPECT_DOUBLE_EQ(cost, search.lastResult().path_cost);
    }
    EXPECT_GT(search.lastResult().sample_hits[0], 0u);
    EXPECT_GT(search.lastResult().sample_hits[1], 0u);
    EXPECT_GT(search.lastResult().sample_hits[2], 0u);
  }
}

TEST(AdvisoryAStar, CachedPhysicalSampleRefreshesAdvisory) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(40, 40, 10));
  size_t refreshes = 0;
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.classification = GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  search.setAdvisoryQuery([&](const Eigen::Vector3d&) {
    ++refreshes;
    GridPlanningRisk risk;
    risk.classification = GridAdvisoryClass::UNKNOWN;
    risk.cost_multiplier = 1.5;
    return risk;
  });
  ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
                                Eigen::Vector3d(1, 0, 1)));
  EXPECT_GT(refreshes, 0u);
  EXPECT_EQ(refreshes, search.lastResult().advisory_refresh_calls +
      search.lastResult().risk_integration_calls);
  EXPECT_GT(search.lastResult().path_cost, 2.0); // metres, including unknown-risk integral
}

TEST(AdvisoryAStar, RealGridRiskCacheRevokesExpiredAndReboundVersionsDuringSearch) {
  GridMapFailureSnapshot snapshot;
  snapshot.origin = Eigen::Vector3d(-2, -2, 0);
  snapshot.max_boundary = Eigen::Vector3d(2, 2, 2);
  snapshot.dimensions = Eigen::Vector3i(40, 40, 20);
  snapshot.resolution_m = 0.1;
  snapshot.cloud_stamp_s = 10;
  snapshot.generation = 1;
  snapshot.frame_id = "map";
  snapshot.cell_flags.assign(40 * 40 * 20, 4);
  auto map = GridMap::fromFailureSnapshot(snapshot);
  GridRiskContext risk_context;
  risk_context.frame_id = "map";
  risk_context.occupancy_generation = map->occupancyGeneration();
  risk_context.reference_time_s = 10;
  risk_context.valid_until_s = 10.5;
  risk_context.reference_position = Eigen::Vector3d::Zero();
  risk_context.predict = [](const Eigen::Vector3d&) {
    GridRiskVoxel risk;
    risk.status = GridRiskStatus::VALID;
    risk.hpl = risk.vpl = 0.1;
    return risk;
  };
  const auto version = map->bindRiskContext(risk_context);
  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 10;
  motion.error_proxy_m = 0.01;
  const auto physical = map->preparePlanningQuery(10, motion);
  GridPlanningRiskPolicy policy;
  size_t queries = 0, stale_hits = 0, changed_hits = 0;
  double now = 10;
  auto risk_query = [&](const Eigen::Vector3d& p, bool cached) {
    if (++queries == 80) now = 10.75;
    if (queries == 160) map->bindRiskContext(risk_context);
    const auto risk = map->queryPlanningRisk(p, version, now, policy);
    if (cached && queries > 80 && queries < 160) {
      EXPECT_NE(risk.classification, GridAdvisoryClass::VALID);
      if (risk.classification == GridAdvisoryClass::STALE_REFERENCE) ++stale_hits;
    }
    if (cached && queries >= 160) {
      EXPECT_EQ(risk.query_status, GridRiskStatus::VERSION_CHANGED);
      EXPECT_EQ(risk.classification, GridAdvisoryClass::UNKNOWN);
      ++changed_hits;
    }
    return risk;
  };
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(30, 30, 10));
  search.setPlanningQuery([&](const Eigen::Vector3d& p) {
    auto cell = map->queryPlanningCell(p, 0, 10, policy, motion, false, &physical);
    if (cell.executable()) cell.advisory = risk_query(p, false);
    return cell;
  });
  search.setAdvisoryQuery([&](const Eigen::Vector3d& p) { return risk_query(p, true); });
  ASSERT_TRUE(search.AstarSearch(0.1, Eigen::Vector3d(-0.8, 0, 1),
      Eigen::Vector3d(0.8, 0, 1)));
  EXPECT_GT(stale_hits, 0u);
  EXPECT_GT(changed_hits, 0u);
}

TEST(AdvisoryAStar, AdvisoryRejectionAfterCacheRefreshKeepsFirstReasonAndPoint) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(30, 30, 10));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.classification = GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier = 1;
    return cell;
  });
  search.setAdvisoryQuery([](const Eigen::Vector3d&) {
    GridPlanningRisk risk;
    risk.classification = GridAdvisoryClass::AVOID;
    risk.cost_multiplier = 1;
    return risk;
  });
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-1, 0, 1),
      Eigen::Vector3d(1, 0, 1)));
  const auto& result = search.lastResult();
  EXPECT_EQ(result.failure, AStar::Failure::ADVISORY_NO_PATH);
  ASSERT_TRUE(result.has_first_rejection);
  EXPECT_EQ(result.first_rejection_cell.execution_reason, GridExecutionReason::OK);
  EXPECT_EQ(result.first_rejection_cell.advisory_class, GridAdvisoryClass::AVOID);
  EXPECT_TRUE(result.first_rejection_position.allFinite());
}

TEST(AdvisoryAStar, LegalRealStartConnectsAroundBlockedRoundedNode) {
  auto map=std::make_shared<GridMap>(); GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map,Eigen::Vector3i(40,40,20));
  const Eigen::Vector3d start(-1.04,0,1), goal(1,0,1);
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell c; c.execution_reason=p.x()>-1.025 && p.x()<-0.975 && std::abs(p.y())<.025
        ? GridExecutionReason::INSUFFICIENT_CLEARANCE : GridExecutionReason::OK;
    c.advisory.classification=GridAdvisoryClass::VALID; c.advisory.cost_multiplier=1.; return c;
  });
  ASSERT_TRUE(search.AstarSearch(.1,start,goal,1.,Eigen::Vector3d(0,0,1)));
  EXPECT_TRUE(search.getPath().front().isApprox(start,1e-10));
}

TEST(AdvisoryAStar, StartRecoveryCannotCrossUnknownConnector) {
  auto map=std::make_shared<GridMap>(); GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map,Eigen::Vector3i(40,40,20));
  const Eigen::Vector3d start(-1.04,0,1);
  search.setPlanningQuery([start](const Eigen::Vector3d& p) {
    GridPlanningCell c; c.execution_reason=(p-start).norm()<.005 || (p-start).norm()>.9
        ? GridExecutionReason::OK : GridExecutionReason::ENVIRONMENT_UNOBSERVED;
    c.advisory.classification=GridAdvisoryClass::VALID; return c;
  });
  EXPECT_FALSE(search.AstarSearch(.1,start,Eigen::Vector3d(1,0,1),1.,Eigen::Vector3d(0,0,1)));
  EXPECT_EQ(search.lastResult().failure,AStar::Failure::START_BLOCKED);
}

TEST(AdvisoryAStar, MultiGoalEvidenceUsesSelectedEndpointLattice) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map);
  AStar search;search.initGridMap(map,Eigen::Vector3i(60,60,10));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;cell.execution_reason=GridExecutionReason::OK;
    cell.advisory.classification=GridAdvisoryClass::UNKNOWN;cell.advisory.cost_multiplier=1.;return cell;
  });
  const std::vector<Eigen::Vector3d> goals={Eigen::Vector3d(-.5,0,1),Eigen::Vector3d(1,0,1)};
  ASSERT_TRUE(search.AstarSearchGoals(.1,Eigen::Vector3d(-1,0,1),goals,1.,Eigen::Vector3d(0,0,1)));
  ASSERT_EQ(search.lastResult().selected_goal,0u);
  EXPECT_TRUE(search.lastResult().end_lattice.isApprox(search.getPath().back(),1e-12));
}

TEST(AdvisoryAStar, CostIsMetricAcrossSearchStepSizes) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map);
  AStar search;search.initGridMap(map,Eigen::Vector3i(60,60,10));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;cell.execution_reason=GridExecutionReason::OK;
    cell.advisory.classification=GridAdvisoryClass::VALID;cell.advisory.cost_multiplier=1.2;return cell;
  });
  for(double step:{.1,.2}) {
    ASSERT_TRUE(search.AstarSearch(step,Eigen::Vector3d(-1,0,1),Eigen::Vector3d(1,0,1),1.,Eigen::Vector3d(0,0,1)));
    EXPECT_NEAR(search.lastResult().path_cost,2.4,1e-9);
  }
}

TEST(AdvisoryAStar, CompleteCostIncludesRealStartAndTerminalConnectors) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map);
  AStar search;search.initGridMap(map,Eigen::Vector3i(60,60,10));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;cell.execution_reason=GridExecutionReason::OK;
    cell.advisory.classification=GridAdvisoryClass::VALID;cell.advisory.cost_multiplier=1.3;return cell;
  });
  ASSERT_TRUE(search.AstarSearch(.1,Eigen::Vector3d(-.98,0,1),Eigen::Vector3d(1.03,0,1),1.,Eigen::Vector3d(0,0,1)));
  const auto route=search.getPath();double length=0.;
  for(size_t i=1;i<route.size();++i)length+=(route[i]-route[i-1]).norm();
  EXPECT_NEAR(search.lastResult().path_cost,1.3*length,1e-9);
}

TEST(AdvisoryAStar, RemainingTaskTermSelectsCompleteObjectiveOnce) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map);
  AStar search;search.initGridMap(map,Eigen::Vector3i(60,60,10));
  search.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell c;c.execution_reason=GridExecutionReason::OK;
    c.advisory.classification=GridAdvisoryClass::VALID;c.advisory.cost_multiplier=1.;return c;
  });
  search.setTaskGoal(Eigen::Vector3d(3,0,1));
  ASSERT_TRUE(search.AstarSearchGoals(.1,Eigen::Vector3d(0,0,1),
      {Eigen::Vector3d(-.5,0,1),Eigen::Vector3d(1.5,0,1)},1.,Eigen::Vector3d(0,0,1)));
  EXPECT_EQ(search.lastResult().selected_goal,1u);
  EXPECT_NEAR(search.lastResult().path_cost,3.,1e-9);
  EXPECT_NEAR(search.lastResult().terminal_cost_m,1.5,1e-9);
  EXPECT_NEAR(search.lastResult().path_length_m,1.5,1e-9);
  EXPECT_TRUE(search.lastResult().optimality_proven);
}

TEST(AdvisoryAStar, DeadlineReturnsCompleteIncumbentWithoutOptimalityClaim) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map);
  AStar search;search.initGridMap(map,Eigen::Vector3i(60,60,10));
  search.setPlanningBudget(std::make_shared<PlanningBudget>(1.5));
  search.setTaskGoal(Eigen::Vector3d(2,0,1));
  search.setPlanningQuery([&](const Eigen::Vector3d&) {
    // Reproduce resource exhaustion only after a complete goal was checked.
    if(search.lastResult().selected_goal!=std::numeric_limits<size_t>::max())
      search.setPlanningBudget(std::make_shared<PlanningBudget>(0));
    GridPlanningCell c;c.execution_reason=GridExecutionReason::OK;
    c.advisory.classification=GridAdvisoryClass::UNKNOWN;c.advisory.cost_multiplier=1.5;return c;
  });
  ASSERT_TRUE(search.AstarSearchGoals(.1,Eigen::Vector3d(0,0,1),
      {Eigen::Vector3d(.2,0,1),Eigen::Vector3d(2,0,1)},1.,Eigen::Vector3d(0,0,1)));
  EXPECT_FALSE(search.lastResult().optimality_proven);
  EXPECT_TRUE(search.lastResult().search_budget_exhausted);
  EXPECT_FALSE(search.lastResult().map_changed);
  EXPECT_EQ(search.lastResult().selected_goal,0u);
  EXPECT_NEAR(search.lastResult().path_cost,2.1,1e-9);
  EXPECT_TRUE(search.getPath().back().isApprox(Eigen::Vector3d(.2,0,1),1e-9));
}

TEST(AdvisoryAStar, BelowWarningPreferenceChoosesLowerRiskRouteWithIdenticalPhysics) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map, Eigen::Vector3i(70, 40, 10));
  const Eigen::Vector3d start(-2, 0, 1), goal(2, 0, 1);
  const auto query = [](bool guidance, const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = std::abs(p.z() - 1) > .06 || std::abs(p.y()) > 1.
        ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
    cell.advisory.classification = GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier = !guidance ? 1. :
        std::abs(p.x()) < 1.5 && std::abs(p.y()) < .3 ? 1.49 : 1.09;
    return cell;
  };
  search.setPlanningQuery([&](const Eigen::Vector3d& p) { return query(false, p); });
  ASSERT_TRUE(search.AstarSearch(.1, start, goal));
  const auto off = search.getPath();
  for (const auto& p : off) EXPECT_NEAR(p.y(), 0., 1e-9);
  search.setPlanningQuery([&](const Eigen::Vector3d& p) { return query(true, p); });
  ASSERT_TRUE(search.AstarSearch(.1, start, goal));
  const auto on = search.getPath();
  double lateral = 0.;
  for (const auto& p : on) {
    EXPECT_TRUE(query(true, p).executable());
    EXPECT_EQ(query(true, p).advisory.classification, GridAdvisoryClass::VALID);
    lateral = std::max(lateral, std::abs(p.y()));
  }
  EXPECT_GE(lateral, .3 - 1e-9);
  EXPECT_GT(search.lastResult().path_length_m, 4.);
  EXPECT_LT(search.lastResult().path_cost, 4. * 1.49);
  EXPECT_TRUE(search.lastResult().optimality_proven);
}

TEST(AdvisoryAStar, SharedGoalVoxelMatchesIndependentOneDimensionalCost) {
  auto map = std::make_shared<GridMap>(); GridMapTestAccess::configure(*map);
  AStar search; search.initGridMap(map, Eigen::Vector3i(40, 20, 10));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = std::abs(p.y()) > .06 || std::abs(p.z() - 1) > .06
        ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
    cell.advisory.classification = GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier = p.x() < .3 ? 1.1 : 1.4;
    return cell;
  });
  const Eigen::Vector3d start(-.03, 0, 1), task(2, 0, 1);
  search.setTaskGoal(task);
  ASSERT_TRUE(search.AstarSearchGoals(.1, start,
      {Eigen::Vector3d(.24, 0, 1), Eigen::Vector3d(.23, 0, 1), Eigen::Vector3d(.8, 0, 1)},
      1., Eigen::Vector3d(0, 0, 1)));
  // In this one-dimensional free component every walk's extra distance costs
  // at least 1.1/m. Enumerating the three endpoint costs gives the shorter of
  // the two connectors in the shared .2m goal voxel as the unique minimum.
  EXPECT_EQ(search.lastResult().selected_goal, 1u);
  EXPECT_NEAR(search.lastResult().path_cost, (.23 + .03) * 1.1 + (2. - .23), 1e-12);
  EXPECT_NEAR(search.lastResult().path_length_m, .26, 1e-12);
  EXPECT_NEAR(search.lastResult().terminal_cost_m, 1.77, 1e-12);
  EXPECT_TRUE(search.lastResult().optimality_proven);
}


TEST(AdvisoryAStar, FineCornerIntegralRejectsWarningBetweenOldHalfVoxelSamples) {
  auto map=std::make_shared<GridMap>();GridMapTestAccess::configure(*map);
  AStar search;search.initGridMap(map,Eigen::Vector3i(8,8,8));
  search.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell c;c.execution_reason=std::abs(p.x()-p.y())<1e-8 &&
        p.x()>-1e-8 && p.x()<.10000001 && std::abs(p.z()-1)<.06 ?
        GridExecutionReason::OK : GridExecutionReason::ENVIRONMENT_UNOBSERVED;
    c.advisory.classification=p.x()>.012 && p.x()<.020 ? GridAdvisoryClass::AVOID : GridAdvisoryClass::VALID;
    c.advisory.cost_multiplier=c.advisory.classification==GridAdvisoryClass::AVOID ? 3. : 1.;return c;
  });
  EXPECT_FALSE(search.AstarSearch(.1,Eigen::Vector3d(0,0,1),Eigen::Vector3d(.1,.1,1),-1,Eigen::Vector3d(0,0,1)));
  EXPECT_GT(search.lastResult().rejected_advisory,0u);
  EXPECT_TRUE(search.getPath().empty());
}
