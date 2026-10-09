#include <gtest/gtest.h>
#include <plan_env/grid_map.h>

TEST(GridCorridorCapture, DenseRepeatedSamplesCompleteWithinBoundedCaptureTime) {
  GridMapFailureSnapshot saved;
  saved.dimensions = Eigen::Vector3i::Constant(40);
  saved.origin = Eigen::Vector3d::Zero();
  saved.max_boundary = Eigen::Vector3d::Constant(4);
  saved.resolution_m = 0.1;
  saved.cloud_stamp_s = 10;
  saved.generation = 1;
  saved.frame_id = "map";
  saved.cell_flags.assign(64000, 4);
  saved.cell_flags[(20 * 40 + 20) * 40 + 20] = 0;
  saved.cell_flags[(21 * 40 + 20) * 40 + 20] = 5;
  auto map = GridMap::fromFailureSnapshot(saved);
  // Release checks densely sample a slow curve. Repeated sample voxels must
  // retain the same raw neighbourhood and point flags without rebuilding it.
  const Eigen::Vector3d p(2.05, 2.05, 2.05);
  const auto reference = map->captureFrozenCorridor({p}, 0.55, {}, true);
  ASSERT_NE(reference, nullptr);
  auto budget = std::make_shared<PlanningBudget>(0.15, 3);
  const auto dense = map->captureFrozenCorridor(
      std::vector<Eigen::Vector3d>(10000, p), 0.55, budget, true);
  ASSERT_NE(dense, nullptr) << "dense capture exhausted its bounded allowance";
  EXPECT_EQ(dense->cells->addresses, reference->cells->addresses);
  EXPECT_EQ(dense->cells->flags, reference->cells->flags);
  EXPECT_EQ(dense->cells->comparison_masks, reference->cells->comparison_masks);
  EXPECT_EQ(dense->cells->raw_addresses, reference->cells->raw_addresses);
  EXPECT_EQ(dense->cells->raw_row_offsets, reference->cells->raw_row_offsets);
  ASSERT_NE(dense->failure_evidence, nullptr);
  EXPECT_EQ(dense->failure_evidence->cell_flags, saved.cell_flags);
  EXPECT_EQ(budget->used(), 0u);
  EXPECT_EQ(map->commitFrozenCorridor(*dense, 10, 1, [] { return true; }),
            GridMap::CorridorCommit::Committed);
  EXPECT_EQ(map->captureFrozenCorridor({p}, 0.55,
                                      std::make_shared<PlanningBudget>(0, 3)),
            nullptr);
}

// Only geometry construction bypasses ROS. All cache operations use the public
// GridMap interface, including real map mutation/reset and query callbacks.
struct GridMapTestAccess {
  static void configure(GridMap& map) {
    map.mp_.map_origin_ = Eigen::Vector3d(-2, -3, 0);
    map.mp_.map_size_ = Eigen::Vector3d(4, 6, 2);
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    map.mp_.map_voxel_num_ = Eigen::Vector3i(4, 6, 2);
    map.mp_.resolution_ = map.mp_.resolution_inv_ = 1.0;
    map.mp_.frame_id_ = "map";
    map.mp_.min_occupancy_log_ = 0.5;
    map.mp_.obstacles_inflation_ = 0;
    map.mp_.virtual_ceil_height_ = -1;
    map.md_.occupancy_buffer_.assign(48, 0);
    map.md_.occupancy_buffer_inflate_.assign(48, 0);
    map.md_.occupancy_buffer_raw_cloud_.assign(48, 0);
    map.md_.observed_buffer_.assign(48, 1);
    map.occupancy_update_sequence_.store(2);
    map.occupancy_cloud_stamp_s_.store(10);
  }
  static size_t riskSize(const GridMap& map) { return map.md_.risk_buffer_.size(); }
  static GridRiskVoxel cached(const GridMap& map, int address) {
    return map.md_.risk_buffer_.at(address);
  }
  static void setObserved(GridMap& map, int address, bool observed) {
    map.md_.observed_buffer_.at(address) = observed ? 1 : 0;
  }
  static void setRawAndInflated(GridMap& map, int address) {
    map.md_.occupancy_buffer_raw_cloud_.at(address) = 1;
    map.md_.occupancy_buffer_inflate_.at(address) = 1;
  }
  static void disableFrozenIndex(GridMap& map) {
    map.frozen_raw_index_generation_ = 0;
  }
};
class GridRiskTest : public testing::Test {
 protected:
  GridMap map;
  Eigen::Vector3d point{-0.2, 0.1, 0.2};
  int calls = 0;
  void SetUp() override { GridMapTestAccess::configure(map); }
  GridRiskContext context() {
    GridRiskContext ctx;
    ctx.frame_id = "map";
    ctx.reference_time_s = 10;
    ctx.valid_until_s = 11;
    ctx.occupancy_generation = map.occupancyGeneration();
    ctx.reference_position = Eigen::Vector3d(0, 0, 0);
    ctx.predict = [this](const Eigen::Vector3d& p) {
      ++calls;
      GridRiskVoxel v;
      v.status = GridRiskStatus::VALID;
      v.hpl = p.norm(); v.vpl = 0.2;
      return v;
    };
    return ctx;
  }
};
TEST_F(GridRiskTest, SameLatticeAndOccupancyIndependent) {
  map.setOccupied(point);
  const auto version = map.bindRiskContext(context());
  const int occupancy = map.getOccupancy(point);
  const int inflated = map.getInflateOccupancy(point);
  const auto result = map.queryRisk(point, version, 10);
  EXPECT_EQ(result.status, GridRiskStatus::VALID);
  Eigen::Vector3i index; map.posToIndex(point, index);
  Eigen::Vector3d center; map.indexToPos(index, center);
  EXPECT_DOUBLE_EQ(result.hpl, center.norm());
  EXPECT_EQ(GridMapTestAccess::riskSize(map), 48u);
  EXPECT_EQ(GridMapTestAccess::cached(map, map.toAddress(index)).version, version);
  EXPECT_EQ(map.getOccupancy(point), occupancy);
  EXPECT_EQ(map.getInflateOccupancy(point), inflated);
  EXPECT_EQ(inflated, 1);
  EXPECT_EQ(map.queryRisk(center, version, 10.1).status, GridRiskStatus::VALID);
  EXPECT_EQ(calls, 1);
}
TEST_F(GridRiskTest, FailureSnapshotKeepsOneGenerationAndAllPhysicalLayers) {
  Eigen::Vector3i index;
  map.posToIndex(point, index);
  const int address = map.toAddress(index);
  GridMapTestAccess::setRawAndInflated(map, address);
  GridMapTestAccess::setObserved(map, address + 1, false);
  const auto version = map.bindRiskContext(context());
  ASSERT_EQ(map.queryRisk(point, version, 10).status, GridRiskStatus::VALID);
  const auto saved = map.captureFailureSnapshot();
  ASSERT_TRUE(saved.has_value());
  EXPECT_EQ(saved->generation, map.occupancyGeneration());
  EXPECT_EQ(saved->cell_flags.size(), 48u);
  EXPECT_EQ(saved->cell_flags.at(address) & 7, 7);
  EXPECT_EQ(saved->cell_flags.at(address + 1) & 4, 0);
  ASSERT_EQ(saved->queried_risk.size(), 1u);
  EXPECT_EQ(saved->queried_risk.front().address,
            static_cast<uint32_t>(address));
  EXPECT_EQ(saved->queried_risk.front().value.version, version);
}
TEST_F(GridRiskTest, FrozenFailureMapUsesSamePlanningCellRule) {
  Eigen::Vector3i obstacle(1, 3, 0);
  GridMapTestAccess::setRawAndInflated(map, map.toAddress(obstacle));
  Eigen::Vector3i unknown(3, 5, 0);
  GridMapTestAccess::setObserved(map, map.toAddress(unknown), false);
  const auto saved = map.captureFailureSnapshot();
  ASSERT_TRUE(saved);
  auto replay = GridMap::fromFailureSnapshot(*saved);
  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 10.0;
  motion.error_proxy_m = 0.02;
  motion.body_radius_m = 0.35;
  motion.tracking_reserve_m = 0.10;
  GridPlanningRiskPolicy policy;
  for (const Eigen::Vector3d point : {
           Eigen::Vector3d(-0.5, 0.5, 0.5),
           Eigen::Vector3d(0.5, 0.5, 0.5),
           Eigen::Vector3d(1.5, 2.5, 0.5)}) {
    const auto original = map.queryPlanningCell(point, 0, 10.0, policy, motion);
    const auto frozen = replay->queryPlanningCell(point, 0, 10.0, policy, motion);
    EXPECT_EQ(frozen.execution_reason, original.execution_reason);
    EXPECT_EQ(frozen.observed, original.observed);
    EXPECT_EQ(frozen.required_clearance_m, original.required_clearance_m);
    if (std::isnan(original.raw_center_clearance_m))
      EXPECT_TRUE(std::isnan(frozen.raw_center_clearance_m));
    else
      EXPECT_EQ(frozen.raw_center_clearance_m, original.raw_center_clearance_m);
  }
}

TEST(GridClearanceIndex, SameExactCubeNearestPointAndRejectionsAsLiveBuffers) {
  GridMapFailureSnapshot saved;
  saved.dimensions = Eigen::Vector3i(32, 24, 16);
  saved.origin = Eigen::Vector3d(-1.6, -1.2, 0);
  saved.resolution_m = 0.1;
  saved.max_boundary = saved.origin + saved.dimensions.cast<double>() * 0.1;
  saved.frame_id = "map";
  saved.generation = 3;
  saved.cloud_stamp_s = 10;
  saved.cell_flags.resize(32 * 24 * 16, 4);
  for (size_t i = 0; i < saved.cell_flags.size(); ++i) {
    if (i % 211 == 0) saved.cell_flags[i] = 5;
    else if (i % 137 == 0) saved.cell_flags[i] = 6;
    else if (i % 17 == 0) saved.cell_flags[i] = 0;
  }
  auto indexed = GridMap::fromFailureSnapshot(saved);
  auto dense = GridMap::fromFailureSnapshot(saved);
  GridMapTestAccess::disableFrozenIndex(*dense);
  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 10;
  motion.error_proxy_m = 0.02;
  GridPlanningRiskPolicy policy;
  const auto compare = [&](const Eigen::Vector3d& p, bool diagnostics) {
    const auto a = indexed->queryPlanningCell(p, 0, 10, policy, motion, diagnostics);
    const auto b = dense->queryPlanningCell(p, 0, 10, policy, motion, diagnostics);
    const auto context = indexed->preparePlanningQuery(10, motion);
    const auto fast = indexed->queryPlanningCell(p, 0, 10, policy, motion,
                                                diagnostics, &context);
    const auto profiled = indexed->queryPlanningCell(p, 0, 10, policy, motion,
                                                    diagnostics, &context, true);
    EXPECT_EQ(fast.execution_reason, b.execution_reason) << p.transpose();
    EXPECT_EQ(fast.execution_reason, profiled.execution_reason);
    EXPECT_EQ(fast.voxel_index, b.voxel_index);
    EXPECT_EQ(fast.observed, b.observed);
    const auto spatial = indexed->queryOccupancyDiagnostic(p, false);
    const auto detailed = indexed->queryOccupancyDiagnostic(p);
    EXPECT_EQ(spatial.available, detailed.available);
    EXPECT_EQ(spatial.generation, detailed.generation);
    EXPECT_EQ(spatial.observed, detailed.observed);
    EXPECT_EQ(spatial.raw_occupied, detailed.raw_occupied);
    EXPECT_EQ(spatial.inflated_occupied, detailed.inflated_occupied);
    EXPECT_EQ(spatial.voxel_index, detailed.voxel_index);
    EXPECT_TRUE(spatial.frame_id.empty());
    EXPECT_FALSE(spatial.voxel_center.allFinite());
    EXPECT_DOUBLE_EQ(fast.required_clearance_m, b.required_clearance_m);
    if (diagnostics && std::isfinite(b.raw_center_clearance_m)) {
      EXPECT_DOUBLE_EQ(fast.raw_center_clearance_m, b.raw_center_clearance_m);
      EXPECT_TRUE(fast.nearest_raw_center == b.nearest_raw_center);
    }
    EXPECT_EQ(a.execution_reason, b.execution_reason);
    EXPECT_EQ(a.observed, b.observed);
    EXPECT_EQ(a.occupancy_generation, b.occupancy_generation);
    EXPECT_EQ(a.required_clearance_m, b.required_clearance_m);
    if (std::isnan(b.raw_center_clearance_m)) {
      EXPECT_TRUE(std::isnan(a.raw_center_clearance_m));
    } else {
      EXPECT_EQ(a.raw_center_clearance_m, b.raw_center_clearance_m);
      if (b.nearest_raw_center.allFinite()) {
        EXPECT_TRUE(a.nearest_raw_center == b.nearest_raw_center);
      }
    }
  };
  // Continuous offsets, exact voxel edges/centers, unknown, raw/inflated,
  // map boundaries and different stencil radii exercise both real paths.
  for (const double radius : {0.0, 0.35, 0.9}) {
    motion.body_radius_m = radius;
    for (int i = 0; i < 600; ++i) {
      Eigen::Vector3d p = saved.origin + Eigen::Vector3d(
          (i * 13 % 33) * 0.1, (i * 19 % 25) * 0.1,
          (i * 7 % 17) * 0.1);
      if (i % 3 == 0) p.array() += 0.05;
      if (i % 3 == 1) p.array() += 0.00123;
      compare(p, false);
      compare(p, true);
    }
  }
  // Generation change invalidates the index, including new fused obstacles.
  const Eigen::Vector3d new_raw(0.15, 0.05, 0.75);
  indexed->setOccupancy(new_raw, 1);
  dense->setOccupancy(new_raw, 1);
  compare(new_raw + Eigen::Vector3d(0.12, 0, 0), true);
}
TEST_F(GridRiskTest, AllCellsShareAddressAndBordersReject) {
  const auto version = map.bindRiskContext(context());
  for (int x = 0; x < 4; ++x) for (int y = 0; y < 6; ++y) for (int z = 0; z < 2; ++z) {
    Eigen::Vector3i index(x,y,z); Eigen::Vector3d p; map.indexToPos(index,p);
    EXPECT_EQ(map.queryRisk(p,version,10).status, GridRiskStatus::VALID);
    EXPECT_DOUBLE_EQ(GridMapTestAccess::cached(map,map.toAddress(index)).hpl, p.norm());
  }
  EXPECT_EQ(calls,48);
  for (const auto& p : {Eigen::Vector3d(-2,-3,0), Eigen::Vector3d(2,0,1), Eigen::Vector3d(0,3,1), Eigen::Vector3d(0,0,2)})
    EXPECT_EQ(map.queryRisk(p,version,10).status,GridRiskStatus::OUT_OF_MAP);
}
TEST_F(GridRiskTest, VersionsMapWritesAndResetInvalidate) {
  const auto old = map.bindRiskContext(context());
  EXPECT_EQ(map.queryRisk(point,old,10).status,GridRiskStatus::VALID);
  const auto current = map.bindRiskContext(context());
  EXPECT_EQ(map.queryRisk(point,old,10).status,GridRiskStatus::VERSION_CHANGED);
  EXPECT_EQ(map.queryRisk(point,current,10).status,GridRiskStatus::VALID);
  EXPECT_EQ(calls,2);
  map.setOccupancy(point);
  EXPECT_EQ(map.queryRisk(point,current,10).status,GridRiskStatus::VERSION_CHANGED);
  const auto next = map.bindRiskContext(context());
  const auto generation = map.occupancyGeneration();
  map.resetBuffer();
  EXPECT_GT(map.occupancyGeneration(),generation);
  EXPECT_EQ(map.queryRisk(point,next,10).status,GridRiskStatus::VERSION_CHANGED);
}
TEST_F(GridRiskTest, FreshnessFrameAndMissingInputs) {
  auto ctx=context(); ctx.frame_id="odom";
  auto version=map.bindRiskContext(ctx);
  EXPECT_EQ(map.queryRisk(point,version,10).status,GridRiskStatus::FRAME_MISMATCH);
  version=map.bindRiskContext(context());
  EXPECT_EQ(map.queryRisk(point,version,9.9).status,GridRiskStatus::STALE);
  EXPECT_EQ(map.queryRisk(point,version,11.1).status,GridRiskStatus::STALE);
  EXPECT_EQ(calls,0);
  ctx=context(); ctx.predict={}; version=map.bindRiskContext(ctx);
  EXPECT_EQ(map.queryRisk(point,version,10).status,GridRiskStatus::UNCOMPUTED);
  EXPECT_TRUE(std::isnan(map.queryRisk(point,version,10).hpl));
}
TEST_F(GridRiskTest, NonFiniteNegativeAndThrowingPredictorNeverBecomeValid) {
  for (double hpl : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -1.0, 1e9, 2e9}) {
    auto ctx=context(); ctx.predict=[hpl](const Eigen::Vector3d&) {
      GridRiskVoxel v; v.hpl=hpl; v.vpl=1; v.status=GridRiskStatus::VALID; return v;
    };
    auto version=map.bindRiskContext(ctx);
    const auto v=map.queryRisk(point,version,10);
    EXPECT_EQ(v.status,GridRiskStatus::INVALID); EXPECT_TRUE(std::isnan(v.hpl));
  }
  auto ctx=context(); ctx.predict=[](const Eigen::Vector3d&) -> GridRiskVoxel { throw std::runtime_error("unavailable"); };
  const auto version=map.bindRiskContext(ctx);
  EXPECT_EQ(map.queryRisk(point,version,10).status,GridRiskStatus::INVALID);
  EXPECT_EQ(map.queryRisk(Eigen::Vector3d::Constant(NAN),version,10).status,GridRiskStatus::INVALID_QUERY);
}
TEST_F(GridRiskTest, InputChangeDuringPredictionDoesNotPublishOldResult) {
  auto ctx=context(); ctx.predict=[this](const Eigen::Vector3d&) {
    map.invalidateRiskContext();
    GridRiskVoxel v; v.hpl=1; v.vpl=2; v.status=GridRiskStatus::VALID; return v;
  };
  const auto version=map.bindRiskContext(ctx);
  const auto v=map.queryRisk(point,version,10);
  EXPECT_EQ(v.status,GridRiskStatus::VERSION_CHANGED);
  EXPECT_TRUE(std::isnan(v.hpl));
}

TEST_F(GridRiskTest, AdvisoryWarningIsDistinctFromUnknownAndExecution) {
  GridPlanningRiskPolicy policy;
  auto ctx = context();
  ctx.predict = [](const Eigen::Vector3d&) {
    GridRiskVoxel value;
    value.status = GridRiskStatus::VALID;
    value.hpl = 0.46;
    value.vpl = 0.30;
    return value;
  };
  const auto version = map.bindRiskContext(ctx);
  const auto warning = map.queryPlanningRisk(point, version, 10.0, policy);
  EXPECT_EQ(warning.classification, GridAdvisoryClass::AVOID);
  EXPECT_EQ(warning.query_status, GridRiskStatus::VALID);

  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 10.0;
  motion.error_proxy_m = 0.05;
  const auto cell = map.queryPlanningCell(point, version, 10.0, policy, motion);
  EXPECT_TRUE(cell.executable());
  EXPECT_EQ(cell.advisory.classification, GridAdvisoryClass::AVOID);

  const auto stale = map.queryPlanningRisk(point, version, 11.1, policy);
  EXPECT_EQ(stale.classification, GridAdvisoryClass::STALE_REFERENCE);
  EXPECT_GT(stale.cost_multiplier, policy.unknown_multiplier);
  const auto unknown = map.queryPlanningRisk(point, version, 12.1, policy);
  EXPECT_EQ(unknown.classification, GridAdvisoryClass::UNKNOWN);
  EXPECT_DOUBLE_EQ(unknown.cost_multiplier, policy.unknown_multiplier);
  EXPECT_TRUE(std::isnan(unknown.hpl));
  EXPECT_EQ(map.getInflateOccupancy(point), 0);
}

TEST_F(GridRiskTest, CurrentMotionAndPhysicalEvidenceAreSeparateConditions) {
  GridPlanningRiskPolicy policy;
  const auto version = map.bindRiskContext(context());
  GridMotionContext motion;
  motion.stamp_s = 10.0;
  motion.error_proxy_m = 0.05;
  EXPECT_EQ(map.queryPlanningCell(point, version, 10, policy, motion).execution_reason,
            GridExecutionReason::CURRENT_MOTION_UNAVAILABLE);
  motion.quality = 2;
  EXPECT_EQ(map.queryPlanningCell(point, version, 10, policy, motion).execution_reason,
            GridExecutionReason::CURRENT_MOTION_UNAVAILABLE);
  motion.allow_bridged = true;
  EXPECT_TRUE(map.queryPlanningCell(point, version, 10, policy, motion).executable());
  motion.quality = 1;
  motion.allow_bridged = false;
  Eigen::Vector3i index;
  map.posToIndex(point, index);
  GridMapTestAccess::setObserved(map, map.toAddress(index), false);
  EXPECT_EQ(map.queryPlanningCell(point, version, 10, policy, motion).execution_reason,
            GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  GridMapTestAccess::setObserved(map, map.toAddress(index), true);
  EXPECT_EQ(map.queryPlanningCell(point, version, 10.6, policy, motion).execution_reason,
            GridExecutionReason::ENVIRONMENT_STALE);
  map.setOccupied(point);
  EXPECT_EQ(map.queryPlanningCell(point, version, 10, policy, motion).execution_reason,
            GridExecutionReason::PHYSICAL_OBSTACLE);
}

TEST_F(GridRiskTest, PreparedConditionsKeepReasonPrecedenceAndInvalidateOnUpdate) {
  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 10;
  motion.error_proxy_m = 0.02;
  GridPlanningRiskPolicy policy;
  for (const double now : {9.0, 10.0, 10.6}) {
    for (const uint8_t quality : {0, 1, 2, 3}) {
      motion.quality = quality;
      for (const double error : {0.02, 0.6}) {
        motion.error_proxy_m = error;
        const auto context = map.preparePlanningQuery(now, motion);
        for (const auto& p : {point, Eigen::Vector3d(2, 0, 1)}) {
          const auto reference = map.queryPlanningCell(p, 0, now, policy, motion);
          const auto prepared = map.queryPlanningCell(p, 0, now, policy, motion,
                                                      false, &context);
          EXPECT_EQ(reference.execution_reason, prepared.execution_reason);
        }
      }
    }
  }
  motion.quality = 1;
  motion.error_proxy_m = 0.02;
  const auto context = map.preparePlanningQuery(10, motion);
  map.setOccupancy(point, 1);
  EXPECT_EQ(map.queryPlanningCell(point, 0, 10, policy, motion, false, &context)
                .execution_reason, GridExecutionReason::ENVIRONMENT_STALE);
}

TEST(GridClearanceBounds, SameVoxelOffsetsDoNotShareAnExactClearanceDecision) {
  GridMapFailureSnapshot saved;
  saved.dimensions = Eigen::Vector3i(20, 20, 20);
  saved.origin = Eigen::Vector3d::Zero();
  saved.max_boundary = Eigen::Vector3d::Constant(2);
  saved.resolution_m = 0.1;
  saved.cloud_stamp_s = 10;
  saved.generation = 1;
  saved.frame_id = "map";
  saved.cell_flags.assign(8000, 4);
  saved.cell_flags[(10 * 20 + 10) * 20 + 10] = 5;
  auto map = GridMap::fromFailureSnapshot(saved);
  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 10;
  motion.body_radius_m = 0.20;
  motion.tracking_reserve_m = 0;
  motion.error_proxy_m = 0;
  GridPlanningRiskPolicy policy;
  const auto context = map->preparePlanningQuery(10, motion);
  const Eigen::Vector3d a(0.701, 1.05, 1.05), b(0.799, 1.05, 1.05);
  auto query = [&](const Eigen::Vector3d& p) {
    const auto exact = map->queryPlanningCell(p, 0, 10, policy, motion);
    const auto fast = map->queryPlanningCell(p, 0, 10, policy, motion, false, &context);
    EXPECT_EQ(exact.execution_reason, fast.execution_reason);
    return fast.execution_reason;
  };
  EXPECT_EQ(query(a), GridExecutionReason::OK);
  EXPECT_EQ(query(b), GridExecutionReason::INSUFFICIENT_CLEARANCE);
  // Empty finite stencils, map borders, exact clearance threshold and adjacent
  // representable doubles must all agree with the original exact query.
  const double boundary = 1.05 - context.required_clearance_m;
  for (const double x : {0.01, 0.15, boundary,
                         std::nextafter(boundary, 0.0),
                         std::nextafter(boundary, 2.0), 1.999})
    query(Eigen::Vector3d(x, 1.05, 1.05));
  const auto stats = map->planningQueryStats();
  EXPECT_GT(stats.bounds_hits, 0u);
  EXPECT_GT(stats.fast_pass, 0u);
  EXPECT_GT(stats.exact_decisions, 0u);
}

TEST_F(GridRiskTest, BelowWarningCostUsesRawProtectionLevelsContinuously) {
  GridPlanningRiskPolicy policy;
  for (double hpl : {0.10, 0.20, 0.40}) {
    auto ctx = context();
    ctx.predict = [hpl](const Eigen::Vector3d&) {
      GridRiskVoxel value;
      value.status = GridRiskStatus::VALID;
      value.hpl = hpl; value.vpl = 0.10;
      return value;
    };
    const auto version = map.bindRiskContext(ctx);
    const auto risk = map.queryPlanningRisk(point, version, 10., policy);
    ASSERT_EQ(risk.classification, GridAdvisoryClass::VALID);
    EXPECT_DOUBLE_EQ(risk.hpl, hpl);
    EXPECT_DOUBLE_EQ(risk.vpl, .10);
    EXPECT_NEAR(risk.cost_multiplier, 1. + .5 * std::max(
        (hpl + policy.reserve_h_m) / policy.hpl_budget_m,
        (.10 + policy.reserve_v_m) / policy.vpl_budget_m), 1e-12);
    EXPECT_EQ(risk.version, version);
  }
}


TEST_F(GridRiskTest, FrozenQueryExportsItsOwnRawSamplesAfterLiveContextChanges) {
  const auto physical = map.captureFailureSnapshot();
  ASSERT_TRUE(physical);
  auto prediction = context();
  prediction.predict = [this](const Eigen::Vector3d&) {
    ++calls;
    GridRiskVoxel raw;
    raw.status = GridRiskStatus::VALID; raw.hpl = .4; raw.vpl = .2;
    raw.source_flags = 3; raw.gnss_raw_valid = false;
    raw.gnss_geometry_status = 3;
    return raw;
  };
  const auto version = map.bindRiskContext(prediction);
  auto frozen = map.capturePlanningRiskQuery(version,10.,{},nullptr,physical->generation);
  const auto actual = frozen(point);
  ASSERT_EQ(actual.query_status,GridRiskStatus::VALID);
  EXPECT_EQ(calls,1);
  map.bindRiskContext(context());
  map.setOccupied(Eigen::Vector3d(1.5,1.5,.5));
  EXPECT_DOUBLE_EQ(frozen(point).hpl,.4);
  EXPECT_EQ(calls,1) << "export/repeated lookup must not re-run prediction";
  const auto raw = frozen.captureEvidence(*physical);
  ASSERT_TRUE(raw);
  EXPECT_TRUE(raw->risk_context_matches_map);
  EXPECT_EQ(raw->risk_version,version);
  EXPECT_DOUBLE_EQ(raw->risk_reference_time_s,10.);
  EXPECT_DOUBLE_EQ(raw->risk_valid_until_s,11.);
  ASSERT_EQ(raw->queried_risk.size(),1u);
  EXPECT_DOUBLE_EQ(raw->queried_risk[0].value.hpl,.4);
  EXPECT_EQ(raw->queried_risk[0].value.source_flags,3u);
  EXPECT_FALSE(raw->queried_risk[0].value.gnss_raw_valid);
  EXPECT_EQ(raw->queried_risk[0].value.gnss_geometry_status,3u);
  auto other = *physical; ++other.generation;
  EXPECT_FALSE(frozen.captureEvidence(other));
  other = *physical; other.origin.x() += 1.;
  EXPECT_FALSE(frozen.captureEvidence(other));
  const auto after = frozen.captureEvidence(*physical);
  ASSERT_TRUE(after);
  EXPECT_EQ(after->queried_risk[0].value.version,version);
  EXPECT_EQ(calls,1);
}

TEST_F(GridRiskTest, FrozenGenerationBindingSurvivesAdvanceBeforeQueryCapture) {
  const auto physical = map.captureFailureSnapshot();
  ASSERT_TRUE(physical);
  const auto version = map.bindRiskContext(context());
  map.setOccupied(Eigen::Vector3d(1.5,1.5,.5));
  ASSERT_NE(map.occupancyGeneration(),physical->generation);
  auto frozen = map.capturePlanningRiskQuery(version,10.,{},nullptr,physical->generation);
  EXPECT_EQ(frozen(point).query_status,GridRiskStatus::VALID);
  const auto evidence = frozen.captureEvidence(*physical);
  ASSERT_TRUE(evidence);
  EXPECT_TRUE(evidence->risk_context_matches_map);
  EXPECT_EQ(evidence->risk_version,version);
  auto wrong = map.capturePlanningRiskQuery(version,10.,{},nullptr,physical->generation+2);
  EXPECT_EQ(wrong(point).query_status,GridRiskStatus::VERSION_CHANGED);
  auto live = map.capturePlanningRiskQuery(version,10.,{});
  EXPECT_EQ(live(point).query_status,GridRiskStatus::VERSION_CHANGED);
}

TEST_F(GridRiskTest, FrozenQueryRetainsInvalidSentinelWithUnknownPreference) {
  auto prediction = context();
  prediction.predict = [](const Eigen::Vector3d&) {
    GridRiskVoxel value; value.status = GridRiskStatus::VALID;
    value.hpl = 1e9; value.vpl = .2; return value;
  };
  const auto physical = map.captureFailureSnapshot();
  ASSERT_TRUE(physical);
  const auto version = map.bindRiskContext(prediction);
  auto frozen = map.capturePlanningRiskQuery(version,10.,{},nullptr,physical->generation);
  const auto cost = frozen(point);
  EXPECT_EQ(cost.query_status,GridRiskStatus::INVALID);
  EXPECT_EQ(cost.classification,GridAdvisoryClass::UNKNOWN);
  EXPECT_DOUBLE_EQ(cost.cost_multiplier,1.5);
  const auto raw = frozen.captureEvidence(*physical);
  ASSERT_TRUE(raw); ASSERT_EQ(raw->queried_risk.size(),1u);
  EXPECT_EQ(raw->queried_risk[0].value.status,GridRiskStatus::INVALID);
  EXPECT_DOUBLE_EQ(raw->queried_risk[0].value.hpl,1e9)
      << "invalid raw evidence is never turned into a valid preference";
  EXPECT_EQ(sizeof(GridRiskVoxel),32u) << "preserve the existing dense voxel cache footprint";
}

TEST_F(GridRiskTest, FrozenClassificationIsComputedOncePerVoxelAndInvalidatedPerContext) {
  const auto physical=map.captureFailureSnapshot(); ASSERT_TRUE(physical);
  auto first=context(); first.predict=[this](const Eigen::Vector3d&) {
    ++calls; GridRiskVoxel value; value.status=GridRiskStatus::VALID;
    value.hpl=.4; value.vpl=.2; return value;
  };
  const auto version=map.bindRiskContext(first);
  auto frozen=map.capturePlanningRiskQuery(version,10.,{},nullptr,physical->generation,true);
  const auto a=frozen(point);
  for(int i=0;i<100;++i) {
    const auto b=frozen(point+Eigen::Vector3d(.01,0,0));
    EXPECT_DOUBLE_EQ(a.cost_multiplier,b.cost_multiplier);
  }
  EXPECT_EQ(frozen.statistics->accesses,101u);
  EXPECT_EQ(frozen.statistics->hits,100u);
  EXPECT_EQ(frozen.statistics->misses,1u);
  EXPECT_EQ(frozen.statistics->predictions,1u);
  EXPECT_EQ(frozen.statistics->classifications,1u);
  auto policy=GridPlanningRiskPolicy{};policy.hpl_budget_m=.3;
  auto other=map.capturePlanningRiskQuery(version,10.,policy,nullptr,physical->generation);
  EXPECT_NE(other(point).classification,a.classification);
  EXPECT_EQ(other.statistics->classifications,1u);
  EXPECT_EQ(calls,2);
  auto stale=map.capturePlanningRiskQuery(version,12.,{},nullptr,physical->generation);
  EXPECT_NE(stale(point).query_status,GridRiskStatus::VALID);
  EXPECT_EQ(stale.statistics->predictions,0u);
  EXPECT_EQ(frozen(point).query_status,GridRiskStatus::VALID);
  EXPECT_EQ(calls,2);
}

TEST_F(GridRiskTest, RouteUnknownRetainsExecuteRefusalAndChecksKnownClearance) {
  Eigen::Vector3i index; map.posToIndex(point,index);
  const int address=map.toAddress(index);
  GridMapTestAccess::setObserved(map,address,false);
  GridMotionContext motion; motion.quality=1; motion.stamp_s=10.; motion.error_proxy_m=.01;
  const auto version=map.bindRiskContext(context());
  auto route=map.queryRouteCell(point,version,10.,{},motion);
  EXPECT_TRUE(route.routable()); EXPECT_FALSE(route.facts.observed);
  EXPECT_EQ(route.facts.execution_reason,GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_EQ(map.queryPlanningCell(point,version,10.,{},motion).execution_reason,
      GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_DOUBLE_EQ(GridSearchCell(route).physical_unknown_cost,1.5);
  GridMapTestAccess::setRawAndInflated(map,address);
  route=map.queryRouteCell(point,version,10.,{},motion);
  EXPECT_FALSE(route.routable()); EXPECT_EQ(route.route_reason,GridExecutionReason::PHYSICAL_OBSTACLE);
}

TEST_F(GridRiskTest, FiniteAdvisoryCostIsContinuousAcrossWarningAndDegradation) {
  GridPlanningRiskPolicy policy; policy.hpl_budget_m=policy.vpl_budget_m=1.;
  policy.reserve_h_m=policy.reserve_v_m=.1;
  for(double ratio:{0.5,1.-1e-6,1.,1.+1e-6,1.5,3.}) {
    auto ctx=context();ctx.predict=[ratio](const Eigen::Vector3d&) {
      GridRiskVoxel value;value.status=GridRiskStatus::VALID;value.hpl=ratio-.1;value.vpl=0.;return value;
    };
    const auto version=map.bindRiskContext(ctx);
    const auto value=map.queryPlanningRisk(point,version,10.,policy);
    EXPECT_NEAR(value.cost_multiplier,ratio<=1 ? 1+.5*ratio : std::min(3.,1.5+1.5*(ratio-1)),1e-12);
    EXPECT_DOUBLE_EQ(gridAdvisoryCostMultiplier(value.classification,value.cost_multiplier),value.cost_multiplier);
  }
}
