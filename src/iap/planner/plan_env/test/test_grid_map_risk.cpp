#include <gtest/gtest.h>
#include <plan_env/grid_map.h>

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
  for (double hpl : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -1.0}) {
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
