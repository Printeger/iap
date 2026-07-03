#include <ego_planner/planner_manager.h>

#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {

class ConstantProvider final : public iap::RiskPredictionProvider {
 public:
  explicit ConstantProvider(double value) : value_(value) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      (void)query;
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = value_;
      result.vpl_pred = value_;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }

 private:
  double value_;
};

iap::RiskGridMapParams params() {
  iap::RiskGridMapParams out;
  out.resolution_m = 1.0;
  out.size_x_m = 4.0;
  out.size_y_m = 4.0;
  out.size_z_m = 4.0;
  out.horizons_s = {0.0, 1.0};
  out.stale_timeout_s = 10.0;
  return out;
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot(double value,
                                                          double stamp_s) {
  iap::RiskGridMap grid(params());
  ConstantProvider provider(value);
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), stamp_s,
                                       provider, &reason))
      << reason;
  auto snapshot = grid.acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

}  // namespace

TEST(PlanningRiskContextTest, ManualContextKeepsGenerationUntilClear) {
  ego_planner::EGOPlannerManager manager;
  auto first = makeSnapshot(1.0, 10.0);
  auto second = makeSnapshot(2.0, 20.0);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  manager.setPlanningRiskContextForTest(first, 10.0);
  EXPECT_TRUE(manager.planningRiskContext().active);
  EXPECT_EQ(manager.currentPlanningRiskSnapshot(), first);
  EXPECT_EQ(manager.currentPlanningGenerationId(), first->generation_id());
  EXPECT_DOUBLE_EQ(manager.currentPlanningQueryBaseTime(), 10.0);

  // A later snapshot exists, but the active planning context remains fixed
  // until the next explicit begin/set/clear.
  EXPECT_NE(manager.currentPlanningRiskSnapshot(), second);
  EXPECT_EQ(manager.currentPlanningGenerationId(), first->generation_id());

  manager.clearPlanningRiskContext();
  EXPECT_FALSE(manager.planningRiskContext().active);
  EXPECT_EQ(manager.currentPlanningRiskSnapshot(), nullptr);
  EXPECT_EQ(manager.currentPlanningGenerationId(), 0u);
}

TEST(PlanningRiskContextTest, BeginWithoutP0RuntimeCreatesDeterministicNullContext) {
  ego_planner::EGOPlannerManager manager;
  const auto& context = manager.beginPlanningRiskContext(42.5);

  EXPECT_TRUE(context.active);
  EXPECT_EQ(context.snapshot, nullptr);
  EXPECT_EQ(context.generation_id, 0u);
  EXPECT_DOUBLE_EQ(context.query_base_time_s, 42.5);

  manager.clearPlanningRiskContext();
  EXPECT_FALSE(manager.planningRiskContext().active);
}
