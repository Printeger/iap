#include <iap/planner/risk_grid_map.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

iap::RiskGridMapParams base_params() {
  iap::RiskGridMapParams params;
  params.resolution_m = 1.0;
  params.size_x_m = 3.0;
  params.size_y_m = 3.0;
  params.size_z_m = 3.0;
  params.horizons_s = {0.0, 1.0};
  params.stale_timeout_s = 10.0;
  params.unknown_cost = 77.0;
  params.cost_max = 1000.0;
  return params;
}

class AffineProvider final : public iap::RiskPredictionProvider {
 public:
  bool fail = false;
  bool mark_unknown = false;

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (fail || results == nullptr) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = !mark_unknown;
      result.valid = !mark_unknown;
      result.stale = false;
      result.hpl_pred = affine(query.position_w, query.horizon_s);
      result.vpl_pred = 0.25 * result.hpl_pred;
      result.reason = result.valid ? "ok" : "forced_unknown";
      results->push_back(result);
    }
    return true;
  }

  static double affine(const Eigen::Vector3d& p, const double horizon_s) {
    return 20.0 + 2.0 * p.x() + 3.0 * p.y() + 4.0 * p.z() +
           5.0 * horizon_s;
  }
};

std::shared_ptr<const iap::RiskGridSnapshot> make_snapshot(
    iap::RiskGridMap* grid,
    const double now_s = 10.0) {
  AffineProvider provider;
  std::string reason;
  EXPECT_TRUE(grid->refreshFromProvider(Eigen::Vector3d::Zero(), now_s,
                                        provider, &reason))
      << reason;
  auto snapshot = grid->acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

}  // namespace

TEST(RiskGridMapTest, IndexRoundTripAndBoundaryBehavior) {
  iap::RiskGridMap grid(base_params());
  AffineProvider provider;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider));
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  EXPECT_EQ(snapshot->voxelNum(), Eigen::Vector3i(3, 3, 3));
  EXPECT_EQ(snapshot->toAddress(Eigen::Vector3i(1, 2, 0)), 15);

  const Eigen::Vector3i id(1, 1, 1);
  const Eigen::Vector3d pos = snapshot->indexToPos(id);
  Eigen::Vector3i round_trip;
  ASSERT_TRUE(snapshot->posToIndex(pos, &round_trip));
  EXPECT_EQ(round_trip, id);

  EXPECT_TRUE(snapshot->isInMap(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  EXPECT_FALSE(snapshot->isInMap(Eigen::Vector3d(-1.5, 0.0, 0.0)));
  EXPECT_FALSE(snapshot->isInMap(Eigen::Vector3d(1.5, 0.0, 0.0)));
  EXPECT_TRUE(snapshot->isInMap(Eigen::Vector3i(2, 2, 2)));
  EXPECT_FALSE(snapshot->isInMap(Eigen::Vector3i(3, 0, 0)));
}

TEST(RiskGridMapTest, TrilinearCostInterpolationAndAnalyticGradient) {
  iap::RiskGridMap grid(base_params());
  auto snapshot = make_snapshot(&grid);

  const Eigen::Vector3d query(0.25, 0.1, -0.2);
  iap::RiskCostSample sample;
  ASSERT_TRUE(snapshot->queryCost(query, 10.0, &sample)) << sample.reason;
  EXPECT_TRUE(sample.valid);
  EXPECT_NEAR(sample.cost, AffineProvider::affine(query, 0.0), 1.0e-9);
  EXPECT_NEAR(sample.grad.x(), 2.0, 1.0e-9);
  EXPECT_NEAR(sample.grad.y(), 3.0, 1.0e-9);
  EXPECT_NEAR(sample.grad.z(), 4.0, 1.0e-9);
}

TEST(RiskGridMapTest, TemporalInterpolationAcrossHorizons) {
  iap::RiskGridMap grid(base_params());
  auto snapshot = make_snapshot(&grid, 10.0);

  const Eigen::Vector3d query(0.0, 0.0, 0.0);
  iap::RiskCostSample sample;
  ASSERT_TRUE(snapshot->queryCost(query, 10.5, &sample)) << sample.reason;
  EXPECT_NEAR(sample.cost, AffineProvider::affine(query, 0.5), 1.0e-9);
  EXPECT_NEAR(sample.grad.x(), 2.0, 1.0e-9);
  EXPECT_NEAR(sample.grad.y(), 3.0, 1.0e-9);
  EXPECT_NEAR(sample.grad.z(), 4.0, 1.0e-9);
}

TEST(RiskGridMapTest, QueryCostAndPredictedPLAreSemanticallySeparate) {
  iap::RiskGridMap grid(base_params());
  auto snapshot = make_snapshot(&grid, 10.0);

  const Eigen::Vector3d query(0.2, -0.2, 0.1);
  iap::RiskCostSample cost;
  iap::PredictedPLSample pl;
  ASSERT_TRUE(snapshot->queryCost(query, 10.25, &cost)) << cost.reason;
  ASSERT_TRUE(snapshot->queryPredictedPL(query, 10.25, &pl)) << pl.reason;

  const double expected_hpl = AffineProvider::affine(query, 0.25);
  EXPECT_NEAR(cost.cost, expected_hpl, 1.0e-9);
  EXPECT_NEAR(pl.hpl_pred, expected_hpl, 1.0e-9);
  EXPECT_NEAR(pl.vpl_pred, 0.25 * expected_hpl, 1.0e-9);
  EXPECT_TRUE(pl.available);
  EXPECT_TRUE(pl.valid);
}

TEST(RiskGridMapTest, UnknownStaleInvalidAndOutOfRangeAreExplicit) {
  iap::RiskGridMapParams params = base_params();
  params.stale_timeout_s = 0.1;
  iap::RiskGridMap grid(params);
  auto snapshot = make_snapshot(&grid, 10.0);

  iap::RiskCostSample stale;
  EXPECT_FALSE(snapshot->queryCost(Eigen::Vector3d::Zero(), 10.2, &stale));
  EXPECT_EQ(stale.reason, "stale_voxel");
  EXPECT_EQ(stale.cost, params.unknown_cost);

  iap::RiskCostSample bad_time;
  EXPECT_FALSE(snapshot->queryCost(Eigen::Vector3d::Zero(),
                                   std::numeric_limits<double>::quiet_NaN(),
                                   &bad_time));
  EXPECT_EQ(bad_time.reason, "invalid_query");

  iap::PredictedPLSample out_of_range;
  EXPECT_FALSE(snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 12.0,
                                          &out_of_range));
  EXPECT_EQ(out_of_range.reason, "time_out_of_horizon");
  EXPECT_FALSE(std::isfinite(out_of_range.hpl_pred));

  iap::RiskGridMap unknown_grid(base_params());
  AffineProvider unknown_provider;
  unknown_provider.mark_unknown = true;
  ASSERT_TRUE(unknown_grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                               unknown_provider));
  auto unknown_snapshot = unknown_grid.acquireSnapshot();
  ASSERT_NE(unknown_snapshot, nullptr);
  iap::RiskCostSample unknown;
  EXPECT_FALSE(unknown_snapshot->queryCost(Eigen::Vector3d::Zero(), 10.0,
                                           &unknown));
  EXPECT_EQ(unknown.reason, "unknown_voxel");
  EXPECT_EQ(unknown.cost, base_params().unknown_cost);
}

TEST(RiskGridMapTest, SnapshotGenerationIdIsStableAfterRefresh) {
  iap::RiskGridMap grid(base_params());
  auto first = make_snapshot(&grid, 10.0);
  const uint64_t first_id = first->generation_id();

  AffineProvider provider;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(1.0, 0.0, 0.0), 11.0,
                                       provider));
  auto second = grid.acquireSnapshot();
  ASSERT_NE(second, nullptr);

  EXPECT_EQ(first->generation_id(), first_id);
  EXPECT_EQ(second->generation_id(), first_id + 1);
}

TEST(RiskGridMapTest, RefreshFailureKeepsPreviousActiveSnapshot) {
  iap::RiskGridMap grid(base_params());
  auto first = make_snapshot(&grid, 10.0);
  const uint64_t first_id = first->generation_id();

  AffineProvider provider;
  provider.fail = true;
  std::string reason;
  EXPECT_FALSE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.5,
                                        provider, &reason));
  EXPECT_EQ(reason, "provider_refresh_failed");
  auto still_active = grid.acquireSnapshot();
  ASSERT_NE(still_active, nullptr);
  EXPECT_EQ(still_active->generation_id(), first_id);
  EXPECT_EQ(grid.health().generation_id, first_id);
  EXPECT_EQ(grid.health().reason, "provider_refresh_failed");
}

