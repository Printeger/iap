#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
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
  uint32_t source_flags = 0u;
  int query_count = 0;
  std::vector<iap::RiskPredictionQuery> last_queries;

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (fail || results == nullptr) {
      return false;
    }
    last_queries = queries;
    query_count += static_cast<int>(queries.size());
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = !mark_unknown;
      result.valid = !mark_unknown;
      result.stale = false;
      result.hpl_pred = affine(query.position_w, query.horizon_s);
      result.vpl_pred = 0.25 * result.hpl_pred;
      result.source_flags = source_flags;
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

class AlternatingStaleProvider final : public iap::RiskPredictionProvider {
 public:
  bool all_stale = false;
  int query_count = 0;

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr) {
      return false;
    }
    query_count += static_cast<int>(queries.size());
    results->clear();
    results->reserve(queries.size());
    for (std::size_t i = 0; i < queries.size(); ++i) {
      iap::RiskPredictionResult result;
      const bool stale = all_stale || i % 2 == 0;
      result.available = !stale;
      result.valid = !stale;
      result.stale = stale;
      result.hpl_pred = 10.0;
      result.vpl_pred = 5.0;
      result.reason = stale ? "stale_gnss_epoch" : "ok";
      results->push_back(result);
    }
    return true;
  }
};

class AlternatingInvalidProvider final : public iap::RiskPredictionProvider {
 public:
  int query_count = 0;

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr) {
      return false;
    }
    query_count += static_cast<int>(queries.size());
    results->clear();
    results->reserve(queries.size());
    for (std::size_t i = 0; i < queries.size(); ++i) {
      iap::RiskPredictionResult result;
      const bool invalid = i % 2 == 0;
      result.available = !invalid;
      result.valid = !invalid;
      result.stale = false;
      result.hpl_pred = 10.0;
      result.vpl_pred = 5.0;
      result.reason = invalid ? "too_few_lidar_normals" : "ok";
      results->push_back(result);
    }
    return true;
  }
};

class BlockingProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    int entry_number = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      entry_number = ++entry_count_;
      entered_.notify_all();
      if (entry_number == 1) {
        released_.wait(lock, [&]() { return released_flag_; });
      }
    }
    if (results == nullptr) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = AffineProvider::affine(query.position_w,
                                               query.horizon_s);
      result.vpl_pred = 0.25 * result.hpl_pred;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }

  bool waitUntilEntryCount(const int expected_count) {
    std::unique_lock<std::mutex> lock(mutex_);
    return entered_.wait_for(lock, std::chrono::seconds(5),
                             [&]() { return entry_count_ >= expected_count; });
  }

  int entryCount() {
    std::lock_guard<std::mutex> lock(mutex_);
    return entry_count_;
  }

  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_flag_ = true;
    released_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable entered_;
  std::condition_variable released_;
  int entry_count_ = 0;
  bool released_flag_ = false;
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

void expect_query_positions_equal(
    const std::vector<iap::RiskPredictionQuery>& lhs,
    const std::vector<iap::RiskPredictionQuery>& rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    EXPECT_EQ(lhs[i].position_w, rhs[i].position_w) << "query " << i;
    EXPECT_DOUBLE_EQ(lhs[i].horizon_s, rhs[i].horizon_s) << "query " << i;
  }
}

void expect_snapshot_voxels_equal(
    const iap::RiskGridSnapshot& lhs,
    const iap::RiskGridSnapshot& rhs) {
  ASSERT_EQ(lhs.horizonCount(), rhs.horizonCount());
  ASSERT_EQ(lhs.voxelNum(), rhs.voxelNum());
  for (int h = 0; h < lhs.horizonCount(); ++h) {
    for (int x = 0; x < lhs.voxelNum().x(); ++x) {
      for (int y = 0; y < lhs.voxelNum().y(); ++y) {
        for (int z = 0; z < lhs.voxelNum().z(); ++z) {
          const Eigen::Vector3i id(x, y, z);
          iap::RiskVoxel lhs_voxel;
          iap::RiskVoxel rhs_voxel;
          ASSERT_TRUE(lhs.voxelAt(h, id, &lhs_voxel));
          ASSERT_TRUE(rhs.voxelAt(h, id, &rhs_voxel));
          EXPECT_DOUBLE_EQ(lhs_voxel.c_pi, rhs_voxel.c_pi);
          EXPECT_DOUBLE_EQ(lhs_voxel.hpl_pred, rhs_voxel.hpl_pred);
          EXPECT_DOUBLE_EQ(lhs_voxel.vpl_pred, rhs_voxel.vpl_pred);
          EXPECT_DOUBLE_EQ(lhs_voxel.stamp_s, rhs_voxel.stamp_s);
          EXPECT_EQ(lhs_voxel.valid, rhs_voxel.valid);
          EXPECT_EQ(lhs_voxel.stale, rhs_voxel.stale);
          EXPECT_EQ(lhs_voxel.unknown, rhs_voxel.unknown);
          EXPECT_EQ(lhs_voxel.source_flags, rhs_voxel.source_flags);
          EXPECT_EQ(lhs_voxel.reason, rhs_voxel.reason);
          EXPECT_EQ(lhs_voxel.occupancy, rhs_voxel.occupancy);
        }
      }
    }
  }
}

}  // namespace

TEST(RiskGridMapTest, FixedLatticeUsesDefaultAnchorEvenSideAndNegativeFloor) {
  auto params = base_params();
  params.resolution_m = 1.0;
  params.size_x_m = 4.0;
  params.size_y_m = 4.0;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0};
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-0.1, 0.2, -0.1),
                                       10.0, provider, &reason))
      << reason;
  const auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const Eigen::Vector3d expected_origin(-3.0, -2.0, -2.0);
  EXPECT_EQ(snapshot->voxelNum(), Eigen::Vector3i(4, 4, 2));
  EXPECT_EQ(snapshot->origin(), expected_origin);
  EXPECT_EQ(grid.origin(), expected_origin);
  EXPECT_EQ(snapshot->indexToPos(Eigen::Vector3i(2, 2, 1)),
            Eigen::Vector3d(-0.5, 0.5, -0.5));
  EXPECT_EQ(provider.query_count, 32);
}

TEST(RiskGridMapTest,
     FixedLatticeKeepsOrderedQueriesForPositiveAndNegativeSubvoxelMotion) {
  auto params = base_params();
  params.resolution_m = 0.75;
  params.size_x_m = 3.0;
  params.size_y_m = 3.0;
  params.size_z_m = 1.5;
  params.horizons_s = {0.0};
  iap::RiskGridMap grid(params);
  AffineProvider provider;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(0.01, 0.02, 0.03),
                                       10.0, provider));
  const Eigen::Vector3d positive_origin = grid.origin();
  const auto positive_queries = provider.last_queries;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(0.01, 0.02, 0.03),
                                       10.0, provider));
  EXPECT_EQ(grid.origin(), positive_origin);
  expect_query_positions_equal(provider.last_queries, positive_queries);
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(0.74, 0.73, 0.72),
                                       10.0, provider));
  EXPECT_EQ(grid.origin(), positive_origin);
  expect_query_positions_equal(provider.last_queries, positive_queries);

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-0.01, -0.02, -0.03),
                                       10.0, provider));
  const Eigen::Vector3d negative_origin = grid.origin();
  const auto negative_queries = provider.last_queries;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-0.74, -0.73, -0.72),
                                       10.0, provider));
  EXPECT_EQ(grid.origin(), negative_origin);
  expect_query_positions_equal(provider.last_queries, negative_queries);
}

TEST(RiskGridMapTest, FixedLatticeCrossingsUseExactIntegerVoxelDeltas) {
  auto params = base_params();
  params.resolution_m = 0.75;
  params.size_x_m = 3.0;
  params.size_y_m = 3.0;
  params.size_z_m = 1.5;
  params.horizons_s = {0.0};
  iap::RiskGridMap grid(params);
  AffineProvider provider;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(0.1, -0.1, 0.1),
                                       10.0, provider));
  const Eigen::Vector3d initial_origin = grid.origin();
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(0.75, -0.76, 0.1),
                                       10.0, provider));
  EXPECT_EQ(grid.origin() - initial_origin,
            Eigen::Vector3d(0.75, -0.75, 0.0));

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(3.1, -2.3, 1.6),
                                       10.0, provider));
  EXPECT_EQ(grid.origin() - initial_origin,
            Eigen::Vector3d(3.0, -2.25, 1.5));
}

TEST(RiskGridMapTest, FixedLatticeSupportsFiniteNonZeroAnchor) {
  auto params = base_params();
  params.resolution_m = 0.5;
  params.size_x_m = 2.0;
  params.size_y_m = 2.0;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0};
  params.lattice_anchor_w = Eigen::Vector3d(0.25, -0.5, 1.25);
  iap::RiskGridMap grid(params);
  AffineProvider provider;

  ASSERT_TRUE(grid.refreshFromProvider(params.lattice_anchor_w, 10.0,
                                       provider));
  EXPECT_EQ(grid.origin(), Eigen::Vector3d(-0.75, -1.5, 0.25));
  ASSERT_NE(grid.acquireSnapshot(), nullptr);
  EXPECT_EQ(grid.acquireSnapshot()->origin(), grid.origin());
}

TEST(RiskGridMapTest, ConfigureRejectsNonFiniteLatticeAnchor) {
  iap::RiskGridMap grid(base_params());
  auto invalid = base_params();
  invalid.lattice_anchor_w.x() =
      std::numeric_limits<double>::quiet_NaN();
  std::string reason;

  EXPECT_FALSE(grid.configure(invalid, &reason));
  EXPECT_EQ(reason, "invalid_lattice_anchor");
  EXPECT_EQ(grid.params().lattice_anchor_w, Eigen::Vector3d::Zero());
}

TEST(RiskGridMapTest,
     ReconfigureResetsGenerationAndRecomputesFixedLatticeGeometry) {
  iap::RiskGridMap grid(base_params());
  const auto first = make_snapshot(&grid);
  ASSERT_NE(first, nullptr);

  auto changed = base_params();
  changed.frame_id = "odom";
  changed.resolution_m = 0.5;
  changed.size_x_m = 2.1;
  changed.size_y_m = 1.1;
  changed.size_z_m = 0.6;
  changed.horizons_s = {0.0};
  changed.lattice_anchor_w = Eigen::Vector3d(0.25, -0.5, 1.25);
  std::string reason;
  ASSERT_TRUE(grid.configure(changed, &reason)) << reason;
  EXPECT_EQ(grid.acquireSnapshot(), nullptr);
  EXPECT_EQ(grid.health().generation_id, 0u);
  EXPECT_EQ(grid.voxelNum(), Eigen::Vector3i(5, 3, 2));
  EXPECT_EQ(grid.origin(), Eigen::Vector3d(-0.75, -1.0, 0.75));

  AffineProvider provider;
  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d(1.3, -1.1, 2.3), 20.0, provider, &reason)) << reason;
  const auto second = grid.acquireSnapshot();
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(second->generation_id(), 1u);
  EXPECT_EQ(second->params().frame_id, "odom");
  EXPECT_EQ(second->origin(), Eigen::Vector3d(0.25, -2.0, 1.75));
  EXPECT_EQ(grid.origin(), second->origin());
}

TEST(RiskGridMapTest,
     InFlightRefreshCannotPublishAcrossConfigureOrDuplicateGenerationIds) {
  iap::RiskGridMap grid(base_params());
  BlockingProvider provider;
  std::string first_reason;
  auto first_refresh = std::async(std::launch::async, [&]() {
    return grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0, provider,
                                    &first_reason);
  });
  if (!provider.waitUntilEntryCount(1)) {
    provider.release();
    FAIL() << "first refresh did not enter provider";
  }

  auto changed = base_params();
  changed.frame_id = "odom";
  changed.lattice_anchor_w = Eigen::Vector3d(0.25, -0.5, 1.25);
  std::string configure_reason;
  const bool configured = grid.configure(changed, &configure_reason);
  if (!configured) {
    provider.release();
    first_refresh.wait();
    FAIL() << configure_reason;
  }
  EXPECT_EQ(grid.acquireSnapshot(), nullptr);
  const Eigen::Vector3d configured_origin(-0.75, -1.5, 0.25);
  EXPECT_EQ(grid.origin(), configured_origin);

  provider.release();
  EXPECT_FALSE(first_refresh.get());
  EXPECT_EQ(first_reason, "configuration_changed");
  EXPECT_EQ(grid.acquireSnapshot(), nullptr);
  EXPECT_EQ(grid.origin(), configured_origin);

  BlockingProvider concurrent_provider;
  std::string second_reason;
  std::string third_reason;
  auto second_refresh = std::async(std::launch::async, [&]() {
    return grid.refreshFromProvider(changed.lattice_anchor_w, 20.0,
                                    concurrent_provider, &second_reason);
  });
  if (!concurrent_provider.waitUntilEntryCount(1)) {
    concurrent_provider.release();
    FAIL() << "concurrent refresh did not enter provider";
  }
  std::promise<void> third_started;
  auto third_started_future = third_started.get_future();
  auto third_refresh = std::async(std::launch::async, [&]() {
    third_started.set_value();
    return grid.refreshFromProvider(changed.lattice_anchor_w, 21.0,
                                    concurrent_provider, &third_reason);
  });
  if (third_started_future.wait_for(std::chrono::seconds(5)) !=
      std::future_status::ready) {
    concurrent_provider.release();
    second_refresh.wait();
    third_refresh.wait();
    FAIL() << "third refresh thread did not start";
  }
  EXPECT_EQ(third_refresh.wait_for(std::chrono::milliseconds(100)),
            std::future_status::timeout);
  EXPECT_EQ(concurrent_provider.entryCount(), 1);
  concurrent_provider.release();
  EXPECT_TRUE(second_refresh.get()) << second_reason;
  EXPECT_TRUE(third_refresh.get()) << third_reason;
  ASSERT_NE(grid.acquireSnapshot(), nullptr);
  EXPECT_EQ(grid.acquireSnapshot()->generation_id(), 2u);
}

TEST(RiskGridMapTest, SnappedOriginPreservesFullRefreshOrderAndScience) {
  auto params = base_params();
  params.resolution_m = 0.75;
  params.size_x_m = 3.0;
  params.size_y_m = 2.25;
  params.size_z_m = 1.5;
  iap::RiskGridMap grid(params);
  AffineProvider provider;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-0.1, 0.8, 0.2),
                                       10.0, provider));
  const auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const int expected_count = snapshot->layerVoxelCount() *
                             snapshot->horizonCount();
  ASSERT_EQ(provider.last_queries.size(),
            static_cast<std::size_t>(expected_count));
  EXPECT_EQ(provider.query_count, expected_count);
  EXPECT_EQ(snapshot->health().provider_query_count,
            static_cast<uint64_t>(expected_count));

  std::size_t query_index = 0;
  for (int h = 0; h < snapshot->horizonCount(); ++h) {
    for (int x = 0; x < snapshot->voxelNum().x(); ++x) {
      for (int y = 0; y < snapshot->voxelNum().y(); ++y) {
        for (int z = 0; z < snapshot->voxelNum().z(); ++z) {
          const Eigen::Vector3i id(x, y, z);
          const auto& query = provider.last_queries[query_index++];
          EXPECT_EQ(query.position_w, snapshot->indexToPos(id));
          EXPECT_DOUBLE_EQ(query.horizon_s,
                           params.horizons_s[static_cast<std::size_t>(h)]);
          iap::RiskVoxel voxel;
          ASSERT_TRUE(snapshot->voxelAt(h, id, &voxel));
          EXPECT_DOUBLE_EQ(
              voxel.hpl_pred,
              AffineProvider::affine(query.position_w, query.horizon_s));
        }
      }
    }
  }
}

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

  EXPECT_TRUE(snapshot->isInMap(Eigen::Vector3d(-0.5, 0.5, 1.5)));
  EXPECT_FALSE(snapshot->isInMap(Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_FALSE(snapshot->isInMap(Eigen::Vector3d(2.0, 0.0, 0.0)));
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

TEST(RiskGridMapTest, SnapshotVoxelAccessorExposesReadOnlyLayerData) {
  iap::RiskGridMap grid(base_params());
  auto snapshot = make_snapshot(&grid, 10.0);

  EXPECT_EQ(snapshot->horizonCount(), 2);
  EXPECT_EQ(snapshot->layerVoxelCount(), 27);

  const Eigen::Vector3i id(1, 1, 1);
  const Eigen::Vector3d p = snapshot->indexToPos(id);
  iap::RiskVoxel voxel;
  ASSERT_TRUE(snapshot->voxelAt(1, id, &voxel));
  EXPECT_TRUE(voxel.valid);
  EXPECT_FALSE(voxel.unknown);
  EXPECT_FALSE(voxel.stale);
  EXPECT_NEAR(voxel.hpl_pred, AffineProvider::affine(p, 1.0), 1.0e-9);
  EXPECT_NEAR(voxel.vpl_pred, 0.25 * voxel.hpl_pred, 1.0e-9);

  EXPECT_FALSE(snapshot->voxelAt(-1, id, &voxel));
  EXPECT_FALSE(snapshot->voxelAt(2, id, &voxel));
  EXPECT_FALSE(snapshot->voxelAt(0, Eigen::Vector3i(3, 0, 0), &voxel));
  EXPECT_FALSE(snapshot->voxelAt(0, id, nullptr));

  const iap::RiskGridSnapshot empty;
  EXPECT_EQ(empty.horizonCount(), 0);
  EXPECT_EQ(empty.layerVoxelCount(), 0);
  EXPECT_FALSE(empty.voxelAt(0, id, &voxel));
}

TEST(RiskGridMapTest, SkipOccupiedVoxelsCanBeDisabled) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = false;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d&) { return true; }, &reason))
      << reason;

  EXPECT_EQ(provider.query_count, 54);
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_DOUBLE_EQ(snapshot->health().valid_ratio, 1.0);
  EXPECT_DOUBLE_EQ(snapshot->health().unknown_ratio, 0.0);

  iap::RiskCostSample cost;
  EXPECT_TRUE(snapshot->queryCost(Eigen::Vector3d::Zero(), 10.0, &cost))
      << cost.reason;
  EXPECT_EQ(cost.reason, "ok");
}

TEST(RiskGridMapTest, SkipOccupiedVoxelsMarksOccupiedAsUnknown) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d&) { return true; }, &reason))
      << reason;

  EXPECT_EQ(provider.query_count, 0);
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_DOUBLE_EQ(snapshot->health().valid_ratio, 0.0);
  EXPECT_DOUBLE_EQ(snapshot->health().unknown_ratio, 1.0);

  iap::RiskCostSample cost;
  EXPECT_FALSE(snapshot->queryCost(Eigen::Vector3d::Zero(), 10.0, &cost));
  EXPECT_EQ(cost.reason, "occupied");
  EXPECT_DOUBLE_EQ(cost.cost, params.unknown_cost);

  iap::PredictedPLSample pl;
  EXPECT_FALSE(
      snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 10.0, &pl));
  EXPECT_EQ(pl.reason, "occupied");
  EXPECT_FALSE(pl.available);
  EXPECT_FALSE(pl.valid);
}

TEST(RiskGridMapTest, QueryTraceAttributesOccupiedInterpolationCorner) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  const auto occupied = [](const Eigen::Vector3d& p) {
    iap::RiskOccupancyDiagnostic diagnostic;
    diagnostic.available = true;
    diagnostic.raw_occupied = p.x() > 1.0;
    diagnostic.inflated_occupied = diagnostic.raw_occupied;
    diagnostic.frame_id = "map";
    diagnostic.cloud_stamp_s = 9.9;
    diagnostic.occupancy_generation = 7;
    diagnostic.resolution_m = 1.0;
    diagnostic.inflation_m = 0.0;
    diagnostic.source = "test_map";
    return diagnostic;
  };
  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider, occupied, &reason)) << reason;

  const auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const Eigen::Vector3d query_point(0.6, 0.6, 0.6);
  EXPECT_FALSE(occupied(query_point).inflated_occupied);

  iap::RiskCostSample cost;
  iap::RiskCostQueryTrace trace;
  EXPECT_FALSE(snapshot->queryCost(query_point, 10.5, &cost, &trace));
  EXPECT_EQ(cost.reason, "occupied");
  EXPECT_FALSE(cost.stale);
  EXPECT_FALSE(trace.success);
  EXPECT_EQ(trace.reason, "occupied");
  ASSERT_EQ(trace.corners.size(), 16u);
  const auto occupied_corners = std::count_if(
      trace.corners.begin(), trace.corners.end(), [](const auto& corner) {
        return corner.invalid_reason == "occupied" &&
               corner.occupancy.inflated_occupied;
      });
  EXPECT_EQ(occupied_corners, 8);
  for (const auto& corner : trace.corners) {
    EXPECT_TRUE(corner.occupancy.available);
    EXPECT_EQ(corner.occupancy.occupancy_generation, 7u);
    EXPECT_EQ(corner.occupancy.frame_id, "map");
    EXPECT_EQ(corner.voxel_index, corner.occupancy.voxel_index);
    EXPECT_TRUE(corner.voxel_position.isApprox(
        corner.occupancy.voxel_center));
  }
}

TEST(RiskGridMapTest, RefreshRejectsChangingOccupancyGeneration) {
  iap::RiskGridMapParams params = base_params();
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  int calls = 0;
  const auto changing_generation = [&calls](const Eigen::Vector3d&) {
    iap::RiskOccupancyDiagnostic diagnostic;
    diagnostic.available = true;
    diagnostic.occupancy_generation = ++calls < 20 ? 11u : 12u;
    diagnostic.source = "test_map";
    return diagnostic;
  };

  EXPECT_FALSE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      changing_generation, &reason));
  EXPECT_EQ(reason, "occupancy_generation_changed");
  EXPECT_EQ(provider.query_count, 0);
  EXPECT_EQ(grid.acquireSnapshot(), nullptr);
  EXPECT_EQ(grid.health().reason, "occupancy_generation_changed");
}

TEST(RiskGridMapTest, SkipOccupiedVoxelsKeepsFreePredicateBehavior) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d&) { return false; }, &reason))
      << reason;

  EXPECT_EQ(provider.query_count, 54);
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_DOUBLE_EQ(snapshot->health().valid_ratio, 1.0);
  EXPECT_DOUBLE_EQ(snapshot->health().unknown_ratio, 0.0);

  iap::PredictedPLSample pl;
  EXPECT_TRUE(
      snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 10.0, &pl))
      << pl.reason;
  EXPECT_EQ(pl.reason, "ok");
}

TEST(RiskGridMapTest, HealthRatiosUseTotalVoxelCountWithPartialOccupiedSkip) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d& p) { return p.x() < 0.0; }, &reason))
      << reason;

  EXPECT_EQ(provider.query_count, 36);
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const auto health = snapshot->health();
  EXPECT_NEAR(health.valid_ratio, 36.0 / 54.0, 1.0e-12);
  EXPECT_NEAR(health.unknown_ratio, 18.0 / 54.0, 1.0e-12);
  EXPECT_NEAR(health.valid_ratio + health.unknown_ratio, 1.0, 1.0e-12);
  EXPECT_EQ(health.provider_query_count, 36u);
  EXPECT_EQ(health.occupied_skip_count, 18u);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_EQ(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.reason, "ok");
  EXPECT_EQ(health.dominant_unknown_reason, "occupied_skip");
  EXPECT_EQ(health.dominant_unknown_count, 18u);
  EXPECT_GE(health.valid_ratio, 0.0);
  EXPECT_LE(health.valid_ratio, 1.0);
  EXPECT_GE(health.unknown_ratio, 0.0);
  EXPECT_LE(health.unknown_ratio, 1.0);
}

TEST(RiskGridMapTest, AllProviderStaleHealthReportsDominantReason) {
  iap::RiskGridMap grid(base_params());
  AlternatingStaleProvider provider;
  provider.all_stale = true;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  const auto health = grid.health();
  EXPECT_TRUE(health.ready);
  EXPECT_FALSE(health.stale);
  EXPECT_DOUBLE_EQ(health.valid_ratio, 0.0);
  EXPECT_DOUBLE_EQ(health.unknown_ratio, 1.0);
  EXPECT_EQ(health.provider_query_count, 54u);
  EXPECT_EQ(health.occupied_skip_count, 0u);
  EXPECT_EQ(health.provider_stale_count, 54u);
  EXPECT_EQ(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.reason, "stale_gnss_epoch");
  EXPECT_EQ(health.dominant_unknown_reason, "stale_gnss_epoch");
  EXPECT_EQ(health.dominant_unknown_count, 54u);
}

TEST(RiskGridMapTest, PartialProviderStaleKeepsOkHealthReasonWithCounters) {
  iap::RiskGridMap grid(base_params());
  AlternatingStaleProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  const auto health = grid.health();
  EXPECT_NEAR(health.valid_ratio, 27.0 / 54.0, 1.0e-12);
  EXPECT_NEAR(health.unknown_ratio, 27.0 / 54.0, 1.0e-12);
  EXPECT_EQ(health.provider_query_count, 54u);
  EXPECT_EQ(health.occupied_skip_count, 0u);
  EXPECT_EQ(health.provider_stale_count, 27u);
  EXPECT_EQ(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.reason, "ok");
  EXPECT_EQ(health.dominant_unknown_reason, "stale_gnss_epoch");
  EXPECT_EQ(health.dominant_unknown_count, 27u);
}

TEST(RiskGridMapTest, PartialProviderInvalidReportsDominantUnknownReason) {
  iap::RiskGridMap grid(base_params());
  AlternatingInvalidProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  const auto health = grid.health();
  EXPECT_NEAR(health.valid_ratio, 27.0 / 54.0, 1.0e-12);
  EXPECT_NEAR(health.unknown_ratio, 27.0 / 54.0, 1.0e-12);
  EXPECT_EQ(health.provider_query_count, 54u);
  EXPECT_EQ(health.provider_invalid_count, 27u);
  EXPECT_EQ(health.reason, "ok");
  EXPECT_EQ(health.dominant_unknown_reason, "too_few_lidar_normals");
  EXPECT_EQ(health.dominant_unknown_count, 27u);
}

TEST(RiskGridMapTest, HealthCountsPredictorSourceFlags) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  provider.source_flags =
      iap::PREDICTOR_RESULT_GNSS_USED |
      iap::PREDICTOR_RESULT_LIDAR_USED |
      iap::PREDICTOR_RESULT_PRIOR_VALID |
      iap::PREDICTOR_RESULT_STALE_CURRENT_PRIOR |
      iap::PREDICTOR_RESULT_REGULARIZED |
      iap::PREDICTOR_RESULT_CONSERVATIVE_MAX;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d& p) { return p.x() < 0.0; }, &reason))
      << reason;

  const auto health = grid.health();
  EXPECT_EQ(health.provider_query_count, 36u);
  EXPECT_EQ(health.occupied_skip_count, 18u);
  EXPECT_EQ(health.predictor_gnss_used_count, 36u);
  EXPECT_EQ(health.predictor_lidar_used_count, 36u);
  EXPECT_EQ(health.predictor_prior_used_count, 36u);
  EXPECT_EQ(health.predictor_stale_current_prior_count, 36u);
  EXPECT_EQ(health.predictor_regularized_count, 36u);
  EXPECT_EQ(health.predictor_conservative_max_count, 36u);
  EXPECT_EQ(health.dominant_unknown_reason, "occupied_skip");
  EXPECT_EQ(health.dominant_unknown_count, 18u);
}

TEST(RiskGridMapTest, P5_3FixtureDisabledByDefaultDoesNotAlterProviderOutput) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 1.0, 1.5, 2.0};
  ASSERT_FALSE(params.p5_3_fixture.enabled);
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const Eigen::Vector3i id(1, 1, 1);
  const Eigen::Vector3d p = snapshot->indexToPos(id);
  iap::RiskVoxel voxel;
  ASSERT_TRUE(snapshot->voxelAt(2, id, &voxel));
  EXPECT_TRUE(voxel.valid);
  EXPECT_FALSE(voxel.stale);
  EXPECT_EQ(voxel.reason, "ok");
  EXPECT_NEAR(voxel.hpl_pred, AffineProvider::affine(p, 1.5), 1.0e-9);
  EXPECT_NEAR(voxel.vpl_pred, 0.25 * AffineProvider::affine(p, 1.5), 1.0e-9);
}

TEST(RiskGridMapTest, P5_3FixtureOnlyAltersCellsInsideBoundsAndTauWindow) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 1.0, 1.5, 2.0};
  params.p5_3_fixture.enabled = true;
  params.p5_3_fixture.name = "future_high_risk_zone_v1";
  params.p5_3_fixture.x_min_m = -0.6;
  params.p5_3_fixture.x_max_m = 0.6;
  params.p5_3_fixture.y_min_m = -0.6;
  params.p5_3_fixture.y_max_m = 0.6;
  params.p5_3_fixture.z_min_m = -0.6;
  params.p5_3_fixture.z_max_m = 0.6;
  params.p5_3_fixture.tau_min_s = 1.2;
  params.p5_3_fixture.tau_max_s = 2.0;
  params.p5_3_fixture.hpl_pred_m = 10.2;
  params.p5_3_fixture.vpl_pred_m = 10.2;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  iap::RiskVoxel center_tau_1_5;
  ASSERT_TRUE(snapshot->voxelAt(2, Eigen::Vector3i(1, 1, 1),
                               &center_tau_1_5));
  EXPECT_TRUE(center_tau_1_5.valid);
  EXPECT_FALSE(center_tau_1_5.stale);
  EXPECT_FALSE(center_tau_1_5.unknown);
  EXPECT_DOUBLE_EQ(center_tau_1_5.hpl_pred, 10.2);
  EXPECT_DOUBLE_EQ(center_tau_1_5.vpl_pred, 10.2);
  EXPECT_EQ(center_tau_1_5.reason, "p5_3_high_risk_zone");

  iap::RiskVoxel center_tau_1_0;
  ASSERT_TRUE(snapshot->voxelAt(1, Eigen::Vector3i(1, 1, 1),
                               &center_tau_1_0));
  const Eigen::Vector3d center_p =
      snapshot->indexToPos(Eigen::Vector3i(1, 1, 1));
  EXPECT_NEAR(center_tau_1_0.hpl_pred,
              AffineProvider::affine(center_p, 1.0), 1.0e-9);
  EXPECT_EQ(center_tau_1_0.reason, "ok");

  iap::RiskVoxel outside_space;
  ASSERT_TRUE(snapshot->voxelAt(2, Eigen::Vector3i(2, 1, 1),
                               &outside_space));
  const Eigen::Vector3d outside_p =
      snapshot->indexToPos(Eigen::Vector3i(2, 1, 1));
  EXPECT_NEAR(outside_space.hpl_pred,
              AffineProvider::affine(outside_p, 1.5), 1.0e-9);
  EXPECT_EQ(outside_space.reason, "ok");
}

TEST(RiskGridMapTest,
     P5_3DefaultFixtureExcludesCurrentPointAndIncludesFutureCorridor) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.2;
  params.size_x_m = 5.0;
  params.size_y_m = 2.5;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 1.2, 1.5, 2.0};
  params.stale_timeout_s = 10.0;
  params.p5_3_fixture.enabled = true;
  ASSERT_EQ(params.p5_3_fixture.name, "future_high_risk_zone_v1");
  ASSERT_DOUBLE_EQ(params.p5_3_fixture.x_min_m, -10.8);
  ASSERT_DOUBLE_EQ(params.p5_3_fixture.x_max_m, -8.7);

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-10.4, 0.0, 1.2),
                                       10.0, provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  const Eigen::Vector3d current_point(-12.0, 0.0, 1.2);
  iap::PredictedPLSample current_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(current_point, 10.0,
                                        &current_sample))
      << current_sample.reason;
  EXPECT_TRUE(current_sample.valid);
  EXPECT_EQ(current_sample.reason, "ok");
  EXPECT_DOUBLE_EQ(current_sample.query_tau_s, 0.0);
  EXPECT_FALSE(current_sample.fixture_match);
  EXPECT_FALSE(std::isfinite(current_sample.fixture_expected_hpl));
  EXPECT_FALSE(std::isfinite(current_sample.fixture_expected_vpl));
  EXPECT_TRUE(current_sample.fixture_expected_reason.empty());
  EXPECT_NEAR(current_sample.hpl_pred,
              AffineProvider::affine(current_point, 0.0), 1.0e-9);

  const Eigen::Vector3d future_point(-10.2, 0.0, 1.2);
  iap::PredictedPLSample current_inside_space_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(future_point, 10.0,
                                        &current_inside_space_sample))
      << current_inside_space_sample.reason;
  EXPECT_TRUE(current_inside_space_sample.valid);
  EXPECT_EQ(current_inside_space_sample.reason, "ok");
  EXPECT_DOUBLE_EQ(current_inside_space_sample.query_tau_s, 0.0);
  EXPECT_FALSE(current_inside_space_sample.fixture_match);
  EXPECT_FALSE(std::isfinite(current_inside_space_sample.fixture_expected_hpl));
  EXPECT_FALSE(std::isfinite(current_inside_space_sample.fixture_expected_vpl));
  EXPECT_TRUE(current_inside_space_sample.fixture_expected_reason.empty());
  EXPECT_NEAR(current_inside_space_sample.hpl_pred,
              AffineProvider::affine(future_point, 0.0), 1.0e-9);

  for (const double tau : {1.2, 1.5, 2.0}) {
    iap::PredictedPLSample future_sample;
    ASSERT_TRUE(snapshot->queryPredictedPL(future_point, 10.0 + tau,
                                          &future_sample))
        << future_sample.reason;
    EXPECT_TRUE(future_sample.valid);
    EXPECT_TRUE(future_sample.available);
    EXPECT_FALSE(future_sample.stale);
    EXPECT_EQ(future_sample.reason, "p5_3_high_risk_zone");
    EXPECT_NEAR(future_sample.query_tau_s, tau, 1.0e-12);
    EXPECT_TRUE(future_sample.fixture_match);
    EXPECT_DOUBLE_EQ(future_sample.fixture_expected_hpl, 10.2);
    EXPECT_DOUBLE_EQ(future_sample.fixture_expected_vpl, 10.2);
    EXPECT_EQ(future_sample.fixture_expected_reason, "p5_3_high_risk_zone");
    EXPECT_DOUBLE_EQ(future_sample.hpl_pred, 10.2);
    EXPECT_DOUBLE_EQ(future_sample.vpl_pred, 10.2);
  }
}

TEST(RiskGridMapTest, P5_3QueryTimeFixtureBypassesOccupiedSkip) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.2;
  params.size_x_m = 5.0;
  params.size_y_m = 2.5;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 1.2, 1.5, 2.0};
  params.stale_timeout_s = 10.0;
  params.skip_occupied_voxels = true;
  params.p5_3_fixture.enabled = true;

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d(-10.4, 0.0, 1.2), 10.0, provider,
      [](const Eigen::Vector3d&) { return true; }, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  iap::PredictedPLSample future_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(Eigen::Vector3d(-10.2, 0.0, 1.2),
                                        11.5, &future_sample))
      << future_sample.reason;
  EXPECT_TRUE(future_sample.fixture_match);
  EXPECT_TRUE(future_sample.valid);
  EXPECT_TRUE(future_sample.available);
  EXPECT_FALSE(future_sample.stale);
  EXPECT_EQ(future_sample.reason, "p5_3_high_risk_zone");
  EXPECT_DOUBLE_EQ(future_sample.query_tau_s, 1.5);
  EXPECT_DOUBLE_EQ(future_sample.fixture_expected_hpl, 10.2);
  EXPECT_DOUBLE_EQ(future_sample.fixture_expected_vpl, 10.2);
  EXPECT_EQ(future_sample.fixture_expected_reason, "p5_3_high_risk_zone");
  EXPECT_DOUBLE_EQ(future_sample.hpl_pred, 10.2);
  EXPECT_DOUBLE_EQ(future_sample.vpl_pred, 10.2);
}

TEST(RiskGridMapTest, P5_4FixtureDisabledByDefaultDoesNotAlterProviderOutput) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 0.6, 0.8, 1.0};
  ASSERT_FALSE(params.p5_4_fixture.enabled);
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const Eigen::Vector3i id(1, 1, 1);
  const Eigen::Vector3d p = snapshot->indexToPos(id);
  iap::RiskVoxel voxel;
  ASSERT_TRUE(snapshot->voxelAt(1, id, &voxel));
  EXPECT_TRUE(voxel.valid);
  EXPECT_FALSE(voxel.stale);
  EXPECT_EQ(voxel.reason, "ok");
  EXPECT_NEAR(voxel.hpl_pred, AffineProvider::affine(p, 0.6), 1.0e-9);
  EXPECT_NEAR(voxel.vpl_pred, 0.25 * AffineProvider::affine(p, 0.6),
              1.0e-9);
}

TEST(RiskGridMapTest, P5_4FixtureOnlyAltersNearRiskCellsInsideBoundsAndTauWindow) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 0.6, 0.8, 1.0};
  params.p5_4_fixture.enabled = true;
  params.p5_4_fixture.name = "near_risk_zone_v1";
  params.p5_4_fixture.x_min_m = -0.6;
  params.p5_4_fixture.x_max_m = 0.6;
  params.p5_4_fixture.y_min_m = -0.6;
  params.p5_4_fixture.y_max_m = 0.6;
  params.p5_4_fixture.z_min_m = -0.6;
  params.p5_4_fixture.z_max_m = 0.6;
  params.p5_4_fixture.tau_min_s = 0.6;
  params.p5_4_fixture.tau_max_s = 0.95;
  params.p5_4_fixture.hpl_pred_m = 10.2;
  params.p5_4_fixture.vpl_pred_m = 10.2;
  ASSERT_FALSE(params.p5_3_fixture.enabled);
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  iap::RiskVoxel center_tau_0_6;
  ASSERT_TRUE(snapshot->voxelAt(1, Eigen::Vector3i(1, 1, 1),
                               &center_tau_0_6));
  EXPECT_TRUE(center_tau_0_6.valid);
  EXPECT_FALSE(center_tau_0_6.stale);
  EXPECT_FALSE(center_tau_0_6.unknown);
  EXPECT_DOUBLE_EQ(center_tau_0_6.hpl_pred, 10.2);
  EXPECT_DOUBLE_EQ(center_tau_0_6.vpl_pred, 10.2);
  EXPECT_EQ(center_tau_0_6.reason, "p5_4_near_risk_zone");

  iap::RiskVoxel center_tau_1_0;
  ASSERT_TRUE(snapshot->voxelAt(3, Eigen::Vector3i(1, 1, 1),
                               &center_tau_1_0));
  const Eigen::Vector3d center_p =
      snapshot->indexToPos(Eigen::Vector3i(1, 1, 1));
  EXPECT_NEAR(center_tau_1_0.hpl_pred,
              AffineProvider::affine(center_p, 1.0), 1.0e-9);
  EXPECT_EQ(center_tau_1_0.reason, "ok");

  iap::RiskVoxel outside_space;
  ASSERT_TRUE(snapshot->voxelAt(1, Eigen::Vector3i(2, 1, 1),
                               &outside_space));
  const Eigen::Vector3d outside_p =
      snapshot->indexToPos(Eigen::Vector3i(2, 1, 1));
  EXPECT_NEAR(outside_space.hpl_pred,
              AffineProvider::affine(outside_p, 0.6), 1.0e-9);
  EXPECT_EQ(outside_space.reason, "ok");
}

TEST(RiskGridMapTest,
     P5_4DefaultFixtureIncludesNearRiskEmergencyWindowWithoutAffectingP5_3) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.2;
  params.size_x_m = 5.0;
  params.size_y_m = 2.5;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 0.6, 0.8, 0.95, 1.0};
  params.stale_timeout_s = 10.0;
  params.p5_4_fixture.enabled = true;
  ASSERT_EQ(params.p5_4_fixture.name, "near_risk_zone_v1");
  ASSERT_DOUBLE_EQ(params.p5_4_fixture.x_min_m, -11.7);
  ASSERT_DOUBLE_EQ(params.p5_4_fixture.x_max_m, -11.1);
  ASSERT_FALSE(params.p5_3_fixture.enabled);

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-11.4, 0.0, 1.2),
                                       10.0, provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  const Eigen::Vector3d near_risk_point(-11.4, 0.0, 1.2);
  iap::PredictedPLSample current_inside_space_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(near_risk_point, 10.0,
                                        &current_inside_space_sample))
      << current_inside_space_sample.reason;
  EXPECT_TRUE(current_inside_space_sample.valid);
  EXPECT_EQ(current_inside_space_sample.reason, "ok");
  EXPECT_DOUBLE_EQ(current_inside_space_sample.query_tau_s, 0.0);
  EXPECT_FALSE(current_inside_space_sample.fixture_match);
  EXPECT_NEAR(current_inside_space_sample.hpl_pred,
              AffineProvider::affine(near_risk_point, 0.0), 1.0e-9);

  for (const double tau : {0.6, 0.8, 0.95}) {
    iap::PredictedPLSample future_sample;
    ASSERT_TRUE(snapshot->queryPredictedPL(near_risk_point, 10.0 + tau,
                                          &future_sample))
        << future_sample.reason;
    EXPECT_TRUE(future_sample.valid);
    EXPECT_TRUE(future_sample.available);
    EXPECT_FALSE(future_sample.stale);
    EXPECT_EQ(future_sample.reason, "p5_4_near_risk_zone");
    EXPECT_NEAR(future_sample.query_tau_s, tau, 1.0e-12);
    EXPECT_TRUE(future_sample.fixture_match);
    EXPECT_DOUBLE_EQ(future_sample.fixture_expected_hpl, 10.2);
    EXPECT_DOUBLE_EQ(future_sample.fixture_expected_vpl, 10.2);
    EXPECT_EQ(future_sample.fixture_expected_reason, "p5_4_near_risk_zone");
    EXPECT_DOUBLE_EQ(future_sample.hpl_pred, 10.2);
    EXPECT_DOUBLE_EQ(future_sample.vpl_pred, 10.2);
  }
}

TEST(RiskGridMapTest, P5_4QueryCanUseP5HorizonTauWithoutChangingP5_3) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.2;
  params.size_x_m = 5.0;
  params.size_y_m = 2.5;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 0.6, 0.8, 0.95, 1.0, 1.5, 2.0};
  params.stale_timeout_s = 10.0;
  params.p5_4_fixture.enabled = true;
  params.p5_3_fixture.enabled = true;
  params.p5_3_fixture.name = "future_high_risk_zone_v1";
  params.p5_3_fixture.x_min_m = -11.7;
  params.p5_3_fixture.x_max_m = -11.1;
  params.p5_3_fixture.y_min_m = -0.75;
  params.p5_3_fixture.y_max_m = 0.75;
  params.p5_3_fixture.z_min_m = 1.0;
  params.p5_3_fixture.z_max_m = 1.35;
  params.p5_3_fixture.tau_min_s = 1.4;
  params.p5_3_fixture.tau_max_s = 1.6;
  params.p5_3_fixture.hpl_pred_m = 12.3;
  params.p5_3_fixture.vpl_pred_m = 12.4;

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-11.4, 0.0, 1.2),
                                       10.0, provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  const Eigen::Vector3d near_risk_point(-11.4, 0.0, 1.2);
  iap::PredictedPLSample p5_4_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(near_risk_point, 11.5,
                                        &p5_4_sample, 0.8))
      << p5_4_sample.reason;
  EXPECT_NEAR(p5_4_sample.query_tau_s, 1.5, 1.0e-12);
  EXPECT_TRUE(p5_4_sample.fixture_match);
  EXPECT_EQ(p5_4_sample.reason, "p5_4_near_risk_zone");
  EXPECT_DOUBLE_EQ(p5_4_sample.hpl_pred, 10.2);
  EXPECT_DOUBLE_EQ(p5_4_sample.vpl_pred, 10.2);

  iap::PredictedPLSample p5_3_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(near_risk_point, 11.5,
                                        &p5_3_sample, 0.2))
      << p5_3_sample.reason;
  EXPECT_NEAR(p5_3_sample.query_tau_s, 1.5, 1.0e-12);
  EXPECT_TRUE(p5_3_sample.fixture_match);
  EXPECT_EQ(p5_3_sample.reason, "p5_3_high_risk_zone");
  EXPECT_DOUBLE_EQ(p5_3_sample.hpl_pred, 12.3);
  EXPECT_DOUBLE_EQ(p5_3_sample.vpl_pred, 12.4);
}

TEST(RiskGridMapTest, P5_4QueryTimeFixtureBypassesOccupiedSkip) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.2;
  params.size_x_m = 5.0;
  params.size_y_m = 2.5;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 0.6, 0.8, 0.95, 1.0};
  params.stale_timeout_s = 10.0;
  params.skip_occupied_voxels = true;
  params.p5_4_fixture.enabled = true;

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d(-11.4, 0.0, 1.2), 10.0, provider,
      [](const Eigen::Vector3d&) { return true; }, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  iap::PredictedPLSample future_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(Eigen::Vector3d(-11.4, 0.0, 1.2),
                                        10.8, &future_sample))
      << future_sample.reason;
  EXPECT_TRUE(future_sample.fixture_match);
  EXPECT_TRUE(future_sample.valid);
  EXPECT_TRUE(future_sample.available);
  EXPECT_FALSE(future_sample.stale);
  EXPECT_EQ(future_sample.reason, "p5_4_near_risk_zone");
  EXPECT_NEAR(future_sample.query_tau_s, 0.8, 1.0e-12);
  EXPECT_DOUBLE_EQ(future_sample.fixture_expected_hpl, 10.2);
  EXPECT_DOUBLE_EQ(future_sample.fixture_expected_vpl, 10.2);
  EXPECT_EQ(future_sample.fixture_expected_reason, "p5_4_near_risk_zone");
  EXPECT_DOUBLE_EQ(future_sample.hpl_pred, 10.2);
  EXPECT_DOUBLE_EQ(future_sample.vpl_pred, 10.2);
}

TEST(RiskGridMapTest, P5_6FixtureDisabledByDefaultDoesNotAlterProviderOutput) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 0.5, 1.0, 1.5, 2.0};
  ASSERT_FALSE(params.p5_6_fixture.enabled);
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const Eigen::Vector3d query(0.0, 0.0, 0.0);
  iap::PredictedPLSample pl;
  ASSERT_TRUE(snapshot->queryPredictedPL(query, 10.5, &pl)) << pl.reason;
  EXPECT_TRUE(pl.available);
  EXPECT_TRUE(pl.valid);
  EXPECT_FALSE(pl.stale);
  EXPECT_FALSE(pl.fixture_match);
  EXPECT_EQ(pl.reason, "ok");
  EXPECT_NEAR(pl.hpl_pred, AffineProvider::affine(query, 0.5), 1.0e-9);
}

TEST(RiskGridMapTest, P5_7FixtureOnlyAltersFinalCandidateQueries) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.2;
  params.size_x_m = 5.0;
  params.size_y_m = 2.5;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 0.6, 1.0, 1.5, 2.0};
  params.stale_timeout_s = 10.0;
  params.p5_7_fixture.enabled = true;
  params.p5_7_fixture.effective_enabled = true;
  ASSERT_EQ(params.p5_7_fixture.name, "rejected_trajectory_zone_v1");
  ASSERT_DOUBLE_EQ(params.p5_7_fixture.x_min_m, -11.7);
  ASSERT_DOUBLE_EQ(params.p5_7_fixture.x_max_m, -8.7);

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d(-10.2, 0.0, 1.2),
                                       10.0, provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  const Eigen::Vector3d rejected_point(-10.2, 0.0, 1.2);

  iap::PredictedPLSample runtime_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(rejected_point, 10.8,
                                        &runtime_sample))
      << runtime_sample.reason;
  EXPECT_TRUE(runtime_sample.valid);
  EXPECT_EQ(runtime_sample.reason, "ok");
  EXPECT_FALSE(runtime_sample.fixture_match);
  EXPECT_NEAR(runtime_sample.hpl_pred,
              AffineProvider::affine(rejected_point, 0.8), 1.0e-9);

  iap::PredictedPLSample final_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(
      rejected_point, 10.8, &final_sample,
      std::numeric_limits<double>::quiet_NaN(), true))
      << final_sample.reason;
  EXPECT_TRUE(final_sample.valid);
  EXPECT_TRUE(final_sample.available);
  EXPECT_FALSE(final_sample.stale);
  EXPECT_EQ(final_sample.reason, "p5_7_rejected_trajectory");
  EXPECT_NEAR(final_sample.query_tau_s, 0.8, 1.0e-12);
  EXPECT_TRUE(final_sample.fixture_match);
  EXPECT_DOUBLE_EQ(final_sample.fixture_expected_hpl, 10.2);
  EXPECT_DOUBLE_EQ(final_sample.fixture_expected_vpl, 10.2);
  EXPECT_EQ(final_sample.fixture_expected_reason,
            "p5_7_rejected_trajectory");
  EXPECT_DOUBLE_EQ(final_sample.hpl_pred, 10.2);
  EXPECT_DOUBLE_EQ(final_sample.vpl_pred, 10.2);
}

TEST(RiskGridMapTest, P5_6FixtureMarksOnlyInBoundsTauSamplesUnknown) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 0.5, 1.0, 1.5, 2.0};
  params.p5_6_fixture.enabled = true;
  params.p5_6_fixture.name = "future_unknown_zone_v1";
  params.p5_6_fixture.x_min_m = -0.6;
  params.p5_6_fixture.x_max_m = 0.6;
  params.p5_6_fixture.y_min_m = -0.6;
  params.p5_6_fixture.y_max_m = 0.6;
  params.p5_6_fixture.z_min_m = -0.6;
  params.p5_6_fixture.z_max_m = 0.6;
  params.p5_6_fixture.tau_min_s = 0.4;
  params.p5_6_fixture.tau_max_s = 1.1;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;

  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);
  iap::RiskVoxel center_tau_0_5;
  ASSERT_TRUE(snapshot->voxelAt(1, Eigen::Vector3i(1, 1, 1),
                               &center_tau_0_5));
  EXPECT_FALSE(center_tau_0_5.valid);
  EXPECT_FALSE(center_tau_0_5.stale);
  EXPECT_TRUE(center_tau_0_5.unknown);
  EXPECT_EQ(center_tau_0_5.reason, "future_unknown");
  EXPECT_FALSE(std::isfinite(center_tau_0_5.hpl_pred));
  EXPECT_FALSE(std::isfinite(center_tau_0_5.vpl_pred));

  iap::PredictedPLSample in_fixture;
  EXPECT_FALSE(snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 10.5,
                                          &in_fixture));
  EXPECT_TRUE(in_fixture.available);
  EXPECT_FALSE(in_fixture.valid);
  EXPECT_FALSE(in_fixture.stale);
  EXPECT_TRUE(in_fixture.fixture_match);
  EXPECT_EQ(in_fixture.reason, "future_unknown");
  EXPECT_EQ(in_fixture.fixture_expected_reason, "future_unknown");
  EXPECT_FALSE(std::isfinite(in_fixture.hpl_pred));
  EXPECT_FALSE(std::isfinite(in_fixture.vpl_pred));
  EXPECT_FALSE(std::isfinite(in_fixture.fixture_expected_hpl));
  EXPECT_FALSE(std::isfinite(in_fixture.fixture_expected_vpl));

  iap::PredictedPLSample current_tau;
  ASSERT_TRUE(snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 10.0,
                                        &current_tau))
      << current_tau.reason;
  EXPECT_TRUE(current_tau.valid);
  EXPECT_EQ(current_tau.reason, "ok");
  EXPECT_FALSE(current_tau.fixture_match);

  iap::RiskVoxel outside_space;
  ASSERT_TRUE(snapshot->voxelAt(1, Eigen::Vector3i(2, 1, 1),
                               &outside_space));
  EXPECT_TRUE(outside_space.valid);
  EXPECT_EQ(outside_space.reason, "ok");
  const Eigen::Vector3d outside_position =
      snapshot->indexToPos(Eigen::Vector3i(2, 1, 1));
  EXPECT_NEAR(outside_space.hpl_pred,
              AffineProvider::affine(outside_position, 0.5),
              1.0e-9);
}

TEST(RiskGridMapTest, P5_6FixtureDoesNotOverrideP5_3OrP5_4Fixtures) {
  iap::RiskGridMapParams params = base_params();
  params.horizons_s = {0.0, 0.6, 1.0, 1.5, 2.0};
  params.p5_3_fixture.enabled = true;
  params.p5_3_fixture.name = "future_high_risk_zone_v1";
  params.p5_3_fixture.x_min_m = -0.6;
  params.p5_3_fixture.x_max_m = 0.6;
  params.p5_3_fixture.y_min_m = -0.6;
  params.p5_3_fixture.y_max_m = 0.6;
  params.p5_3_fixture.z_min_m = -0.6;
  params.p5_3_fixture.z_max_m = 0.6;
  params.p5_3_fixture.tau_min_s = 1.4;
  params.p5_3_fixture.tau_max_s = 1.6;
  params.p5_3_fixture.hpl_pred_m = 12.3;
  params.p5_3_fixture.vpl_pred_m = 12.4;
  params.p5_4_fixture.enabled = true;
  params.p5_4_fixture.name = "near_risk_zone_v1";
  params.p5_4_fixture.x_min_m = -0.6;
  params.p5_4_fixture.x_max_m = 0.6;
  params.p5_4_fixture.y_min_m = -0.6;
  params.p5_4_fixture.y_max_m = 0.6;
  params.p5_4_fixture.z_min_m = -0.6;
  params.p5_4_fixture.z_max_m = 0.6;
  params.p5_4_fixture.tau_min_s = 0.5;
  params.p5_4_fixture.tau_max_s = 0.7;
  params.p5_4_fixture.hpl_pred_m = 10.3;
  params.p5_4_fixture.vpl_pred_m = 10.4;
  params.p5_6_fixture.enabled = true;
  params.p5_6_fixture.name = "future_unknown_zone_v1";
  params.p5_6_fixture.x_min_m = -0.6;
  params.p5_6_fixture.x_max_m = 0.6;
  params.p5_6_fixture.y_min_m = -0.6;
  params.p5_6_fixture.y_max_m = 0.6;
  params.p5_6_fixture.z_min_m = -0.6;
  params.p5_6_fixture.z_max_m = 0.6;
  params.p5_6_fixture.tau_min_s = 0.5;
  params.p5_6_fixture.tau_max_s = 1.6;

  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider, &reason))
      << reason;
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  iap::PredictedPLSample p5_4_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 10.6,
                                        &p5_4_sample))
      << p5_4_sample.reason;
  EXPECT_TRUE(p5_4_sample.valid);
  EXPECT_EQ(p5_4_sample.reason, "p5_4_near_risk_zone");
  EXPECT_DOUBLE_EQ(p5_4_sample.hpl_pred, 10.3);
  EXPECT_DOUBLE_EQ(p5_4_sample.vpl_pred, 10.4);

  iap::PredictedPLSample p5_3_sample;
  ASSERT_TRUE(snapshot->queryPredictedPL(Eigen::Vector3d::Zero(), 11.5,
                                        &p5_3_sample))
      << p5_3_sample.reason;
  EXPECT_TRUE(p5_3_sample.valid);
  EXPECT_EQ(p5_3_sample.reason, "p5_3_high_risk_zone");
  EXPECT_DOUBLE_EQ(p5_3_sample.hpl_pred, 12.3);
  EXPECT_DOUBLE_EQ(p5_3_sample.vpl_pred, 12.4);
}

TEST(RiskGridMapTest, AllOccupiedSkipHealthReportsDominantReason) {
  iap::RiskGridMapParams params = base_params();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  AffineProvider provider;
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d&) { return true; }, &reason))
      << reason;

  const auto health = grid.health();
  EXPECT_DOUBLE_EQ(health.valid_ratio, 0.0);
  EXPECT_DOUBLE_EQ(health.unknown_ratio, 1.0);
  EXPECT_EQ(health.provider_query_count, 0u);
  EXPECT_EQ(health.occupied_skip_count, 54u);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_EQ(health.provider_invalid_count, 0u);
  EXPECT_EQ(health.reason, "occupied_skip");
  EXPECT_EQ(health.dominant_unknown_reason, "occupied_skip");
  EXPECT_EQ(health.dominant_unknown_count, 54u);
}

TEST(RiskGridMapTest, AllProviderInvalidHealthReportsDominantReason) {
  iap::RiskGridMap grid(base_params());
  AffineProvider provider;
  provider.mark_unknown = true;

  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0,
                                       provider));

  const auto health = grid.health();
  EXPECT_DOUBLE_EQ(health.valid_ratio, 0.0);
  EXPECT_DOUBLE_EQ(health.unknown_ratio, 1.0);
  EXPECT_EQ(health.provider_query_count, 54u);
  EXPECT_EQ(health.occupied_skip_count, 0u);
  EXPECT_EQ(health.provider_stale_count, 0u);
  EXPECT_EQ(health.provider_invalid_count, 54u);
  EXPECT_EQ(health.reason, "forced_unknown");
  EXPECT_EQ(health.dominant_unknown_reason, "forced_unknown");
  EXPECT_EQ(health.dominant_unknown_count, 54u);
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
  const Eigen::Vector3d first_origin = first->origin();

  AffineProvider provider;
  provider.fail = true;
  std::string reason;
  EXPECT_FALSE(grid.refreshFromProvider(Eigen::Vector3d(8.0, -7.0, 6.0), 10.5,
                                        provider, &reason));
  EXPECT_EQ(reason, "provider_refresh_failed");
  auto still_active = grid.acquireSnapshot();
  ASSERT_NE(still_active, nullptr);
  EXPECT_EQ(still_active->generation_id(), first_id);
  EXPECT_EQ(still_active->origin(), first_origin);
  EXPECT_EQ(grid.origin(), first_origin);
  expect_snapshot_voxels_equal(*first, *still_active);
  EXPECT_EQ(grid.health().generation_id, first_id);
  EXPECT_EQ(grid.health().reason, "provider_refresh_failed");
}

TEST(RiskGridMapTest,
     SourceValidatorRunsAtStartAndImmediatelyBeforeAtomicPublish) {
  iap::RiskGridMap grid(base_params());
  AffineProvider provider;
  int validator_calls = 0;
  const auto validator = [&]() {
    ++validator_calls;
    return iap::RiskGridSourceValidation::VALID;
  };
  std::string reason;

  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      iap::RiskGridMap::OccupancyDiagnosticQuery{}, validator, &reason))
      << reason;
  EXPECT_EQ(validator_calls, 2);
  EXPECT_EQ(reason, "ok");
  ASSERT_NE(grid.acquireSnapshot(), nullptr);
  EXPECT_EQ(grid.acquireSnapshot()->generation_id(), 1u);
}

TEST(RiskGridMapTest,
     PriorGenerationFailureKeepsPreviousActiveSnapshot) {
  iap::RiskGridMap grid(base_params());
  const auto first = make_snapshot(&grid, 10.0);
  ASSERT_NE(first, nullptr);
  const uint64_t first_id = first->generation_id();
  iap::RiskCostSample first_sample;
  ASSERT_TRUE(first->queryCost(Eigen::Vector3d::Zero(), 10.0,
                               &first_sample));

  AffineProvider provider;
  int validator_calls = 0;
  for (const auto failure : {
           iap::RiskGridSourceValidation::OCCUPANCY_GENERATION_CHANGED,
           iap::RiskGridSourceValidation::PRIOR_GENERATION_CHANGED}) {
    validator_calls = 0;
    const auto validator = [&]() {
      return ++validator_calls == 1
          ? iap::RiskGridSourceValidation::VALID
          : failure;
    };
    std::string reason;
    EXPECT_FALSE(grid.refreshFromProvider(
        Eigen::Vector3d(8.0, -7.0, 6.0), 10.5, provider,
        iap::RiskGridMap::OccupancyDiagnosticQuery{}, validator, &reason));
    EXPECT_EQ(validator_calls, 2);
    EXPECT_EQ(reason,
              failure ==
                      iap::RiskGridSourceValidation::OCCUPANCY_GENERATION_CHANGED
                  ? "occupancy_generation_changed"
                  : "prior_generation_changed");

    const auto still_active = grid.acquireSnapshot();
    ASSERT_NE(still_active, nullptr);
    EXPECT_EQ(still_active->generation_id(), first_id);
    EXPECT_EQ(still_active->origin(), first->origin());
    EXPECT_EQ(grid.origin(), first->origin());
    expect_snapshot_voxels_equal(*first, *still_active);
    iap::RiskCostSample retained_sample;
    ASSERT_TRUE(still_active->queryCost(Eigen::Vector3d::Zero(), 10.0,
                                        &retained_sample));
    EXPECT_DOUBLE_EQ(retained_sample.cost, first_sample.cost);
    EXPECT_EQ(grid.health().generation_id, first_id);
    EXPECT_EQ(grid.health().reason, reason);
  }
}
