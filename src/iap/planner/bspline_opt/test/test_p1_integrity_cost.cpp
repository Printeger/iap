#include <bspline_opt/bspline_optimizer.h>
#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

class AffineProvider : public iap::RiskPredictionProvider {
 public:
  AffineProvider(const double base, const Eigen::Vector3d& grad)
      : base_(base), grad_(grad) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      const double value = base_ + grad_.dot(query.position_w);
      result.hpl_pred = value;
      result.vpl_pred = value;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }

 private:
  double base_;
  Eigen::Vector3d grad_;
};

class UnknownProvider : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    results->assign(queries.size(), iap::RiskPredictionResult{});
    for (auto& result : *results) {
      result.reason = "test_unknown";
    }
    return true;
  }
};

class StaleProvider : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    results->clear();
    results->reserve(queries.size());
    for (std::size_t i = 0; i < queries.size(); ++i) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = true;
      result.hpl_pred = 10.0;
      result.vpl_pred = 10.0;
      result.reason = "test_stale";
      results->push_back(result);
    }
    return true;
  }
};

iap::RiskGridMapParams makeParams() {
  iap::RiskGridMapParams params;
  params.resolution_m = 1.0;
  params.size_x_m = 12.0;
  params.size_y_m = 12.0;
  params.size_z_m = 8.0;
  params.horizons_s = {0.0, 0.5, 1.0};
  params.stale_timeout_s = 10.0;
  params.unknown_cost = 7.0;
  params.cost_max = 100.0;
  return params;
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot(
    iap::RiskPredictionProvider& provider, const double stamp_s = 10.0) {
  iap::RiskGridMap grid(makeParams());
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), stamp_s, provider, &reason))
      << reason;
  auto snapshot = grid.acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

Eigen::MatrixXd makeControlPoints() {
  Eigen::MatrixXd q(3, 7);
  q << -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  return q;
}

void ensureRclcpp() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

std::unique_ptr<ego_planner::BsplineOptimizer> makeOptimizer(
    const ego_planner::BsplineOptimizer::P1IntegrityConfig& p1_config,
    ego_planner::SwarmTrajData* swarm_trajs) {
  ensureRclcpp();
  static int node_id = 0;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("optimization/lambda_smooth", 1.0),
      rclcpp::Parameter("optimization/lambda_collision", 0.5),
      rclcpp::Parameter("optimization/lambda_feasibility", 0.1),
      rclcpp::Parameter("optimization/lambda_fitness", 1.0),
      rclcpp::Parameter("optimization/dist0", 0.5),
      rclcpp::Parameter("optimization/swarm_clearance", 0.5),
      rclcpp::Parameter("optimization/max_vel", 100.0),
      rclcpp::Parameter("optimization/max_acc", 100.0),
      rclcpp::Parameter("optimization/order", 3),
      rclcpp::Parameter("p1.use_integrity_cost", p1_config.use_integrity_cost),
      rclcpp::Parameter("p1.metrics_only", p1_config.metrics_only),
      rclcpp::Parameter("p1.lambda_integrity", p1_config.lambda_integrity),
      rclcpp::Parameter("p1.sample_dt_min_s", p1_config.sample_dt_min_s),
      rclcpp::Parameter("p1.sample_dt_scale", p1_config.sample_dt_scale),
      rclcpp::Parameter("p1.max_samples_per_eval", p1_config.max_samples_per_eval),
      rclcpp::Parameter("p1.integrity_cost_max", p1_config.integrity_cost_max),
      rclcpp::Parameter("p1.integrity_grad_norm_max", p1_config.integrity_grad_norm_max),
      rclcpp::Parameter("p1.unknown_policy", p1_config.unknown_policy),
      rclcpp::Parameter("p1.unknown_soft_penalty", p1_config.unknown_soft_penalty),
      rclcpp::Parameter("p1.debug_csv_enable", p1_config.debug_csv_enable),
      rclcpp::Parameter("p1.debug_csv_path", p1_config.debug_csv_path),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "test_p1_integrity_cost_" + std::to_string(node_id++), options);

  auto optimizer = std::make_unique<ego_planner::BsplineOptimizer>();
  optimizer->setParam(node);
  optimizer->setSwarmTrajs(swarm_trajs);
  optimizer->setDroneId(0);
  const Eigen::MatrixXd q = makeControlPoints();
  optimizer->setLocalTargetPt((q.col(q.cols() - 3) + 4.0 * q.col(q.cols() - 2) +
                               q.col(q.cols() - 1)) /
                              6.0);
  return optimizer;
}

bool evaluate(ego_planner::BsplineOptimizer& optimizer,
              const Eigen::MatrixXd& q,
              double* cost,
              Eigen::MatrixXd* gradient) {
  return optimizer.evaluateReboundCostForTest(q, 0.1, *cost, *gradient);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

std::vector<std::map<std::string, std::string>> readCsvRows(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  std::vector<std::map<std::string, std::string>> rows;
  if (!std::getline(in, line)) {
    return rows;
  }
  const auto header = splitCsvLine(line);
  while (std::getline(in, line)) {
    const auto fields = splitCsvLine(line);
    std::map<std::string, std::string> row;
    for (std::size_t i = 0; i < header.size(); ++i) {
      row[header[i]] = i < fields.size() ? fields[i] : "";
    }
    rows.push_back(row);
  }
  return rows;
}

std::string tempProfilePath(const std::string& name) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return "/tmp/" + name + "_" + std::to_string(now) + ".csv";
}

ego_planner::BsplineOptimizer::P1IntegrityConfig disabledConfig() {
  ego_planner::BsplineOptimizer::P1IntegrityConfig config;
  config.use_integrity_cost = false;
  config.metrics_only = false;
  config.lambda_integrity = 0.0;
  return config;
}

}  // namespace

TEST(P1IntegrityCostTest, DisabledMatchesOriginalObjectiveAndGradient) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  auto original = makeOptimizer(disabledConfig(), &swarm);
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));

  double disabled_cost = 0.0;
  Eigen::MatrixXd disabled_gradient;
  auto disabled = makeOptimizer(disabledConfig(), &swarm);
  disabled->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*disabled, q, &disabled_cost, &disabled_gradient));

  EXPECT_DOUBLE_EQ(disabled_cost, original_cost);
  EXPECT_TRUE(disabled_gradient.isApprox(original_gradient, 0.0));
  EXPECT_EQ(disabled->getLastP1IntegrityMetrics().sample_count, 0);
}

TEST(P1IntegrityCostTest, MetricsOnlyDoesNotChangeObjectiveOrGradient) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  auto original = makeOptimizer(disabledConfig(), &swarm);
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));

  auto config = disabledConfig();
  config.metrics_only = true;
  config.lambda_integrity = 3.0;
  double metrics_cost = 0.0;
  Eigen::MatrixXd metrics_gradient;
  auto metrics_only = makeOptimizer(config, &swarm);
  metrics_only->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*metrics_only, q, &metrics_cost, &metrics_gradient));

  const auto& metrics = metrics_only->getLastP1IntegrityMetrics();
  EXPECT_DOUBLE_EQ(metrics_cost, original_cost);
  EXPECT_TRUE(metrics_gradient.isApprox(original_gradient, 0.0));
  EXPECT_GT(metrics.sample_count, 0);
  EXPECT_GT(metrics.hit_count, 0);
  EXPECT_FALSE(metrics.applied_to_objective);
  EXPECT_EQ(metrics.snapshot_generation_id, snapshot->generation_id());
  const auto& breakdown = metrics_only->getLastOptimizerCostBreakdown();
  EXPECT_DOUBLE_EQ(breakdown.original_cost, original_cost);
  EXPECT_DOUBLE_EQ(breakdown.total_cost, original_cost);
  EXPECT_DOUBLE_EQ(breakdown.integrity_cost, 0.0);
}

TEST(P1IntegrityCostTest, EnabledAddsExpectedCostAndGradient) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  auto original = makeOptimizer(disabledConfig(), &swarm);
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 2.0;
  double enabled_cost = 0.0;
  Eigen::MatrixXd enabled_gradient;
  auto enabled = makeOptimizer(config, &swarm);
  enabled->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*enabled, q, &enabled_cost, &enabled_gradient));

  const auto& metrics = enabled->getLastP1IntegrityMetrics();
  EXPECT_TRUE(metrics.applied_to_objective);
  EXPECT_NEAR(enabled_cost, original_cost + metrics.weighted_f_integrity, 1.0e-9);
  EXPECT_GT((enabled_gradient - original_gradient).norm(), 0.0);
  EXPECT_GT(metrics.weighted_grad_integrity_norm, 0.0);
  const auto& breakdown = enabled->getLastOptimizerCostBreakdown();
  EXPECT_DOUBLE_EQ(breakdown.original_cost, original_cost);
  EXPECT_NEAR(breakdown.total_cost, enabled_cost, 1.0e-12);
  EXPECT_NEAR(breakdown.integrity_cost, metrics.weighted_f_integrity, 1.0e-12);
}

TEST(P1IntegrityCostTest, DisabledCostBreakdownHasNoIntegrityCost) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto disabled = makeOptimizer(disabledConfig(), &swarm);
  disabled->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*disabled, q, &cost, &gradient));

  const auto& breakdown = disabled->getLastOptimizerCostBreakdown();
  EXPECT_DOUBLE_EQ(breakdown.original_cost, cost);
  EXPECT_DOUBLE_EQ(breakdown.total_cost, cost);
  EXPECT_DOUBLE_EQ(breakdown.integrity_cost, 0.0);
}

TEST(P1IntegrityCostTest, SampleCountIsCapped) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  auto config = disabledConfig();
  config.metrics_only = true;
  config.sample_dt_min_s = 0.001;
  config.sample_dt_scale = 0.01;
  config.max_samples_per_eval = 3;
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));

  EXPECT_EQ(optimizer->getLastP1IntegrityMetrics().sample_count, 3);
}

TEST(P1IntegrityCostTest, SnapshotGenerationStaysFixedUntilReset) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider1(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  AffineProvider provider2(20.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  iap::RiskGridMap grid(makeParams());
  std::string reason;
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0, provider1, &reason));
  auto snapshot1 = grid.acquireSnapshot();
  ASSERT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0, provider2, &reason));
  auto snapshot2 = grid.acquireSnapshot();
  ASSERT_NE(snapshot1->generation_id(), snapshot2->generation_id());

  auto config = disabledConfig();
  config.metrics_only = true;
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot1, snapshot1->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));
  EXPECT_EQ(optimizer->getLastP1IntegrityMetrics().snapshot_generation_id,
            snapshot1->generation_id());

  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));
  EXPECT_EQ(optimizer->getLastP1IntegrityMetrics().snapshot_generation_id,
            snapshot1->generation_id());

  optimizer->setRiskSnapshot(snapshot2, snapshot2->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));
  EXPECT_EQ(optimizer->getLastP1IntegrityMetrics().snapshot_generation_id,
            snapshot2->generation_id());
}

TEST(P1IntegrityCostTest, GradientClippingIsRecorded) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(100.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  auto config = disabledConfig();
  config.metrics_only = true;
  config.integrity_grad_norm_max = 0.05;
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));

  const auto& metrics = optimizer->getLastP1IntegrityMetrics();
  EXPECT_GT(metrics.hit_count, 0);
  EXPECT_GT(metrics.clipped_grad_count, 0);
  EXPECT_LE(metrics.clipped_grad_count, metrics.hit_count);
}

TEST(P1IntegrityCostTest, UnknownSkipAddsNoCostOrGradient) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  UnknownProvider provider;
  auto snapshot = makeSnapshot(provider);

  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  auto original = makeOptimizer(disabledConfig(), &swarm);
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));

  auto config = disabledConfig();
  config.metrics_only = true;
  config.unknown_policy = "skip";
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));

  const auto& metrics = optimizer->getLastP1IntegrityMetrics();
  EXPECT_DOUBLE_EQ(cost, original_cost);
  EXPECT_TRUE(gradient.isApprox(original_gradient, 0.0));
  EXPECT_EQ(metrics.hit_count, 0);
  EXPECT_EQ(metrics.miss_count, metrics.sample_count);
  EXPECT_DOUBLE_EQ(metrics.f_integrity, 0.0);
}

TEST(P1IntegrityCostTest, UnknownSmallPenaltyAddsCostWithZeroGradient) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  UnknownProvider provider;
  auto snapshot = makeSnapshot(provider);

  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  auto original = makeOptimizer(disabledConfig(), &swarm);
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 2.0;
  config.unknown_policy = "small_penalty";
  config.unknown_soft_penalty = 3.0;
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));

  const auto& metrics = optimizer->getLastP1IntegrityMetrics();
  EXPECT_TRUE(metrics.applied_to_objective);
  EXPECT_NEAR(metrics.f_integrity, 3.0, 1.0e-12);
  EXPECT_NEAR(cost, original_cost + 6.0, 1.0e-12);
  EXPECT_TRUE(gradient.isApprox(original_gradient, 0.0));
}

TEST(P1IntegrityCostTest, MetricsOnlyAcceptedProfileWritesFiniteSamples) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_metrics_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = true;
  config.lambda_integrity = 3.0;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ego_planner::UniformBspline trajectory(q, 3, 0.1);

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      trajectory, 7, 42, 123.0));
  const auto rows = readCsvRows(optimizer->p1AcceptedTrajectoryRiskProfilePath());
  ASSERT_EQ(rows.size(), 200U);
  int finite_count = 0;
  for (const auto& row : rows) {
    EXPECT_EQ(row.at("profile_seq"), "7");
    EXPECT_EQ(row.at("trajectory_id"), "42");
    EXPECT_EQ(row.at("applied_to_objective"), "0");
    EXPECT_EQ(row.at("metrics_only"), "1");
    EXPECT_EQ(row.at("hit"), "1");
    EXPECT_EQ(row.at("valid"), "1");
    EXPECT_EQ(row.at("stale"), "0");
    EXPECT_FALSE(row.at("c_pi").empty());
    finite_count++;
  }
  EXPECT_EQ(finite_count, 200);
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
}

TEST(P1IntegrityCostTest, EnabledAcceptedProfileRecordsAppliedObjectiveAndLambda) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_enabled_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ego_planner::UniformBspline trajectory(q, 3, 0.1);

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      trajectory, 8, 43, 124.0));
  const auto rows = readCsvRows(optimizer->p1AcceptedTrajectoryRiskProfilePath());
  ASSERT_EQ(rows.size(), 200U);
  for (const auto& row : rows) {
    EXPECT_EQ(row.at("applied_to_objective"), "1");
    EXPECT_EQ(row.at("metrics_only"), "0");
    EXPECT_DOUBLE_EQ(std::stod(row.at("lambda_integrity")), 0.00001);
    EXPECT_FALSE(row.at("c_pi").empty());
  }
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
}

TEST(P1IntegrityCostTest, AcceptedProfileLeavesCpiBlankForStaleMisses) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  StaleProvider provider;
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_stale_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ego_planner::UniformBspline trajectory(q, 3, 0.1);

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      trajectory, 9, 44, 125.0));
  const auto rows = readCsvRows(optimizer->p1AcceptedTrajectoryRiskProfilePath());
  ASSERT_EQ(rows.size(), 200U);
  for (const auto& row : rows) {
    EXPECT_EQ(row.at("hit"), "0");
    EXPECT_EQ(row.at("valid"), "0");
    EXPECT_EQ(row.at("stale"), "1");
    EXPECT_TRUE(row.at("c_pi").empty());
  }
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
}
