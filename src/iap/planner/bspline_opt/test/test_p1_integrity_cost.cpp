#include <bspline_opt/bspline_optimizer.h>
#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
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

class SpatiotemporalAffineProvider : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      const double value = 20.0 + 2.0 * query.position_w.x() +
                           3.0 * query.position_w.y() +
                           4.0 * query.position_w.z() +
                           5.0 * query.horizon_s;
      result.hpl_pred = value;
      result.vpl_pred = value;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
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

class InvalidProvider : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    results->assign(queries.size(), iap::RiskPredictionResult{});
    for (auto& result : *results) {
      result.available = true;
      result.valid = false;
      result.stale = false;
      result.reason = "invalid_cost";
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

Eigen::MatrixXd makeFixedLambdaConflictControlPoints() {
  Eigen::MatrixXd q = makeControlPoints();
  // The test optimizer's terminal target remains on Y=0.  Starting the whole
  // curve at Y=1 therefore makes the base terminal descent pull the active
  // suffix toward -Y, while a -Y affine risk field rewards motion toward +Y.
  q.row(1).setConstant(1.0);
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
      rclcpp::Parameter("p1.objective_aggregation_mode", p1_config.objective_aggregation_mode),
      rclcpp::Parameter("p1.smooth_max_temperature", p1_config.smooth_max_temperature),
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

TEST(P1IntegrityCostTest,
     AffineWeightedGradientMatchesCentralDifferenceForEveryFreeControlPoint) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  SpatiotemporalAffineProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  config.max_samples_per_eval = 9;
  auto enabled = makeOptimizer(config, &swarm);
  enabled->setRiskSnapshot(snapshot, snapshot->stamp_s());
  auto original = makeOptimizer(disabledConfig(), &swarm);

  double enabled_cost = 0.0;
  double original_cost = 0.0;
  Eigen::MatrixXd enabled_gradient;
  Eigen::MatrixXd original_gradient;
  ASSERT_TRUE(evaluate(*enabled, q, &enabled_cost, &enabled_gradient));
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));
  const Eigen::MatrixXd integrity_gradient =
      enabled_gradient - original_gradient;
  constexpr double kStep = 1.0e-5;
  for (int column = 3; column < q.cols(); ++column) {
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::MatrixXd plus = q;
      Eigen::MatrixXd minus = q;
      plus(axis, column) += kStep;
      minus(axis, column) -= kStep;
      double plus_enabled = 0.0, plus_original = 0.0;
      double minus_enabled = 0.0, minus_original = 0.0;
      Eigen::MatrixXd ignored;
      ASSERT_TRUE(evaluate(*enabled, plus, &plus_enabled, &ignored));
      ASSERT_TRUE(evaluate(*original, plus, &plus_original, &ignored));
      ASSERT_TRUE(evaluate(*enabled, minus, &minus_enabled, &ignored));
      ASSERT_TRUE(evaluate(*original, minus, &minus_original, &ignored));
      const double finite_difference =
          ((plus_enabled - plus_original) -
           (minus_enabled - minus_original)) /
          (2.0 * kStep);
      EXPECT_NEAR(integrity_gradient(axis, column), finite_difference, 2.0e-8)
          << "axis=" << axis << " control_point=" << column;
    }
  }
}

TEST(P1IntegrityCostTest, FixedLambdaNegativeGradientStepLowersIntegrityCost) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  SpatiotemporalAffineProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  double before_total = 0.0;
  Eigen::MatrixXd total_gradient;
  ASSERT_TRUE(evaluate(*optimizer, q, &before_total, &total_gradient));
  const double before_integrity =
      optimizer->getLastP1IntegrityMetrics().f_integrity;
  auto no_p1_config = config;
  no_p1_config.use_integrity_cost = false;
  no_p1_config.lambda_integrity = 0.0;
  auto no_p1 = makeOptimizer(no_p1_config, &swarm);
  no_p1->setRiskSnapshot(snapshot, snapshot->stamp_s());
  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  ASSERT_TRUE(evaluate(*no_p1, q, &original_cost, &original_gradient));
  Eigen::MatrixXd moved = q;
  Eigen::MatrixXd direction = total_gradient - original_gradient;
  direction.leftCols(3).setZero();
  ASSERT_GT(direction.norm(), 0.0);
  moved -= 1.0e-3 * direction / direction.norm();
  double after_total = 0.0;
  Eigen::MatrixXd after_gradient;
  ASSERT_TRUE(evaluate(*optimizer, moved, &after_total, &after_gradient));
  const double after_integrity =
      optimizer->getLastP1IntegrityMetrics().f_integrity;
  EXPECT_LT(after_integrity, before_integrity);
  EXPECT_LT((direction.array() * (moved - q).array()).sum(), 0.0);
}

TEST(P1IntegrityCostTest,
     ObjectiveAppliedLbfgsMovesAlongDescentWithinIterationBudget) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd before = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  double initial_cost = 0.0;
  Eigen::MatrixXd initial_gradient;
  ASSERT_TRUE(evaluate(*optimizer, before, &initial_cost, &initial_gradient));

  Eigen::MatrixXd after = before;
  double final_cost = 0.0;
  int iterations = 0;
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(optimizer->optimizeReboundCostForTest(
      after, 0.1, 40, final_cost, iterations));
  const double elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  const Eigen::MatrixXd displacement = after - before;
  EXPECT_GT(iterations, 0);
  EXPECT_LE(iterations, 40);
  EXPECT_LT(elapsed_s, 1.0);
  EXPECT_GT(displacement.rightCols(before.cols() - 3).norm(), 0.0);
  EXPECT_LT((initial_gradient.array() * displacement.array()).sum(), 0.0);
  EXPECT_LT(final_cost, initial_cost);
}

TEST(P1IntegrityCostTest,
     FixedLambdaConflictFixtureCharacterizesLegacyRegression) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd initial = makeFixedLambdaConflictControlPoints();
  AffineProvider provider(20.0, Eigen::Vector3d(0.0, -1.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  config.objective_aggregation_mode = "fixed_200_mean";
  auto optimizer = makeOptimizer(config, &swarm);
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.snapshot = snapshot;
  context.query_base_time_s = snapshot->stamp_s();
  context.planning_start_s = snapshot->stamp_s();
  context.planning_attempt_id = 92;
  context.candidate_id = 1;
  optimizer->setP1PlanningRiskContext(context);

  double raw_cost = 0.0;
  Eigen::MatrixXd raw_gradient;
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(
      initial, 0.1, raw_cost, raw_gradient));
  Eigen::MatrixXd active_gradient = Eigen::MatrixXd::Zero(
      initial.rows(), initial.cols());
  active_gradient.rightCols(initial.cols() - optimizer->getOrder()) =
      raw_gradient.rightCols(initial.cols() - optimizer->getOrder());
  ASSERT_GT(active_gradient.norm(), 0.0);

  constexpr double kProbeDisplacementM = 0.025;
  const double max_column_norm = active_gradient.colwise().norm().maxCoeff();
  const Eigen::MatrixXd probe = initial -
      (kProbeDisplacementM / max_column_norm) * active_gradient;
  double probe_cost = 0.0;
  Eigen::MatrixXd ignored;
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(
      probe, 0.1, probe_cost, ignored));
  EXPECT_LT(probe_cost, raw_cost);

  Eigen::MatrixXd terminal = initial;
  double final_cost = 0.0;
  int iterations = 0;
  ASSERT_TRUE(optimizer->optimizeReboundCostForTest(
      terminal, 0.1, 80, final_cost, iterations));
  const auto trace = optimizer->getLastP1OptimizationTrace();
  ASSERT_TRUE(trace.support_full_valid);
  EXPECT_GT(trace.grad_integrity_dot_displacement, 0.0);
  EXPECT_GT(trace.post_mean_c_pi, trace.pre_mean_c_pi);
  EXPECT_GT(trace.post_max_c_pi, trace.pre_max_c_pi);
  EXPECT_LT(trace.post_total_objective, trace.pre_total_objective);
}

TEST(P1IntegrityCostTest, FixedLambdaTwoStagePreferenceDescendsRisk) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd topology_seed =
      makeFixedLambdaConflictControlPoints();
  AffineProvider provider(20.0, Eigen::Vector3d(0.0, -1.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  config.objective_aggregation_mode = "fixed_200_mean";
  auto optimizer = makeOptimizer(config, &swarm);
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.snapshot = snapshot;
  context.query_base_time_s = snapshot->stamp_s();
  context.planning_start_s = snapshot->stamp_s();
  context.planning_attempt_id = 92;
  optimizer->setP1PlanningRiskContext(context);

  Eigen::MatrixXd base = topology_seed;
  double base_cost = 0.0;
  int base_iterations = 0;
  ASSERT_TRUE(optimizer->optimizeP1BasePrepassForTest(
      base, 0.1, 80, base_cost, base_iterations));
  const auto base_prepass = optimizer->getLastP1BasePrepassTrace();
  ASSERT_TRUE(base_prepass.success);
  EXPECT_LT(base_prepass.post_objective, base_prepass.pre_objective);

  double raw_cost = 0.0;
  Eigen::MatrixXd raw_gradient;
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(
      base, 0.1, raw_cost, raw_gradient));
  raw_gradient.leftCols(optimizer->getOrder()).setZero();
  const double max_column_norm = raw_gradient.colwise().norm().maxCoeff();
  ASSERT_GT(max_column_norm, 0.0);

  std::vector<Eigen::MatrixXd> initial_candidates;
  for (const double displacement : {0.0, 0.025, 0.05, 0.10})
    initial_candidates.push_back(
        base - displacement * raw_gradient / max_column_norm);
  for (std::size_t left = 0; left < initial_candidates.size(); ++left)
    for (std::size_t right = left + 1; right < initial_candidates.size(); ++right)
      EXPECT_GT((initial_candidates[left] - initial_candidates[right]).norm(),
                1.0e-6);

  std::vector<Eigen::MatrixXd> finals;
  std::vector<ego_planner::BsplineOptimizer::P1OptimizationTrace> traces;
  for (std::size_t index = 0; index < initial_candidates.size(); ++index) {
    context.candidate_id = index + 1;
    optimizer->setP1PlanningRiskContext(context);
    std::string reason;
    ASSERT_TRUE(optimizer->prepareP1NormalizedStage(
        initial_candidates[index], 0.1, base_prepass, &reason)) << reason;
    Eigen::MatrixXd terminal = initial_candidates[index];
    double terminal_cost = 0.0;
    int iterations = 0;
    ASSERT_TRUE(optimizer->optimizeReboundCostForTest(
        terminal, 0.1, 80, terminal_cost, iterations));
    finals.push_back(terminal);
    traces.push_back(optimizer->getLastP1OptimizationTrace());
    optimizer->clearP1NormalizedStage();
  }

  const auto eligible = std::count_if(
      traces.begin(), traces.end(), [](const auto& trace) {
        return trace.optimization_success && trace.support_full_valid &&
            trace.post_total_objective <= trace.pre_total_objective &&
            trace.post_mean_c_pi <= trace.pre_mean_c_pi &&
            trace.post_max_c_pi <= trace.pre_max_c_pi &&
            (trace.post_mean_c_pi < trace.pre_mean_c_pi ||
             trace.post_max_c_pi < trace.pre_max_c_pi) &&
            trace.grad_integrity_dot_displacement < 0.0;
      });
  EXPECT_GE(eligible, 1);
  EXPECT_TRUE(std::all_of(traces.begin(), traces.end(), [](const auto& trace) {
    return trace.normalization_mode == "base_improvement_budget_v1" &&
        trace.normalization_reference_lambda == 1.0e-5 &&
        trace.normalization_budget_fraction == 0.10 &&
        trace.normalization_reference_displacement_m == 0.025 &&
        trace.normalization_scale > 0.0;
  }));
  bool distinguishable = false;
  for (std::size_t left = 0; left < finals.size(); ++left)
    for (std::size_t right = left + 1; right < finals.size(); ++right)
      distinguishable = distinguishable ||
          (finals[left] - finals[right]).norm() > 1.0e-4;
  EXPECT_TRUE(distinguishable);
}

TEST(P1IntegrityCostTest, EvidenceV4WritesAllCandidateSidecars) {
  ego_planner::SwarmTrajData swarm;
  Eigen::MatrixXd points = makeFixedLambdaConflictControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(0.0, -1.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_v4_sidecar_debug");

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  config.evidence_schema_version = "p1_evidence_provenance_v4";
  config.evidence_run_id = "v4-sidecar-test";
  config.evidence_manifest_path = "/tmp/v4-sidecar-manifest.json";
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setP1IntegrityConfigForTest(config);
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.snapshot = snapshot;
  context.query_base_time_s = snapshot->stamp_s();
  context.planning_start_s = snapshot->stamp_s();
  context.planning_attempt_id = 101;
  context.candidate_id = 1;
  optimizer->setP1PlanningRiskContext(context);

  const std::vector<std::string> sidecars = {
      debug_path,
      optimizer->p1CandidateOptimizationPath(),
      optimizer->p1CandidateControlPointsPath(),
      optimizer->p1CandidateProfilePath(),
      optimizer->p1CandidatePairwisePath(),
      optimizer->p1OptimizerCheckpointPath(),
      optimizer->p0OccupancyQueryEvidencePath(),
  };
  for (const auto& path : sidecars) std::remove(path.c_str());

  double base_cost = 0.0;
  int base_iterations = 0;
  ASSERT_TRUE(optimizer->optimizeP1BasePrepassForTest(
      points, 0.1, 80, base_cost, base_iterations));
  const auto base_prepass = optimizer->getLastP1BasePrepassTrace();
  std::string normalization_reason;
  ASSERT_TRUE(optimizer->prepareP1NormalizedStage(
      points, 0.1, base_prepass, &normalization_reason))
      << normalization_reason;
  double final_cost = 0.0;
  int iterations = 0;
  ASSERT_TRUE(optimizer->optimizeReboundCostForTest(
      points, 0.1, 80, final_cost, iterations));
  auto trace = optimizer->getLastP1OptimizationTrace();
  trace.selected = true;
  trace.optimization_success = true;
  trace.rank_eligible = true;
  trace.replacement_accepted = true;
  optimizer->writeP1OptimizationTrace(trace);

  const auto candidate_rows = readCsvRows(optimizer->p1CandidateOptimizationPath());
  ASSERT_EQ(candidate_rows.size(), 1U);
  {
    std::ifstream candidate_file(optimizer->p1CandidateOptimizationPath());
    std::string header_line;
    std::string value_line;
    ASSERT_TRUE(std::getline(candidate_file, header_line));
    ASSERT_TRUE(std::getline(candidate_file, value_line));
    EXPECT_EQ(splitCsvLine(header_line).size(), splitCsvLine(value_line).size());
  }
  EXPECT_EQ(candidate_rows.front().at("schema_version"),
            "p1_evidence_provenance_v4");
  EXPECT_EQ(candidate_rows.front().at("normalization_mode"),
            "base_improvement_budget_v1");
  EXPECT_FALSE(candidate_rows.front().at(
      "pre_full_total_gradient_norm").empty());
  const auto control_rows = readCsvRows(optimizer->p1CandidateControlPointsPath());
  ASSERT_EQ(control_rows.size(), static_cast<std::size_t>(2 * points.cols()));
  const auto profile_rows = readCsvRows(optimizer->p1CandidateProfilePath());
  ASSERT_EQ(profile_rows.size(), 400U);
  EXPECT_EQ(std::count_if(profile_rows.begin(), profile_rows.end(),
      [](const auto& row) { return row.at("phase") == "initial"; }), 200);
  EXPECT_TRUE(std::all_of(profile_rows.begin(), profile_rows.end(),
      [](const auto& row) {
        return row.at("valid") == "1" && !row.at("c_pi").empty() &&
            row.at("invalid_reason") == "none";
      }));
  const auto pairwise_rows = readCsvRows(optimizer->p1CandidatePairwisePath());
  ASSERT_EQ(pairwise_rows.size(), 2U);
  const auto checkpoint_rows = readCsvRows(optimizer->p1OptimizerCheckpointPath());
  EXPECT_TRUE(std::any_of(checkpoint_rows.begin(), checkpoint_rows.end(),
      [](const auto& row) {
        return row.at("stage") == "base_prepass" &&
            row.at("checkpoint") == "terminal";
      }));
  EXPECT_TRUE(std::any_of(checkpoint_rows.begin(), checkpoint_rows.end(),
      [](const auto& row) {
        return row.at("stage") == "p1_stage" &&
            row.at("checkpoint") == "first_accepted_step";
      }));
  const auto occupancy_rows = readCsvRows(
      optimizer->p0OccupancyQueryEvidencePath());
  EXPECT_GE(occupancy_rows.size(), 3200U);
  EXPECT_TRUE(std::all_of(occupancy_rows.begin(), occupancy_rows.end(),
      [](const auto& row) {
        return row.at("schema_version") == "p1_evidence_provenance_v4";
      }));

  for (const auto& path : sidecars) std::remove(path.c_str());
}

TEST(P1IntegrityCostTest,
     FixedLambdaFeedbackGateUsesOneSnapshotAndRecordsAuthoritativeDiagnostic) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd initial = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_fixed_lambda_feedback_debug");
  // Persist stable, inspectable artifacts after CTest.  They are deliberately
  // outside the source tree and overwritten at the next deterministic run.
  const std::string diagnostic_csv_path =
      "/tmp/iap_p1_fixed_lambda_feedback_diagnostic.csv";
  const std::string diagnostic_path =
      "/tmp/iap_p1_fixed_lambda_feedback_diagnostic.json";
  std::remove(debug_path.c_str());
  std::remove(diagnostic_csv_path.c_str());

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.lambda_integrity = 0.00001;
  config.integrity_grad_norm_max = 100.0;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.snapshot = snapshot;
  context.query_base_time_s = snapshot->stamp_s();
  context.planning_start_s = snapshot->stamp_s();
  context.planning_attempt_id = 91;
  context.candidate_id = 1;

  auto metrics_config = config;
  metrics_config.metrics_only = true;
  auto metrics_only = makeOptimizer(metrics_config, &swarm);
  metrics_only->setP1PlanningRiskContext(context);
  Eigen::MatrixXd metrics_points = initial;
  double metrics_final_cost = 0.0;
  int metrics_iterations = 0;
  ASSERT_TRUE(metrics_only->optimizeReboundCostForTest(
      metrics_points, 0.1, 40, metrics_final_cost, metrics_iterations));
  auto metrics_trace = metrics_only->getLastP1OptimizationTrace();
  metrics_trace.selected = true;
  metrics_trace.selection_score = metrics_final_cost;
  metrics_trace.selection_reason = "deterministic_single_candidate";

  auto enabled_config = config;
  enabled_config.metrics_only = false;
  auto enabled = makeOptimizer(enabled_config, &swarm);
  enabled->setP1PlanningRiskContext(context);
  Eigen::MatrixXd enabled_points = initial;
  double enabled_final_cost = 0.0;
  int enabled_iterations = 0;
  ASSERT_TRUE(enabled->optimizeReboundCostForTest(
      enabled_points, 0.1, 40, enabled_final_cost, enabled_iterations));
  auto enabled_trace = enabled->getLastP1OptimizationTrace();
  enabled_trace.selected = true;
  enabled_trace.selection_score = enabled_final_cost;
  enabled_trace.selection_reason = "deterministic_single_candidate";
  std::ofstream diagnostic_csv(diagnostic_csv_path, std::ios::trunc);
  ASSERT_TRUE(diagnostic_csv.good());
  diagnostic_csv << std::setprecision(17)
      << "mode,lambda_integrity,planning_attempt_id,candidate_id,snapshot_generation_id,"
         "query_base_time_s,support_signature,initial_control_points_hash,p1_config_hash,"
         "pre_base_objective,post_base_objective,pre_total_objective,post_total_objective,"
         "pre_raw_p1_cost,post_raw_p1_cost,pre_weighted_p1_cost,post_weighted_p1_cost,"
         "pre_base_gradient_norm,post_base_gradient_norm,pre_raw_p1_gradient_norm,"
         "post_raw_p1_gradient_norm,pre_weighted_p1_gradient_norm,"
         "post_weighted_p1_gradient_norm,pre_total_gradient_norm,post_total_gradient_norm,"
         "displacement_norm,grad_integrity_dot_displacement,pre_mean_c_pi,post_mean_c_pi,"
         "pre_max_c_pi,post_max_c_pi,support_sample_count,pre_support_valid_count,"
         "post_support_valid_count,pre_support_coverage,post_support_coverage,support_full_valid,"
         "optimization_success,selected,solver_result,iteration_count,termination_reason,"
         "objective_allowed,objective_applied,selection_score,selection_reason,fallback_reason\n";
  const auto write_csv_trace = [&diagnostic_csv](
      const char* mode, const ego_planner::BsplineOptimizer::P1OptimizationTrace& trace) {
    diagnostic_csv << mode << ",0.00001," << trace.planning_attempt_id << ','
                   << trace.candidate_id << ',' << trace.snapshot_generation_id << ','
                   << trace.query_base_time_s << ',' << trace.support_signature << ','
                   << trace.initial_control_points_hash << ',' << trace.p1_config_hash << ','
                   << trace.pre_base_objective << ',' << trace.post_base_objective << ','
                   << trace.pre_total_objective << ',' << trace.post_total_objective << ','
                   << trace.pre_raw_p1_cost << ',' << trace.post_raw_p1_cost << ','
                   << trace.pre_weighted_p1_cost << ',' << trace.post_weighted_p1_cost << ','
                   << trace.pre_base_gradient_norm << ',' << trace.post_base_gradient_norm << ','
                   << trace.pre_raw_p1_gradient_norm << ','
                   << trace.post_raw_p1_gradient_norm << ','
                   << trace.pre_weighted_p1_gradient_norm << ','
                   << trace.post_weighted_p1_gradient_norm << ','
                   << trace.pre_total_gradient_norm << ',' << trace.post_total_gradient_norm << ','
                   << trace.displacement_norm << ',' << trace.grad_integrity_dot_displacement << ','
                   << trace.pre_mean_c_pi << ',' << trace.post_mean_c_pi << ','
                   << trace.pre_max_c_pi << ',' << trace.post_max_c_pi << ','
                   << trace.support_sample_count << ',' << trace.pre_support_valid_count << ','
                   << trace.post_support_valid_count << ',' << trace.pre_support_coverage << ','
                   << trace.post_support_coverage << ',' << (trace.support_full_valid ? 1 : 0) << ','
                   << (trace.optimization_success ? 1 : 0) << ',' << (trace.selected ? 1 : 0) << ','
                   << trace.solver_result << ',' << trace.iteration_count << ','
                   << trace.termination_reason << ',' << (trace.objective_allowed ? 1 : 0) << ','
                   << (trace.objective_applied ? 1 : 0) << ',' << trace.selection_score << ','
                   << trace.selection_reason << ',' << trace.fallback_reason << '\n';
  };
  write_csv_trace("metrics_only", metrics_trace);
  write_csv_trace("enabled", enabled_trace);
  diagnostic_csv.close();
  const auto rows = readCsvRows(diagnostic_csv_path);
  ASSERT_EQ(rows.size(), 2U);
  ASSERT_TRUE(std::ifstream(diagnostic_csv_path).good());
  ASSERT_EQ(metrics_trace.support_signature, enabled_trace.support_signature);
  ASSERT_EQ(metrics_trace.snapshot_generation_id, enabled_trace.snapshot_generation_id);
  ASSERT_EQ(metrics_trace.query_base_time_s, enabled_trace.query_base_time_s);
  ASSERT_EQ(metrics_trace.planning_attempt_id, enabled_trace.planning_attempt_id);
  ASSERT_EQ(metrics_trace.candidate_id, enabled_trace.candidate_id);
  ASSERT_EQ(metrics_trace.initial_control_points_hash,
            enabled_trace.initial_control_points_hash);
  ASSERT_TRUE(metrics_trace.support_full_valid);
  ASSERT_TRUE(enabled_trace.support_full_valid);
  EXPECT_TRUE(metrics_trace.selected);
  EXPECT_TRUE(enabled_trace.selected);
  EXPECT_GT(enabled_trace.pre_weighted_p1_cost, 0.0);
  EXPECT_GT(enabled_trace.pre_weighted_p1_gradient_norm, 0.0);
  EXPECT_LT(enabled_trace.post_mean_c_pi, enabled_trace.pre_mean_c_pi);
  EXPECT_LT(enabled_trace.post_max_c_pi, enabled_trace.pre_max_c_pi);
  EXPECT_TRUE(std::isfinite(enabled_trace.grad_integrity_dot_displacement));
  const double objective_ratio = enabled_trace.pre_weighted_p1_cost /
      std::max(std::abs(enabled_trace.pre_base_objective), 1.0e-12);
  const double gradient_ratio = enabled_trace.pre_weighted_p1_gradient_norm /
      std::max(std::abs(enabled_trace.pre_base_gradient_norm), 1.0e-12);
  const double rounding_ratio_scale = 100.0 * std::numeric_limits<double>::epsilon();
  const bool fixed_lambda_unmeasurable =
      objective_ratio <= 1.0e-6 && gradient_ratio <= 1.0e-6 &&
      objective_ratio <= rounding_ratio_scale && gradient_ratio <= rounding_ratio_scale;
  EXPECT_FALSE(fixed_lambda_unmeasurable);

  std::ofstream diagnostic(diagnostic_path, std::ios::trunc);
  ASSERT_TRUE(diagnostic.good());
  diagnostic << std::setprecision(17);
  const auto write_trace = [&diagnostic](
      const char* mode, const ego_planner::BsplineOptimizer::P1OptimizationTrace& trace,
      const bool trailing_comma) {
    diagnostic << "  \"" << mode << "\": {\n"
               << "    \"planning_attempt_id\": " << trace.planning_attempt_id << ",\n"
               << "    \"candidate_id\": " << trace.candidate_id << ",\n"
               << "    \"snapshot_generation_id\": " << trace.snapshot_generation_id << ",\n"
               << "    \"query_base_time_s\": " << trace.query_base_time_s << ",\n"
               << "    \"support_signature\": \"" << trace.support_signature << "\",\n"
               << "    \"initial_control_points_hash\": \"" << trace.initial_control_points_hash << "\",\n"
               << "    \"p1_config_hash\": \"" << trace.p1_config_hash << "\",\n"
               << "    \"pre_base_objective\": " << trace.pre_base_objective << ",\n"
               << "    \"post_base_objective\": " << trace.post_base_objective << ",\n"
               << "    \"pre_total_objective\": " << trace.pre_total_objective << ",\n"
               << "    \"post_total_objective\": " << trace.post_total_objective << ",\n"
               << "    \"pre_raw_p1_cost\": " << trace.pre_raw_p1_cost << ",\n"
               << "    \"post_raw_p1_cost\": " << trace.post_raw_p1_cost << ",\n"
               << "    \"pre_weighted_p1_cost\": " << trace.pre_weighted_p1_cost << ",\n"
               << "    \"post_weighted_p1_cost\": " << trace.post_weighted_p1_cost << ",\n"
               << "    \"pre_base_gradient_norm\": " << trace.pre_base_gradient_norm << ",\n"
               << "    \"post_base_gradient_norm\": " << trace.post_base_gradient_norm << ",\n"
               << "    \"pre_raw_p1_gradient_norm\": " << trace.pre_raw_p1_gradient_norm << ",\n"
               << "    \"post_raw_p1_gradient_norm\": " << trace.post_raw_p1_gradient_norm << ",\n"
               << "    \"pre_weighted_p1_gradient_norm\": " << trace.pre_weighted_p1_gradient_norm << ",\n"
               << "    \"post_weighted_p1_gradient_norm\": " << trace.post_weighted_p1_gradient_norm << ",\n"
               << "    \"pre_total_gradient_norm\": " << trace.pre_total_gradient_norm << ",\n"
               << "    \"post_total_gradient_norm\": " << trace.post_total_gradient_norm << ",\n"
               << "    \"displacement_norm\": " << trace.displacement_norm << ",\n"
               << "    \"grad_integrity_dot_displacement\": " << trace.grad_integrity_dot_displacement << ",\n"
               << "    \"pre_mean_c_pi\": " << trace.pre_mean_c_pi << ",\n"
               << "    \"post_mean_c_pi\": " << trace.post_mean_c_pi << ",\n"
               << "    \"pre_max_c_pi\": " << trace.pre_max_c_pi << ",\n"
               << "    \"post_max_c_pi\": " << trace.post_max_c_pi << ",\n"
               << "    \"support_sample_count\": " << trace.support_sample_count << ",\n"
               << "    \"pre_support_valid_count\": " << trace.pre_support_valid_count << ",\n"
               << "    \"post_support_valid_count\": " << trace.post_support_valid_count << ",\n"
               << "    \"support_full_valid\": " << (trace.support_full_valid ? "true" : "false") << ",\n"
               << "    \"optimization_success\": " << (trace.optimization_success ? "true" : "false") << ",\n"
               << "    \"selected\": " << (trace.selected ? "true" : "false") << ",\n"
               << "    \"objective_allowed\": " << (trace.objective_allowed ? "true" : "false") << ",\n"
               << "    \"objective_applied\": " << (trace.objective_applied ? "true" : "false") << ",\n"
               << "    \"solver_result\": " << trace.solver_result << ",\n"
               << "    \"iteration_count\": " << trace.iteration_count << ",\n"
               << "    \"termination_reason\": \"" << trace.termination_reason << "\",\n"
               << "    \"selection_score\": " << trace.selection_score << ",\n"
               << "    \"selection_reason\": \"" << trace.selection_reason << "\",\n"
               << "    \"fallback_reason\": \"" << trace.fallback_reason << "\"\n"
               << "  }" << (trailing_comma ? "," : "") << "\n";
  };
  diagnostic << "{\n"
             << "  \"lambda_integrity\": 0.00001,\n"
             << "  \"same_snapshot_generation\": true,\n"
             << "  \"same_query_base\": true,\n"
             << "  \"same_initial_control_points_hash\": true,\n"
             << "  \"same_support_signature\": true,\n";
  write_trace("metrics_only", metrics_trace, true);
  write_trace("enabled", enabled_trace, true);
  diagnostic << "  \"weighted_p1_base_objective_ratio\": " << objective_ratio << ",\n"
             << "  \"weighted_p1_base_gradient_ratio\": " << gradient_ratio << ",\n"
             << "  \"rounding_ratio_scale\": " << rounding_ratio_scale << ",\n"
             << "  \"fixed_lambda_unmeasurable\": "
             << (fixed_lambda_unmeasurable ? "true" : "false") << "\n"
             << "}\n";
  diagnostic.close();
  EXPECT_TRUE(std::ifstream(diagnostic_path).good());
  std::remove(debug_path.c_str());
}

TEST(P1IntegrityCostTest,
     ObjectiveAppliedConvergenceResolvesFixedLambdaWhileMetricsOnlyIsUnchanged) {
  ego_planner::SwarmTrajData swarm;
  auto applied_config = disabledConfig();
  applied_config.use_integrity_cost = true;
  applied_config.metrics_only = false;
  applied_config.lambda_integrity = 0.00001;
  auto applied = makeOptimizer(applied_config, &swarm);
  const double applied_epsilon =
      applied->p1LbfgsGradientEpsilon(5.0e-7, 2.0);
  EXPECT_GE(applied_epsilon, 1.0e-8);
  EXPECT_LT(applied_epsilon, 1.0e-6);

  auto metrics_config = applied_config;
  metrics_config.metrics_only = true;
  auto metrics_only = makeOptimizer(metrics_config, &swarm);
  EXPECT_DOUBLE_EQ(
      metrics_only->p1LbfgsGradientEpsilon(5.0e-7, 2.0), 0.01);
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
  config.objective_aggregation_mode = "adaptive_mean";
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));

  EXPECT_EQ(optimizer->getLastP1IntegrityMetrics().sample_count, 3);
}

TEST(P1IntegrityCostTest, FixedLatticeModeAndRiskGradientSupplementAreDeterministic) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.objective_aggregation_mode = "fixed_200_mean";
  // This H4 finite-difference check validates sampling/projection, not the
  // separately tested conservative gradient clip.
  config.integrity_grad_norm_max = 100.0;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  optimizer->setBsplineInterval(0.1);
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));
  EXPECT_EQ(optimizer->getLastP1IntegrityMetrics().sample_count, 200);

  ego_planner::ControlPoints base;
  base.resize(q.cols());
  base.points = q;
  const auto candidates = optimizer->supplementP1RiskGradientCandidates(
      base, snapshot, snapshot->stamp_s(), 7);
  ASSERT_EQ(candidates.size(), 3U);
  EXPECT_GT((candidates[0].points - base.points).norm(), 0.0);
  EXPECT_LT((candidates[0].points - base.points).norm(),
            (candidates[2].points - base.points).norm());
  EXPECT_EQ(optimizer->lastP1FanoutDiagnostics().supplemental_candidate_count, 3);
}

TEST(P1IntegrityCostTest, IncumbentRiskUsesOnlyFutureTrajectoryWindow) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.objective_aggregation_mode = "fixed_200_mean";
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());

  ego_planner::UniformBspline trajectory(q, 3, 0.1);
  const double future_start_t_s = trajectory.getTimeSum() * 0.5;
  const auto whole = optimizer->evaluateP1FixedLatticeRisk(trajectory);
  const auto future = optimizer->evaluateP1FixedLatticeRisk(
      trajectory, future_start_t_s);
  const auto shared_forward_window = optimizer->evaluateP1FixedLatticeRisk(
      trajectory, future_start_t_s, trajectory.getTimeSum() * 0.1);

  ASSERT_TRUE(whole.full_support);
  ASSERT_TRUE(future.full_support);
  ASSERT_TRUE(shared_forward_window.full_support);
  EXPECT_GT(future.mean_c_pi, whole.mean_c_pi);
  EXPECT_LT(shared_forward_window.mean_c_pi, future.mean_c_pi);
  EXPECT_DOUBLE_EQ(future.max_c_pi, whole.max_c_pi);
  EXPECT_LT(shared_forward_window.max_c_pi, future.max_c_pi);
}

TEST(P1IntegrityCostTest, Fixed200RawGradientProbeDescends) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.objective_aggregation_mode = "fixed_200_mean";
  // Validate lattice/projection itself, independently of the conservative
  // per-sample clip (covered by GradientClippingIsRecorded).
  config.integrity_grad_norm_max = 100.0;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  double before = 0.0;
  Eigen::MatrixXd gradient;
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(q, 0.1, before, gradient));
  // Match the L-BFGS variable map: the spline prefix is immutable.  This is
  // deliberately not a whole-control-point test, because that could hide a
  // projection error in the optimizer callback.
  Eigen::MatrixXd active_gradient = Eigen::MatrixXd::Zero(q.rows(), q.cols());
  active_gradient.rightCols(q.cols() - optimizer->getOrder()) =
      gradient.rightCols(q.cols() - optimizer->getOrder());
  ASSERT_GT(active_gradient.norm(), 0.0);
  Eigen::MatrixXd probe = q - 0.01 * active_gradient / active_gradient.norm();
  double after = 0.0;
  Eigen::MatrixXd after_gradient;
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(probe, 0.1, after, after_gradient));
  EXPECT_LT(after, before);

  // H4: on the same fixed-200 lattice, the raw callback gradient must agree
  // with a central finite difference along the actual active variable block.
  constexpr double kFiniteDifferenceStep = 1.0e-5;
  const Eigen::MatrixXd direction = active_gradient / active_gradient.norm();
  double plus = 0.0, minus = 0.0;
  Eigen::MatrixXd ignored;
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(
      q + kFiniteDifferenceStep * direction, 0.1, plus, ignored));
  ASSERT_TRUE(optimizer->evaluateP1RawCostForTest(
      q - kFiniteDifferenceStep * direction, 0.1, minus, ignored));
  EXPECT_NEAR((plus - minus) / (2.0 * kFiniteDifferenceStep),
              gradient.cwiseProduct(direction).sum(), 1.0e-6);
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

TEST(P1IntegrityCostTest, InvalidMissIsConservativeWithoutInventedGradient) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  InvalidProvider provider;
  auto snapshot = makeSnapshot(provider);

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 2.0;
  config.unknown_policy = "skip";
  config.unknown_soft_penalty = 3.0;
  double cost = 0.0;
  Eigen::MatrixXd gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &cost, &gradient));

  const auto& metrics = optimizer->getLastP1IntegrityMetrics();
  EXPECT_GT(metrics.f_integrity, 0.0);
  EXPECT_EQ(metrics.grad_norm_integrity, 0.0);
}

TEST(P1IntegrityCostTest, SpatialOutOfMapUsesCappedInwardBarrierWithSkipPolicy) {
  ego_planner::SwarmTrajData swarm;
  Eigen::MatrixXd q = makeControlPoints();
  q.row(0).setConstant(20.0);  // outside the authoritative snapshot bounds
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);

  double original_cost = 0.0;
  Eigen::MatrixXd original_gradient;
  auto original = makeOptimizer(disabledConfig(), &swarm);
  ASSERT_TRUE(evaluate(*original, q, &original_cost, &original_gradient));

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 1.0;
  config.unknown_policy = "skip";
  config.unknown_soft_penalty = 1.0;
  config.integrity_grad_norm_max = 0.05;
  double barrier_cost = 0.0;
  Eigen::MatrixXd barrier_gradient;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ASSERT_TRUE(evaluate(*optimizer, q, &barrier_cost, &barrier_gradient));

  const auto& metrics = optimizer->getLastP1IntegrityMetrics();
  EXPECT_TRUE(metrics.applied_to_objective);
  EXPECT_EQ(metrics.hit_count, 0);
  EXPECT_GT(metrics.f_integrity, 0.0);
  EXPECT_GT(barrier_cost, original_cost);
  // At x > max, d(cost)/dx is positive; gradient descent therefore moves x
  // inward, toward the snapshot interior.
  EXPECT_GT(metrics.weighted_grad_integrity_norm, 0.0);
  const auto& samples = optimizer->getLastP1IntegrityVizSamples();
  ASSERT_FALSE(samples.empty());
  EXPECT_GT(samples.front().grad.x(), 0.0);
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

TEST(P1IntegrityCostTest,
     MetricsOnlyReferenceObservationSamplesOnlyRemainingTrajectoryWindow) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_reference_observation_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  const std::string context_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile_context.csv";
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(context_path.c_str());

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = true;
  config.lambda_integrity = 0.00001;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  ego_planner::UniformBspline trajectory(q, 3, 0.1);
  const Eigen::Vector3d expected_start = trajectory.evaluateDeBoorT(0.2);
  const Eigen::Vector3d expected_end = trajectory.evaluateDeBoorT(0.35);

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      trajectory, 8, 43, 10.2, 10.1, "map", 10.0, 0.2, 0.15));
  const auto rows = readCsvRows(profile_path);
  ASSERT_EQ(rows.size(), 200U);
  EXPECT_NEAR(std::stod(rows.front().at("t_s")), 0.0, 1e-12);
  EXPECT_NEAR(std::stod(rows.back().at("t_s")), 0.15, 1e-12);
  EXPECT_NEAR(std::stod(rows.front().at("x")), expected_start.x(), 1e-12);
  EXPECT_NEAR(std::stod(rows.back().at("x")), expected_end.x(), 1e-12);
  const auto context_rows = readCsvRows(context_path);
  ASSERT_EQ(context_rows.size(), 1U);
  EXPECT_NEAR(std::stod(context_rows.front().at("trajectory_time_max_s")),
              0.15, 1e-12);
  EXPECT_NEAR(std::stod(context_rows.front().at("trajectory_start_stamp_s")),
              10.0, 1e-12);
  EXPECT_NEAR(std::stod(context_rows.front().at("accepted_stamp_s")),
              10.2, 1e-12);
  EXPECT_EQ(context_rows.front().at("temporal_in_horizon"), "1");

  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(context_path.c_str());
}

TEST(P1IntegrityCostTest,
     AcceptedProfileContextAtomicallyRetainsEarlierPublishBindings) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_context_history_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  const std::string context_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile_context.csv";
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(context_path.c_str());

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
      trajectory, 8, 43, 10.2, 10.1, "map", 10.0));
  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      trajectory, 9, 44, 11.2, 11.1, "map", 11.0));

  const auto context_rows = readCsvRows(context_path);
  ASSERT_EQ(context_rows.size(), 2U);
  EXPECT_EQ(context_rows[0].at("profile_seq"), "8");
  EXPECT_EQ(context_rows[1].at("profile_seq"), "9");
  EXPECT_NEAR(std::stod(context_rows[0].at("trajectory_start_stamp_s")),
              10.0, 1e-12);
  EXPECT_NEAR(std::stod(context_rows[1].at("trajectory_start_stamp_s")),
              11.0, 1e-12);

  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(context_path.c_str());
}

TEST(P1IntegrityCostTest, BaseFallbackWithoutSnapshotWritesFailClosedProfile) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  const std::string debug_path = tempProfilePath("p1_base_fallback_debug");
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
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.objective_allowed = false;
  context.fallback_reason = "snapshot_unavailable";
  optimizer->setP1PlanningRiskContext(context);

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      ego_planner::UniformBspline(q, 3, 0.1), 8, 43, 123.0));
  const auto rows = readCsvRows(profile_path);
  ASSERT_EQ(rows.size(), 200U);
  for (const auto& row : rows) {
    EXPECT_EQ(row.at("hit"), "0");
    EXPECT_EQ(row.at("reason"), "snapshot_unavailable");
    EXPECT_EQ(row.at("objective_requested"), "1");
    EXPECT_EQ(row.at("objective_applied"), "0");
    EXPECT_EQ(row.at("p1_fallback"), "1");
    EXPECT_EQ(row.at("fallback_reason"), "snapshot_unavailable");
  }
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
}

TEST(P1IntegrityCostTest,
     AcceptedFallbackWritesOccupiedInterpolationCornerEvidence) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto params = makeParams();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  const auto occupancy = [](const Eigen::Vector3d& p) {
    iap::RiskOccupancyDiagnostic diagnostic;
    diagnostic.available = true;
    diagnostic.raw_occupied = p.x() > 0.5;
    diagnostic.inflated_occupied = diagnostic.raw_occupied;
    diagnostic.frame_id = "map";
    diagnostic.cloud_stamp_s = 9.9;
    diagnostic.occupancy_generation = 17;
    diagnostic.resolution_m = 1.0;
    diagnostic.inflation_m = 0.0;
    diagnostic.source = "accepted_fallback_test";
    return diagnostic;
  };
  std::string refresh_reason;
  ASSERT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider, occupancy, &refresh_reason))
      << refresh_reason;
  auto snapshot = grid.acquireSnapshot();
  ASSERT_NE(snapshot, nullptr);

  const std::string debug_path = tempProfilePath("p1_occupied_fallback_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  config.evidence_schema_version = "p1_evidence_provenance_v4";
  config.evidence_run_id = "accepted-fallback-occupancy-test";
  config.evidence_manifest_path = "/tmp/accepted-fallback-manifest.json";
  auto optimizer = makeOptimizer(config, &swarm);
  optimizer->setP1IntegrityConfigForTest(config);
  optimizer->setRiskSnapshot(snapshot, snapshot->stamp_s());
  std::remove(profile_path.c_str());
  std::remove(optimizer->p0OccupancyQueryEvidencePath().c_str());

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      ego_planner::UniformBspline(q, 3, 0.1), 9, 44, 10.5));
  const auto rows = readCsvRows(optimizer->p0OccupancyQueryEvidencePath());
  ASSERT_GE(rows.size(), 1600U);
  EXPECT_TRUE(std::all_of(rows.begin(), rows.end(), [](const auto& row) {
    return row.at("phase") == "accepted";
  }));
  EXPECT_TRUE(std::any_of(rows.begin(), rows.end(), [](const auto& row) {
    return row.at("invalid_reason") == "occupied" &&
        row.at("inflated_occupied") == "1" &&
        row.at("occupancy_generation") == "17";
  }));

  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(optimizer->p0OccupancyQueryEvidencePath().c_str());
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
    EXPECT_NEAR(std::stod(row.at("grad_x")), 1.0, 1e-9);
    EXPECT_NEAR(std::stod(row.at("neg_grad_x")), -1.0, 1e-9);
    EXPECT_NEAR(std::stod(row.at("neg_grad_y")), 0.0, 1e-9);
    EXPECT_NEAR(std::stod(row.at("neg_grad_z")), 0.0, 1e-9);
  }
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
}

TEST(P1IntegrityCostTest, AcceptedProfileWritesBoundedContextSidecar) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd q = makeControlPoints();
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_context_debug");
  const std::string profile_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile.csv";
  const std::string context_path =
      "/tmp/planner_p1_accepted_trajectory_risk_profile_context.csv";
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(context_path.c_str());

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
      trajectory, 10, 45, 126.0, 125.5));
  const auto context_rows =
      readCsvRows(optimizer->p1AcceptedTrajectoryRiskProfileContextPath());
  ASSERT_EQ(context_rows.size(), 1U);
  const auto& context = context_rows.front();
  EXPECT_EQ(context.at("profile_seq"), "10");
  EXPECT_EQ(context.at("trajectory_id"), "45");
  EXPECT_EQ(context.at("expected_sample_count"), "200");
  EXPECT_EQ(context.at("matched_sample_count"), "200");
  EXPECT_EQ(context.at("snapshot_generation_id"),
            std::to_string(snapshot->generation_id()));
  EXPECT_LT(std::stod(context.at("snapshot_x_min")),
            std::stod(context.at("snapshot_x_max")));
  std::remove(debug_path.c_str());
  std::remove(profile_path.c_str());
  std::remove(context_path.c_str());
}

TEST(P1IntegrityCostTest, AcceptedProfileBindsGradientToActualDisplacement) {
  ego_planner::SwarmTrajData swarm;
  const Eigen::MatrixXd before = makeControlPoints();
  Eigen::MatrixXd after = before;
  after.row(0).array() -= 0.1;
  AffineProvider provider(10.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  auto snapshot = makeSnapshot(provider);
  const std::string debug_path = tempProfilePath("p1_direction_debug");
  std::remove(debug_path.c_str());
  std::remove("/tmp/planner_p1_accepted_trajectory_risk_profile.csv");

  auto config = disabledConfig();
  config.use_integrity_cost = true;
  config.metrics_only = false;
  config.lambda_integrity = 0.00001;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  auto optimizer = makeOptimizer(config, &swarm);
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.snapshot = snapshot;
  context.query_base_time_s = snapshot->stamp_s();
  context.planning_attempt_id = 51;
  context.candidate_id = 7;
  optimizer->setP1PlanningRiskContext(context);
  optimizer->setP1PreOptimizationTrajectoryForTest(before, 0.1);

  ASSERT_TRUE(optimizer->writeP1AcceptedTrajectoryRiskProfile(
      ego_planner::UniformBspline(after, 3, 0.1), 11, 46, 10.5));
  const auto rows = readCsvRows(optimizer->p1AcceptedTrajectoryRiskProfilePath());
  ASSERT_EQ(rows.size(), 200U);
  for (const auto &row : rows)
  {
    EXPECT_EQ(row.at("trace_available"), "1");
    EXPECT_NEAR(std::stod(row.at("disp_x")), -0.1, 1e-9);
    EXPECT_NEAR(std::stod(row.at("grad_dot_displacement")), -0.1, 1e-9);
    EXPECT_NEAR(std::stod(row.at("delta_c_pi")), -0.1, 1e-9);
  }
  std::remove(debug_path.c_str());
  std::remove("/tmp/planner_p1_accepted_trajectory_risk_profile.csv");
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

TEST(P1IntegrityCostTest,
     IncumbentPresenceDoesNotDependOnComparisonSupport) {
  ego_planner::BsplineOptimizer::P1OptimizationTrace trace;
  EXPECT_FALSE(trace.incumbent_available);

  // The published incumbent still exists when an occupied interpolation
  // corner prevents a full fixed-200 comparison.  Its presence must close
  // retained-trajectory identity independently from comparison statistics.
  trace.markIncumbentAvailable();

  EXPECT_TRUE(trace.incumbent_available);
  EXPECT_EQ(trace.replacement_comparison_mode, "full_profile");
  EXPECT_DOUBLE_EQ(trace.replacement_comparison_duration_s, 0.0);
}
