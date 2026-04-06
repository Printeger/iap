// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for planner consumption of the continuous-time trajectory view.

#include <gtest/gtest.h>

#include <iap/planner/integrity_planner.hpp>

namespace {

class MockTrajectoryView : public iap::ContinuousTrajectoryView {
 public:
  explicit MockTrajectoryView(std::vector<iap::TrajectorySample> samples) : samples_(std::move(samples)) {}

  bool empty() const override { return samples_.empty(); }
  double start_time() const override { return samples_.front().stamp; }
  double end_time() const override { return samples_.back().stamp; }
  iap::SplineMeta meta() const override { return {}; }

  std::optional<iap::TrajectorySample> sample(double stamp) const override {
    for (const auto& sample : samples_) {
      if (std::abs(sample.stamp - stamp) < 1e-9) {
        return sample;
      }
    }
    return std::nullopt;
  }
  std::optional<iap::TrajectorySample> latest_sample() const override {
    if (samples_.empty()) {
      return std::nullopt;
    }
    return samples_.back();
  }
  std::vector<iap::TrajectorySample> sample_range(double start, double end, double) const override {
    std::vector<iap::TrajectorySample> selected;
    for (const auto& sample : samples_) {
      if (sample.stamp >= start && sample.stamp <= end) {
        selected.push_back(sample);
      }
    }
    return selected;
  }

 private:
  std::vector<iap::TrajectorySample> samples_;
};

class MockControlAccess : public iap::SplineControlAccess {
 public:
  explicit MockControlAccess(std::vector<iap::SplineControlPoint> control_points)
  : control_points_(std::move(control_points)) {}

  iap::SplineMeta meta() const override { return {}; }
  std::vector<double> knot_vector() const override { return {}; }
  std::vector<iap::SplineControlPoint> control_points() const override { return control_points_; }
  iap::SplineWindowSnapshot clone_window() const override {
    iap::SplineWindowSnapshot snapshot;
    snapshot.control_points = control_points_;
    return snapshot;
  }

 private:
  std::vector<iap::SplineControlPoint> control_points_;
};

}  // namespace

TEST(IntegrityPlannerTest, PlanSeedsCurrentStateFromContinuousTrajectoryView) {
  iap::TrajectoryGenerator::Params gen_params;
  gen_params.horizon = 0.2;
  gen_params.dt = 0.2;
  gen_params.speeds = {1.0};
  gen_params.yaw_rates = {0.0};
  gen_params.alt_rates = {0.0};

  iap::IntegrityPlanner::Params planner_params;
  planner_params.use_araim_pl = false;
  planner_params.w_integrity = 0.0;
  planner_params.w_mission = 0.0;
  planner_params.w_smooth = 0.0;
  planner_params.w_turn = 0.0;
  planner_params.w_infeasible = 0.0;

  iap::IntegrityPlanner planner(planner_params, gen_params);

  iap::TrajectorySample sample;
  sample.stamp = 10.0;
  sample.pose = Eigen::Isometry3d::Identity();
  sample.pose.translation() = Eigen::Vector3d(5.0, -1.0, 0.5);
  sample.vel = Eigen::Vector3d(2.0, 0.5, 0.0);
  sample.yaw = 0.7;
  sample.sigma = 1.25;
  planner.set_trajectory_view(std::make_shared<MockTrajectoryView>(std::vector<iap::TrajectorySample>{sample}));

  const auto chosen = planner.plan(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.0,
    Eigen::Vector3d::Zero(),
    0.1,
    nullptr);

  ASSERT_FALSE(chosen.points.empty());
  EXPECT_NEAR(chosen.points.front().pos.x(), 5.0, 1e-9);
  EXPECT_NEAR(chosen.points.front().pos.y(), -1.0, 1e-9);
  EXPECT_NEAR(chosen.points.front().pos.z(), 0.5, 1e-9);
  EXPECT_NEAR(chosen.points.front().vel.x(), 2.0, 1e-9);
  EXPECT_NEAR(chosen.points.front().vel.y(), 0.5, 1e-9);
  EXPECT_NEAR(chosen.points.front().yaw, 0.7, 1e-9);
  ASSERT_FALSE(chosen.sigma_pred.empty());
  EXPECT_NEAR(chosen.sigma_pred.front(), 1.25, 1e-9);
}

TEST(IntegrityPlannerTest, ContinuousTrajectorySeedVelocityInfluencesCandidateSelection) {
  iap::TrajectoryGenerator::Params gen_params;
  gen_params.horizon = 0.2;
  gen_params.dt = 0.2;
  gen_params.speeds = {0.0, 2.0};
  gen_params.yaw_rates = {0.0};
  gen_params.alt_rates = {0.0};

  iap::IntegrityPlanner::Params planner_params;
  planner_params.use_araim_pl = false;
  planner_params.w_integrity = 0.0;
  planner_params.w_mission = 0.0;
  planner_params.w_smooth = 1.0;
  planner_params.w_turn = 0.0;
  planner_params.w_infeasible = 0.0;

  iap::IntegrityPlanner planner(planner_params, gen_params);

  const auto without_view = planner.plan(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.0,
    Eigen::Vector3d::Zero(),
    0.1,
    nullptr);
  EXPECT_EQ(without_view.id, 0);

  iap::TrajectorySample sample;
  sample.stamp = 1.0;
  sample.pose = Eigen::Isometry3d::Identity();
  sample.vel = Eigen::Vector3d(2.0, 0.0, 0.0);
  sample.yaw = 0.0;
  planner.set_trajectory_view(std::make_shared<MockTrajectoryView>(std::vector<iap::TrajectorySample>{sample}));

  const auto with_view = planner.plan(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.0,
    Eigen::Vector3d::Zero(),
    0.1,
    nullptr);
  EXPECT_EQ(with_view.id, 1);
}

TEST(IntegrityPlannerTest, FutureTrajectorySigmaRaisesPredictedUncertainty) {
  iap::TrajectoryGenerator::Params gen_params;
  gen_params.horizon = 0.2;
  gen_params.dt = 0.2;
  gen_params.speeds = {1.0};
  gen_params.yaw_rates = {0.0};
  gen_params.alt_rates = {0.0};

  iap::IntegrityPlanner::Params planner_params;
  planner_params.use_araim_pl = false;
  planner_params.w_integrity = 0.0;
  planner_params.w_mission = 0.0;
  planner_params.w_smooth = 0.0;
  planner_params.w_turn = 0.0;
  planner_params.w_ct_align = 0.0;
  planner_params.w_infeasible = 0.0;

  iap::IntegrityPlanner planner(planner_params, gen_params);

  iap::TrajectorySample now;
  now.stamp = 1.0;
  now.pose = Eigen::Isometry3d::Identity();
  now.sigma = 0.1;

  iap::TrajectorySample future = now;
  future.stamp = 1.2;
  future.sigma = 5.0;

  planner.set_trajectory_view(std::make_shared<MockTrajectoryView>(std::vector<iap::TrajectorySample>{now, future}));
  iap::SplineControlPoint cp0;
  cp0.stamp = 1.0;
  iap::SplineControlPoint cp1;
  cp1.stamp = 1.2;
  planner.set_control_access(std::make_shared<MockControlAccess>(std::vector<iap::SplineControlPoint>{cp0, cp1}));

  const auto chosen = planner.plan(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.0,
    Eigen::Vector3d::Zero(),
    0.1,
    nullptr);

  ASSERT_GE(chosen.sigma_pred.size(), 2U);
  EXPECT_NEAR(chosen.sigma_pred[0], 0.1, 1e-9);
  EXPECT_GE(chosen.sigma_pred[1], 5.0);
  EXPECT_LT(chosen.sigma_pred[1], 5.001);
}

TEST(IntegrityPlannerTest, FutureTrajectoryVelocityInfluencesScoring) {
  iap::TrajectoryGenerator::Params gen_params;
  gen_params.horizon = 0.2;
  gen_params.dt = 0.2;
  gen_params.speeds = {0.0, 2.0};
  gen_params.yaw_rates = {0.0};
  gen_params.alt_rates = {0.0};

  iap::IntegrityPlanner::Params planner_params;
  planner_params.use_araim_pl = false;
  planner_params.w_integrity = 0.0;
  planner_params.w_mission = 0.0;
  planner_params.w_smooth = 0.0;
  planner_params.w_turn = 0.0;
  planner_params.w_ct_align = 1.0;
  planner_params.w_infeasible = 0.0;

  iap::IntegrityPlanner planner(planner_params, gen_params);

  const auto no_future = planner.plan(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.0,
    Eigen::Vector3d::Zero(),
    0.1,
    nullptr);
  EXPECT_EQ(no_future.id, 0);

  iap::TrajectorySample now;
  now.stamp = 1.0;
  now.pose = Eigen::Isometry3d::Identity();
  now.vel = Eigen::Vector3d::Zero();
  now.yaw = 0.0;
  now.sigma = 0.1;

  iap::TrajectorySample future = now;
  future.stamp = 1.2;
  future.vel = Eigen::Vector3d(2.0, 0.0, 0.0);

  planner.set_trajectory_view(std::make_shared<MockTrajectoryView>(std::vector<iap::TrajectorySample>{now, future}));
  iap::SplineControlPoint cp0;
  cp0.stamp = 1.0;
  iap::SplineControlPoint cp1;
  cp1.stamp = 1.2;
  planner.set_control_access(std::make_shared<MockControlAccess>(std::vector<iap::SplineControlPoint>{cp0, cp1}));

  const auto with_future = planner.plan(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.0,
    Eigen::Vector3d::Zero(),
    0.1,
    nullptr);
  EXPECT_EQ(with_future.id, 1);
}
