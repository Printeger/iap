// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT compact backend handoff surface.

#include <gtest/gtest.h>

#include <iap/gnss/gnss_types.hpp>
#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/ct_compact_backend.hpp>
#include <iap/odometry/spline_state_layout.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace {

// Build a minimal valid SplineStateLayout covering [0, 1] with 4 control points.
iap::CTLocalFrontendResult make_local_result_with_gnss_coverage() {
  iap::CTLocalFrontendResult result;

  // Cubic B-spline: 4 controls + 4 degree = 8 knots for a clamped [0,1] domain.
  // Knot vector: {0,0,0,0, 1,1,1,1} — single span covering [0,1].
  std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<iap::BSplineControlPointState> controls;
  for (std::size_t i = 0; i < 4; ++i) {
    controls.push_back(iap::BSplineControlPointState{
      i,
      static_cast<double>(i) * 0.25,
      gtsam::Pose3(),
    });
  }
  result.layout.set_knots(std::move(knots));
  result.layout.set_controls(std::move(controls));

  // Register a GNSS sensor model with zero time offset.
  iap::SplineSensorModel gnss_model;
  gnss_model.id = iap::SplineSensorId::Gnss;
  gnss_model.time_offset = 0.0;
  result.layout.set_sensor_model(iap::SplineSensorId::Gnss, gnss_model);

  // Seed control-point values.
  for (std::size_t i = 0; i < 4; ++i) {
    result.local_values.insert(iap::bspline_control_point_key(i), gtsam::Pose3());
  }

  result.backend_summary.pose_key_count = 4;
  return result;
}

iap::CTCompactBackend::Input make_gnss_input_with_one_sat() {
  iap::CTCompactBackend::Input input;
  input.gnss_anchor_initialized = true;
  input.ecef_origin = gtsam::Vector3(0.0, 0.0, 0.0);
  input.ecef_rot = gtsam::Rot3::Identity();
  input.gnss_pr_noise_base = 1.0;
  input.gnss_dop_noise_base = 0.1;
  input.gnss_min_elevation = 0.0;

  iap::GnssEpoch epoch;
  epoch.stamp = 0.5;  // inside [0,1] domain
  epoch.gps_sec = 0.0;

  iap::SatObs sat;
  sat.sat_id = 1;
  sat.constellation = 'G';
  sat.pr_meas = 1000.0;
  sat.dop_meas = 0.0;
  sat.sat_pos = Eigen::Vector3d(1000.0, 0.0, 0.0);
  sat.sat_vel = Eigen::Vector3d::Zero();
  sat.elevation = 0.5;  // ~28 degrees, above min
  sat.excluded = false;

  epoch.sats.push_back(sat);
  input.gnss_epochs.push_back(std::move(epoch));
  return input;
}

}  // namespace

TEST(CTCompactBackendContract, BackendDoesNotOwnRawLidarBuckets) {
  iap::CTBackendSummary summary;
  summary.lidar_factor_count = 16;
  summary.pose_key_count = 4;

  iap::CTCompactBackend backend;
  const auto stats = backend.debug_stats(summary);

  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_GT(stats.summary_pose_count, 0U);
}

TEST(CTCompactBackendContract, BackendAcceptsGnssFactorsOnly) {
  iap::CTCompactBackend backend;
  iap::CTBackendSummary summary;
  summary.pose_key_count = 3;

  const auto stats = backend.debug_stats(summary);

  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_EQ(stats.summary_pose_count, 3U);
  EXPECT_EQ(stats.gnss_factor_count, 0U);
}

// IAP-RQ-300 / IAP-RQ-410: update() with empty gnss_epochs must not add any factors.
TEST(CTCompactBackendUpdate, UpdateWithNoGnssEpochsDoesNothing) {
  iap::CTCompactBackend backend;
  const auto local_result = make_local_result_with_gnss_coverage();

  iap::CTCompactBackend::Input input;
  input.gnss_anchor_initialized = true;
  input.gnss_epochs.clear();  // empty

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  backend.update(local_result, input, &graph, &values);

  EXPECT_EQ(graph.size(), 0U);

  iap::CTBackendSummary summary;
  summary.pose_key_count = 4;
  const auto stats = backend.debug_stats(summary);
  EXPECT_EQ(stats.gnss_factor_count, 0U);
}

// IAP-RQ-300 / IAP-RQ-410: update() with one epoch + one satellite must add factors.
TEST(CTCompactBackendUpdate, UpdateWithGnssEpochsAddsFactors) {
  iap::CTCompactBackend backend;
  const auto local_result = make_local_result_with_gnss_coverage();
  const auto input = make_gnss_input_with_one_sat();

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  backend.update(local_result, input, &graph, &values);

  EXPECT_EQ(graph.size(), 2U);

  iap::CTBackendSummary summary;
  summary.pose_key_count = 4;
  const auto stats = backend.debug_stats(summary);
  EXPECT_GT(stats.gnss_factor_count, 0U);
}

// IAP-RQ-300 / IAP-RQ-410: update() must never add raw LiDAR factors.
TEST(CTCompactBackendUpdate, UpdateNeverAddsRawLidarFactors) {
  iap::CTCompactBackend backend;
  const auto local_result = make_local_result_with_gnss_coverage();
  const auto input = make_gnss_input_with_one_sat();

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  backend.update(local_result, input, &graph, &values);

  iap::CTBackendSummary summary;
  summary.pose_key_count = 4;
  const auto stats = backend.debug_stats(summary);
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
}

// IAP-RQ-300 / IAP-RQ-410: update() with null graph or values must be a no-op (no crash).
TEST(CTCompactBackendContract, NullGraphOrValuesIsNoOp) {
  iap::CTCompactBackend backend;
  iap::CTLocalFrontendResult result;
  iap::CTCompactBackend::Input input;
  input.gnss_anchor_initialized = true;
  // Should not crash
  ASSERT_NO_THROW(backend.update(result, input, nullptr, nullptr));
}
