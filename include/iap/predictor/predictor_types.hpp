#pragma once
// Independent advisory predictor query API.
//
// This module is planner-side and advisory only. It does not publish current
// certified monitor PL, build grids, compute AL/IM/PI cost, or own planner
// topic schemas.

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <iap/gnss/visibility_predictor.hpp>
#include <iap/predictor/advisory_fim_types.hpp>
#include <iap/predictor/gnss_geometry_pl_predictor.hpp>
#include <iap/planner/integrity_snapshot.hpp>
#include <iap/predictor/lidar_observability_fim.hpp>

namespace iap {

struct GnssAdvisoryPredictorParams {
  GnssGeometryPlPredictorParams geometry_params;
  VisibilityPredictor::Params visibility_params;
  double fallback_pl = 5.0;
  double fim_clock_epsilon = 1.0e-6;
  double fim_psd_epsilon = 1.0e-9;
};

struct LidarAdvisoryPredictorParams {
  LidarObservabilityFim::Params fim_params;
  bool enable_legacy_observability = true;
};

struct FusionAdvisoryPredictorParams {
  double fim_epsilon = 1.0e-6;
  double K_H_adv = 5.0;
  double K_V_adv = 5.0;
  double b_H_pred = 0.0;
  double b_V_pred = 0.0;
  double s_H_pred = 0.0;
  double s_V_pred = 0.0;
  bool conservative_max_with_gnss = false;
};

struct PredictorFreshnessGuardParams {
  bool enabled = false;
  double max_odom_age_s = 0.5;
  double max_integrity_age_s = 0.5;
  double max_gnss_age_s = 0.5;
  double max_snapshot_age_s = 0.5;
};

enum class PredictorSourceMode {
  Fusion = 0,
  GnssOnly = 1,
  LidarOnly = 2,
};

enum class PredictorGnssEpochPolicy {
  Auto = 0,
  Required = 1,
  Optional = 2,
  Disabled = 3,
};

struct PredictorParams {
  GnssAdvisoryPredictorParams gnss;
  LidarAdvisoryPredictorParams lidar;
  FusionAdvisoryPredictorParams fusion;
  PredictorFreshnessGuardParams freshness;
  PredictorSourceMode source_mode = PredictorSourceMode::Fusion;
  PredictorGnssEpochPolicy gnss_epoch_policy =
      PredictorGnssEpochPolicy::Auto;
};

enum class PredictorInformationState {
  // Common fusion state for Predictor: 3D position-only information in the
  // map/ENU frame. GNSS clock and any LiDAR pose states must be eliminated
  // before writing lambda_* into Predictor results.
  Position3MapEnu = 0,
};

struct GnssAdvisoryResult {
  bool available = false;
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  PredictorInformationState information_state =
      PredictorInformationState::Position3MapEnu;

  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double pl_scalar = std::numeric_limits<double>::quiet_NaN();
  double pl_e = std::numeric_limits<double>::quiet_NaN();
  double pl_n = std::numeric_limits<double>::quiet_NaN();
  double pl_u = std::numeric_limits<double>::quiet_NaN();
  double pl_ff_h = std::numeric_limits<double>::quiet_NaN();
  double pl_ff_v = std::numeric_limits<double>::quiet_NaN();
  double sigma_h = std::numeric_limits<double>::quiet_NaN();
  double sigma_v = std::numeric_limits<double>::quiet_NaN();
  double pdop = std::numeric_limits<double>::quiet_NaN();
  double hdop = std::numeric_limits<double>::quiet_NaN();
  double vdop = std::numeric_limits<double>::quiet_NaN();
  double effective_sigma_mean = std::numeric_limits<double>::quiet_NaN();
  double effective_sigma_max = std::numeric_limits<double>::quiet_NaN();

  int n_visible = 0;
  int n_used = 0;
  int n_hypotheses = 0;
  int n_excluded = 0;
  std::vector<int> visible_sat_ids;
  std::vector<int> used_sat_ids;
  std::vector<int> excluded_sat_ids;

  // R^{3x3} position-only map/ENU information after eliminating receiver
  // clock from the 4D GNSS normal matrix.
  Eigen::Matrix3d lambda_gnss = Eigen::Matrix3d::Zero();
  bool fim_valid = false;
  bool fim_regularized = false;
  double lambda_trace = 0.0;
  double lambda_min_eig = 0.0;
  double lambda_max_eig = 0.0;
  double lambda_condition = 1.0e12;
  std::string fim_fallback_reason = "not_evaluated";
};

struct LidarAdvisoryResult {
  bool available = false;
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  PredictorInformationState information_state =
      PredictorInformationState::Position3MapEnu;

  // R^{3x3} position-only map/ENU LiDAR advisory information. A future 6D
  // pose FIM source must be projected or marginalized to this state first.
  Eigen::Matrix3d lambda_lidar = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d legacy_delta_lambda = Eigen::Matrix3d::Zero();

  bool fim_valid = false;
  bool legacy_valid = false;
  bool fim_regularized = false;
  double lidar_alpha = 0.0;
  double tdop_proxy = 20.0;
  double condition = 1.0e6;
  int n_primitives = 0;
  int n_valid_normals = 0;
  double bias_h = 0.0;
  double bias_v = 0.0;

  double lambda_trace = 0.0;
  double lambda_min_eig = 0.0;
  double lambda_max_eig = 0.0;
  double lambda_condition = 1.0e12;
};

struct FusionAdvisoryResult {
  bool available = false;
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  PredictorInformationState information_state =
      PredictorInformationState::Position3MapEnu;

  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double pl_scalar = std::numeric_limits<double>::quiet_NaN();
  double sigma_h = std::numeric_limits<double>::quiet_NaN();
  double sigma_v = std::numeric_limits<double>::quiet_NaN();

  // All matrices below are R^{3x3} position-only map/ENU information or
  // covariance over the common Predictor fusion state.
  Eigen::Matrix3d lambda_prior = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d lambda_gnss = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d lambda_lidar = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d lambda_pred = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d sigma_pos = Eigen::Matrix3d::Identity();

  bool prior_valid = false;
  bool gnss_used = false;
  bool lidar_used = false;
  bool epsilon_applied = false;
  bool degeneracy_regularized = false;
  bool conservative_max_applied = false;
  AdvisoryFusionMode fusion_mode = AdvisoryFusionMode::FimAdd;

  double lambda_prior_trace = 0.0;
  double lambda_gnss_trace = 0.0;
  double lambda_lidar_trace = 0.0;
  double lambda_pred_trace = 0.0;
  double lambda_pred_min_eig = 0.0;
  double lambda_pred_max_eig = 0.0;
  double lambda_pred_condition = 1.0e12;
};

struct PredictorQueryInput {
  PredictorQueryInput(Eigen::Vector3d query_position_map_in,
                      IntegritySnapshot snapshot_in,
                      const double query_time_s_in,
                      const double horizon_s_in = 0.0,
                      std::string frame_id_in = "map",
                      const double freshness_reference_time_s_in =
                          std::numeric_limits<double>::quiet_NaN())
      : query_position_map(std::move(query_position_map_in)),
        snapshot(std::move(snapshot_in)),
        query_time_s(query_time_s_in),
        horizon_s(horizon_s_in),
        frame_id(std::move(frame_id_in)),
        freshness_reference_time_s(freshness_reference_time_s_in) {}

  Eigen::Vector3d query_position_map;
  IntegritySnapshot snapshot;
  double query_time_s;
  double horizon_s;
  std::string frame_id = "map";
  double freshness_reference_time_s =
      std::numeric_limits<double>::quiet_NaN();
};

enum PredictorResultFlags : uint32_t {
  PREDICTOR_RESULT_VALID = 1u << 0,
  PREDICTOR_RESULT_FALLBACK = 1u << 1,
  PREDICTOR_RESULT_GNSS_VALID = 1u << 2,
  PREDICTOR_RESULT_LIDAR_VALID = 1u << 3,
  PREDICTOR_RESULT_FUSION_VALID = 1u << 4,
  PREDICTOR_RESULT_PRIOR_VALID = 1u << 5,
  PREDICTOR_RESULT_GNSS_USED = 1u << 6,
  PREDICTOR_RESULT_LIDAR_USED = 1u << 7,
  PREDICTOR_RESULT_REGULARIZED = 1u << 8,
  PREDICTOR_RESULT_CONSERVATIVE_MAX = 1u << 9,
  PREDICTOR_RESULT_AVAILABLE = 1u << 10,
};

struct PredictorQueryResult {
  bool available = false;
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  std::string query_source = "direct";
  Eigen::Vector3d query_position_map =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double horizon_s = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id = "map";
  uint32_t source_flags = 0u;

  GnssAdvisoryResult gnss;
  LidarAdvisoryResult lidar;
  FusionAdvisoryResult fused;
};

}  // namespace iap
