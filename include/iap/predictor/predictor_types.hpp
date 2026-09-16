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
  // A received GNSS epoch is direct evidence that its non-excluded signals
  // were usable at the receiver. It is not evidence that they were LOS: the
  // measured per-satellite sigma and integrity exclusions remain authoritative
  // for LOS/NLOS quality. This radius must cover only the voxel containing the
  // measured receiver pose; all other queries require online map support.
  double measured_epoch_support_radius_m = 0.0;
  // The current integrity report must belong to the same measurement epoch
  // before receiver-local support or its FDE exclusions may be applied.
  double measured_epoch_integrity_max_delta_s = 0.25;
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

struct EmpiricalCovarianceGrowthParams {
  double sigma_grow_m_sqrt_s =
      std::numeric_limits<double>::quiet_NaN();
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
  EmpiricalCovarianceGrowthParams covariance_growth;
  PredictorSourceMode source_mode = PredictorSourceMode::Fusion;
  PredictorGnssEpochPolicy gnss_epoch_policy =
      PredictorGnssEpochPolicy::Auto;
};

// Authoritative description of which spatial evidence the configured
// Predictor can consume. Cache identity and production transaction guards
// share this projection so inactive sources cannot accidentally invalidate
// active work.
struct PredictorSpatialSourceUsage {
  bool gnss = false;
  bool lidar = false;
  bool legacy_lidar = false;
};

inline PredictorSpatialSourceUsage predictorSpatialSourceUsage(
    const PredictorParams& params) {
  PredictorSpatialSourceUsage usage;
  usage.gnss = params.source_mode != PredictorSourceMode::LidarOnly &&
               params.gnss_epoch_policy !=
                   PredictorGnssEpochPolicy::Disabled;
  usage.lidar = params.source_mode != PredictorSourceMode::GnssOnly;
  usage.legacy_lidar =
      usage.lidar && params.lidar.enable_legacy_observability;
  return usage;
}

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
  GnssGeometryStatus geometry_status =
      GnssGeometryStatus::NOT_EVALUATED;
  std::vector<int> degenerate_satellite_ids;
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
  double weighted_geometry_condition =
      std::numeric_limits<double>::quiet_NaN();
  int worst_excluded_sat_h = -1;
  int worst_excluded_sat_v = -1;

  // The planner GNSS channel is anchored to the current certified monitor.
  // Raw advisory values remain available for spatial-delta diagnostics and
  // pre-conservative ranking; they are never an absolute safety authority.
  double raw_hpl = std::numeric_limits<double>::quiet_NaN();
  double raw_vpl = std::numeric_limits<double>::quiet_NaN();
  double receiver_raw_hpl = std::numeric_limits<double>::quiet_NaN();
  double receiver_raw_vpl = std::numeric_limits<double>::quiet_NaN();
  double anchor_hpl = std::numeric_limits<double>::quiet_NaN();
  double anchor_vpl = std::numeric_limits<double>::quiet_NaN();
  double spatial_delta_h = std::numeric_limits<double>::quiet_NaN();
  double spatial_delta_v = std::numeric_limits<double>::quiet_NaN();
  double temporal_growth_h = 0.0;
  double temporal_growth_v = 0.0;
  bool anchor_consistent = false;
  double anchor_epoch_delta_s = std::numeric_limits<double>::quiet_NaN();

  int n_visible = 0;
  int n_unknown_support = 0;
  int n_known_support = 0;
  int n_blocked = 0;
  int n_used = 0;
  int n_hypotheses = 0;
  int n_excluded = 0;
  bool measured_epoch_support_used = false;
  LocalMapSupportAuthority support_authority =
      LocalMapSupportAuthority::STRICT_OBSERVATION;
  LocalMapSupportStatus support_status =
      LocalMapSupportStatus::FRAME_INVALID;
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
  // Diagnostic channels retain the information before the conservative GNSS
  // safety floor. They are not substitutes for hpl/vpl in P5.
  double prior_only_hpl = std::numeric_limits<double>::quiet_NaN();
  double prior_only_vpl = std::numeric_limits<double>::quiet_NaN();
  double lidar_only_hpl = std::numeric_limits<double>::quiet_NaN();
  double lidar_only_vpl = std::numeric_limits<double>::quiet_NaN();
  double pre_conservative_hpl = std::numeric_limits<double>::quiet_NaN();
  double pre_conservative_vpl = std::numeric_limits<double>::quiet_NaN();
  double floor_increment_h = 0.0;
  double floor_increment_v = 0.0;
  std::string floor_source_h = "none";
  std::string floor_source_v = "none";

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
                      const double evaluation_time_s_in =
                          std::numeric_limits<double>::quiet_NaN())
      : query_position_map(std::move(query_position_map_in)),
        snapshot(std::move(snapshot_in)),
        query_time_s(query_time_s_in),
        horizon_s(horizon_s_in),
        frame_id(std::move(frame_id_in)),
        evaluation_time_s(evaluation_time_s_in) {}

  Eigen::Vector3d query_position_map;
  IntegritySnapshot snapshot;
  double query_time_s;
  double horizon_s;
  std::string frame_id = "map";
  // Wall/ROS simulation time at which this prediction round evaluates input
  // freshness. This is distinct from the candidate arrival query_time_s.
  double evaluation_time_s =
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
  PREDICTOR_RESULT_STALE_CURRENT_PRIOR = 1u << 11,
};

enum class CovarianceGrowthStatus {
  NOT_REQUIRED_TAU_ZERO = 0,
  APPLIED,
  INVALID_HORIZON,
  INVALID_PARAMETER,
  MISSING_PRIOR,
  STALE_PRIOR,
  INVALID_PRIOR,
  NUMERICAL_FAILURE,
  NOT_EVALUATED,
};

enum class PredictorFreshnessStatus {
  NOT_EVALUATED = 0,
  FRESH,
  STALE,
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
  CovarianceGrowthStatus covariance_growth_status =
      CovarianceGrowthStatus::NOT_EVALUATED;
  PredictorFreshnessStatus freshness_status =
      PredictorFreshnessStatus::NOT_EVALUATED;

  GnssAdvisoryResult gnss;
  LidarAdvisoryResult lidar;
  FusionAdvisoryResult fused;
};

enum class ForwardRiskSafetyState {
  SAFE = 0,
  UNSAFE,
  UNKNOWN,
};

enum class ForwardRiskRankingState {
  COMPARABLE = 0,
  INCOMPLETE,
};

enum class ForwardRiskFailureReason {
  NONE = 0,
  SAFETY_LIMIT_EXCEEDED,
  GNSS_ANCHOR_INCONSISTENT,
  GNSS_LOCAL_USABLE_SATS_LT_MIN,
  GNSS_SKY_UNKNOWN,
  GNSS_GEOMETRY_DEGENERATE,
  OCCUPANCY_UNKNOWN,
  OCCUPIED,
  LIDAR_SUPPORT_MISSING,
  FIM_SUPPORT_MISSING,
  STALE,
  GENERATION_CHANGED,
  EVIDENCE_IDENTITY_MISMATCH,
  COMPUTE_BUDGET_EXCEEDED,
};

const char* forwardRiskFailureReasonName(ForwardRiskFailureReason reason);

struct ForwardRiskQueryPoint {
  Eigen::Vector3d position_map = Eigen::Vector3d::Zero();
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double horizon_s = 0.0;
  std::uint64_t candidate_group_id = 0;
  // A physical space/time sample. Repeated IDs are evaluated once and may be
  // assigned to two satellite windows to certify a handover with both cores.
  // Zero preserves the legacy behavior of treating every request row as a
  // distinct evidence point.
  std::uint64_t evidence_point_id = 0;
  // Deterministic execution-commitment window. Required by
  // BRAKING_WINDOW_CORE and ignored by the legacy policies.
  std::uint64_t satellite_window_id = 0;
};

enum class ForwardRiskSatelliteSetPolicy {
  // Coarse RiskGrid construction keeps the best locally supported set at
  // each voxel.
  PER_POINT = 0,
  // Execution authorization uses one conservative set that is usable at
  // every point of the remaining curve.
  COMMON_CORE,
  // Execution authorization intersects locally usable satellites only over
  // one reaction-and-braking commitment window. Transition samples appear
  // once per adjacent window and must pass with both exact cores.
  BRAKING_WINDOW_CORE,
};

struct GnssRiskSatelliteDiagnostic {
  int sat_id = 0;
  bool epoch_excluded = false;
  bool above_elevation_mask = false;
  bool support_known = false;
  bool visible = false;
  bool blocked = false;
  bool used = false;
  Eigen::Vector3d los_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double elevation_rad = std::numeric_limits<double>::quiet_NaN();
  double azimuth_rad = std::numeric_limits<double>::quiet_NaN();
  double kappa = std::numeric_limits<double>::quiet_NaN();
  double epoch_pr_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double canopy_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_eff_m = std::numeric_limits<double>::quiet_NaN();
  std::string sigma_source = "not_evaluated";
  std::string exclusion_reason = "not_evaluated";
};

struct ForwardRiskBatchRequest {
  std::string combined_snapshot_identity;
  IntegritySnapshot snapshot;
  std::vector<ForwardRiskQueryPoint> points;
  double hal = 10.0;
  double val = 20.0;
  double evaluation_time_s =
      std::numeric_limits<double>::quiet_NaN();
  double compute_budget_ms = std::numeric_limits<double>::infinity();
  ForwardRiskSatelliteSetPolicy satellite_set_policy =
      ForwardRiskSatelliteSetPolicy::PER_POINT;
};

struct ForwardRiskPointResult {
  ForwardRiskSafetyState safety_state = ForwardRiskSafetyState::UNKNOWN;
  ForwardRiskRankingState ranking_state =
      ForwardRiskRankingState::INCOMPLETE;
  ForwardRiskFailureReason failure_reason =
      ForwardRiskFailureReason::GNSS_SKY_UNKNOWN;
  PredictorQueryResult prediction;
  double safety_ratio = std::numeric_limits<double>::quiet_NaN();
  double fim_ratio = std::numeric_limits<double>::quiet_NaN();
  bool gnss_supported = false;
  bool lidar_supported = false;
  bool fim_supported = false;
  double gnss_support_ray_length_m =
      std::numeric_limits<double>::quiet_NaN();
  bool gnss_hard_occlusion = false;
  // Online evidence retained even when this point's local satellite set is
  // insufficient. It is diagnostic/advisory only and never changes UNKNOWN
  // into SAFE.
  int gnss_visible_satellite_count = 0;
  int gnss_blocked_satellite_count = 0;
  int gnss_attenuated_satellite_count = 0;
  int gnss_unknown_satellite_count = 0;
  int gnss_known_satellite_count = 0;
  int gnss_used_satellite_count = 0;
  std::uint64_t local_satellite_set_hash = 0;
  std::vector<GnssRiskSatelliteDiagnostic> gnss_satellites;
  bool known_hazard_evidence = false;
  double known_gnss_degradation_ratio = 0.0;
  double known_fim_ratio = std::numeric_limits<double>::quiet_NaN();
  double unknown_coverage = 1.0;
};

struct ForwardRiskWindowResult {
  std::uint64_t satellite_window_id = 0;
  std::vector<int> satellite_ids;
  std::uint64_t satellite_set_hash = 0;
  std::size_t point_count = 0;
  bool complete = false;
  std::size_t first_failure_index = std::numeric_limits<std::size_t>::max();
  ForwardRiskFailureReason failure_reason = ForwardRiskFailureReason::NONE;
};

struct ForwardRiskBatchTiming {
  std::size_t unique_evidence_point_count = 0;
  std::size_t evidence_reuse_count = 0;
  std::size_t receiver_cache_hit_count = 0;
  std::size_t candidate_cache_hit_count = 0;
  double evidence_ms = 0.0;
  double core_construction_ms = 0.0;
  double advisory_ms = 0.0;
  double transition_advisory_ms = 0.0;
  double total_ms = 0.0;
};

struct ForwardRiskBatchResult {
  bool complete = false;
  std::string combined_snapshot_identity;
  // For COMMON_CORE, the exact sorted IDs used by every point. For
  // BRAKING_WINDOW_CORE, the diagnostic whole-request intersection computed
  // from the same evidence pass but not used for authorization. Empty for
  // PER_POINT requests or incomplete evidence.
  std::vector<int> common_satellite_ids;
  // Populated for BRAKING_WINDOW_CORE in first-appearance order. Each entry
  // contains the exact sorted IDs used by every row assigned to that window.
  std::vector<ForwardRiskWindowResult> windows;
  ForwardRiskBatchTiming timing;
  std::size_t first_failure_index = std::numeric_limits<std::size_t>::max();
  ForwardRiskFailureReason failure_reason = ForwardRiskFailureReason::NONE;
  std::vector<ForwardRiskPointResult> points;
};

}  // namespace iap
