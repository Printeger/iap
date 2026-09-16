#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <iap/map/trusted_local_map_support.hpp>
#include <iap/predictor/gnss_geometry_pl_predictor.hpp>

namespace iap {

enum class RiskGridSourceValidation {
  VALID = 0,
  OCCUPANCY_GENERATION_CHANGED,
  PRIOR_GENERATION_CHANGED,
  PREDICTOR_SPATIAL_SOURCE_CHANGED,
  COMPUTE_BUDGET_EXCEEDED,
};

constexpr uint32_t RISK_GRID_SOURCE_OCCUPIED_SKIP = 1u << 31;

enum class RiskCostQueryPolicy {
  LEGACY_STRICT = 0,
  CONSERVATIVE_OCCUPIED_COST_SUPPORT,
};

enum class RiskProviderCostSource {
  LEGACY_SAFETY_PL = 0,
  PRE_CONSERVATIVE_FIM_RATIO,
};

enum class RiskOccupancyState : uint8_t {
  UNKNOWN = 0,
  OBSERVED_FREE = 1,
  OCCUPIED = 2,
};

struct PlanningLatticeGeometry {
  std::string frame_id;
  Eigen::Vector3d origin_w = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d extent_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3i voxel_dimensions = Eigen::Vector3i::Zero();
  double resolution_m = std::numeric_limits<double>::quiet_NaN();
  std::string geometry_id;

  bool valid() const {
    return !frame_id.empty() && !geometry_id.empty() &&
        origin_w.allFinite() && extent_m.allFinite() &&
        (extent_m.array() > 0.0).all() &&
        (voxel_dimensions.array() > 0).all() &&
        std::isfinite(resolution_m) && resolution_m > 0.0;
  }
};

struct P5_3HighRiskZoneFixtureConfig {
  bool enabled = false;
  std::string name = "future_high_risk_zone_v1";
  double x_min_m = -10.8;
  double x_max_m = -8.7;
  double y_min_m = -0.75;
  double y_max_m = 0.75;
  double z_min_m = 1.0;
  double z_max_m = 1.35;
  double tau_min_s = 1.2;
  double tau_max_s = 2.0;
  double hpl_pred_m = 10.2;
  double vpl_pred_m = 10.2;
};

struct P5_4NearRiskZoneFixtureConfig {
  bool enabled = false;
  std::string name = "near_risk_zone_v1";
  double x_min_m = -11.7;
  double x_max_m = -11.1;
  double y_min_m = -0.75;
  double y_max_m = 0.75;
  double z_min_m = 1.0;
  double z_max_m = 1.35;
  double tau_min_s = 0.6;
  double tau_max_s = 0.95;
  double hpl_pred_m = 10.2;
  double vpl_pred_m = 10.2;
};

struct P5_6FutureUnknownZoneFixtureConfig {
  bool enabled = false;
  std::string name = "future_unknown_zone_v1";
  double x_min_m = -1.0;
  double x_max_m = 12.5;
  double y_min_m = -15.0;
  double y_max_m = 15.0;
  double z_min_m = -3.0;
  double z_max_m = 3.0;
  double tau_min_s = 0.2;
  double tau_max_s = 2.0;
};

struct P5_7RejectedTrajectoryFixtureConfig {
  bool enabled = false;
  bool effective_enabled = false;
  std::string name = "rejected_trajectory_zone_v1";
  double x_min_m = -11.7;
  double x_max_m = -8.7;
  double y_min_m = -0.75;
  double y_max_m = 0.75;
  double z_min_m = 1.0;
  double z_max_m = 1.35;
  double tau_min_s = 0.6;
  double tau_max_s = 2.0;
  double hpl_pred_m = 10.2;
  double vpl_pred_m = 10.2;
};

struct RiskGridMapParams {
  std::string frame_id = "map";
  std::string alert_limit_policy_id = "legacy_unspecified";
  double alert_limit_h_m = 10.0;
  double alert_limit_v_m = 20.0;
  Eigen::Vector3d lattice_anchor_w = Eigen::Vector3d::Zero();
  bool use_fixed_origin = false;
  Eigen::Vector3d fixed_origin_w = Eigen::Vector3d::Zero();
  std::string geometry_id;
  double resolution_m = 0.75;
  double size_x_m = 30.0;
  double size_y_m = 30.0;
  double size_z_m = 6.0;
  std::vector<double> horizons_s = {0.0, 0.5, 1.0, 1.5, 2.0};
  double refresh_period_s = 0.5;
  double stale_timeout_s = 1.0;
  double unknown_cost = 10.0;
  double cost_max = 100.0;
  bool skip_occupied_voxels = true;
  bool require_observed_support = false;
  RiskProviderCostSource provider_cost_source =
      RiskProviderCostSource::LEGACY_SAFETY_PL;
  bool require_safety_ratio_below_one_for_cost = false;
  bool use_predictor_batch_query = true;
  P5_3HighRiskZoneFixtureConfig p5_3_fixture;
  P5_4NearRiskZoneFixtureConfig p5_4_fixture;
  P5_6FutureUnknownZoneFixtureConfig p5_6_fixture;
  P5_7RejectedTrajectoryFixtureConfig p5_7_fixture;
};

struct RiskGridHealth {
  bool ready = false;
  bool stale = true;
  double age_s = std::numeric_limits<double>::infinity();
  double valid_ratio = 0.0;
  double unknown_ratio = 1.0;
  uint64_t generation_id = 0;
  uint64_t provider_query_count = 0;
  uint64_t occupied_skip_count = 0;
  uint64_t provider_stale_count = 0;
  uint64_t provider_invalid_count = 0;
  uint64_t predictor_gnss_used_count = 0;
  uint64_t predictor_lidar_used_count = 0;
  uint64_t predictor_prior_used_count = 0;
  uint64_t predictor_stale_current_prior_count = 0;
  uint64_t predictor_regularized_count = 0;
  uint64_t predictor_conservative_max_count = 0;
  uint64_t predictor_lidar_map_point_count = 0;
  uint64_t predictor_lidar_fim_primitive_count = 0;
  uint64_t predictor_lidar_fim_valid_normal_count = 0;
  double occupancy_support_scan_ms = 0.0;
  double query_layout_ms = 0.0;
  double provider_batch_ms = 0.0;
  double voxel_materialization_ms = 0.0;
  double commit_ms = 0.0;
  double build_total_ms = 0.0;
  std::string predictor_lidar_fim_fallback_reason = "not_evaluated";
  std::string dominant_unknown_reason = "";
  uint64_t dominant_unknown_count = 0;
  std::string reason = "not_ready";
};

// Immutable provenance captured with one risk generation.  These values are
// evidence only: the generation itself remains the atomic planning contract.
struct RiskGridSourceIdentity {
  uint64_t occupancy_generation = 0;
  double occupancy_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t prior_generation = 0;
  double prior_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t gnss_generation = 0;
  double gnss_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t gnss_epoch_identity = 0;
  uint64_t lidar_generation = 0;
  double lidar_stamp_s = std::numeric_limits<double>::quiet_NaN();
  // Identity of the immutable local-map support envelope used by the
  // predictor. It changes when pose, validity, extent or model version
  // changes, even if the obstacle points themselves do not.
  std::string local_map_support_identity;
  std::string predictor_algorithm_identity = "legacy_predictor";
  std::string alert_limit_policy_id = "legacy_unspecified";
};

// Canonical identities shared by P0 health, P4 admission and runner evidence.
// Keep hashing in the owning risk-grid module so every consumer binds exactly
// the same safety and provenance fields.
std::string canonicalRiskGridConfigHash(const RiskGridMapParams& params);
std::string canonicalRiskGridSourceIdentityHash(
    const RiskGridSourceIdentity& identity);

struct RiskOccupancyDiagnostic;

struct RiskSourcePrediction {
  bool available = false;
  bool valid = false;
  bool stale = false;
  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double risk_ratio = std::numeric_limits<double>::quiet_NaN();
  double information_trace = std::numeric_limits<double>::quiet_NaN();
  std::string reason = "not_evaluated";
};

struct RiskVoxel {
  double c_pi = std::numeric_limits<double>::quiet_NaN();
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  double hal = 10.0;
  double val = 20.0;
  double risk_ratio = std::numeric_limits<double>::quiet_NaN();
  RiskSourcePrediction gnss;
  RiskSourcePrediction lidar;
  RiskSourcePrediction prior;
  RiskSourcePrediction fim_fused;
  double floor_increment_h = 0.0;
  double floor_increment_v = 0.0;
  std::string floor_source_h = "none";
  std::string floor_source_v = "none";
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool stale = true;
  bool unknown = true;
  uint32_t source_flags = 0u;
  std::string reason = "not_evaluated";
  GnssGeometryStatus gnss_geometry_status =
      GnssGeometryStatus::NOT_EVALUATED;
  LocalMapSupportAuthority gnss_support_authority =
      LocalMapSupportAuthority::STRICT_OBSERVATION;
  LocalMapSupportStatus gnss_support_status =
      LocalMapSupportStatus::FRAME_INVALID;
  std::vector<int> gnss_used_satellite_ids;
  uint64_t gnss_local_satellite_set_hash = 0;
  double gnss_weighted_geometry_condition =
      std::numeric_limits<double>::quiet_NaN();
  int gnss_worst_excluded_sat_h = -1;
  int gnss_worst_excluded_sat_v = -1;
  std::shared_ptr<const RiskOccupancyDiagnostic> occupancy;
};

struct RiskCostSample {
  bool valid = false;
  bool stale = true;
  double cost = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  uint64_t generation_id = 0;
  std::string reason = "not_evaluated";
};

inline constexpr char RISK_COST_DECOMPOSITION_SCHEMA_V2[] =
    "risk_cost_decomposition_v2";

// Versioned P4-v2 query result. Provider risk is kept separate from map
// occupancy and unavailable support so neither can masquerade as c_pi.
struct RiskCostDecomposition {
  std::string schema_version = RISK_COST_DECOMPOSITION_SCHEMA_V2;
  bool valid = false;
  double provider_c_pi = std::numeric_limits<double>::quiet_NaN();
  double provider_support_weight = 0.0;
  double occupied_support_weight = 0.0;
  double unknown_support_weight = 0.0;
  uint64_t generation_id = 0;
  std::string reason = "not_evaluated";
};

struct RiskOccupancyDiagnostic {
  bool available = false;
  bool observed = false;
  bool raw_occupied = false;
  bool inflated_occupied = false;
  RiskOccupancyState state = RiskOccupancyState::UNKNOWN;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double resolution_m = std::numeric_limits<double>::quiet_NaN();
  double inflation_m = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t occupancy_generation = 0;
  std::string source = "unavailable";
  // Independent from the ray-derived observed/state fields. A trusted
  // hit-only local-map model may complete prediction support without
  // fabricating OBSERVED_FREE occupancy evidence.
  LocalMapSupportQuery model_support;
};

struct RiskCostQueryCornerTrace {
  int temporal_layer = -1;
  int horizon_id = -1;
  double horizon_s = std::numeric_limits<double>::quiet_NaN();
  double temporal_weight = 0.0;
  int corner_id = -1;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double spatial_weight = 0.0;
  uint32_t source_flags = 0u;
  double c_pi = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool stale = true;
  bool unknown = true;
  bool gnss_supported = false;
  bool lidar_supported = false;
  bool fim_supported = false;
  std::string invalid_reason = "not_evaluated";
  RiskOccupancyDiagnostic occupancy;
};

struct RiskCostQueryTrace {
  Eigen::Vector3d query_point = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double query_tau_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t risk_generation_id = 0;
  std::string frame_id;
  bool success = false;
  std::string reason = "not_evaluated";
  std::vector<RiskCostQueryCornerTrace> corners;
};

enum class RiskGridInterpolationStatus {
  NOT_EVALUATED = 0,
  INTERPOLATED,
  DIRECT_RECHECK_REQUIRED,
  GEOMETRY_DEGENERATE,
  INVALID_SUPPORT,
};

const char* riskGridInterpolationStatusName(
    RiskGridInterpolationStatus status);

struct PredictedPLQueryCornerTrace {
  int temporal_layer = -1;
  int horizon_id = -1;
  double horizon_s = std::numeric_limits<double>::quiet_NaN();
  double temporal_weight = 0.0;
  int corner_id = -1;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double spatial_weight = 0.0;
  double combined_weight = 0.0;
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  uint32_t source_flags = 0u;
  bool valid = false;
  bool stale = true;
  bool unknown = true;
  GnssGeometryStatus gnss_geometry_status =
      GnssGeometryStatus::NOT_EVALUATED;
  LocalMapSupportAuthority gnss_support_authority =
      LocalMapSupportAuthority::STRICT_OBSERVATION;
  LocalMapSupportStatus gnss_support_status =
      LocalMapSupportStatus::FRAME_INVALID;
  std::vector<int> gnss_used_satellite_ids;
  uint64_t gnss_local_satellite_set_hash = 0;
  double gnss_weighted_geometry_condition =
      std::numeric_limits<double>::quiet_NaN();
  int gnss_worst_excluded_sat_h = -1;
  int gnss_worst_excluded_sat_v = -1;
  std::string invalid_reason = "not_evaluated";
};

struct PredictedPLQueryTrace {
  Eigen::Vector3d query_point = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double query_tau_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t risk_generation_id = 0;
  RiskGridInterpolationStatus interpolation_status =
      RiskGridInterpolationStatus::NOT_EVALUATED;
  std::string reason = "not_evaluated";
  std::vector<PredictedPLQueryCornerTrace> corners;
};

struct PredictedPLSample {
  bool available = false;
  bool valid = false;
  bool stale = true;
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double query_tau_s = std::numeric_limits<double>::quiet_NaN();
  bool fixture_match = false;
  double fixture_expected_hpl = std::numeric_limits<double>::quiet_NaN();
  double fixture_expected_vpl = std::numeric_limits<double>::quiet_NaN();
  std::string fixture_expected_reason;
  uint64_t generation_id = 0;
  RiskGridInterpolationStatus interpolation_status =
      RiskGridInterpolationStatus::NOT_EVALUATED;
  std::string reason = "not_evaluated";
};

struct RiskPredictionQuery {
  Eigen::Vector3d position_w = Eigen::Vector3d::Zero();
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double horizon_s = std::numeric_limits<double>::quiet_NaN();
};

struct RiskPredictionResult {
  bool available = false;
  bool valid = false;
  bool stale = true;
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  double hal = 10.0;
  double val = 20.0;
  RiskSourcePrediction gnss;
  RiskSourcePrediction lidar;
  RiskSourcePrediction prior;
  RiskSourcePrediction fim_fused;
  RiskSourcePrediction safety_fused;
  double floor_increment_h = 0.0;
  double floor_increment_v = 0.0;
  std::string floor_source_h = "none";
  std::string floor_source_v = "none";
  uint32_t source_flags = 0u;
  GnssGeometryStatus gnss_geometry_status =
      GnssGeometryStatus::NOT_EVALUATED;
  LocalMapSupportAuthority gnss_support_authority =
      LocalMapSupportAuthority::STRICT_OBSERVATION;
  LocalMapSupportStatus gnss_support_status =
      LocalMapSupportStatus::FRAME_INVALID;
  std::vector<int> gnss_used_satellite_ids;
  uint64_t gnss_local_satellite_set_hash = 0;
  double gnss_weighted_geometry_condition =
      std::numeric_limits<double>::quiet_NaN();
  int gnss_worst_excluded_sat_h = -1;
  int gnss_worst_excluded_sat_v = -1;
  std::string reason = "not_evaluated";
};

class RiskPredictionProvider {
 public:
  virtual ~RiskPredictionProvider() = default;
  virtual bool batchQuery(const std::vector<RiskPredictionQuery>& queries,
                          std::vector<RiskPredictionResult>* results) = 0;

  // Structured Cartesian query used by RiskGrid. Results are horizon-major:
  // result[h * positions.size() + p]. The default adapter preserves existing
  // providers; production providers override this to avoid rebuilding and
  // hashing the same spatial positions for every time layer.
  virtual bool batchQueryPositionHorizons(
      const std::vector<Eigen::Vector3d>& positions_w,
      const std::vector<double>& horizons_s,
      double evaluation_time_s,
      std::vector<RiskPredictionResult>* results) {
    std::vector<RiskPredictionQuery> queries;
    queries.reserve(positions_w.size() * horizons_s.size());
    for (const double horizon_s : horizons_s) {
      for (const auto& position_w : positions_w) {
        queries.push_back(RiskPredictionQuery{
            position_w, evaluation_time_s + horizon_s, horizon_s});
      }
    }
    return batchQuery(queries, results);
  }
};

class RiskGridSnapshot {
 public:
  struct Generation;

  RiskGridSnapshot() = default;

  RiskGridHealth health() const;
  double stamp_s() const;
  uint64_t generation_id() const;
  int horizonCount() const;
  int layerVoxelCount() const;

  const RiskGridMapParams& params() const;
  const RiskGridSourceIdentity& sourceIdentity() const;
  const Eigen::Vector3d& origin() const;
  const Eigen::Vector3i& voxelNum() const;

  bool posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i* id) const;
  Eigen::Vector3d indexToPos(const Eigen::Vector3i& id) const;
  int toAddress(const Eigen::Vector3i& id) const;
  bool isInMap(const Eigen::Vector3d& pos) const;
  bool isInMap(const Eigen::Vector3i& idx) const;

  bool queryCost(const Eigen::Vector3d& p_w,
                 double query_time_s,
                 RiskCostSample* out) const;
  bool queryCost(const Eigen::Vector3d& p_w,
                 double query_time_s,
                 RiskCostSample* out,
                 RiskCostQueryTrace* trace) const;
  bool queryCost(const Eigen::Vector3d& p_w,
                 double query_time_s,
                 RiskCostSample* out,
                 RiskCostQueryPolicy policy) const;
  bool queryCost(const Eigen::Vector3d& p_w,
                 double query_time_s,
                 RiskCostSample* out,
                 RiskCostQueryPolicy policy,
                 RiskCostQueryTrace* trace) const;

  bool queryRiskCostDecomposition(const Eigen::Vector3d& p_w,
                                  double query_time_s,
                                  RiskCostDecomposition* out) const;

  bool queryPredictedPL(const Eigen::Vector3d& p_w,
                        double query_time_s,
                        PredictedPLSample* out,
                        double p5_4_fixture_horizon_s =
                            std::numeric_limits<double>::quiet_NaN(),
                        bool p5_7_final_candidate = false,
                        PredictedPLQueryTrace* trace = nullptr) const;

  bool voxelAt(int horizon_id,
               const Eigen::Vector3i& id,
               RiskVoxel* out) const;

 private:
  explicit RiskGridSnapshot(std::shared_ptr<const Generation> generation);

  std::shared_ptr<const Generation> generation_;

  friend class RiskGridMap;
};

class RiskGridMap {
 public:
  using OccupancyPredicate = std::function<bool(const Eigen::Vector3d&)>;
  using OccupancyDiagnosticQuery =
      std::function<RiskOccupancyDiagnostic(const Eigen::Vector3d&)>;
  using SourceValidator = std::function<RiskGridSourceValidation()>;

  RiskGridMap();
  explicit RiskGridMap(RiskGridMapParams params);

  bool configure(RiskGridMapParams params, std::string* reason = nullptr);

  RiskGridHealth health() const;
  RiskGridHealth health(double now_s) const;
  std::shared_ptr<const RiskGridSnapshot> acquireSnapshot() const;

  const RiskGridMapParams& params() const { return params_; }
  const Eigen::Vector3i& voxelNum() const { return voxel_num_; }
  Eigen::Vector3d origin() const;

  bool posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i* id) const;
  Eigen::Vector3d indexToPos(const Eigen::Vector3i& id) const;
  int toAddress(const Eigen::Vector3i& id) const;
  bool isInMap(const Eigen::Vector3d& pos) const;
  bool isInMap(const Eigen::Vector3i& idx) const;

  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           std::string* reason = nullptr);
  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           const OccupancyPredicate& is_occupied,
                           std::string* reason = nullptr);
  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           const OccupancyDiagnosticQuery& occupancy_query,
                           std::string* reason = nullptr);
  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           const OccupancyDiagnosticQuery& occupancy_query,
                           const SourceValidator& source_validator,
                           std::string* reason = nullptr);
  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           const OccupancyDiagnosticQuery& occupancy_query,
                           const SourceValidator& source_validator,
                           const RiskGridSourceIdentity& source_identity,
                           std::string* reason = nullptr);

  void markRefreshFailure(double now_s, const std::string& reason);

 private:
  bool validateParams(const RiskGridMapParams& params,
                      std::string* reason) const;

  mutable std::mutex mutex_;
  std::mutex refresh_mutex_;
  RiskGridMapParams params_;
  Eigen::Vector3i voxel_num_ = Eigen::Vector3i::Zero();
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  uint64_t configuration_epoch_ = 0;
  uint64_t next_generation_id_ = 1;
  RiskGridHealth health_;
  std::shared_ptr<const RiskGridSnapshot::Generation> active_;
};

}  // namespace iap
