#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace iap {

enum class RiskGridSourceValidation {
  VALID = 0,
  OCCUPANCY_GENERATION_CHANGED,
  PRIOR_GENERATION_CHANGED,
};

constexpr uint32_t RISK_GRID_SOURCE_OCCUPIED_SKIP = 1u << 31;

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
  Eigen::Vector3d lattice_anchor_w = Eigen::Vector3d::Zero();
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
  std::string predictor_lidar_fim_fallback_reason = "not_evaluated";
  std::string dominant_unknown_reason = "";
  uint64_t dominant_unknown_count = 0;
  std::string reason = "not_ready";
};

struct RiskOccupancyDiagnostic;

struct RiskVoxel {
  double c_pi = std::numeric_limits<double>::quiet_NaN();
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool stale = true;
  bool unknown = true;
  uint32_t source_flags = 0u;
  std::string reason = "not_evaluated";
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

struct RiskOccupancyDiagnostic {
  bool available = false;
  bool raw_occupied = false;
  bool inflated_occupied = false;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double resolution_m = std::numeric_limits<double>::quiet_NaN();
  double inflation_m = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t occupancy_generation = 0;
  std::string source = "unavailable";
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
  uint32_t source_flags = 0u;
  std::string reason = "not_evaluated";
};

class RiskPredictionProvider {
 public:
  virtual ~RiskPredictionProvider() = default;
  virtual bool batchQuery(const std::vector<RiskPredictionQuery>& queries,
                          std::vector<RiskPredictionResult>* results) = 0;
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

  bool queryPredictedPL(const Eigen::Vector3d& p_w,
                        double query_time_s,
                        PredictedPLSample* out,
                        double p5_4_fixture_horizon_s =
                            std::numeric_limits<double>::quiet_NaN(),
                        bool p5_7_final_candidate = false) const;

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
