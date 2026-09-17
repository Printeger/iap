#ifndef _P0_RISK_GRID_RUNTIME_H_
#define _P0_RISK_GRID_RUNTIME_H_

#include <cstddef>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ego_planner/safety_rviz_publisher.h>
#include <ego_planner/p0_occupancy_epoch_adapter.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_ionosphere_parameter.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/planner/integrity_snapshot.hpp>
#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>
#include <iap/predictor/gnss_satellite_admission.hpp>
#include <iap/predictor/rolling_spatial_advisory_window.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace ego_planner {

class P0RiskGridWorkerPool;

enum class P0ExecutionSnapshotAttemptStatus : std::uint8_t {
  PUBLISHED = 0,
  DEDUPLICATED,
  CAPTURE_UNAVAILABLE,
  CAPTURE_ADAPTER_INVALID,
  OCCUPANCY_INVALID,
  OCCUPANCY_STALE,
  INTEGRITY_UNAVAILABLE,
  INTEGRITY_UNSAFE,
  MAP_POINTS_MISSING,
  FINAL_FRESHNESS_FAILED,
  SUPERSEDED,
  WORKER_EXCEPTION,
  WORKER_STOPPED,
};

const char* p0ExecutionSnapshotAttemptStatusName(
    P0ExecutionSnapshotAttemptStatus status);

struct P0ExecutionSnapshotAttemptEvidence {
  std::uint64_t attempt_id = 0;
  P0ExecutionSnapshotAttemptStatus status =
      P0ExecutionSnapshotAttemptStatus::CAPTURE_UNAVAILABLE;
  std::string reason = "not_attempted";
  std::uint64_t requested_occupancy_generation = 0;
  double requested_occupancy_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  std::uint64_t captured_occupancy_generation = 0;
  double captured_occupancy_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  double support_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t support_generation = 0;
  double lidar_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t lidar_generation = 0;
  double lidar_receive_steady_s =
      std::numeric_limits<double>::quiet_NaN();
  double gnss_epoch_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t gnss_epoch_generation = 0;
  std::uint64_t gnss_epoch_identity = 0;
  double integrity_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t integrity_generation = 0;
  std::uint64_t published_execution_snapshot_id = 0;
  double request_ros_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double start_ros_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double occupancy_capture_finish_ros_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  double predictor_ready_ros_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  double snapshot_publish_ros_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  double finish_ros_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double request_steady_s = std::numeric_limits<double>::quiet_NaN();
  double start_steady_s = std::numeric_limits<double>::quiet_NaN();
  double occupancy_capture_finish_steady_s =
      std::numeric_limits<double>::quiet_NaN();
  double predictor_ready_steady_s =
      std::numeric_limits<double>::quiet_NaN();
  double finish_steady_s = std::numeric_limits<double>::quiet_NaN();
  double queue_delay_ms = std::numeric_limits<double>::quiet_NaN();
  double build_duration_ms = std::numeric_limits<double>::quiet_NaN();
  double publish_age_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t pending_overwrite_count_at_request = 0;
  std::uint64_t pending_overwrite_count = 0;
  std::uint64_t risk_grid_yield_count = 0;
  double risk_grid_yield_duration_ms = 0.0;
  std::string frame_contract_id;
  std::string geometry_id;
  std::string support_identity;
};

struct P0ExecutionRiskSnapshot {
  struct LocalObstacleSourceBound {
    std::int64_t frame_id = -1;
    double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
    bool certified = false;
    Eigen::Vector3d lidar_pl_enu_m = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    bool icp_degenerate = true;
    double icp_rmse_m = std::numeric_limits<double>::quiet_NaN();
    double icp_condition = std::numeric_limits<double>::quiet_NaN();
    double icp_gamma = std::numeric_limits<double>::quiet_NaN();
    std::shared_ptr<const std::vector<Eigen::Vector3d>> occupied_centers;
    std::string identity;
  };

  std::uint64_t execution_snapshot_id = 0;
  double evaluation_time_s = std::numeric_limits<double>::quiet_NaN();
  double publish_time_s = std::numeric_limits<double>::quiet_NaN();
  std::shared_ptr<const P0OccupancyEpoch> occupancy;
  iap::IntegritySnapshot integrity_anchor;
  iap::RiskGridSourceIdentity source_identity;
  iap::RiskGridMapParams risk_policy;
  double gnss_max_age_s = -1.0;
  std::uint64_t lidar_generation = 0;
  double lidar_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::string frame_contract_id;
  std::string geometry_id;
  std::string predictor_algorithm_identity;
  // Per-source-frame local-relative bounds.  They are paired by source stamp
  // with the immutable registered-map contribution; a missing pair remains
  // uncertified and must fail closed in controlled-degraded execution.
  std::vector<LocalObstacleSourceBound> local_obstacle_source_bounds;
  std::function<iap::ForwardRiskBatchResult(
      const iap::ForwardRiskBatchRequest&)> forward_risk_batch;
  std::function<iap::ForwardRiskBatchResult(
      const iap::ForwardRiskBatchRequest&)> diagnostic_forward_risk_batch;

  bool freshAt(double now_s, double gnss_max_age_s) const {
    if (execution_snapshot_id == 0u || !std::isfinite(now_s) ||
        !std::isfinite(evaluation_time_s) || now_s < evaluation_time_s ||
        !occupancy || !forward_risk_batch || risk_policy.frame_id != "map" ||
        geometry_id.empty() || !integrity_anchor.current.valid ||
        !std::isfinite(integrity_anchor.current.stamp)) {
      return false;
    }
    const double timeout = risk_policy.stale_timeout_s;
    const auto fresh_stamp = [now_s, timeout](const double stamp) {
      const double age = now_s - stamp;
      return std::isfinite(stamp) && age >= -1.0e-6 &&
          (timeout < 0.0 || age <= timeout);
    };
    if (!fresh_stamp(occupancy->cloud_stamp_s) ||
        !fresh_stamp(integrity_anchor.current.stamp)) {
      return false;
    }
    if (occupancy->trusted_local_map_support &&
        !occupancy->trusted_local_map_support->freshAt(now_s)) {
      return false;
    }
    if (integrity_anchor.has_epoch) {
      const double gnss_age = now_s - integrity_anchor.gnss_epoch.stamp;
      if (!std::isfinite(integrity_anchor.gnss_epoch.stamp) ||
          gnss_age < -1.0e-6 ||
          (gnss_max_age_s >= 0.0 && gnss_age > gnss_max_age_s)) {
        return false;
      }
    }
    return lidar_generation == 0u || fresh_stamp(lidar_stamp_s);
  }
  bool freshAt(double now_s) const { return freshAt(now_s, gnss_max_age_s); }
};

struct P0PlanningSnapshot {
  std::shared_ptr<const iap::RiskGridSnapshot> risk;
  std::shared_ptr<const P0OccupancyEpoch> occupancy;
  iap::IntegritySnapshot integrity_anchor;
  double gnss_support_ray_length_m =
      std::numeric_limits<double>::quiet_NaN();
  bool gnss_hard_occlusion = false;
  std::function<iap::ForwardRiskBatchResult(
      const iap::ForwardRiskBatchRequest&)> forward_risk_batch;
  // Diagnostic-only map/epoch cross-product query. Unlike the production
  // callback above, this honors input.snapshot. It must never authorize motion.
  std::function<iap::ForwardRiskBatchResult(
      const iap::ForwardRiskBatchRequest&)> diagnostic_forward_risk_batch;
  // Exact execution authority used to construct this search grid. A newer
  // execution snapshot may be published while this grid remains active.
  std::shared_ptr<const P0ExecutionRiskSnapshot> execution;
};

class P0RiskGridRuntime {
 public:
  struct P0_6FixtureConfig {
    bool enabled = false;
    std::string name;
    double x_min_m = -1.5;
    double x_max_m = 1.5;
    double y_min_m = -0.75;
    double y_max_m = 0.75;
    double z_min_m = 1.0;
    double z_max_m = 2.0;
    double raw_hpl_m = 1.0;
    double raw_vpl_m = 1.2;
    double raw_c_pi = 1.2;
    double low_raw_cost_threshold = 2.0;
  };

  struct Config {
    bool enable_risk_grid = false;
    bool debug_metrics_enable = false;
    bool online_mapping_mode = false;
    bool fit_grid_to_map_cloud = false;
    double refresh_start_delay_s = 0.0;
    iap::RiskGridMapParams grid;
    std::string odom_topic = "/drone_0_visual_slam/odom";
    std::string integrity_topic = "/iap/integrity";
    std::string range_meas_topic = "/ublox_driver/range_meas";
    std::string ephem_topic = "/ublox_driver/ephem";
    std::string glo_ephem_topic = "/ublox_driver/glo_ephem";
    std::string receiver_lla_topic = "/ublox_driver/receiver_lla";
    std::string iono_topic = "/ublox_driver/iono_params";
    std::string map_topic = "/map_generator/global_cloud";
    // This is an evidence topic, not a debug topic.  Keep it absolute so a
    // namespaced planner cannot silently move it away from the bag contract.
    std::string health_topic = "/planning/risk_grid_health";
    double gnss_epoch_max_age_s = 2.0;
    double gnss_pr_noise_base_m = 5.0;
    double gnss_dop_noise_base_mps = 0.5;
    iap::PredictorSourceMode predictor_source_mode =
        iap::PredictorSourceMode::Fusion;
    iap::PredictorGnssEpochPolicy predictor_gnss_epoch_policy =
        iap::PredictorGnssEpochPolicy::Auto;
    double predictor_gnss_measured_epoch_support_radius_m = 0.0;
    double predictor_gnss_measured_epoch_integrity_max_delta_s = 0.25;
    double predictor_gnss_clearance_transition_m = 0.0;
    bool predictor_use_current_integrity_prior = true;
    bool predictor_conservative_max_with_gnss = false;
    double predictor_hal_m = 10.0;
    double predictor_val_m = 20.0;
    bool predictor_lidar_legacy_observability = true;
    double predictor_lidar_fim_radius_m =
        iap::LidarObservabilityFim::Params{}.fim_radius_m;
    double predictor_sigma_grow_m_sqrt_s =
        std::numeric_limits<double>::quiet_NaN();
    double predictor_gnss_spatial_ttl_s =
        std::numeric_limits<double>::quiet_NaN();
    double predictor_legacy_current_spatial_ttl_s =
        std::numeric_limits<double>::quiet_NaN();
    double predictor_full_refresh_watchdog_s =
        std::numeric_limits<double>::quiet_NaN();
    int predictor_requested_worker_count = 1;
    int predictor_effective_worker_count = 1;
    double execution_snapshot_period_s = 0.05;
    double execution_batch_budget_ms = 150.0;
    double risk_grid_build_budget_ms = 500.0;
    int predictor_gnss_admission_epochs = 3;
    P0_6FixtureConfig p0_6_fixture;
  };

  static Config declareAndReadConfig(const rclcpp::Node::SharedPtr& node);
  static bool geometryMatchesRiskOverlay(
      const iap::PlanningLatticeGeometry& geometry,
      const iap::RiskGridMapParams& risk_grid,
      int required_ego_voxels_per_axis = 5);
  static std::unique_ptr<P0RiskGridRuntime> createIfEnabled(
      const rclcpp::Node::SharedPtr& node);

  P0RiskGridRuntime(
      rclcpp::Node::SharedPtr node,
      Config config,
      std::unique_ptr<iap::RiskPredictionProvider> provider = nullptr);
  ~P0RiskGridRuntime();
  void shutdown();

  bool enabled() const { return config_.enable_risk_grid; }
  double gnssClearanceTransitionM() const {
    return config_.predictor_gnss_clearance_transition_m;
  }
  const iap::RiskGridMap& riskGrid() const { return risk_grid_; }
  iap::RiskGridMap& riskGrid() { return risk_grid_; }
  std::shared_ptr<const iap::RiskGridSnapshot> acquireSnapshot() const;
  std::shared_ptr<const P0PlanningSnapshot> acquirePlanningSnapshot() const;
  std::shared_ptr<const P0ExecutionRiskSnapshot>
  acquireExecutionRiskSnapshot() const;
  // Select a causal execution authority for the caller's ROS/simulation
  // evaluation instant. The latest snapshot may already contain the next
  // sensor timestamp when mutually-exclusive executor callbacks are observed
  // out of timestamp order; in that case the execution snapshot bound to the
  // latest completed RiskGrid is the bounded fallback.
  std::shared_ptr<const P0ExecutionRiskSnapshot>
  acquireExecutionRiskSnapshotForEvaluation(double evaluation_time_s) const;
  static std::shared_ptr<const P0ExecutionRiskSnapshot>
  selectExecutionRiskSnapshotForEvaluation(
      const std::shared_ptr<const P0ExecutionRiskSnapshot>& latest,
      const std::shared_ptr<const P0ExecutionRiskSnapshot>& grid_bound,
      double evaluation_time_s);
  static std::shared_ptr<const P0ExecutionRiskSnapshot>
  selectExecutionRiskSnapshotHistoryForEvaluation(
      const std::deque<std::shared_ptr<const P0ExecutionRiskSnapshot>>&
          completed,
      const std::shared_ptr<const P0ExecutionRiskSnapshot>& grid_bound,
      double evaluation_time_s);
  bool gnssEpochFreshAt(double epoch_stamp_s,
                        double evaluation_time_s) const;
  bool executionSnapshotFreshAt(
      const std::shared_ptr<const P0ExecutionRiskSnapshot>& snapshot,
      double evaluation_time_s) const;
  // Return the newest certified monitor sample at or before this evaluation
  // instant, only when it is valid and fresh.  ROS callbacks carrying the
  // next sensor timestamp can run before a planner callback for the preceding
  // timestamp; selecting by timestamp prevents that benign ordering from
  // looking like a missing input. Unsafe PL values are intentionally returned:
  // callers must distinguish an unsafe current monitor from stale predictive
  // inputs and revoke immediately.
  bool currentIntegrityForExecution(
      double evaluation_time_s, iap::CurrentIntegrityState* current) const;
  iap::RiskGridHealth health() const;
  bool refreshOnceForTest();
  void setOccupancyPredicate(iap::RiskGridMap::OccupancyPredicate predicate);
  void setOccupancyDiagnosticQuery(
      iap::RiskGridMap::OccupancyDiagnosticQuery query);
  void setOccupancyDiagnosticQueryFactory(
      std::function<iap::RiskGridMap::OccupancyDiagnosticQuery()> factory);
  void setOccupancyEpochFactory(
      std::function<P0OccupancyEpochCapture()> factory);
  void setExecutionOccupancyEpochFactory(
      std::function<P0OccupancyEpochCapture()> factory);
  void setOccupancyGenerationProvider(std::function<uint64_t()> provider);
  void notifyOccupancyCommitted(uint64_t generation, double source_stamp_s);
  P0ExecutionSnapshotAttemptEvidence lastExecutionSnapshotAttempt() const;

 private:
  friend class P0RiskGridRuntimeStampTest;

  void createRosInterfaces();
  void executionSnapshotTimerCallback();
  struct ExecutionSnapshotRequest {
    uint64_t generation = 0;
    double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double request_ros_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double request_steady_s = std::numeric_limits<double>::quiet_NaN();
    uint64_t overwritten_count = 0;
  };
  void executionSnapshotWorkerLoop();
  void buildAndPublishExecutionSnapshot(const ExecutionSnapshotRequest& request);
  void recordExecutionSnapshotAttempt(
      P0ExecutionSnapshotAttemptEvidence evidence);
  bool yieldRiskGridToExecutionSnapshot(
      std::chrono::steady_clock::time_point deadline);
  void refreshTimerCallback();
  void healthTimerCallback();
  void publishHealth(const iap::RiskGridHealth& health, double now_s);
  void recordInputCallback();

  struct InputReadiness {
    bool odom_seen = false;
    bool odom_valid = false;
    bool odom_fresh = false;
    double odom_stamp_s = std::numeric_limits<double>::quiet_NaN();
    bool current_integrity_seen = false;
    bool current_integrity_valid = false;
    bool current_integrity_fresh = false;
    double current_integrity_stamp_s = std::numeric_limits<double>::quiet_NaN();
    bool gnss_epoch_seen = false;
    bool gnss_epoch_valid = false;
    bool gnss_epoch_fresh = false;
    double gnss_epoch_stamp_s = std::numeric_limits<double>::quiet_NaN();
    uint64_t gnss_epoch_satellite_count = 0;
    bool origin_seen = false;
    bool origin_valid = false;
    bool origin_fresh = false;
    double origin_stamp_s = std::numeric_limits<double>::quiet_NaN();
    bool map_seen = false;
    bool map_valid = false;
    bool map_fresh = false;
    double map_stamp_s = std::numeric_limits<double>::quiet_NaN();
    uint64_t map_point_count = 0;
  };

  struct HealthPublicationState {
    double refresh_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_start_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_end_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double health_callback_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double publish_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_scheduled_steady_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_start_steady_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_end_steady_s = std::numeric_limits<double>::quiet_NaN();
    double health_callback_steady_s = std::numeric_limits<double>::quiet_NaN();
    double publish_steady_s = std::numeric_limits<double>::quiet_NaN();
    double last_grid_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_elapsed_ms = std::numeric_limits<double>::quiet_NaN();
    double refresh_queue_delay_ms = std::numeric_limits<double>::quiet_NaN();
    double provider_batch_duration_ms = std::numeric_limits<double>::quiet_NaN();
    double generation_interval_ms = std::numeric_limits<double>::quiet_NaN();
    uint64_t execution_snapshot_id = 0;
    double execution_snapshot_evaluation_stamp_s =
        std::numeric_limits<double>::quiet_NaN();
    double execution_snapshot_publish_stamp_s =
        std::numeric_limits<double>::quiet_NaN();
    double execution_snapshot_publish_latency_ms =
        std::numeric_limits<double>::quiet_NaN();
    double input_callback_age_s = std::numeric_limits<double>::quiet_NaN();
    double process_cpu_delta_ms = std::numeric_limits<double>::quiet_NaN();
    double health_callback_duration_ms = std::numeric_limits<double>::quiet_NaN();
    double health_callback_queue_delay_ms = std::numeric_limits<double>::quiet_NaN();
    double health_state_mutex_wait_ms = std::numeric_limits<double>::quiet_NaN();
    double health_state_mutex_hold_ms = std::numeric_limits<double>::quiet_NaN();
    std::size_t refresh_query_count = 0;
    std::size_t predictor_unique_positions = 0;
    std::size_t predictor_lidar_evaluations = 0;
    std::size_t predictor_lidar_cache_hits = 0;
    std::size_t predictor_spatial_advisory_recompute_count = 0;
    std::size_t predictor_spatial_advisory_reuse_count = 0;
    std::size_t predictor_gnss_advisory_invocation_count = 0;
    std::size_t predictor_lidar_advisory_invocation_count = 0;
    std::size_t predictor_horizon_fusion_count = 0;
    std::size_t predictor_spatial_retained_position_count = 0;
    std::size_t predictor_spatial_entered_position_count = 0;
    std::size_t predictor_spatial_evicted_position_count = 0;
    std::size_t predictor_spatial_full_invalidation_count = 0;
    std::size_t predictor_spatial_exact_retained_position_count = 0;
    std::size_t predictor_spatial_ttl_retained_position_count = 0;
    std::size_t predictor_spatial_gnss_ttl_expired_position_count = 0;
    std::size_t predictor_spatial_legacy_current_ttl_expired_position_count = 0;
    std::size_t predictor_spatial_watchdog_forced_full_rebuild_count = 0;
    std::size_t predictor_spatial_invalid_source_provenance_count = 0;
    std::string predictor_spatial_invalidation_reason = "none";
    uint64_t input_callback_count = 0;
    uint64_t health_callback_count = 0;
    bool snapshot_available = false;
  };
  enum class RefreshEvidenceState {
    PRE_REFRESH,
    IN_PROGRESS,
    COMPLETED_SUCCESS,
    COMPLETED_FAILURE,
  };
  struct RefreshEvidenceRecord {
    uint64_t refresh_attempt_id = 1;
    RefreshEvidenceState state = RefreshEvidenceState::PRE_REFRESH;
    uint64_t result_generation_id = 0;
    uint64_t previous_successful_generation_id = 0;
    HealthPublicationState publication;
    InputReadiness readiness;
    iap::RiskGridHealth health;
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot;
    std::shared_ptr<const P0PlanningSnapshot> planning_snapshot;
    std::string snapshot_failure_reason = "none";
  };
  HealthPublicationState healthPublicationStateSnapshot() const;
  RefreshEvidenceRecord refreshEvidenceRecordSnapshot() const;
  void completeRefreshEvidence(const iap::RiskGridHealth& health,
                               double now_s, bool succeeded);
  static const char* refreshEvidenceStateName(RefreshEvidenceState state);
  InputReadiness inputReadiness(double now_s) const;
  InputReadiness inputReadinessLocked(double now_s) const;
  std::string snapshotFailureReason(double now_s) const;

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void integrityCallback(const iap::msg::IntegrityReport::ConstSharedPtr msg);
  void rangeCallback(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg);
  void pruneGnssEpochHistoryLocked(double newest_stamp_s);
  void ephemCallback(const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg);
  void gloEphemCallback(
      const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg);
  void receiverLlaCallback(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void ionoCallback(
      const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  bool initializeGridDimensionsFromMapCloud(
      const std::vector<Eigen::Vector3d>& points,
      const std::string& frame_id,
      std::string* reason = nullptr);

  iap::CurrentIntegrityState currentFromMsg(
      const iap::msg::IntegrityReport& msg) const;
  struct PredictorSourceCapture {
    uint64_t current_generation = 0;
    double current_stamp = std::numeric_limits<double>::quiet_NaN();
    uint64_t gnss_epoch_generation = 0;
    double gnss_epoch_stamp = std::numeric_limits<double>::quiet_NaN();
  };
  const iap::GnssEpoch* activeGnssEpoch(double query_stamp) const;
  Eigen::Matrix3d currentPriorInformation(
      const iap::CurrentIntegrityState& current) const;
  bool buildSnapshot(
      double now_s, iap::IntegritySnapshot* snapshot,
      PredictorSourceCapture* source_capture = nullptr,
      InputReadiness* readiness_capture = nullptr) const;
  iap::RiskGridHealth addLidarPredictorInputHealth(
      iap::RiskGridHealth health) const;
  iap::RiskGridHealth addLidarPredictorInputHealthLocked(
      iap::RiskGridHealth health) const;
  bool p0_6_fixture_occupied(const Eigen::Vector3d& pos) const;
  iap::RiskGridMap::OccupancyPredicate combinedOccupancyPredicate() const;
  iap::RiskGridMap::OccupancyDiagnosticQuery
  combinedOccupancyDiagnosticQuery(
      iap::RiskGridMap::OccupancyDiagnosticQuery base_query = {}) const;
  double currentMessageStamp() const;
  double currentRefreshStamp() const;
  double diagnosticRosNowSeconds() const;
  double liveNowSeconds() const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  iap::RiskGridMap risk_grid_;
  std::unique_ptr<iap::RiskPredictionProvider> provider_;
  std::unique_ptr<P0RiskGridWorkerPool> risk_grid_worker_pool_;
  // Runtime callbacks start before production map adapters are attached.
  // Keep each replaceable source callable behind one synchronization seam so
  // the timer, dense-grid refresh and execution worker never race a setter.
  mutable std::mutex source_factory_mutex_;
  iap::RiskGridMap::OccupancyPredicate occupancy_predicate_;
  iap::RiskGridMap::OccupancyDiagnosticQuery occupancy_diagnostic_query_;
  std::function<iap::RiskGridMap::OccupancyDiagnosticQuery()>
      occupancy_diagnostic_query_factory_;
  std::function<P0OccupancyEpochCapture()> occupancy_epoch_factory_;
  std::function<P0OccupancyEpochCapture()>
      execution_occupancy_epoch_factory_;
  std::function<uint64_t()> occupancy_generation_provider_;
  mutable std::mutex planning_snapshot_mutex_;
  std::shared_ptr<const P0PlanningSnapshot> planning_snapshot_;
  std::shared_ptr<const P0ExecutionRiskSnapshot> execution_snapshot_;
  // Published results are not pending work. Retain only enough completed
  // snapshots to select the newest causal tuple when ROS callbacks for the
  // next sensor stamp overtake an execution query for the preceding stamp.
  std::deque<std::shared_ptr<const P0ExecutionRiskSnapshot>>
      execution_snapshot_history_;
  std::atomic<std::uint64_t> next_execution_snapshot_id_{1};
  std::atomic<std::uint64_t> next_execution_snapshot_attempt_id_{1};
  mutable std::mutex execution_snapshot_worker_mutex_;
  std::condition_variable execution_snapshot_worker_cv_;
  std::optional<ExecutionSnapshotRequest> pending_execution_snapshot_request_;
  std::thread execution_snapshot_worker_;
  bool execution_snapshot_worker_stop_ = false;
  bool execution_snapshot_worker_in_flight_ = false;
  // Reserve two low-priority RiskGrid workers for uninterrupted forward
  // progress while the remaining workers make bounded scheduling yields to
  // the execution snapshot. Dynamic work stealing prevents yielded workers
  // from retaining a slow fixed partition.
  std::size_t risk_grid_yield_waiter_count_ = 0;
  uint64_t last_requested_occupancy_generation_ = 0;
  uint64_t execution_snapshot_pending_overwrite_count_ = 0;
  P0ExecutionSnapshotAttemptEvidence last_execution_snapshot_attempt_;
  P0ExecutionSnapshotAttemptEvidence last_execution_snapshot_failure_;
  uint64_t execution_snapshot_attempt_count_ = 0;
  uint64_t execution_snapshot_publish_count_ = 0;
  uint64_t execution_snapshot_failure_count_ = 0;
  uint64_t risk_grid_yield_count_ = 0;
  double risk_grid_yield_duration_ms_ = 0.0;
  // refreshOnceForTest() intentionally executes one frozen transaction.  It
  // must not invoke the independent live timer (which would capture the
  // occupancy factory twice), while still exposing that transaction's direct
  // execution snapshot to deterministic tests.
  std::atomic<bool> synchronous_test_refresh_{false};
  iap::IntegritySnapshotBuilder snapshot_builder_;
  iap::RollingSpatialAdvisoryWindow rolling_spatial_window_;
  std::shared_ptr<const iap::LocalOccupancyGrid>
      rolling_occupancy_owner_;
  std::shared_ptr<const P0RawOccupancyIdentity>
      rolling_raw_occupancy_identity_;
  P0OccupancyEpoch::SourceOwner rolling_occupancy_source_owner_;
  iap::PlanningLatticeGeometry rolling_occupancy_geometry_;
  std::string rolling_occupancy_frame_contract_id_;
  uint64_t rolling_occupancy_generation_ = 0;
  double rolling_occupancy_stamp_ =
      std::numeric_limits<double>::quiet_NaN();
  uint64_t rolling_occupancy_content_identity_ = 0;
  std::string rolling_support_identity_;

  // Inputs, heavy refresh, and health publication deliberately use distinct
  // execution paths.  Refresh may take longer than a sensor period, but must
  // never prevent the next input state from being recorded.
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  // GNSS epoch construction and dense-map conversion can each be
  // substantially more expensive than recording odometry/current integrity.
  // Keep all three workloads independent so map refresh cannot stale GNSS.
  rclcpp::CallbackGroup::SharedPtr predictor_input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr map_input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr execution_snapshot_callback_group_;
  rclcpp::CallbackGroup::SharedPtr refresh_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;
  rclcpp::TimerBase::SharedPtr refresh_start_timer_;
  rclcpp::TimerBase::SharedPtr execution_snapshot_timer_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::shared_ptr<SafetyRvizPublisher> safety_viz_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      execution_snapshot_attempt_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssMeasMsg>::SharedPtr range_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr
      glo_ephem_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
      receiver_lla_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr
      iono_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  double latest_odom_stamp_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d latest_odom_p_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond latest_odom_q_ = Eigen::Quaterniond::Identity();
  bool latest_odom_pose_valid_ = false;
  bool odom_seen_ = false;
  double latest_origin_stamp_ = std::numeric_limits<double>::quiet_NaN();
  double latest_map_stamp_ = std::numeric_limits<double>::quiet_NaN();
  bool map_seen_ = false;
  bool scene_grid_dimensions_ready_ = false;
  bool online_geometry_bound_ = false;
  iap::CurrentIntegrityState latest_current_;
  bool latest_current_valid_ = false;
  bool current_integrity_seen_ = false;
  uint64_t latest_current_generation_ = 0;
  std::deque<std::pair<uint64_t, iap::CurrentIntegrityState>>
      current_integrity_history_;
  double latest_gnss_epoch_stamp_ = std::numeric_limits<double>::quiet_NaN();
  uint64_t latest_gnss_epoch_satellite_count_ = 0;
  uint64_t latest_gnss_epoch_generation_ = 0;
  double last_refresh_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_grid_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_elapsed_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_queue_delay_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_provider_batch_duration_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_generation_interval_ms_ = std::numeric_limits<double>::quiet_NaN();
  uint64_t last_execution_snapshot_id_ = 0;
  double last_execution_snapshot_evaluation_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_execution_snapshot_publish_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_execution_snapshot_publish_latency_ms_ =
      std::numeric_limits<double>::quiet_NaN();
  bool last_snapshot_available_ = false;
  bool last_refresh_succeeded_ = false;
  std::string last_snapshot_failure_reason_ = "none";
  std::size_t last_refresh_query_count_ = 0;
  std::size_t last_predictor_unique_positions_ = 0;
  std::size_t last_predictor_lidar_evaluations_ = 0;
  std::size_t last_predictor_lidar_cache_hits_ = 0;
  std::size_t last_predictor_spatial_advisory_recompute_count_ = 0;
  std::size_t last_predictor_spatial_advisory_reuse_count_ = 0;
  std::size_t last_predictor_gnss_advisory_invocation_count_ = 0;
  std::size_t last_predictor_lidar_advisory_invocation_count_ = 0;
  std::size_t last_predictor_horizon_fusion_count_ = 0;
  iap::RollingSpatialRefreshDiagnostics last_rolling_spatial_diagnostics_;
  double last_refresh_start_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_end_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_publish_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_start_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_scheduled_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double next_refresh_scheduled_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_end_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_generation_end_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_input_callback_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_publish_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_duration_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_queue_delay_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_state_mutex_wait_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_state_mutex_hold_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_process_cpu_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_process_cpu_delta_ms_ = std::numeric_limits<double>::quiet_NaN();
  uint64_t input_callback_count_ = 0;
  uint64_t health_callback_count_ = 0;
  uint64_t next_refresh_attempt_id_ = 1;
  uint64_t last_successful_generation_id_ = 0;
  InputReadiness refresh_input_readiness_;
  RefreshEvidenceRecord refresh_evidence_;
  bool origin_set_ = false;
  bool origin_seen_ = false;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
  std::vector<double> iono_params_;
  std::optional<iap::GnssEpoch> latest_epoch_;
  iap::GnssSatelliteAdmissionHysteresis gnss_satellite_admission_{3};
  std::deque<std::pair<uint64_t, iap::GnssEpoch>> gnss_epoch_history_;
  bool gnss_epoch_seen_ = false;

  mutable std::mutex lidar_predictor_input_mutex_;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      latest_lidar_map_points_;
  std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
      latest_lidar_fim_primitives_;
  iap::LidarFimPrimitiveGenerationDiagnostics
      latest_lidar_fim_diagnostics_;
  std::size_t latest_lidar_map_point_count_ = 0;
  std::size_t latest_lidar_fim_primitive_count_ = 0;
  std::size_t latest_lidar_fim_valid_normal_count_ = 0;
  std::string latest_lidar_fim_fallback_reason_ = "not_evaluated";
  uint64_t latest_lidar_generation_ = 0;
  double latest_lidar_stamp_ = std::numeric_limits<double>::quiet_NaN();
  mutable std::mutex health_state_mutex_;
};

}  // namespace ego_planner

#endif
