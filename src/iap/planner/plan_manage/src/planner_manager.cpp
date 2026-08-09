// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <ego_planner/p1_candidate_selection.h>
#include <ego_planner/p1_soft_fallback_policy.h>
#include <ego_planner/p0_risk_grid_runtime.h>
#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/safety_rviz_publisher.h>
#include <iap/planner/risk_grid_map.hpp>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <thread>
#include <utility>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{
  namespace
  {
    std::vector<Eigen::Vector3d> matrixColumnsToPoints(const Eigen::MatrixXd &points)
    {
      std::vector<Eigen::Vector3d> out;
      if (points.rows() != 3)
      {
        return out;
      }
      out.reserve(static_cast<std::size_t>(points.cols()));
      for (int i = 0; i < points.cols(); ++i)
      {
        out.push_back(points.col(i));
      }
      return out;
    }

    SafetyVizP1Metrics toSafetyVizP1Metrics(
        const BsplineOptimizer::P1IntegrityMetrics &metrics)
    {
      SafetyVizP1Metrics out;
      out.sample_count = metrics.sample_count;
      out.hit_count = metrics.hit_count;
      out.miss_count = metrics.miss_count;
      out.stale_count = metrics.stale_count;
      out.f_integrity = metrics.f_integrity;
      out.weighted_f_integrity = metrics.weighted_f_integrity;
      out.grad_ratio = metrics.grad_ratio;
      out.snapshot_generation_id = metrics.snapshot_generation_id;
      out.applied_to_objective = metrics.applied_to_objective;
      out.fallback_reason = metrics.fallback_reason;
      return out;
    }

    P1CandidateEvidence toP1CandidateEvidence(
        const BsplineOptimizer::P1OptimizationTrace &trace)
    {
      P1CandidateEvidence evidence;
      evidence.planning_attempt_id = trace.planning_attempt_id;
      evidence.candidate_id = trace.candidate_id;
      evidence.snapshot_generation_id = trace.snapshot_generation_id;
      evidence.pre_base_objective = trace.pre_base_objective;
      evidence.post_base_objective = trace.post_base_objective;
      evidence.pre_raw_p1_objective = trace.pre_raw_p1_cost;
      evidence.post_raw_p1_objective = trace.post_raw_p1_cost;
      evidence.pre_weighted_p1_objective = trace.pre_weighted_p1_cost;
      evidence.post_weighted_p1_objective = trace.post_weighted_p1_cost;
      evidence.pre_total_objective = trace.pre_total_objective;
      evidence.post_total_objective = trace.post_total_objective;
      evidence.pre_mean_c_pi = trace.pre_mean_c_pi;
      evidence.post_mean_c_pi = trace.post_mean_c_pi;
      evidence.pre_max_c_pi = trace.pre_max_c_pi;
      evidence.post_max_c_pi = trace.post_max_c_pi;
      evidence.gradient_dot_displacement = trace.grad_integrity_dot_displacement;
      evidence.optimization_success = trace.optimization_success;
      evidence.full_support = trace.support_full_valid;
      return evidence;
    }

    std::vector<SafetyVizP1Sample> toSafetyVizP1Samples(
        const std::vector<BsplineOptimizer::P1IntegrityVizSample> &samples)
    {
      std::vector<SafetyVizP1Sample> out;
      out.reserve(samples.size());
      for (const auto &sample : samples)
      {
        SafetyVizP1Sample viz;
        viz.position = sample.position;
        viz.grad = sample.grad;
        viz.push = sample.push;
        viz.cost = sample.cost;
        viz.t_s = sample.t_s;
        viz.hit = sample.hit;
        viz.stale = sample.stale;
        viz.unknown = sample.unknown;
        viz.reason = sample.reason;
        out.push_back(viz);
      }
      return out;
    }

    std::vector<SafetyVizP4Guide> toSafetyVizP4Guides(
        const std::vector<BsplineOptimizer::P4GuideViz> &guides)
    {
      std::vector<SafetyVizP4Guide> out;
      out.reserve(guides.size());
      for (const auto &guide : guides)
      {
        SafetyVizP4Guide viz;
        viz.original_path = guide.original_path;
        viz.risk_path = guide.risk_path;
        viz.selected_path = guide.selected_path;
        viz.segment_start = guide.segment_start;
        viz.segment_end = guide.segment_end;
        viz.path_length_ratio = guide.metrics.path_length_ratio;
        viz.risk_selected = guide.risk_selected;
        viz.reason = guide.metrics.fallback_reason;
        out.push_back(viz);
      }
      return out;
    }
  } // namespace

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::setTimeProvider(TimeProvider provider)
  {
    time_provider_ = std::move(provider);
  }

  rclcpp::Time EGOPlannerManager::plannerNow() const
  {
    if (time_provider_)
    {
      return time_provider_();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/p1_collision_fanout_clearance_m", 0.0);
    node->declare_parameter("manager/p1_collision_fanout_preserve_homotopies", false);
    node->declare_parameter("manager/p1_collision_fanout_mirror_y", false);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);
    node->declare_parameter("p2.enable_candidate_ranking", false);
    node->declare_parameter("p2.metrics_only", true);
    node->declare_parameter("p2.sample_dt_s", 0.2);
    node->declare_parameter("p2.lambda_candidate_integrity", 1.0);
    node->declare_parameter("p2.w_max_cost", 0.25);
    node->declare_parameter("p2.w_unknown", 5.0);
    node->declare_parameter("p2.w_stale", 2.0);
    node->declare_parameter("p2.min_valid_ratio", 0.3);
    node->declare_parameter("p2.debug_csv_enable", false);
    node->declare_parameter("p2.debug_csv_path", "");
    node->declare_parameter("p3.enable_local_reference_bias", false);
    node->declare_parameter("p3.enable_global_reference_bias", false);
    node->declare_parameter("p3.local_bias_radius_m", 1.5);
    node->declare_parameter("p3.min_improvement_ratio", 0.05);
    node->declare_parameter("p3.w_risk", 1.0);
    node->declare_parameter("p3.w_detour", 0.25);
    node->declare_parameter("p3.w_unknown", 5.0);
    node->declare_parameter("p3.min_corridor_valid_ratio", 0.8);
    node->declare_parameter("p3.station_spacing_m", 2.0);
    node->declare_parameter("p3.lateral_sample_step_m", 1.0);
    node->declare_parameter("p3.lateral_sample_count_each_side", 3);
    node->declare_parameter("p3.beam_width", 5);
    node->declare_parameter("p3.max_detour_ratio", 1.5);
    node->declare_parameter("p3.debug_csv_enable", false);
    node->declare_parameter("p3.debug_csv_path", "");

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/p1_collision_fanout_clearance_m",
                        pp_.p1_collision_fanout_clearance_m_);
    node->get_parameter("manager/p1_collision_fanout_preserve_homotopies",
                        pp_.p1_collision_fanout_preserve_homotopies_);
    node->get_parameter("manager/p1_collision_fanout_mirror_y",
                        pp_.p1_collision_fanout_mirror_y_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);
    node->get_parameter("p2.enable_candidate_ranking", p2_config_.enable_candidate_ranking);
    node->get_parameter("p2.metrics_only", p2_config_.metrics_only);
    node->get_parameter("p2.sample_dt_s", p2_config_.sample_dt_s);
    node->get_parameter("p2.lambda_candidate_integrity", p2_config_.lambda_candidate_integrity);
    node->get_parameter("p2.w_max_cost", p2_config_.w_max_cost);
    node->get_parameter("p2.w_unknown", p2_config_.w_unknown);
    node->get_parameter("p2.w_stale", p2_config_.w_stale);
    node->get_parameter("p2.min_valid_ratio", p2_config_.min_valid_ratio);
    node->get_parameter("p2.debug_csv_enable", p2_config_.debug_csv_enable);
    node->get_parameter("p2.debug_csv_path", p2_config_.debug_csv_path);
    node->get_parameter("p3.enable_local_reference_bias", p3_config_.enable_local_reference_bias);
    node->get_parameter("p3.enable_global_reference_bias", p3_config_.enable_global_reference_bias);
    node->get_parameter("p3.local_bias_radius_m", p3_config_.local_bias_radius_m);
    node->get_parameter("p3.min_improvement_ratio", p3_config_.min_improvement_ratio);
    node->get_parameter("p3.w_risk", p3_config_.w_risk);
    node->get_parameter("p3.w_detour", p3_config_.w_detour);
    node->get_parameter("p3.w_unknown", p3_config_.w_unknown);
    node->get_parameter("p3.min_corridor_valid_ratio", p3_config_.min_corridor_valid_ratio);
    node->get_parameter("p3.station_spacing_m", p3_config_.station_spacing_m);
    node->get_parameter("p3.lateral_sample_step_m", p3_config_.lateral_sample_step_m);
    node->get_parameter("p3.lateral_sample_count_each_side", p3_config_.lateral_sample_count_each_side);
    node->get_parameter("p3.beam_width", p3_config_.beam_width);
    node->get_parameter("p3.max_detour_ratio", p3_config_.max_detour_ratio);
    node->get_parameter("p3.debug_csv_enable", p3_config_.debug_csv_enable);
    node->get_parameter("p3.debug_csv_path", p3_config_.debug_csv_path);
    safety_viz_ = std::make_shared<SafetyRvizPublisher>(
        node, SafetyRvizPublisher::declareAndReadConfig(node));

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);
    node->get_parameter("grid_map/frame_id", trajectory_frame_id_);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    p0_risk_grid_runtime_ = P0RiskGridRuntime::createIfEnabled(node);
    if (p0_risk_grid_runtime_)
    {
      p0_risk_grid_runtime_->setOccupancyPredicate(
          [this](const Eigen::Vector3d &pos)
          {
            return grid_map_ && grid_map_->getInflateOccupancy(pos) > 0;
          });
      p0_risk_grid_runtime_->setOccupancyDiagnosticQueryFactory(
          [this]() -> iap::RiskGridMap::OccupancyDiagnosticQuery
          {
            if (!grid_map_)
              return {};
            auto frozen_query = grid_map_->captureOccupancyDiagnosticQuery();
            if (!frozen_query)
              return {};
            return [frozen_query = std::move(frozen_query)](
                       const Eigen::Vector3d &pos)
            {
              iap::RiskOccupancyDiagnostic out;
              const auto source = frozen_query(pos);
              out.available = source.available;
              out.raw_occupied = source.raw_occupied;
              out.inflated_occupied = source.inflated_occupied;
              out.voxel_index = source.voxel_index;
              out.voxel_center = source.voxel_center;
              out.resolution_m = source.resolution_m;
              out.inflation_m = source.inflation_m;
              out.frame_id = source.frame_id;
              out.cloud_stamp_s = source.cloud_stamp_s;
              out.occupancy_generation = source.generation;
              out.source = source.source;
              return out;
            };
          });
    }
    p5_integrity_gate_ = P5RuntimeIntegrityGate::createIfEnabled(node);
    if (p5_integrity_gate_)
    {
      p5_integrity_gate_->setPredAlertLimitEnvironment(
          [this](const Eigen::Vector3d &pos)
          {
            return grid_map_ && grid_map_->getInflateOccupancy(pos) > 0;
          },
          [this](Eigen::Vector3d *origin, Eigen::Vector3d *size)
          {
            if (!grid_map_ || !origin || !size)
            {
              return false;
            }
            grid_map_->getRegion(*origin, *size);
            return origin->allFinite() && size->allFinite();
          },
          [this]()
          {
            return grid_map_ ? grid_map_->getResolution()
                             : std::numeric_limits<double>::quiet_NaN();
          });
    }

    visualization_ = vis;
  }

  std::shared_ptr<const iap::RiskGridSnapshot> EGOPlannerManager::acquireRiskGridSnapshot() const
  {
    if (!p0_risk_grid_runtime_)
    {
      return nullptr;
    }
    return p0_risk_grid_runtime_->acquireSnapshot();
  }

  const EGOPlannerManager::PlanningRiskContext &
  EGOPlannerManager::beginPlanningRiskContext(const double now_s)
  {
    return beginPlanningRiskContextWithSnapshot(now_s, acquireRiskGridSnapshot());
  }

  const EGOPlannerManager::PlanningRiskContext &
  EGOPlannerManager::beginPlanningRiskContextWithSnapshot(
      const double now_s,
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
      const uint64_t planning_attempt_id)
  {
    planning_risk_context_ = PlanningRiskContext{};
    planning_risk_context_.active = true;
    planning_risk_context_.planning_start_s = now_s;
    planning_risk_context_.snapshot_acquired_s = now_s;
    planning_risk_context_.planning_attempt_id = planning_attempt_id
        ? planning_attempt_id : ++p1_planning_attempt_seq_;
    p1_planning_attempt_seq_ = std::max(
        p1_planning_attempt_seq_, planning_risk_context_.planning_attempt_id);
    planning_risk_context_.query_base_time_s = now_s;
    planning_risk_context_.snapshot = std::move(snapshot);
    if (planning_risk_context_.snapshot)
    {
      planning_risk_context_.generation_id =
          planning_risk_context_.snapshot->generation_id();
      const double snapshot_stamp_s = planning_risk_context_.snapshot->stamp_s();
      planning_risk_context_.snapshot_stamp_s = snapshot_stamp_s;
      if (std::isfinite(snapshot_stamp_s))
      {
        planning_risk_context_.query_base_time_s = snapshot_stamp_s;
      }
    }

    // P0 may legitimately publish startup health before the planner admits
    // its first P1 attempt.  Record the boundary once so offline validation
    // never guesses from CSV ordering.
    if (!p1_activation_recorded_)
    {
      appendPlanningRiskContextTimeline("planner_activation", now_s,
          "activated", "p1_planner_ready");
      p1_activation_recorded_ = true;
    }

    cout << "[RiskContext] planning_generation_id="
         << planning_risk_context_.generation_id
         << ", query_base_time_s="
         << planning_risk_context_.query_base_time_s
         << ", snapshot_available="
         << static_cast<int>(static_cast<bool>(planning_risk_context_.snapshot))
         << endl;
    appendPlanningRiskContextTimeline("acquire", now_s, "acquired",
        planning_risk_context_.snapshot ? "ok" : "snapshot_unavailable");
    return planning_risk_context_;
  }

  void EGOPlannerManager::clearPlanningRiskContext()
  {
    planning_risk_context_ = PlanningRiskContext{};
  }

  std::string EGOPlannerManager::p1PlanningContextTimelinePath() const
  {
    const std::string profile_path = bspline_optimizer_
        ? bspline_optimizer_->p1AcceptedTrajectoryRiskProfilePath()
        : "planner_p1_accepted_trajectory_risk_profile.csv";
    const std::string suffix = "planner_p1_accepted_trajectory_risk_profile.csv";
    const auto found = profile_path.rfind(suffix);
    return found == std::string::npos
        ? profile_path + ".planning_context_timeline.csv"
        : profile_path.substr(0, found) + "planner_p1_planning_context_timeline.csv";
  }

  std::string EGOPlannerManager::p1PreAdmissionAttemptPath() const
  {
    const std::string profile_path = bspline_optimizer_
        ? bspline_optimizer_->p1AcceptedTrajectoryRiskProfilePath()
        : "planner_p1_accepted_trajectory_risk_profile.csv";
    const std::string suffix = "planner_p1_accepted_trajectory_risk_profile.csv";
    const auto found = profile_path.rfind(suffix);
    return found == std::string::npos
        ? profile_path + ".pre_admission_attempt.csv"
        : profile_path.substr(0, found) + "planner_p1_pre_admission_attempt.csv";
  }

  void EGOPlannerManager::writeP1PreAdmissionAttempt(
      const std::string &stage, const uint64_t candidate_id,
      const UniformBspline &initial_trajectory,
      const iap::P1AcceptedContextValidation &initial_validation,
      const UniformBspline *base_optimized_trajectory,
      const bool base_optimizer_success, const std::string &base_reason,
      const std::string &p1_admission_verdict,
      const std::string &p1_admission_reason) const
  {
    if (!bspline_optimizer_) return;
    const auto &p1 = bspline_optimizer_->p1IntegrityConfig();
    if (!p1.debug_csv_enable || p1.debug_csv_path.empty()) return;
    const std::string path = p1PreAdmissionAttemptPath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return;
    // UniformBspline predates const-qualified accessors.  Copying for these
    // scalar queries preserves the immutable pre-admission input.
    UniformBspline initial_duration_probe = initial_trajectory;
    const double initial_duration = initial_duration_probe.getTimeSum();
    double base_duration = std::numeric_limits<double>::quiet_NaN();
    if (base_optimized_trajectory) {
      UniformBspline base_duration_probe = *base_optimized_trajectory;
      base_duration = base_duration_probe.getTimeSum();
    }
    const double snapshot_time_min = planning_risk_context_.query_base_time_s +
        initial_validation.horizon_min_s;
    const double snapshot_time_max = planning_risk_context_.query_base_time_s +
        initial_validation.horizon_max_s;
    const double initial_time_max = planning_risk_context_.query_base_time_s +
        initial_duration;
    const double base_time_max = planning_risk_context_.query_base_time_s +
        base_duration;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,stage,planning_attempt_id,candidate_id,"
             "snapshot_generation_id,snapshot_stamp_s,query_base_time_s,snapshot_time_min_s,snapshot_time_max_s,"
             "initial_duration_s,initial_time_min_s,initial_time_max_s,initial_temporal_margin_s,"
             "expected_sample_count,matched_sample_count,spatial_miss_count,temporal_miss_count,occupied_miss_count,stale_miss_count,invalid_miss_count,"
             "p1_admission_verdict,p1_admission_reason,base_optimizer_success,base_optimizer_reason,"
             "base_duration_s,base_time_min_s,base_time_max_s,base_temporal_margin_s,base_full_p1_support\n";
    }
    const bool base_full_support = base_optimized_trajectory &&
        base_optimizer_success &&
        bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
            *base_optimized_trajectory, plannerNow().seconds(),
            trajectory_frame_id_).valid;
    out << p1.evidence_schema_version << ',' << p1.evidence_run_id << ','
        << p1.evidence_manifest_path << ',' << stage << ','
        << planning_risk_context_.planning_attempt_id << ',' << candidate_id << ','
        << planning_risk_context_.generation_id << ','
        << planning_risk_context_.snapshot_stamp_s << ','
        << planning_risk_context_.query_base_time_s << ','
        << snapshot_time_min << ',' << snapshot_time_max << ','
        << initial_duration << ',' << planning_risk_context_.query_base_time_s << ','
        << initial_time_max << ',' << (snapshot_time_max - initial_time_max) << ','
        << initial_validation.expected_sample_count << ','
        << initial_validation.covered_sample_count << ','
        << initial_validation.spatial_miss_count << ','
        << initial_validation.temporal_miss_count << ','
        << initial_validation.occupied_miss_count << ','
        << initial_validation.stale_miss_count << ','
        << initial_validation.invalid_miss_count << ','
        << p1_admission_verdict << ',' << p1_admission_reason << ','
        << (base_optimizer_success ? 1 : 0) << ',' << base_reason << ','
        << base_duration << ',' << planning_risk_context_.query_base_time_s << ','
        << base_time_max << ',' << (snapshot_time_max - base_time_max) << ','
        << (base_full_support ? 1 : 0) << '\n';
  }

  void EGOPlannerManager::appendPlanningRiskContextTimeline(
      const std::string &stage, const double stamp_s, const std::string &outcome,
      const std::string &reason, const std::string &fallback_branch,
      const PlanningRiskContext *context_override) const
  {
    // Unit-level lifecycle checks intentionally construct a manager before
    // the optimizer/artifact registry is initialized.  Those checks must not
    // dereference a null writer merely to emit optional runtime evidence.
    if (!bspline_optimizer_)
      return;
    const std::string path = p1PlanningContextTimelinePath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,stage,stamp_s,planning_attempt_id,candidate_id,snapshot_generation_id,"
             "snapshot_stamp_s,query_base_time_s,context_age_s,stale_threshold_s,"
             "outcome,reason,fallback_branch\n";
    }
    const auto &ctx = context_override ? *context_override
                                       : planning_risk_context_;
    const double threshold = ctx.snapshot ? ctx.snapshot->params().stale_timeout_s
                                          : std::numeric_limits<double>::quiet_NaN();
    const double age = std::isfinite(ctx.snapshot_stamp_s) ? stamp_s - ctx.snapshot_stamp_s
                                                            : std::numeric_limits<double>::quiet_NaN();
    const auto &p1 = bspline_optimizer_->p1IntegrityConfig();
    out << p1.evidence_schema_version << ',' << p1.evidence_run_id << ','
        << p1.evidence_manifest_path << ',' << stage << ',' << stamp_s << ',' << ctx.planning_attempt_id << ','
        << ctx.candidate_id << ',' << ctx.generation_id << ',' << ctx.snapshot_stamp_s << ','
        << ctx.query_base_time_s << ',' << age << ',' << threshold << ','
        << outcome << ',' << reason << ',' << fallback_branch << '\n';
  }

  bool EGOPlannerManager::planningRiskContextFresh(
      const double now_s, std::string *reason) const
  {
    // Safety-off/P0-off planning keeps its historical behavior: there is no
    // P0-derived candidate to guard. Once P0 is enabled, absence of its
    // snapshot is fail-closed for the P1 evidence path.
    if (!p0_risk_grid_runtime_ && !planning_risk_context_.snapshot)
    {
      if (reason) *reason = "risk_grid_disabled";
      return true;
    }
    const auto &ctx = planning_risk_context_;
    if (!ctx.active || !ctx.snapshot || !std::isfinite(ctx.snapshot_stamp_s) ||
        !std::isfinite(now_s)) {
      if (reason) *reason = "planning_risk_context_unavailable";
      return false;
    }
    const double stale_timeout_s = ctx.snapshot->params().stale_timeout_s;
    const double age_s = now_s - ctx.snapshot_stamp_s;
    if (!std::isfinite(age_s) || age_s < 0.0 ||
        (stale_timeout_s >= 0.0 && age_s > stale_timeout_s)) {
      if (reason) *reason = "stale_planning_risk_context";
      return false;
    }
    if (reason) *reason = "ok";
    return true;
  }

  bool EGOPlannerManager::preparePlanningRiskPublish(
      const double now_s, std::string *reason)
  {
    planning_risk_context_.pre_publish_s = now_s;
    const bool fresh = planningRiskContextFresh(now_s, reason);
    const bool blocks_publish = !fresh && planning_risk_context_.p1_objective_applied;
    last_p1_rejection_requires_new_generation_ = blocks_publish && reason &&
        (*reason == "stale_planning_risk_context" ||
         *reason == "planning_risk_context_unavailable");
    appendPlanningRiskContextTimeline("pre_publish", now_s,
        fresh ? "fresh" : (blocks_publish ? "rejected" : "base_fallback"),
        reason ? *reason : "",
        blocks_publish ? "existing_trajectory" : "p1_soft_fallback");
    return fresh || !blocks_publish;
  }

  bool EGOPlannerManager::finalizeP1AcceptedRiskProfile(
      const double publish_stamp_s)
  {
    // Freshness is checked immediately before the ROS publish.  Do not move a
    // second check here: it could create a published trajectory without the
    // evidence row for the same already-approved candidate.
    planning_risk_context_.accepted_s = publish_stamp_s;
    planning_risk_context_.publish_s = publish_stamp_s;
    const bool written = bspline_optimizer_->writeP1AcceptedTrajectoryRiskProfile(
        local_data_.position_traj_, ++p1_accepted_profile_seq_, local_data_.traj_id_,
        publish_stamp_s, planning_risk_context_.planning_start_s,
        trajectory_frame_id_, local_data_.start_time_.seconds());
    appendPlanningRiskContextTimeline("publish", publish_stamp_s,
        written ? "published" : "published_without_profile",
        written ? "ok" : "accepted_profile_write_failed");
    bspline_optimizer_->clearRiskSnapshot();
    return written;
  }

  bool EGOPlannerManager::recordP1MetricsOnlyReferenceObservation(
      const double observation_stamp_s)
  {
    if (!bspline_optimizer_ || !planning_risk_context_.active ||
        !planning_risk_context_.snapshot || local_data_.traj_id_ == 0 ||
        !std::isfinite(observation_stamp_s) ||
        !std::isfinite(local_data_.start_time_.seconds()) ||
        !std::isfinite(local_data_.duration_))
      return false;

    const auto &config = bspline_optimizer_->p1IntegrityConfig();
    std::string freshness_reason;
    if (!planningRiskContextFresh(observation_stamp_s, &freshness_reason))
      return false;
    const double trajectory_start_t_s = std::clamp(
        observation_stamp_s - local_data_.start_time_.seconds(),
        0.0, std::max(0.0, local_data_.duration_));
    const double remaining_duration_s =
        std::max(0.0, local_data_.duration_ - trajectory_start_t_s);
    const auto &horizons = planning_risk_context_.snapshot->params().horizons_s;
    const auto horizon_max_it = std::max_element(horizons.begin(), horizons.end());
    const double snapshot_horizon_s = horizon_max_it == horizons.end()
        ? std::numeric_limits<double>::quiet_NaN() : *horizon_max_it;
    if (!shouldRecordP1MetricsOnlyReferenceObservation(
            config.metrics_only, local_data_.traj_id_,
            p1_metrics_reference_observed_trajectory_id_,
            remaining_duration_s, snapshot_horizon_s))
      return false;

    planning_risk_context_.candidate_id = 0;
    planning_risk_context_.p1_objective_allowed = false;
    planning_risk_context_.p1_objective_applied = false;
    planning_risk_context_.p1_fallback_reason =
        "metrics_only_reference_observation";
    BsplineOptimizer::P1PlanningRiskContext context;
    context.snapshot = planning_risk_context_.snapshot;
    context.query_base_time_s = planning_risk_context_.query_base_time_s;
    context.planning_start_s = planning_risk_context_.planning_start_s;
    context.planning_attempt_id = planning_risk_context_.planning_attempt_id;
    context.candidate_id = 0;
    context.objective_allowed = false;
    context.fallback_reason = "metrics_only_reference_observation";
    bspline_optimizer_->setP1PlanningRiskContext(std::move(context));

    // Reserve the sequence before the multi-file write.  A failed context
    // rename must not let a later retry append a second 200-row profile under
    // the same identity.
    const uint64_t profile_seq = ++p1_accepted_profile_seq_;
    const bool written = bspline_optimizer_->writeP1AcceptedTrajectoryRiskProfile(
        local_data_.position_traj_, profile_seq, local_data_.traj_id_,
        observation_stamp_s, planning_risk_context_.planning_start_s,
        trajectory_frame_id_, local_data_.start_time_.seconds(),
        trajectory_start_t_s, remaining_duration_s);
    if (!written)
    {
      appendPlanningRiskContextTimeline(
          "reference_observation", observation_stamp_s, "write_failed",
          "accepted_profile_write_failed", "existing_trajectory");
      return false;
    }

    p1_metrics_reference_observed_trajectory_id_ = local_data_.traj_id_;
    appendPlanningRiskContextTimeline(
        "reference_observation", observation_stamp_s, "recorded",
        "metrics_only_reference_observation", "existing_trajectory");
    // Formal scene evidence must bind the same immutable snapshot used for
    // this read-only incumbent observation, independent of the periodic RViz
    // publisher phase.
    if (safety_viz_)
      safety_viz_->publishPredictedPLCloud(
          planning_risk_context_.snapshot,
          local_data_.position_traj_.evaluateDeBoorT(trajectory_start_t_s).z(),
          observation_stamp_s, true);
    return true;
  }

  bool EGOPlannerManager::p1AdmissionEnabled() const
  {
    if (!bspline_optimizer_) return false;
    return bspline_optimizer_->getP1IntegrityConfig().use_integrity_cost;
  }

  void EGOPlannerManager::recordP1RetryDeferred(
      const std::string &reason, const double stamp_s,
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot)
  {
    PlanningRiskContext deferred_context;
    deferred_context.snapshot = std::move(snapshot);
    deferred_context.query_base_time_s = stamp_s;
    if (deferred_context.snapshot)
    {
      deferred_context.generation_id = deferred_context.snapshot->generation_id();
      deferred_context.snapshot_stamp_s = deferred_context.snapshot->stamp_s();
      deferred_context.query_base_time_s = deferred_context.snapshot_stamp_s;
    }
    appendPlanningRiskContextTimeline("retry_deferred", stamp_s, "deferred",
        reason, "existing_polynomial", &deferred_context);
  }

  void EGOPlannerManager::recordP1StaleRejection(
      const std::string &reason, const double stamp_s)
  {
    appendPlanningRiskContextTimeline("stale_rejection", stamp_s, "rejected",
        reason, "existing_trajectory");
  }

  void EGOPlannerManager::setPlanningRiskContextForTest(
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
      const double query_base_time_s)
  {
    planning_risk_context_ = PlanningRiskContext{};
    planning_risk_context_.active = true;
    planning_risk_context_.planning_start_s = query_base_time_s;
    planning_risk_context_.snapshot_acquired_s = query_base_time_s;
    planning_risk_context_.planning_attempt_id = ++p1_planning_attempt_seq_;
    planning_risk_context_.query_base_time_s = query_base_time_s;
    planning_risk_context_.snapshot = std::move(snapshot);
    if (planning_risk_context_.snapshot)
    {
      planning_risk_context_.generation_id =
          planning_risk_context_.snapshot->generation_id();
      planning_risk_context_.snapshot_stamp_s =
          planning_risk_context_.snapshot->stamp_s();
    }
  }

  bool EGOPlannerManager::applyLocalTargetP3ReferenceBias(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
      Eigen::Vector3d &local_target_pt, Eigen::Vector3d &local_target_vel)
  {
    (void)local_target_vel;
    if (!p3_config_.enable_local_reference_bias)
    {
      return false;
    }
    const auto now = plannerNow();
    P3LocalBiasInput input;
    input.start_pt = start_pt;
    input.end_pt = end_pt;
    input.nominal_target = local_target_pt;
    input.max_vel = pp_.max_vel_;
    const auto snapshot = planning_risk_context_.active
                              ? currentPlanningRiskSnapshot()
                              : acquireRiskGridSnapshot();
    const auto result = applyP3LocalReferenceBias(
        input, p3_config_, snapshot,
        [this](const Eigen::Vector3d &pos)
        {
          return grid_map_ && grid_map_->getInflateOccupancy(pos) == 0;
        },
        planning_risk_context_.active ? currentPlanningQueryBaseTime()
                                      : now.seconds(),
        ++p3_batch_id_);
    if (p3_config_.enable_local_reference_bias)
    {
      cout << "[P3-local] reason=" << result.reason
           << ", used=" << result.used_bias
           << ", improvement=" << result.improvement_ratio << endl;
    }
    if (safety_viz_)
    {
      SafetyVizP3ReferenceBias viz;
      viz.local = true;
      viz.used_bias = result.used_bias;
      viz.start = result.start_pt;
      viz.end = result.end_pt;
      viz.nominal_target = result.nominal_target;
      viz.biased_target = result.target;
      viz.improvement_ratio = result.improvement_ratio;
      viz.reason = result.reason;
      safety_viz_->publishP3ReferenceBias(viz, now.seconds());
    }
    if (result.used_bias)
    {
      local_target_pt = result.target;
      return true;
    }
    return false;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    last_p1_rejection_reason_.clear();
    last_p1_rejection_requires_new_generation_ = false;
    const auto p1_config = bspline_optimizer_->getP1IntegrityConfig();
    const bool has_existing_trajectory =
        local_data_.traj_id_ > 0 && local_data_.duration_ > 0.0;
    bool p1_objective_allowed =
        p1_config.use_integrity_cost && !p1_config.metrics_only;
    std::string p1_fallback_reason = "none";
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    const bool created_local_risk_context = !planning_risk_context_.active;
    if (created_local_risk_context)
    {
      beginPlanningRiskContext(plannerNow().seconds());
    }
    struct LocalRiskContextGuard
    {
      EGOPlannerManager *manager = nullptr;
      bool enabled = false;
      ~LocalRiskContextGuard()
      {
        if (enabled && manager)
        {
          manager->clearPlanningRiskContext();
        }
      }
    } local_risk_context_guard{this, created_local_risk_context};

    const auto planning_snapshot = currentPlanningRiskSnapshot();
    const double planning_query_base_time_s = currentPlanningQueryBaseTime();
    double incumbent_start_t_s = 0.0;
    if (has_existing_trajectory &&
        std::isfinite(planning_risk_context_.planning_start_s) &&
        std::isfinite(local_data_.start_time_.seconds()) &&
        std::isfinite(local_data_.duration_))
    {
      incumbent_start_t_s = std::clamp(
          planning_risk_context_.planning_start_s -
              local_data_.start_time_.seconds(),
          0.0, std::max(0.0, local_data_.duration_));
    }
    const auto set_p1_context = [this, planning_snapshot,
                                 planning_query_base_time_s,
                                 &p1_objective_allowed,
                                 &p1_fallback_reason,
                                 &p1_config](const uint64_t candidate_id)
    {
      planning_risk_context_.candidate_id = candidate_id;
      BsplineOptimizer::P1PlanningRiskContext context;
      context.snapshot = planning_snapshot;
      context.query_base_time_s = planning_query_base_time_s;
      context.planning_start_s = planning_risk_context_.planning_start_s;
      context.planning_attempt_id = planning_risk_context_.planning_attempt_id;
      context.candidate_id = candidate_id;
      context.objective_allowed = p1_objective_allowed;
      context.fallback_reason = p1_fallback_reason;
      planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
      planning_risk_context_.p1_objective_applied =
          p1_objective_allowed && p1_config.use_integrity_cost;
      planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
      bspline_optimizer_->setP1PlanningRiskContext(std::move(context));
    };

    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (plannerNow() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将轨迹变为B样条轨迹
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    const UniformBspline initial_candidate(ctrl_pts, 3, ts);
    iap::P1AcceptedContextValidation initial_p1_validation;

    if (p1_config.use_integrity_cost)
    {
      set_p1_context(0);
      initial_p1_validation =
          bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
              initial_candidate, plannerNow().seconds(), trajectory_frame_id_);
      const bool defer_admission_until_base_prepass =
          !p1_config.metrics_only && p1_config.lambda_integrity != 0.0 &&
          planning_snapshot && std::isfinite(planning_query_base_time_s) &&
          canP1BasePrepassRecoverSupport(initial_p1_validation);
      if (defer_admission_until_base_prepass)
      {
        p1_objective_allowed = true;
        p1_fallback_reason = "pending_base_prepass";
      }
      else
      {
        const auto fallback = decideP1SoftFallback({
            p1_config.metrics_only, false, has_existing_trajectory,
            initial_p1_validation});
        p1_objective_allowed = fallback.objective_allowed;
        p1_fallback_reason = fallback.reason;
      }
      planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
      planning_risk_context_.p1_objective_applied = false;
      planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
      appendPlanningRiskContextTimeline(
          defer_admission_until_base_prepass ? "p1_admission_pending"
                                             : "p1_admission",
          plannerNow().seconds(),
          defer_admission_until_base_prepass ? "base_prepass_pending" :
          p1_objective_allowed ? "p1_objective" : "base_fallback",
          p1_fallback_reason,
          defer_admission_until_base_prepass ? "none" :
          p1_objective_allowed ? "none" : "p1_soft_fallback");
      // Record the actual pre-admission seed.  A later published base
      // trajectory cannot be used to reconstruct this H1/H2/H3/H4 evidence.
      writeP1PreAdmissionAttempt(
          "initial_admission", 0, initial_candidate, initial_p1_validation, nullptr,
          false, "not_run", p1_objective_allowed ? "p1_objective" : "base_fallback",
          p1_fallback_reason);
      bspline_optimizer_->clearRiskSnapshot();
    }
    // The candidate CSV is the strict fixed-lattice P1 evidence contract. A
    // base fallback remains visible in the lifecycle timeline and accepted
    // profile sidecar, but it is not a P1 optimizer start because it lacks
    // full P1 lattice support. Metrics-only remains evidence after valid
    // admission, even though its objective is not applied.
    bool write_p1_candidate_trace =
        p1_objective_allowed || p1_fallback_reason == "metrics_only";

    vector<std::pair<int, int>> segments;
    if (bspline_optimizer_->getP4RiskAStarConfig().enable_risk_aware_astar)
    {
      bspline_optimizer_->setP4RiskSnapshot(planning_snapshot, planning_query_base_time_s);
    }
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    if (safety_viz_)
    {
      safety_viz_->publishP4Guides(
          toSafetyVizP4Guides(bspline_optimizer_->getLastP4GuideViz()),
          plannerNow().seconds());
    }
    bspline_optimizer_->clearP4RiskSnapshot();
    // 计算时间差并更新时间
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    bool p1_preference_rejected = false;
    bool p1_candidate_traces_deferred = false;
    uint64_t selected_p1_candidate_id = 1;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    std::vector<BsplineOptimizer::P1OptimizationTrace> p1_candidate_traces;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      std::vector<BsplineOptimizer::P1BasePrepassTrace> candidate_prepasses;
      const int candidate_limit = std::clamp(
          p1_config.max_candidates_per_attempt, 1, 8);
      const auto fanout_before_supplement =
          bspline_optimizer_->lastP1FanoutDiagnostics();
      bool collision_fanout_active = false;
      if (trajs.size() == 1 &&
          fanout_before_supplement.singleton_due_to_empty_segments &&
          (initial_p1_validation.occupied_miss_count > 0 ||
           pp_.p1_collision_fanout_preserve_homotopies_) &&
          pp_.p1_collision_fanout_clearance_m_ > 0.0) {
        const auto fanout = makeP1CollisionClearanceFanout(
            trajs.front().points,
            std::max<std::size_t>(initial_p1_validation.occupied_miss_count, 1),
            pp_.p1_collision_fanout_clearance_m_, candidate_limit,
            pp_.p1_collision_fanout_mirror_y_ ? 1.0 : -1.0,
            pp_.p1_collision_fanout_preserve_homotopies_);
        const ControlPoints prototype = trajs.front();
        trajs.clear();
        trajs.reserve(fanout.size());
        for (const auto& points : fanout) {
          ControlPoints candidate = prototype;
          candidate.points = points;
          for (int column = 3; column < points.cols() - 3; ++column) {
            const Eigen::Vector3d displacement =
                points.col(column) - prototype.points.col(column);
            if (displacement.norm() <= 1.0e-9) continue;
            const Eigen::Vector3d direction = displacement.normalized();
            candidate.base_point[column].push_back(
                makeP1CollisionConstraintBasePoint(
                    prototype.points.col(column), points.col(column), direction,
                    pp_.p1_collision_fanout_clearance_m_));
            candidate.direction[column].push_back(direction);
            candidate.clearance = std::max(
                candidate.clearance, pp_.p1_collision_fanout_clearance_m_);
          }
          trajs.push_back(std::move(candidate));
        }
        collision_fanout_active = trajs.size() > 1;
      }
      bool normalized_p1_stage = p1_objective_allowed &&
          !p1_config.metrics_only && p1_config.lambda_integrity != 0.0;
      if (normalized_p1_stage)
      {
        std::vector<ControlPoints> base_candidates;
        std::vector<ControlPoints> unsupported_base_candidates;
        std::vector<BsplineOptimizer::P1BasePrepassTrace> base_prepasses;
        base_candidates.reserve(trajs.size());
        unsupported_base_candidates.reserve(trajs.size());
        base_prepasses.reserve(trajs.size());
        for (std::size_t topology_index = 0;
             topology_index < trajs.size(); ++topology_index)
        {
          set_p1_context(static_cast<uint64_t>(topology_index + 1));
          const double base_start_s = plannerNow().seconds();
          appendPlanningRiskContextTimeline(
              "base_prepass_start", base_start_s, "started", "ok");
          Eigen::MatrixXd base_points;
          double base_cost = 0.0;
          const bool base_success =
              bspline_optimizer_->BsplineOptimizeTrajBasePrepass(
                  base_points, base_cost, trajs[topology_index], ts);
          const auto base_prepass =
              bspline_optimizer_->getLastP1BasePrepassTrace();
          bool base_full_support = false;
          ControlPoints base_control_points;
          if (base_success)
          {
            const auto base_summary =
                bspline_optimizer_->evaluateP1FixedLatticeRisk(
                    UniformBspline(base_points, 3, ts));
            base_full_support = base_summary.full_support;
            base_control_points = bspline_optimizer_->getControlPoints();
          }
          appendPlanningRiskContextTimeline(
              "base_prepass_end", plannerNow().seconds(),
              base_success && base_full_support ? "candidate_success"
                                                : "candidate_failure",
              !base_success ? "optimizer_failure" :
              !base_full_support ? "fixed_support_not_full" : "ok");
          bspline_optimizer_->clearRiskSnapshot();
          if (!base_success)
            continue;
          base_control_points.points = base_points;
          if (!base_full_support)
          {
            unsupported_base_candidates.push_back(std::move(base_control_points));
            continue;
          }
          base_candidates.push_back(std::move(base_control_points));
          base_prepasses.push_back(base_prepass);
        }
        trajs = std::move(base_candidates);
        candidate_prepasses = std::move(base_prepasses);

        const auto prepass_fallback = decideP1BasePrepassFallback({
            !trajs.empty() || !unsupported_base_candidates.empty(),
            !trajs.empty(), has_existing_trajectory,
            has_p1_preference_incumbent_});
        normalized_p1_stage =
            prepass_fallback.action == P1SoftFallbackAction::USE_P1_CANDIDATE;
        if (prepass_fallback.action ==
            P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE)
        {
          trajs = std::move(unsupported_base_candidates);
          candidate_prepasses.clear();
        }
        else if (!normalized_p1_stage)
        {
          trajs.clear();
          candidate_prepasses.clear();
        }
        if (prepass_fallback.action ==
                P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY &&
            has_p1_preference_incumbent_)
        {
          p1_preference_rejected = true;
          last_p1_rejection_reason_ = prepass_fallback.reason;
          last_p1_rejection_requires_new_generation_ = true;
          appendPlanningRiskContextTimeline(
              "replacement", plannerNow().seconds(), "rejected",
              prepass_fallback.reason, "existing_trajectory");
        }

        // The supplement is generated only after a collision-feasible,
        // full-support base prepass.  Each active control point follows its
        // own projected fixed-200 raw P1 gradient; the base seed is retained.
        if (normalized_p1_stage && trajs.size() == 1 &&
            (fanout_before_supplement.singleton_due_to_empty_segments ||
             fanout_before_supplement.singleton_due_to_degenerate_segments ||
             fanout_before_supplement.singleton_due_to_opposite_direction_unavailable))
        {
          bspline_optimizer_->setRiskSnapshot(
              planning_snapshot, planning_query_base_time_s);
          const auto supplemental =
              bspline_optimizer_->supplementP1RiskGradientCandidates(
                  trajs.front(), planning_snapshot, planning_query_base_time_s,
                  candidate_limit - static_cast<int>(trajs.size()));
          bspline_optimizer_->clearRiskSnapshot();
          const auto prepass = candidate_prepasses.front();
          trajs.insert(trajs.end(), supplemental.begin(), supplemental.end());
          candidate_prepasses.insert(
              candidate_prepasses.end(), supplemental.size(), prepass);
        }
        const bool p1_admitted = normalized_p1_stage && !trajs.empty();
        p1_objective_allowed = prepass_fallback.objective_allowed;
        p1_fallback_reason = p1_admitted ? "none" : prepass_fallback.reason;
        planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
        planning_risk_context_.p1_objective_applied = false;
        planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
        write_p1_candidate_trace =
            p1_objective_allowed || p1_fallback_reason == "metrics_only";
        appendPlanningRiskContextTimeline(
            "p1_admission", plannerNow().seconds(),
            p1_admitted ? "p1_objective" : "base_fallback",
            p1_fallback_reason,
            p1_admitted ? "none" : "p1_soft_fallback");
      }
      if (static_cast<int>(trajs.size()) > candidate_limit)
      {
        trajs.resize(static_cast<std::size_t>(candidate_limit));
        if (candidate_prepasses.size() > trajs.size())
          candidate_prepasses.resize(trajs.size());
      }
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost;
      std::vector<P2CandidateInput> p2_candidates;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        const uint64_t candidate_id = static_cast<uint64_t>(trajs.size() - i);
        set_p1_context(candidate_id);
        planning_risk_context_.optimizer_start_s = plannerNow().seconds();
        appendPlanningRiskContextTimeline(
            write_p1_candidate_trace ? "optimizer_start" : "base_optimizer_start",
            planning_risk_context_.optimizer_start_s, "started", "ok");
        std::string optimizer_reason = "ok";
        bool p1_candidate_success = normalized_p1_stage
            ? bspline_optimizer_->BsplineOptimizeTrajNormalizedP1(
                  ctrl_pts_temp, final_cost, trajs[i], ts,
                  candidate_prepasses[static_cast<std::size_t>(i)],
                  &optimizer_reason)
            : bspline_optimizer_->BsplineOptimizeTrajRebound(
                  ctrl_pts_temp, final_cost, trajs[i], ts);
        if (p1_candidate_success && collision_fanout_active)
        {
          const auto support = bspline_optimizer_->evaluateP1FixedLatticeRisk(
              UniformBspline(ctrl_pts_temp, 3, ts));
          p1_candidate_success = support.full_support;
          if (!p1_candidate_success) optimizer_reason =
              support.occupied_sample_count > 0 &&
              support.evidence_miss_count == 0
              ? "p0_collision_support_not_full"
              : "p0_collision_support_unavailable";
        }
        planning_risk_context_.optimizer_end_s = plannerNow().seconds();
        appendPlanningRiskContextTimeline(
            write_p1_candidate_trace ? "optimizer_end" : "base_optimizer_end",
            planning_risk_context_.optimizer_end_s,
            p1_candidate_success ? "candidate_success" : "candidate_failure",
            p1_candidate_success ? "ok" :
            optimizer_reason == "ok" ? "optimizer_failure" : optimizer_reason);
        if (write_p1_candidate_trace)
        {
          p1_candidate_traces.push_back(
              bspline_optimizer_->getLastP1OptimizationTrace());
        }
        bspline_optimizer_->clearRiskSnapshot();
        if (safety_viz_)
        {
          safety_viz_->publishP1IntegrityViz(
              toSafetyVizP1Samples(bspline_optimizer_->getLastP1IntegrityVizSamples()),
              toSafetyVizP1Metrics(bspline_optimizer_->getLastP1IntegrityMetrics()),
              plannerNow().seconds());
        }
        if (p1_candidate_success)
        {

          if (!p1_objective_allowed && p1_config.use_integrity_cost)
          {
            // Diagnostic-only: establish whether a collision-feasible base
            // result would have 200/200 support before changing admission.
            bspline_optimizer_->setRiskSnapshot(
                planning_snapshot, planning_query_base_time_s);
            const UniformBspline base_trajectory(ctrl_pts_temp, 3, ts);
            const auto base_validation =
                bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
                    base_trajectory, plannerNow().seconds(), trajectory_frame_id_);
            writeP1PreAdmissionAttempt(
                "base_optimizer_result", candidate_id, initial_candidate,
                initial_p1_validation, &base_trajectory, true, "ok",
                base_validation.valid ? "p1_objective_candidate" : "base_fallback",
                base_validation.valid ? "ok" : p1FallbackReason(base_validation));
            bspline_optimizer_->clearRiskSnapshot();
          }

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          P2CandidateInput p2_candidate;
          p2_candidate.candidate_id = static_cast<int>(candidate_id);
          p2_candidate.control_points = ctrl_pts_temp;
          p2_candidate.final_cost = final_cost;
          p2_candidate.cost_breakdown = bspline_optimizer_->getLastOptimizerCostBreakdown();
          p2_candidates.push_back(p2_candidate);

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          if (!p1_objective_allowed && p1_config.use_integrity_cost)
          {
            writeP1PreAdmissionAttempt(
                "base_optimizer_result", candidate_id, initial_candidate,
                initial_p1_validation, nullptr, false, "optimizer_failure",
                "base_fallback", p1_fallback_reason);
          }
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      if (!p2_candidates.empty())
      {
        std::shared_ptr<const iap::RiskGridSnapshot> p2_snapshot;
        if (p2_config_.enable_candidate_ranking)
        {
          p2_snapshot = planning_snapshot;
        }
        const auto p2_result = rankP2Candidates(
            p2_candidates, p2_config_, p2_snapshot, planning_query_base_time_s, ts,
            plannerNow().seconds(), ++p2_batch_id_);
        if (safety_viz_)
        {
          std::vector<SafetyVizP2Candidate> viz_candidates;
          viz_candidates.reserve(p2_candidates.size());
          for (std::size_t candidate_idx = 0; candidate_idx < p2_candidates.size();
               ++candidate_idx)
          {
            SafetyVizP2Candidate viz;
            viz.candidate_id = p2_candidates[candidate_idx].candidate_id;
            viz.selected =
                static_cast<int>(candidate_idx) == p2_result.selected_index;
            viz.fallback = p2_result.fallback;
            viz.reason = p2_result.fallback_reason;
            viz.control_points =
                matrixColumnsToPoints(p2_candidates[candidate_idx].control_points);
            for (const auto &metrics : p2_result.metrics)
            {
              if (metrics.candidate_id == viz.candidate_id)
              {
                viz.selected = metrics.selected;
                viz.score = metrics.candidate_score;
                viz.valid_ratio = metrics.valid_ratio;
                viz.reason = metrics.fallback_reason;
                break;
              }
            }
            viz_candidates.push_back(viz);
          }
          safety_viz_->publishP2Candidates(viz_candidates,
                                           plannerNow().seconds());
        }
        if (p2_result.selected_index >= 0 &&
            p2_result.selected_index < static_cast<int>(p2_candidates.size()))
        {
          ctrl_pts = p2_candidates[p2_result.selected_index].control_points;
          selected_p1_candidate_id = static_cast<uint64_t>(
              p2_candidates[p2_result.selected_index].candidate_id);
          if (p2_config_.enable_candidate_ranking)
          {
            cout << "[P2] selected_candidate="
                 << p2_candidates[p2_result.selected_index].candidate_id
                 << ", fallback=" << p2_result.fallback_reason
                 << ", metrics_only=" << p2_config_.metrics_only << endl;
          }
        }
        if (pp_.p1_collision_fanout_preserve_homotopies_ &&
            p1_config.metrics_only && planning_snapshot &&
            std::isfinite(planning_query_base_time_s))
        {
          for (const auto &candidate : p2_candidates)
          {
            set_p1_context(static_cast<uint64_t>(candidate.candidate_id));
            bspline_optimizer_->writeP1PrequalificationCandidateProfile(
                UniformBspline(candidate.control_points, 3, ts),
                candidate.candidate_id ==
                    static_cast<int>(selected_p1_candidate_id));
          }
          set_p1_context(selected_p1_candidate_id);
        }
        if (!p1_candidate_traces.empty())
        {
          std::vector<P1CandidateEvidence> evidence;
          evidence.reserve(p1_candidate_traces.size());
          for (const auto &trace : p1_candidate_traces)
            evidence.push_back(toP1CandidateEvidence(trace));
          P1CandidateEvidence incumbent_evidence;
          const P1CandidateEvidence *incumbent = nullptr;
          if (has_existing_trajectory)
          {
            for (auto &trace : p1_candidate_traces)
              trace.markIncumbentAvailable();
            incumbent = &incumbent_evidence;
            // This marker makes a missing per-candidate shared-window tuple
            // reject closed instead of falling back to unequal full profiles.
            incumbent_evidence.replacement_comparison_available = true;
            // Candidate optimizations clear their temporary optimizer
            // context. Re-bind the immutable planning snapshot solely for a
            // shared forward-time replacement comparison. Candidate
            // self-descent/ranking remains on each full fixed-200 profile.
            bspline_optimizer_->setRiskSnapshot(
                planning_snapshot, planning_query_base_time_s);
            const double incumbent_remaining_duration_s = std::max(
                0.0, local_data_.duration_ - incumbent_start_t_s);
            for (std::size_t index = 0; index < evidence.size(); ++index)
            {
              const auto candidate_it = std::find_if(
                  p2_candidates.begin(), p2_candidates.end(),
                  [&evidence, index](const auto &candidate) {
                    return candidate.candidate_id ==
                        static_cast<int>(evidence[index].candidate_id);
                  });
              if (candidate_it == p2_candidates.end())
                continue;
              UniformBspline candidate(
                  candidate_it->control_points, 3, ts);
              const double comparison_duration_s = std::min(
                  candidate.getTimeSum(), incumbent_remaining_duration_s);
              if (!(comparison_duration_s > 0.0))
                continue;
              const auto candidate_summary =
                  bspline_optimizer_->evaluateP1FixedLatticeRisk(
                      candidate, 0.0, comparison_duration_s);
              const auto incumbent_summary =
                  bspline_optimizer_->evaluateP1FixedLatticeRisk(
                      local_data_.position_traj_, incumbent_start_t_s,
                      comparison_duration_s);
              if (!candidate_summary.full_support)
                continue;
              if (!incumbent_summary.full_support)
              {
                evidence[index].replacement_incumbent_collision_infeasible =
                    incumbent_summary.occupied_sample_count > 0 &&
                    incumbent_summary.evidence_miss_count == 0;
                continue;
              }
              evidence[index].replacement_comparison_available = true;
              evidence[index].replacement_mean_c_pi =
                  candidate_summary.mean_c_pi;
              evidence[index].replacement_max_c_pi =
                  candidate_summary.max_c_pi;
              evidence[index].replacement_incumbent_mean_c_pi =
                  incumbent_summary.mean_c_pi;
              evidence[index].replacement_incumbent_max_c_pi =
                  incumbent_summary.max_c_pi;
              incumbent_evidence.full_support = true;
              incumbent_evidence.pre_mean_c_pi = incumbent_summary.mean_c_pi;
              incumbent_evidence.post_mean_c_pi = incumbent_summary.mean_c_pi;
              incumbent_evidence.pre_max_c_pi = incumbent_summary.max_c_pi;
              incumbent_evidence.post_max_c_pi = incumbent_summary.max_c_pi;
              auto &trace = p1_candidate_traces[index];
              trace.incumbent_mean_c_pi = incumbent_summary.mean_c_pi;
              trace.incumbent_max_c_pi = incumbent_summary.max_c_pi;
              trace.replacement_comparison_mode =
                  "shared_forward_time_window";
              trace.replacement_comparison_duration_s = comparison_duration_s;
              trace.replacement_candidate_mean_c_pi =
                  candidate_summary.mean_c_pi;
              trace.replacement_candidate_max_c_pi =
                  candidate_summary.max_c_pi;
            }
            bspline_optimizer_->clearRiskSnapshot();
          }
          const auto decisions = selectP1Candidates(evidence, incumbent);
          for (std::size_t index = 0; index < p1_candidate_traces.size(); ++index)
          {
            auto &trace = p1_candidate_traces[index];
            const auto &decision = decisions[index];
            trace.selected = decision.selected;
            trace.selection_score = trace.post_total_objective;
            trace.selection_reason = decision.selection_reason;
            trace.candidate_rank = decision.rank;
            trace.p1_descent = decision.p1_descent;
            trace.rank_eligible = decision.rank_eligible;
            trace.replacement_accepted = decision.replace_published_trajectory;
            trace.replacement_reason = decision.replacement_reason;
            if (decision.selected)
            {
              // Candidate optimization leaves the shared planning context on
              // the last candidate that ran. Rebind it to the actual winner
              // before any replacement/rejection lifecycle evidence is
              // emitted so every downstream identity names the same result.
              set_p1_context(trace.candidate_id);
              selected_p1_candidate_id = trace.candidate_id;
              for (const auto &candidate : p2_candidates)
              {
                if (candidate.candidate_id == static_cast<int>(trace.candidate_id))
                {
                  ctrl_pts = candidate.control_points;
                  break;
                }
              }
              p1_preference_rejected = p1_objective_allowed &&
                  !p1_config.metrics_only && has_existing_trajectory &&
                  !decision.replace_published_trajectory;
              if (p1_preference_rejected)
              {
                last_p1_rejection_reason_ = decision.replacement_reason;
                last_p1_rejection_requires_new_generation_ = true;
                appendPlanningRiskContextTimeline(
                    "replacement", plannerNow().seconds(), "rejected",
                    decision.replacement_reason, "existing_trajectory");
                // Preserve the selected-but-rejected candidate and the
                // incumbent on the identical fixed 200-sample lattice.  The
                // accepted-profile artifact remains reserved for publishes.
                bspline_optimizer_->setRiskSnapshot(
                    planning_snapshot, planning_query_base_time_s);
                const UniformBspline selected_candidate(
                    ctrl_pts, 3, ts);
                bspline_optimizer_->writeP1CandidateRetainedProfile(
                    selected_candidate, trace.planning_attempt_id, trace.candidate_id,
                    &local_data_.position_traj_, local_data_.traj_id_,
                    "retained_incumbent", incumbent_start_t_s);
                bspline_optimizer_->writeP1ReplacementDecision(
                    trace, local_data_.traj_id_, local_data_.start_time_.seconds(),
                    "retained_incumbent",
                    "incumbent:" + std::to_string(local_data_.traj_id_));
                bspline_optimizer_->clearRiskSnapshot();
              }
            }
            trace.fanout = bspline_optimizer_->lastP1FanoutDiagnostics();
            trace.fanout.optimizer_success_count =
                static_cast<int>(p2_candidates.size());
            trace.fanout.full_support_count = static_cast<int>(std::count_if(
                p1_candidate_traces.begin(), p1_candidate_traces.end(),
                [](const auto &item) { return item.support_full_valid; }));
            trace.fanout.p1_descent_eligible_count = static_cast<int>(std::count_if(
                decisions.begin(), decisions.end(),
                [](const auto &item) { return item.rank_eligible; }));
            trace.fanout.optimizer_selected_candidate =
                std::to_string(selected_p1_candidate_id);
            trace.fanout.replacement_acceptance =
                decision.replace_published_trajectory ? "accepted" : "rejected";
          }
          if (p1_preference_rejected)
          {
            for (const auto &trace : p1_candidate_traces)
              bspline_optimizer_->writeP1OptimizationTrace(trace);
          }
          else
          {
            p1_candidate_traces_deferred = true;
          }
        }
      }
      else
      {
        // Failed optimizations are still evidence for an admitted candidate;
        // write one definitive unselected row for each of them.
        for (const auto &trace : p1_candidate_traces)
        {
          auto failed_trace = trace;
          failed_trace.selected = false;
          failed_trace.selection_score = failed_trace.post_total_objective;
          failed_trace.selection_reason = "no_successful_candidate";
          failed_trace.fanout = bspline_optimizer_->lastP1FanoutDiagnostics();
          failed_trace.fanout.optimizer_success_count = 0;
          failed_trace.fanout.full_support_count = static_cast<int>(std::count_if(
              p1_candidate_traces.begin(), p1_candidate_traces.end(),
              [](const auto &item) { return item.support_full_valid; }));
          failed_trace.fanout.optimizer_selected_candidate = "none";
          failed_trace.fanout.replacement_acceptance = "not_evaluated";
          bspline_optimizer_->writeP1OptimizationTrace(failed_trace);
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      set_p1_context(selected_p1_candidate_id);
      bool normalized_p1_stage = p1_objective_allowed &&
          !p1_config.metrics_only && p1_config.lambda_integrity != 0.0;
      BsplineOptimizer::P1BasePrepassTrace base_prepass;
      ControlPoints p1_seed;
      double normalized_final_cost = 0.0;
      bool base_prepass_ready = !normalized_p1_stage;
      bool base_candidate_ready = base_prepass_ready;
      if (normalized_p1_stage)
      {
        appendPlanningRiskContextTimeline(
            "base_prepass_start", plannerNow().seconds(), "started", "ok");
        const ControlPoints topology_seed = bspline_optimizer_->getControlPoints();
        Eigen::MatrixXd base_points;
        double base_cost = 0.0;
        const bool base_success =
            bspline_optimizer_->BsplineOptimizeTrajBasePrepass(
                base_points, base_cost, topology_seed, ts);
        base_prepass = bspline_optimizer_->getLastP1BasePrepassTrace();
        bool base_full_support = false;
        if (base_success)
        {
          base_full_support = bspline_optimizer_->evaluateP1FixedLatticeRisk(
              UniformBspline(base_points, 3, ts)).full_support;
          p1_seed = bspline_optimizer_->getControlPoints();
        }
        base_prepass_ready = base_success && base_full_support;
        const auto prepass_fallback = decideP1BasePrepassFallback({
            base_success, base_full_support, has_existing_trajectory,
            has_p1_preference_incumbent_});
        normalized_p1_stage =
            prepass_fallback.action == P1SoftFallbackAction::USE_P1_CANDIDATE;
        base_candidate_ready = normalized_p1_stage ||
            prepass_fallback.action ==
                P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE;
        p1_objective_allowed = prepass_fallback.objective_allowed;
        p1_fallback_reason = base_prepass_ready ? "none" : prepass_fallback.reason;
        planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
        planning_risk_context_.p1_objective_applied = false;
        planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
        write_p1_candidate_trace =
            p1_objective_allowed || p1_fallback_reason == "metrics_only";
        appendPlanningRiskContextTimeline(
            "base_prepass_end", plannerNow().seconds(),
            base_prepass_ready ? "candidate_success" : "candidate_failure",
            !base_success ? "optimizer_failure" :
            !base_full_support ? "fixed_support_not_full" : "ok");
        appendPlanningRiskContextTimeline(
            "p1_admission", plannerNow().seconds(),
            base_prepass_ready ? "p1_objective" : "base_fallback",
            p1_fallback_reason,
            base_prepass_ready ? "none" : "p1_soft_fallback");
        if (prepass_fallback.action ==
                P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY &&
            has_p1_preference_incumbent_)
        {
          p1_preference_rejected = true;
          last_p1_rejection_reason_ = prepass_fallback.reason;
          last_p1_rejection_requires_new_generation_ = true;
          appendPlanningRiskContextTimeline(
              "replacement", plannerNow().seconds(), "rejected",
              prepass_fallback.reason, "existing_trajectory");
        }
        p1_seed.points = base_points;
        set_p1_context(selected_p1_candidate_id);
      }
      planning_risk_context_.optimizer_start_s = plannerNow().seconds();
      appendPlanningRiskContextTimeline(
          write_p1_candidate_trace ? "optimizer_start" : "base_optimizer_start",
          planning_risk_context_.optimizer_start_s, "started", "ok");
      std::string optimizer_reason = "ok";
      flag_step_1_success = base_candidate_ready && (normalized_p1_stage
          ? bspline_optimizer_->BsplineOptimizeTrajNormalizedP1(
                ctrl_pts, normalized_final_cost, p1_seed, ts, base_prepass,
                &optimizer_reason)
          : bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts));
      planning_risk_context_.optimizer_end_s = plannerNow().seconds();
      appendPlanningRiskContextTimeline(
          write_p1_candidate_trace ? "optimizer_end" : "base_optimizer_end",
          planning_risk_context_.optimizer_end_s,
          flag_step_1_success ? "candidate_success" : "candidate_failure",
          flag_step_1_success ? "ok" :
          !base_prepass_ready ? "base_prepass_failed" :
          optimizer_reason == "ok" ? "optimizer_failure" : optimizer_reason);
      auto trace = bspline_optimizer_->getLastP1OptimizationTrace();
      P1CandidateEvidence candidate_evidence;
      candidate_evidence.planning_attempt_id = trace.planning_attempt_id;
      candidate_evidence.candidate_id = trace.candidate_id;
      candidate_evidence.snapshot_generation_id = trace.snapshot_generation_id;
      candidate_evidence.pre_base_objective = trace.pre_base_objective;
      candidate_evidence.post_base_objective = trace.post_base_objective;
      candidate_evidence.pre_raw_p1_objective = trace.pre_raw_p1_cost;
      candidate_evidence.post_raw_p1_objective = trace.post_raw_p1_cost;
      candidate_evidence.pre_weighted_p1_objective = trace.pre_weighted_p1_cost;
      candidate_evidence.post_weighted_p1_objective = trace.post_weighted_p1_cost;
      candidate_evidence.pre_total_objective = trace.pre_total_objective;
      candidate_evidence.post_total_objective = trace.post_total_objective;
      candidate_evidence.pre_mean_c_pi = trace.pre_mean_c_pi;
      candidate_evidence.post_mean_c_pi = trace.post_mean_c_pi;
      candidate_evidence.pre_max_c_pi = trace.pre_max_c_pi;
      candidate_evidence.post_max_c_pi = trace.post_max_c_pi;
      candidate_evidence.gradient_dot_displacement =
          trace.grad_integrity_dot_displacement;
      candidate_evidence.optimization_success = flag_step_1_success;
      candidate_evidence.full_support = trace.support_full_valid;
      P1CandidateEvidence incumbent_evidence;
      const P1CandidateEvidence *incumbent = nullptr;
      if (has_existing_trajectory)
      {
        trace.markIncumbentAvailable();
        incumbent = &incumbent_evidence;
        // Require the candidate-specific shared-window tuple below.  If the
        // risk evaluation is incomplete, replacement rejects closed.
        incumbent_evidence.replacement_comparison_available = true;
        UniformBspline candidate(ctrl_pts, 3, ts);
        const double comparison_duration_s = std::min(
            candidate.getTimeSum(),
            std::max(0.0, local_data_.duration_ - incumbent_start_t_s));
        if (comparison_duration_s > 0.0)
        {
          const auto candidate_summary =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  candidate, 0.0, comparison_duration_s);
          const auto incumbent_summary =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  local_data_.position_traj_, incumbent_start_t_s,
                  comparison_duration_s);
          if (candidate_summary.full_support && !incumbent_summary.full_support)
          {
            candidate_evidence.replacement_incumbent_collision_infeasible =
                incumbent_summary.occupied_sample_count > 0 &&
                incumbent_summary.evidence_miss_count == 0;
          }
          else if (candidate_summary.full_support && incumbent_summary.full_support)
          {
            // Replacement only compares P1 risk.  Objective fields are kept
            // finite so the policy can diagnose this as an incumbent evidence
            // tuple without inventing a cross-trajectory base-cost ordering.
            incumbent_evidence.full_support = true;
            incumbent_evidence.pre_mean_c_pi = incumbent_summary.mean_c_pi;
            incumbent_evidence.post_mean_c_pi = incumbent_summary.mean_c_pi;
            incumbent_evidence.pre_max_c_pi = incumbent_summary.max_c_pi;
            incumbent_evidence.post_max_c_pi = incumbent_summary.max_c_pi;
            candidate_evidence.replacement_comparison_available = true;
            candidate_evidence.replacement_mean_c_pi = candidate_summary.mean_c_pi;
            candidate_evidence.replacement_max_c_pi = candidate_summary.max_c_pi;
            candidate_evidence.replacement_incumbent_mean_c_pi =
                incumbent_summary.mean_c_pi;
            candidate_evidence.replacement_incumbent_max_c_pi =
                incumbent_summary.max_c_pi;
            trace.incumbent_mean_c_pi = incumbent_summary.mean_c_pi;
            trace.incumbent_max_c_pi = incumbent_summary.max_c_pi;
            trace.replacement_comparison_mode = "shared_forward_time_window";
            trace.replacement_comparison_duration_s = comparison_duration_s;
            trace.replacement_candidate_mean_c_pi = candidate_summary.mean_c_pi;
            trace.replacement_candidate_max_c_pi = candidate_summary.max_c_pi;
          }
        }
      }
      const auto decisions = selectP1Candidates({candidate_evidence}, incumbent);
      const auto &decision = decisions.front();
      trace.selected = decision.selected;
      trace.selection_score = trace.post_total_objective;
      trace.selection_reason = decision.selection_reason;
      trace.candidate_rank = decision.rank;
      trace.p1_descent = decision.p1_descent;
      trace.rank_eligible = decision.rank_eligible;
      trace.replacement_accepted = decision.replace_published_trajectory;
      trace.replacement_reason = decision.replacement_reason;
      // A P1-enabled replan must not overwrite a usable published trajectory
      // merely because the low-weight total objective converged.  This is a
      // publication preference only; it neither calls nor changes P5.
      p1_preference_rejected = flag_step_1_success && p1_objective_allowed &&
          !p1_config.metrics_only && has_existing_trajectory &&
          !decision.replace_published_trajectory;
      if (p1_preference_rejected) {
        last_p1_rejection_reason_ = decision.replacement_reason;
        last_p1_rejection_requires_new_generation_ = true;
        appendPlanningRiskContextTimeline(
            "replacement", plannerNow().seconds(), "rejected",
            decision.replacement_reason, "existing_trajectory");
        bspline_optimizer_->writeP1CandidateRetainedProfile(
            UniformBspline(ctrl_pts, 3, ts), trace.planning_attempt_id,
            trace.candidate_id,
            &local_data_.position_traj_, local_data_.traj_id_,
            "retained_incumbent", incumbent_start_t_s);
        bspline_optimizer_->writeP1ReplacementDecision(
            trace, local_data_.traj_id_, local_data_.start_time_.seconds(),
            "retained_incumbent",
            "incumbent:" + std::to_string(local_data_.traj_id_));
      }
      trace.fanout = bspline_optimizer_->lastP1FanoutDiagnostics();
      trace.fanout.optimizer_success_count = flag_step_1_success ? 1 : 0;
      trace.fanout.full_support_count = trace.support_full_valid ? 1 : 0;
      trace.fanout.p1_descent_eligible_count = decision.rank_eligible ? 1 : 0;
      trace.fanout.optimizer_selected_candidate = std::to_string(trace.candidate_id);
      trace.fanout.replacement_acceptance =
          decision.replace_published_trajectory ? "accepted" : "rejected";
      if (write_p1_candidate_trace)
      {
        p1_candidate_traces.push_back(trace);
        if (p1_preference_rejected || !flag_step_1_success)
          bspline_optimizer_->writeP1OptimizationTrace(trace);
        else
          p1_candidate_traces_deferred = true;
      }
      bspline_optimizer_->clearRiskSnapshot();
      if (safety_viz_)
      {
        safety_viz_->publishP1IntegrityViz(
            toSafetyVizP1Samples(bspline_optimizer_->getLastP1IntegrityVizSamples()),
            toSafetyVizP1Metrics(bspline_optimizer_->getLastP1IntegrityMetrics()),
            plannerNow().seconds());
      }
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (p1_preference_rejected)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = rclcpp::Clock().now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;

    // STEP3 is allowed to change both the control points and the interval.
    // Close the P1 publication decision over that actual final trajectory,
    // on the same immutable snapshot and fixed 200-sample lattice.  This is
    // a soft-preference publication check; collision/feasibility/P5 remain
    // independently authoritative.
    if (p1_candidate_traces_deferred)
    {
      set_p1_context(selected_p1_candidate_id);
      auto selected_trace = std::find_if(
          p1_candidate_traces.begin(), p1_candidate_traces.end(),
          [selected_p1_candidate_id](const auto &trace) {
            return trace.selected &&
                trace.candidate_id == selected_p1_candidate_id;
          });
      if (selected_trace == p1_candidate_traces.end())
      {
        last_p1_rejection_reason_ = "p1_refinement_selected_trace_missing";
        last_p1_rejection_requires_new_generation_ = true;
        appendPlanningRiskContextTimeline(
            "replacement", plannerNow().seconds(), "rejected",
            last_p1_rejection_reason_, "existing_trajectory");
        for (const auto &trace : p1_candidate_traces)
          bspline_optimizer_->writeP1OptimizationTrace(trace);
        bspline_optimizer_->clearRiskSnapshot();
        continous_failures_count_++;
        return false;
      }

      const auto refined_summary =
          bspline_optimizer_->evaluateP1FixedLatticeRisk(pos);
      P1RefinementRiskEvidence refinement_evidence{
          refined_summary.full_support,
          selected_trace->pre_mean_c_pi,
          selected_trace->pre_max_c_pi,
          refined_summary.mean_c_pi,
          refined_summary.max_c_pi,
          has_existing_trajectory,
          selected_trace->incumbent_mean_c_pi,
          selected_trace->incumbent_max_c_pi};
      if (has_existing_trajectory)
      {
        const double comparison_duration_s = std::min(
            pos.getTimeSum(),
            std::max(0.0, local_data_.duration_ - incumbent_start_t_s));
        if (comparison_duration_s > 0.0)
        {
          const auto refined_comparison =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  pos, 0.0, comparison_duration_s);
          const auto incumbent_comparison =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  local_data_.position_traj_, incumbent_start_t_s,
                  comparison_duration_s);
          if (refined_comparison.full_support && incumbent_comparison.full_support)
          {
            refinement_evidence.replacement_comparison_available = true;
            refinement_evidence.replacement_candidate_mean_c_pi =
                refined_comparison.mean_c_pi;
            refinement_evidence.replacement_candidate_max_c_pi =
                refined_comparison.max_c_pi;
            refinement_evidence.replacement_incumbent_mean_c_pi =
                incumbent_comparison.mean_c_pi;
            refinement_evidence.replacement_incumbent_max_c_pi =
                incumbent_comparison.max_c_pi;
            selected_trace->incumbent_mean_c_pi = incumbent_comparison.mean_c_pi;
            selected_trace->incumbent_max_c_pi = incumbent_comparison.max_c_pi;
            selected_trace->replacement_comparison_mode =
                "shared_forward_time_window";
            selected_trace->replacement_comparison_duration_s =
                comparison_duration_s;
            selected_trace->replacement_candidate_mean_c_pi =
                refined_comparison.mean_c_pi;
            selected_trace->replacement_candidate_max_c_pi =
                refined_comparison.max_c_pi;
          }
          else if (refined_comparison.full_support &&
                   incumbent_comparison.occupied_sample_count > 0 &&
                   incumbent_comparison.evidence_miss_count == 0)
          {
            refinement_evidence.replacement_incumbent_collision_infeasible = true;
          }
        }
        // STEP3 replacement evidence is part of full support.  Never reuse
        // the STEP1 tuple after control points or timing have changed.
        if (!refinement_evidence.replacement_comparison_available &&
            !refinement_evidence.replacement_incumbent_collision_infeasible)
          refinement_evidence.full_support = false;
      }
      const auto refinement_decision =
          decideP1RefinementRisk(refinement_evidence);
      selected_trace->replacement_accepted = refinement_decision.accept;
      selected_trace->replacement_reason = refinement_decision.reason;
      selected_trace->fanout.replacement_acceptance =
          refinement_decision.accept ? "accepted" : "rejected";

      if (!refinement_decision.accept)
      {
        last_p1_rejection_reason_ = refinement_decision.reason;
        last_p1_rejection_requires_new_generation_ = true;
        appendPlanningRiskContextTimeline(
            "replacement", plannerNow().seconds(), "rejected",
            refinement_decision.reason,
            has_existing_trajectory ? "existing_trajectory"
                                    : "no_publish_no_incumbent");
        if (has_existing_trajectory)
        {
          bspline_optimizer_->writeP1CandidateRetainedProfile(
              pos, selected_trace->planning_attempt_id,
              selected_trace->candidate_id,
              &local_data_.position_traj_, local_data_.traj_id_,
              "retained_incumbent", incumbent_start_t_s);
          // The decision artifact names the actual refined trajectory risk;
          // optimizer candidate metrics remain unchanged in the candidate
          // table and are recoverable from its profile sidecar.
          auto final_decision_trace = *selected_trace;
          final_decision_trace.post_mean_c_pi = refined_summary.mean_c_pi;
          final_decision_trace.post_max_c_pi = refined_summary.max_c_pi;
          bspline_optimizer_->writeP1ReplacementDecision(
              final_decision_trace, local_data_.traj_id_,
              local_data_.start_time_.seconds(), "retained_incumbent",
              "incumbent:" + std::to_string(local_data_.traj_id_));
        }
        else
        {
          bspline_optimizer_->writeP1CandidateRetainedProfile(
              pos, selected_trace->planning_attempt_id,
              selected_trace->candidate_id, nullptr, 0,
              "no_publish_no_incumbent", 0.0);
          auto final_decision_trace = *selected_trace;
          final_decision_trace.post_mean_c_pi = refined_summary.mean_c_pi;
          final_decision_trace.post_max_c_pi = refined_summary.max_c_pi;
          bspline_optimizer_->writeP1ReplacementDecision(
              final_decision_trace, 0, 0.0, "no_publish_no_incumbent",
              "none");
        }
        for (const auto &trace : p1_candidate_traces)
          bspline_optimizer_->writeP1OptimizationTrace(trace);
        bspline_optimizer_->clearRiskSnapshot();
        continous_failures_count_++;
        return false;
      }

      appendPlanningRiskContextTimeline(
          "replacement", plannerNow().seconds(), "accepted",
          refinement_decision.reason, "refined_candidate");
      for (const auto &trace : p1_candidate_traces)
        bspline_optimizer_->writeP1OptimizationTrace(trace);
    }

    // Bind the final candidate to a newly acquired immutable context tuple,
    // then fail closed before it can mutate LocalTrajData or profile evidence.
    const auto accepted_time = plannerNow();
    set_p1_context(selected_p1_candidate_id);
    std::string freshness_reason;
    const bool objective_applied =
        p1_objective_allowed && p1_config.use_integrity_cost &&
        !p1_config.metrics_only && p1_config.lambda_integrity != 0.0;
    planning_risk_context_.p1_objective_applied = objective_applied;
    if (!planningRiskContextFresh(accepted_time.seconds(), &freshness_reason) &&
        objective_applied)
    {
      last_p1_rejection_reason_ = freshness_reason;
      last_p1_rejection_requires_new_generation_ =
          freshness_reason == "stale_planning_risk_context" ||
          freshness_reason == "planning_risk_context_unavailable";
      appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
          "rejected", freshness_reason, "existing_poly_random_failure_budget");
      bspline_optimizer_->clearRiskSnapshot();
      clearPlanningRiskContext();
      continous_failures_count_++;
      return false;
    }
    const auto accepted_context =
        bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
            pos, accepted_time.seconds(), trajectory_frame_id_);
    if (!accepted_context.valid)
    {
      const auto fallback = decideP1SoftFallback({
          p1_config.metrics_only, objective_applied,
          has_existing_trajectory, accepted_context});
      last_p1_rejection_reason_ = fallback.reason;
      p1_fallback_reason = fallback.reason;
      planning_risk_context_.p1_fallback_reason = fallback.reason;
      planning_risk_context_.p1_objective_applied = false;
      if (!fallback.publish_candidate)
      {
        last_p1_rejection_requires_new_generation_ =
            fallback.retry_base_on_new_generation || !accepted_context.fresh ||
            accepted_context.stale_miss_count > 0;
        appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
            "rejected", last_p1_rejection_reason_,
            fallback.retry_base_on_new_generation
                ? "base_initial_fallback_next_generation"
                : "existing_trajectory");
        bspline_optimizer_->clearRiskSnapshot();
        continous_failures_count_++;
        return false;
      }
      appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
          "base_fallback", fallback.reason, "p1_soft_fallback");
    }
    else
    {
      planning_risk_context_.accepted_s = accepted_time.seconds();
      appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
          objective_applied ? "fresh" : "base_fallback",
          objective_applied ? "ok" : p1_fallback_reason,
          objective_applied ? "none" : "p1_soft_fallback");
    }
    // The formal scene must contain the exact immutable snapshot used by the
    // accepted trajectory.  Periodic RViz throttling is intentionally bypassed
    // for this one evidence publication; the snapshot header retains its own
    // generation stamp and P0 values/occupancy semantics are unchanged.
    if (safety_viz_ && planning_snapshot)
      safety_viz_->publishPredictedPLCloud(
          planning_snapshot, pos.evaluateDeBoorT(0.0).z(),
          accepted_time.seconds(), true);
    updateTrajInfo(pos, accepted_time);
    if (objective_applied)
      has_p1_preference_incumbent_ = true;

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), plannerNow());
    has_p1_preference_incumbent_ = false;

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = plannerNow();

    has_p1_preference_incumbent_ = false;
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWithP3ReferenceBias(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pos,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    if (!p3_config_.enable_global_reference_bias)
    {
      return planGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
    }

    const auto now = plannerNow();
    P3GlobalBiasInput input;
    input.start_pos = start_pos;
    input.end_pos = end_pos;
    input.max_vel = pp_.max_vel_;
    const auto result = computeP3GlobalReferenceBias(
        input, p3_config_, acquireRiskGridSnapshot(),
        [this](const Eigen::Vector3d &pos)
        {
          return grid_map_ && grid_map_->getInflateOccupancy(pos) == 0;
        },
        now.seconds(), ++p3_batch_id_);

    cout << "[P3-global] reason=" << result.reason
         << ", used=" << result.used_bias
         << ", corridor_valid=" << result.corridor_valid_ratio
         << ", detour=" << result.detour_ratio << endl;
    if (safety_viz_)
    {
      SafetyVizP3ReferenceBias viz;
      viz.local = false;
      viz.used_bias = result.used_bias;
      viz.start = result.start_pos;
      viz.end = result.end_pos;
      viz.nominal_target = result.end_pos;
      viz.biased_target =
          result.biased_waypoints.empty() ? result.end_pos
                                          : result.biased_waypoints.back();
      viz.biased_waypoints = result.biased_waypoints;
      viz.improvement_ratio = result.improvement_ratio;
      viz.reason = result.reason;
      safety_viz_->publishP3ReferenceBias(viz, now.seconds());
    }

    if (result.used_bias && !result.biased_waypoints.empty())
    {
      if (planGlobalTrajWaypoints(start_pos, start_vel, start_acc,
                                  result.biased_waypoints, end_vel, end_acc))
      {
        return true;
      }
      cout << "[P3-global] planGlobalTrajWaypoints failed; falling back to original global trajectory" << endl;
    }

    return planGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = plannerNow();

    has_p1_preference_incumbent_ = false;
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
