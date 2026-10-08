// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <iap/util/run_log_manager.hpp>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{
  namespace {
  std::string clearanceText(const double value) {
    if (std::isnan(value)) return "not_measured";
    if (std::isinf(value)) return "no_raw_obstacle_in_scan";
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
  }
  }

  void EGOPlannerManager::recordCurveStage(const std::string& stage,
      const Eigen::MatrixXd& controls, double interval, const LocalTarget& target,
      double feasibility_ratio, const TrajectoryAssessment* assessment, bool optimization_exit,
      std::optional<double> nominal_interval, bool geometry_revision) {
    // Every geometry revision invalidates the previous final assessment, even
    // when export is disabled. An early return must never pair it with a new curve.
    if(geometry_revision) { last_candidate_assessment_={}; last_release_assessment_.reset(); }
    if (!capture_failure_map_ || controls.rows()!=3 || controls.cols()<4 ||
        !controls.allFinite() || !std::isfinite(interval) || interval<=0) return;
    UniformBspline curve(controls,3,interval);
    failed_candidate_curve_=curve;
    CurveStageEvidence evidence{stage,curve,target,
        planning_budget_ ? planning_budget_->elapsed() : 0.,
        planning_budget_ ? planning_budget_->used() : 0,
        optimization_exit ? bspline_optimizer_->lastOptimizationResult() : std::nullopt,
        optimization_exit ? bspline_optimizer_->lastOptimizationReason() : std::string{},
        feasibility_ratio};
    evidence.guide=bspline_optimizer_->recoveryGuide();
    if(nominal_interval) {
      evidence.nominal_interval_s=nominal_interval;
      evidence.guide_sampling_model=kGuideInitializationSamplingModel;
    }
    if(!global_data_.global_traj_.getTimes().empty())
      evidence.terminal_stop=(target.position-global_data_.getPosition(global_data_.global_duration_)).norm()<1e-6;
    if(assessment) {
      evidence.guide_retention=assessment->guide_retention;
      evidence.physical_checked=assessment->physical_epoch && assessment->sampled_points>0;
      evidence.physical_reason=assessment->execution_reason;
      evidence.first_physical_position=assessment->first_execution_position;
      evidence.first_physical_time_s=assessment->first_execution_time_s;
      evidence.physical_generation=assessment->evaluated_generation;
      evidence.physical_evaluation_time_s=assessment->evaluation_time_s;
      evidence.physical_check_scope=assessment->physical_check_scope;
      evidence.first_physical_section=assessment->first_execution_section;
      evidence.first_stopping_distance_m=assessment->first_execution_stopping_distance_m;
    }
    if (curve_stages_.size()<24) curve_stages_.push_back(std::move(evidence));
    else { curve_stages_.back()=std::move(evidence); ++dropped_curve_stages_; }
  }

  void EGOPlannerManager::captureFailureMap(
      const std::string& kind, const Eigen::Vector3d& point,
      const Eigen::Vector3d& other, const GridPlanningCell& cell,
      const AStar::Result* search,
      const BsplineOptimizer::SearchFailureContext* context,
      const UniformBspline* trajectory,
      const TrajectoryAssessment* assessment)
  {
    if (!capture_failure_map_) return;
    auto* artifacts = glim::RunLogManager::get_if_initialized();
    if (!artifacts) {
      RCLCPP_ERROR(node_->get_logger(),
                   "planner failure map capture requires IAP run artifacts");
      return;
    }
    auto snapshot = assessment ? assessment->failure_snapshot :
        std::shared_ptr<const GridMapFailureSnapshot>{};
    // FSM may add tracking/swarm rejection after a successful physical scan.
    // Its immutable epoch still owns the proof; do not recapture a newer map.
    if(!snapshot && assessment && assessment->physical_epoch)
      snapshot=assessment->physical_epoch->failure_evidence;
    if (!snapshot && planning_view_ && (!search ||
        planning_view_->generation == search->occupancy_generation))
      snapshot = planning_view_->snapshot;
    if (!snapshot) {
      const auto live = grid_map_->captureFailureSnapshot(true);
      if (live) snapshot = std::make_shared<const GridMapFailureSnapshot>(*live);
    }
    const uint64_t expected = search ? search->occupancy_generation :
        cell.occupancy_generation;
    if (!snapshot || snapshot->generation != expected ||
        (assessment && (assessment->map_changed ||
         snapshot->generation != assessment->evaluated_generation)) ||
        (cell.occupancy_generation != 0 &&
         snapshot->generation != cell.occupancy_generation)) {
      RCLCPP_ERROR(node_->get_logger(),
          "planner failure map %s capture failed: occupancy evidence mismatch expected=%lu cell=%lu actual=%lu assessment=%lu assessment_map_changed=%d",
          kind.c_str(), static_cast<unsigned long>(expected),
          static_cast<unsigned long>(cell.occupancy_generation),
          static_cast<unsigned long>(snapshot ? snapshot->generation : 0),
          static_cast<unsigned long>(assessment ? assessment->evaluated_generation : 0),
          assessment && assessment->map_changed);
      return;
    }
    std::optional<GridPlanningCell> first_rejection_detail;
    if (search && search->has_first_rejection && planning_view_ &&
        planning_view_->generation == search->occupancy_generation) {
      first_rejection_detail = grid_map_->queryPlanningCell(
          search->first_rejection_position, 0, planning_view_->time_s,
          planning_risk_policy_, planning_view_->motion, true, &planning_view_->physical_context);
    }
    const auto& capture_motion = assessment ? assessment->evaluated_motion :
        (search && planning_view_ ? planning_view_->motion : planning_motion_);
    const double capture_time = assessment ? assessment->evaluation_time_s :
        (search && planning_view_ ? planning_view_->time_s : planning_time_s_);
    const auto search_value = search ? std::optional<AStar::Result>(*search) : std::nullopt;
    const auto context_value = context ? std::optional<BsplineOptimizer::SearchFailureContext>(*context) : std::nullopt;
    const auto curve_value = trajectory ? std::optional<UniformBspline>(*trajectory) : std::nullopt;
    const bool candidate_trace=kind=="attempt_failure" || kind=="attempt_failure_curve" || kind=="candidate";
    const auto stages=candidate_trace ? curve_stages_ : std::vector<CurveStageEvidence>{};
    const unsigned dropped_stages=candidate_trace ? dropped_curve_stages_ : 0;
    const double velocity_limit=pp_.max_vel_,acceleration_limit=pp_.max_acc_;
    const double feasibility_tolerance=pp_.feasibility_tolerance_;
    const auto assessment_value = assessment ? std::optional<TrajectoryAssessment>(*assessment) : std::nullopt;
    const auto attempt_id = planning_attempt_id_;
    const auto start_p = failure_start_p_, start_v = failure_start_v_, start_a = failure_start_a_;
    const int executing_id = local_data_.traj_id_, feedback_id = server_feedback_id_;
    const int candidate_id = pending_trajectory_ ? pending_trajectory_->traj_id_ : 0;
    const double effective_time = connection_time_ ? connection_time_->seconds() : 0.;
    const double budget_elapsed = planning_budget_ ? planning_budget_->elapsed() : 0.;
    const unsigned repairs = planning_budget_ ? planning_budget_->used() : 0;
    const unsigned failure_phase = static_cast<unsigned>(last_plan_failure_);
    const auto guide = bspline_optimizer_ && bspline_optimizer_->a_star_->lastResult().occupancy_generation==snapshot->generation
        ? bspline_optimizer_->recoveryGuide() : std::vector<Eigen::Vector3d>{};
    std::vector<Eigen::Vector3d> goal_positions;
    for(const auto& target : planning_targets_) goal_positions.push_back(target.position);
    const double fitting_reserve_m=.5*snapshot->resolution_m;
    const auto state_json = std::exchange(failure_state_json_, std::string{});
    const auto* checked=candidate_trace ? (assessment ? assessment : last_release_assessment_
        ? &*last_release_assessment_ : &last_candidate_assessment_) : nullptr;
    const bool has_final_check=checked &&
        (checked->sampled_points || checked->execution_reason!=GridExecutionReason::OK) &&
        checked->physical_epoch;
    const std::string final_precondition_reason=checked && !has_final_check &&
        checked->execution_reason!=GridExecutionReason::OK ? gridExecutionReasonName(checked->execution_reason) : "";
    const auto final_check=has_final_check ? std::optional<TrajectoryAssessment>(*checked) : std::nullopt;
    const auto final_snapshot=final_check && final_check->physical_epoch
        ? final_check->physical_epoch->failure_evidence : std::shared_ptr<const GridMapFailureSnapshot>{};
    // Capture the same frozen raw cache on the serialized planning thread.
    // The writer receives values, never a mutable query or a live re-query.
    const auto risk_evidence=planning_view_ && planning_view_->advisory_query.captureEvidence
        ? planning_view_->advisory_query.captureEvidence(*snapshot) : std::optional<GridRiskEvidence>{};
    auto input_binding=std::atomic_load(&planning_input_binding_);
    if(!planning_view_ || !input_binding || input_binding->attempt_id!=attempt_id ||
        input_binding->risk_version!=planning_view_->risk_version ||
        (risk_evidence && input_binding->risk_version!=risk_evidence->risk_version) ||
        input_binding->input.occupancy->generation!=snapshot->generation ||
        input_binding->input.reference_time_s!=capture_time)
      input_binding.reset();
    const auto node = node_;
    auto write = [=](const std::string& label) mutable {
    const auto* search = search_value ? &*search_value : nullptr;
    const auto* context = context_value ? &*context_value : nullptr;
    const auto* trajectory = curve_value ? &*curve_value : nullptr;
    const auto* assessment = assessment_value ? &*assessment_value : nullptr;
    const std::string relative = "planner/failure_map/" + label;

    const auto directory = artifacts->export_path(relative);
    const auto manifest_path = artifacts->metadata_path(
        "manifests/planner_failure_map_" + label + ".json");
    const auto number = [](const double value) {
      if (!std::isfinite(value)) return std::string("null");
      std::ostringstream text;
      text << std::setprecision(17) << value;
      return text.str();
    };
    const auto vector = [&number](const Eigen::Vector3d& value) {
      return "[" + number(value.x()) + "," + number(value.y()) +
          "," + number(value.z()) + "]";
    };
    const auto retention = [&](const BsplineOptimizer::GuideRetention& value) {
      std::ostringstream out;
      out << "{\"source_contribution_coverage\":\"not_available\",\"checked\":" << (value.checked ? "true" : "false")
          << ",\"budget_exhausted\":" << (value.budget_exhausted ? "true" : "false")
          << ",\"comparable_valid_risk\":" << (value.comparable_valid_risk ? "true" : "false")
          << ",\"comparable_model_cost\":" << (value.comparable_model_cost ? "true" : "false")
          << ",\"risk_version\":" << value.risk_version
          << ",\"guide_length_m\":" << number(value.guide_length_m)
          << ",\"curve_length_m\":" << number(value.curve_length_m)
          << ",\"guide_risk_cost_m\":" << number(value.guide_risk_cost_m)
          << ",\"curve_risk_cost_m\":" << number(value.curve_risk_cost_m)
          << ",\"guide_valid_fraction\":" << number(value.guide_valid_fraction)
          << ",\"curve_valid_fraction\":" << number(value.curve_valid_fraction)
          << ",\"max_deviation_m\":" << number(value.max_deviation_m)
          << ",\"corridor_m\":" << number(value.corridor_m)
          << ",\"quadrature_uncertainty\":" << number(value.quadrature_uncertainty)
          << ",\"route_lost\":" << (value.route_lost ? "true" : "false")
          << ",\"risk_preference_lost\":" << (value.risk_preference_lost ? "true" : "false")
          << ",\"samples\":" << value.samples << '}';
      return out.str();
    };
    try {
      if (std::filesystem::exists(directory)) {
        if (!std::filesystem::exists(directory / "snapshot.json") ||
            !std::filesystem::exists(directory / "queried_risk.csv") ||
            !std::filesystem::exists(directory / "cells.bin") ||
            std::filesystem::file_size(directory / "cells.bin") !=
                snapshot->cell_flags.size() ||
            (!snapshot->observation_sources.empty() &&
             (!std::filesystem::exists(directory / "observation_sources.bin") ||
              std::filesystem::file_size(directory / "observation_sources.bin") !=
                  snapshot->cell_flags.size())) ||
            (snapshot->current_frame &&
             (!std::filesystem::exists(directory / "current_frame_hits.csv") ||
              !std::filesystem::exists(directory / "current_frame_beams.csv"))) ||
            !std::filesystem::exists(manifest_path))
          throw std::runtime_error("existing failure map artifact is incomplete");
        return;
      }
      const auto pending = artifacts->export_path(relative + ".pending");
      if (std::filesystem::exists(pending))
        throw std::runtime_error("previous pending failure map artifact exists");
      std::filesystem::create_directories(pending);
      // The existing bounded writer owns encoding and disk I/O, outside the
      // planning budget/thread. Both files retain the same planning binding.
      if(input_binding) {
        const auto payload=encodePredictionInput(input_binding->input);
        std::ofstream stream(pending/"planning_input.bin",std::ios::binary);
        stream.write(reinterpret_cast<const char*>(payload.data()),payload.size());
        stream.close(); if(!stream) throw std::runtime_error("planning_input.bin write failed");
      }
      std::ofstream cells(pending / "cells.bin", std::ios::binary);
      cells.write(reinterpret_cast<const char*>(snapshot->cell_flags.data()),
                  snapshot->cell_flags.size());
      cells.close();
      if (!cells) throw std::runtime_error("cells.bin write failed");
      if(final_snapshot && final_snapshot->generation!=snapshot->generation) {
        std::ofstream final_cells(pending/"final_check_cells.bin",std::ios::binary);
        final_cells.write(reinterpret_cast<const char*>(final_snapshot->cell_flags.data()),final_snapshot->cell_flags.size());
        final_cells.close(); if(!final_cells) throw std::runtime_error("final_check_cells.bin write failed");
      }
      std::ofstream risk(pending / "queried_risk.csv");
      risk << "address,hpl_m,vpl_m,status,version,source_flags,gnss_raw_valid,gnss_geometry_status\n";
      risk << std::setprecision(17);
      for (const auto& sample : risk_evidence ? risk_evidence->queried_risk : snapshot->queried_risk)
        risk << sample.address << ',' << sample.value.hpl << ','
             << sample.value.vpl << ','
             << static_cast<unsigned>(sample.value.status) << ','
             << sample.value.version << ',' << sample.value.source_flags << ','
             << sample.value.gnss_raw_valid << ','
             << static_cast<unsigned>(sample.value.gnss_geometry_status) << '\n';
      risk.close();
      if (!risk) throw std::runtime_error("queried_risk.csv write failed");
      if (!snapshot->observation_sources.empty()) {
        auto sources = snapshot->observation_sources;
        // Replay diagnostic rays after the occupancy lock was released. This
        // preserves sensor callback progress and never expands the live mask.
        if (snapshot->current_frame) {
          RegisteredLidarWindow::Geometry geometry;
          geometry.origin = snapshot->origin;
          geometry.dimensions = snapshot->dimensions;
          geometry.resolution_m = snapshot->resolution_m;
          geometry.frame_contract_id = snapshot->current_frame->frame_contract_id;
          RegisteredLidarWindow replay(geometry);
          const auto raw = replay.unthinnedObservationMask(*snapshot->current_frame);
          for (size_t i = 0; i < sources.size(); ++i)
            if (raw[i]) sources[i] |= 128;
        }
        std::ofstream evidence(pending / "observation_sources.bin", std::ios::binary);
        evidence.write(reinterpret_cast<const char*>(sources.data()), sources.size());
        evidence.close();
        if (!evidence) throw std::runtime_error("observation_sources.bin write failed");
      }
      if (snapshot->current_frame) {
        const auto& frame = *snapshot->current_frame;
        std::ofstream hits(pending / "current_frame_hits.csv");
        hits << std::setprecision(17) << "lidar_x,lidar_y,lidar_z,map_x,map_y,map_z\n";
        for (const auto& hit : frame.hits_lidar) {
          const Eigen::Vector3d world = frame.T_map_lidar * hit;
          hits << hit.x() << ',' << hit.y() << ',' << hit.z() << ','
               << world.x() << ',' << world.y() << ',' << world.z() << '\n';
        }
        hits.close();
        std::ofstream beams(pending / "current_frame_beams.csv");
        beams << std::setprecision(17)
              << "lidar_dx,lidar_dy,lidar_dz,outcome,range_m,map_dx,map_dy,map_dz\n";
        for (const auto& beam : frame.beams) {
          const Eigen::Vector3d world = frame.T_map_lidar.linear() * beam.direction_lidar;
          beams << beam.direction_lidar.x() << ',' << beam.direction_lidar.y() << ','
                << beam.direction_lidar.z() << ',' << static_cast<unsigned>(beam.outcome)
                << ',' << beam.range_m << ',' << world.x() << ',' << world.y() << ','
                << world.z() << '\n';
        }
        beams.close();
        if (!hits || !beams) throw std::runtime_error("current frame evidence write failed");
      }
      if (!state_json.empty()) {
        std::ofstream state(pending / "state.json"); state << state_json;
        state.close(); if (!state) throw std::runtime_error("state.json write failed");
      }
      std::ofstream metadata(pending / "snapshot.json");
      metadata << "{\n  \"schema_version\": \"iap_gridmap_failure_v3\",\n"
          << "  \"kind\": " << std::quoted(kind) << ",\n"
          << "  \"planning_attempt_id\": " << attempt_id << ",\n"
          << "  \"planning_input_file\": " << (input_binding ? "\"planning_input.bin\"" : "null") << ",\n"
          << "  \"planning_input_risk_version\": " << (input_binding ? std::to_string(input_binding->risk_version) : "null") << ",\n"
          << "  \"run_id\": " << std::quoted(artifacts->run_dir().filename().string()) << ",\n"
          << "  \"run_manifest\": \"../../../../metadata/run_manifest.json\",\n"
          << "  \"artifact_label\": " << std::quoted(label) << ",\n"
          << "  \"executing_trajectory_id\": " << executing_id << ",\n"
          << "  \"server_feedback_id\": " << feedback_id << ",\n"
          << "  \"candidate_trajectory_id\": " << (candidate_id>0 ? std::to_string(candidate_id) : "null") << ",\n"
          << "  \"expected_effective_time_s\": " << number(effective_time) << ",\n"
          << "  \"real_start_p_m\": " << vector(start_p) << ",\n"
          << "  \"real_start_v_mps\": " << vector(start_v) << ",\n"
          << "  \"real_start_a_mps2\": " << vector(start_a) << ",\n"
          << "  \"shared_budget_elapsed_s\": " << number(budget_elapsed) << ",\n"
          << "  \"shared_budget_repairs\": " << repairs << ",\n"
          << "  \"plan_failure_phase\": " << failure_phase << ",\n"
          << "  \"guide_m\": [";
      for(size_t i=0;i<guide.size();++i) metadata << (i ? "," : "") << vector(guide[i]);
      metadata << "],\n  \"guide_fitting_reserve_m\": " << number(fitting_reserve_m)
          << ",\n  \"guide_reserve_taper_distance_m\": 0.5,\n  \"planning_goals_m\": [";
      for(size_t i=0;i<goal_positions.size();++i) metadata << (i ? "," : "") << vector(goal_positions[i]);
      metadata << "],\n"
          << "  \"frame_id\": " << std::quoted(snapshot->frame_id) << ",\n"
          << "  \"generation\": " << snapshot->generation << ",\n"
          << "  \"cloud_stamp_s\": " << number(snapshot->cloud_stamp_s) << ",\n"
          << "  \"planning_time_s\": " << number(capture_time) << ",\n"
          << "  \"environment_max_age_s\": "
          << number(capture_motion.max_environment_age_s) << ",\n"
          << "  \"motion_quality\": "
          << static_cast<unsigned>(capture_motion.quality) << ",\n"
          << "  \"origin_m\": " << vector(snapshot->origin) << ",\n"
          << "  \"max_boundary_m\": " << vector(snapshot->max_boundary) << ",\n"
          << "  \"dimensions\": [" << snapshot->dimensions.x() << ','
          << snapshot->dimensions.y() << ',' << snapshot->dimensions.z()
          << "],\n  \"resolution_m\": " << number(snapshot->resolution_m)
          << ",\n  \"virtual_ceiling_height_m\": " << number(snapshot->virtual_ceiling_height_m)
          << ",\n  \"inflation_radius_m\": " << number(snapshot->inflation_radius_m)
          << ",\n  \"cell_flags_file\": \"cells.bin\",\n"
          << "  \"cell_flag_bits\": {\"raw\": 1, \"inflated\": 2, \"observed\": 4},\n"
          << "  \"risk_version\": " << (risk_evidence ? risk_evidence->risk_version : snapshot->risk_version) << ",\n"
          << "  \"risk_context_matches_map\": "
          << ((risk_evidence ? risk_evidence->risk_context_matches_map : snapshot->risk_context_matches_map) ? "true" : "false") << ",\n"
          << "  \"risk_reference_time_s\": "
          << number(risk_evidence ? risk_evidence->risk_reference_time_s : snapshot->risk_reference_time_s) << ",\n"
          << "  \"risk_valid_until_s\": "
          << number(risk_evidence ? risk_evidence->risk_valid_until_s : snapshot->risk_valid_until_s) << ",\n"
          << "  \"risk_samples_file\": \"queried_risk.csv\",\n"
          << "  \"risk_samples_authority\": "
          << std::quoted(risk_evidence ? "FROZEN_PLANNING_QUERY_CACHE" : "GLOBAL_GRIDMAP_CACHE_HISTORY") << ",\n"
          << "  \"motion_allow_bridged\": "
          << (capture_motion.allow_bridged ? "true" : "false") << ",\n"
          << "  \"motion_stamp_s\": " << number(capture_motion.stamp_s) << ",\n"
          << "  \"motion_error_proxy_m\": "
          << number(capture_motion.error_proxy_m) << ",\n"
          << "  \"motion_body_radius_m\": "
          << number(capture_motion.body_radius_m) << ",\n"
          << "  \"motion_tracking_reserve_m\": "
          << number(capture_motion.tracking_reserve_m) << ",\n"
          << "  \"motion_budget_m\": "
          << number(capture_motion.motion_budget_m) << ",\n"
          << "  \"motion_max_age_s\": "
          << number(capture_motion.max_motion_age_s) << ",\n"
          << "  \"failure_position_m\": " << vector(point) << ",\n"
          << "  \"failure_voxel_index\": [" << cell.voxel_index.x() << ','
          << cell.voxel_index.y() << ',' << cell.voxel_index.z() << "],\n"
          << "  \"other_endpoint_m\": " << vector(other) << ",\n"
          << "  \"execution_reason\": "
          << std::quoted(gridExecutionReasonName(cell.execution_reason)) << ",\n"
          << "  \"required_clearance_m\": " << number(cell.required_clearance_m) << ",\n"
          << "  \"nearest_raw_center_distance_m\": "
          << number(cell.raw_center_clearance_m) << ",\n"
          << "  \"nearest_raw_center_m\": " << vector(cell.nearest_raw_center)
          << ",\n  \"advisory_class\": "
          << static_cast<unsigned>(cell.advisory.classification) << ",\n"
          << "  \"search_failure\": "
          << (search ? std::string("\"") + AStar::failureName(search->failure) + "\""
                     : "null") << ",\n"
          << "  \"search_map_changed\": " << (search && search->map_changed ? "true" : "false") << ",\n"
          << "  \"search_generation\": " << (search ? search->occupancy_generation : 0) << ",\n"
          << "  \"search_live_generation_at_finish\": " << (search ? search->live_generation_at_finish : 0) << ",\n"
          << "  \"search_stage\": "
          << (context ? std::string("\"") + context->stage + "\"" : "null")
          << ",\n  \"search_step_size_m\": "
          << number(search ? search->step_size_m : 0.0)
          << ",\n  \"search_pool_dimensions\": ["
          << (search ? search->pool_dimensions.x() : 0) << ','
          << (search ? search->pool_dimensions.y() : 0) << ','
          << (search ? search->pool_dimensions.z() : 0) << "],\n"
          << "  \"search_pool_center_m\": "
          << vector(search ? search->pool_center : Eigen::Vector3d::Zero())
          << ",\n  \"search_start_lattice_m\": " << (search ? vector(search->start_lattice) : "null")
          << ",\n  \"search_end_lattice_m\": " << (search ? vector(search->end_lattice) : "null")
          << ",\n  \"search_start_attachment_recovered\": " << (search && search->start_attachment_recovered ? "true" : "false")
          << ",\n  \"search_requested_start_m\": "
          << vector(search ? search->requested_start : Eigen::Vector3d::Zero())
          << ",\n  \"search_requested_end_m\": "
          << vector(search ? search->requested_end : Eigen::Vector3d::Zero())
          << ",\n  \"search_first_rejection_position_m\": "
          << (search && search->has_first_rejection
              ? vector(search->first_rejection_position) : "null")
          << ",\n  \"search_first_rejection_reason\": "
          << (search && search->has_first_rejection
              ? std::string("\"") + gridExecutionReasonName(search->first_rejection_cell.execution_reason) + "\"" : "null")
          << ",\n  \"search_first_rejection_advisory_class\": "
          << (search && search->has_first_rejection
              ? std::to_string(static_cast<unsigned>(search->first_rejection_cell.advisory_class)) : "null")
          << ",\n  \"search_first_rejection_required_clearance_m\": "
          << number(first_rejection_detail ? first_rejection_detail->required_clearance_m : NAN)
          << ",\n  \"search_first_rejection_nearest_raw_center_distance_m\": "
          << number(first_rejection_detail ? first_rejection_detail->raw_center_clearance_m : NAN)
          << ",\n  \"search_first_rejection_nearest_raw_center_m\": "
          << (first_rejection_detail ? vector(first_rejection_detail->nearest_raw_center) : "null")
          << ",\n  \"segment_start_index\": "
          << (context ? context->segment_start : -1)
          << ",\n  \"segment_end_index\": "
          << (context ? context->segment_end : -1)
          << ",\n  \"control_points_m\": [";
      if (context) for (size_t i = 0; i < context->control_points.size(); ++i)
        metadata << (i ? "," : "") << vector(context->control_points[i]);
      metadata << "],\n"
          << "  \"search_expanded\": " << (search ? search->expanded : 0)
          << ",\n  \"search_query_calls\": "
          << (search ? search->query_calls : 0) << ",\n"
          << "  \"search_cache_hits\": "
          << (search ? search->cache_hits : 0) << ",\n"
          << "  \"search_performance_diagnostics\": "
          << (search && search->performance_diagnostics ? "true" : "false") << ",\n"
          << "  \"search_advisory_query_calls\": "
          << (search ? search->advisory_query_calls : 0) << ",\n"
          << "  \"search_advisory_refresh_calls\": "
          << (search ? search->advisory_refresh_calls : 0) << ",\n"
          << "  \"search_queue_pushes\": " << (search ? search->queue_pushes : 0) << ",\n"
          << "  \"search_queue_pops\": " << (search ? search->queue_pops : 0) << ",\n"
          << "  \"search_rejected_execution\": [";
      for (size_t i = 0; i < 10; ++i)
        metadata << (i ? "," : "") <<
            (search ? search->rejected_execution[i] : 0);
      metadata << "],\n  \"search_rejected_advisory\": "
               << (search ? search->rejected_advisory : 0) << ",\n"
               << "  \"search_occupancy_query_s\": "
               << number(search ? search->occupancy_query_s : 0.0) << ",\n"
               << "  \"search_clearance_query_s\": "
               << number(search ? search->clearance_query_s : 0.0) << ",\n"
               << "  \"search_advisory_query_s\": "
               << number(search ? search->advisory_query_s : 0.0) << ",\n"
               << "  \"search_duration_s\": "
               << number(search ? search->duration_s : 0.0) << ",\n";
      metadata << "  \"search_path_cost_m\": " << number(search ? search->path_cost : NAN)
          << ",\n  \"search_path_length_m\": " << number(search ? search->path_length_m : NAN)
          << ",\n  \"search_risk_cost_m\": " << number(search ? search->risk_cost_m : NAN)
          << ",\n  \"search_terminal_cost_m\": " << number(search ? search->terminal_cost_m : NAN)
          << ",\n  \"search_optimality_proven\": " << (search && search->optimality_proven ? "true" : "false")
          << ",\n  \"search_budget_exhausted\": " << (search && search->search_budget_exhausted ? "true" : "false")
          << ",\n  \"search_advisory_changed\": " << (search && search->advisory_changed ? "true" : "false") << ",\n";
      metadata << "  \"first_unobserved_time_s\": "
          << number(assessment ? assessment->first_unobserved_time_s : NAN) << ",\n"
          << "  \"first_unobserved_position_m\": "
          << (assessment && std::isfinite(assessment->first_unobserved_time_s)
              ? vector(assessment->first_unobserved_position) : "null") << ",\n"
          << "  \"first_unobserved_voxel_index\": ";
      if (assessment && std::isfinite(assessment->first_unobserved_time_s)) {
        const auto& index = assessment->first_unobserved_cell.voxel_index;
        metadata << '[' << index.x() << ',' << index.y() << ',' << index.z() << ']';
      } else metadata << "null";
      metadata << ",\n  \"curve_evaluation_time_s\": "
          << number(assessment ? assessment->evaluation_time_s : NAN)
          << ",\n  \"curve_execution_reason\": "
          << (assessment ? std::string("\"") +
              gridExecutionReasonName(assessment->execution_reason) + "\"" : "null")
          << ",\n  \"curve_first_execution_time_s\": "
          << number(assessment ? assessment->first_execution_time_s : NAN)
          << ",\n  \"curve_first_execution_position_m\": "
          << (assessment ? vector(assessment->first_execution_position) : "null")
          << ",\n  \"curve_checked_from_time_s\": "
          << number(assessment ? assessment->checked_from_time_s : NAN)
          << ",\n  \"curve_checked_to_time_s\": "
          << number(assessment ? assessment->checked_to_time_s : NAN)
          << ",\n  \"curve_sample_step_s\": "
          << number(assessment ? assessment->sample_step_s : NAN)
          << ",\n  \"actual_curve\": ";
      if (trajectory) {
        auto curve = *trajectory;
        const auto points = curve.getControlPoint();
        const auto knots = curve.getKnot();
        metadata << "{\"degree\":" << knots.size() - points.cols() - 1
                 << ",\"interval_s\":" << number(curve.getInterval())
                 << ",\"control_points_m\":[";
        for (int i = 0; i < points.cols(); ++i)
          metadata << (i ? "," : "") << vector(points.col(i));
        metadata << "],\"knots_s\":[";
        for (int i = 0; i < knots.size(); ++i)
          metadata << (i ? "," : "") << number(knots[i]);
        metadata << "]}";
      } else metadata << "null";
      metadata << ",\n  \"curve_generation_state\": " << std::quoted(curve_value ? "generated" : "not_generated")
          << ",\n  \"final_check_state\": " << std::quoted(candidate_trace ? (final_check ? "checked" : "not_checked") : "not_applicable")
          << ",\n  \"final_check_precondition_reason\": "
          << (final_precondition_reason.empty() ? "null" : std::string("\"")+final_precondition_reason+"\"")
          << ",\n  \"curve_stages_dropped\": " << dropped_stages
          << ",\n  \"curve_stages\": [";
      for(size_t s=0;s<stages.size();++s) {
        const auto& evidence=stages[s]; auto curve=evidence.curve;
        const auto controls=curve.getControlPoint(); const auto knots=curve.getKnot();
        auto velocity=curve.getDerivative(); auto acceleration=velocity.getDerivative();
        double vmax=0,amax=0,vt=0,at=0;
        // Exact component extrema; the independent gate still uses derivative-control bounds.
        for(int k=3;k<knots.size()-4;++k) {
          const double left=knots[k]-knots[3],right=knots[k+1]-knots[3];
          if(right<=left) continue;
          std::vector<double> times{left,right};
          const Eigen::Vector3d a0=acceleration.evaluateDeBoorT(left),a1=acceleration.evaluateDeBoorT(right);
          for(int axis=0;axis<3;++axis) if(std::abs(a1[axis]-a0[axis])>1e-14) {
            const double alpha=-a0[axis]/(a1[axis]-a0[axis]);
            if(alpha>0 && alpha<1) times.push_back(left+(right-left)*alpha);
          }
          for(double t:times) {
            const double v=velocity.evaluateDeBoorT(t).cwiseAbs().maxCoeff();
            const double a=acceleration.evaluateDeBoorT(t).cwiseAbs().maxCoeff();
            if(v>vmax) {vmax=v;vt=t;} if(a>amax) {amax=a;at=t;}
          }
        }
        metadata << (s ? "," : "") << "{\"stage\":" << std::quoted(evidence.stage)
            << ",\"degree\":3,\"interval_s\":" << number(curve.getInterval())
            << ",\"nominal_interval_s\":" << (evidence.nominal_interval_s ? number(*evidence.nominal_interval_s) : "null")
            << ",\"guide_sampling_model\":" << (evidence.guide_sampling_model.empty() ? "null" : std::string("\"")+evidence.guide_sampling_model+"\"")
            << ",\"elapsed_s\":" << number(evidence.elapsed_s) << ",\"repairs\":" << evidence.repairs
            << ",\"solver_result\":" << (evidence.solver_result ? std::to_string(*evidence.solver_result) : "null")
            << ",\"solver_reason\":" << std::quoted(evidence.solver_reason)
            << ",\"feasibility_ratio\":" << number(evidence.feasibility_ratio)
            << ",\"physical_check_reason\":" << (evidence.physical_checked ? std::string("\"")+gridExecutionReasonName(evidence.physical_reason)+"\"" : "null")
            << ",\"physical_precondition_reason\":" << (!evidence.physical_checked && evidence.physical_reason!=GridExecutionReason::OK
                ? std::string("\"")+gridExecutionReasonName(evidence.physical_reason)+"\"" : "null")
            << ",\"first_physical_position_m\":" << vector(evidence.first_physical_position)
            << ",\"first_physical_time_s\":" << number(evidence.first_physical_time_s)
            << ",\"physical_generation\":" << evidence.physical_generation
            << ",\"physical_evaluation_time_s\":" << number(evidence.physical_evaluation_time_s)
            << ",\"physical_check_scope\":" << std::quoted(evidence.physical_check_scope)
            << ",\"first_physical_section\":" << std::quoted(evidence.first_physical_section)
            << ",\"first_stopping_distance_m\":" << number(evidence.first_stopping_distance_m)
            << ",\"target_p_m\":" << vector(evidence.target.position)
            << ",\"target_v_mps\":" << vector(evidence.target.velocity)
            << ",\"target_a_mps2\":" << vector(evidence.target.acceleration)
            << ",\"terminal_stop\":" << (evidence.terminal_stop ? (*evidence.terminal_stop ? "true" : "false") : "null")
            << ",\"max_component_velocity_mps\":" << number(vmax) << ",\"max_velocity_time_s\":" << number(vt)
            << ",\"max_component_acceleration_mps2\":" << number(amax) << ",\"max_acceleration_time_s\":" << number(at)
            << ",\"velocity_control_bound_mps\":" << number(velocity.getControlPoint().cwiseAbs().maxCoeff())
            << ",\"acceleration_control_bound_mps2\":" << number(acceleration.getControlPoint().cwiseAbs().maxCoeff())
            << ",\"velocity_limit_mps\":" << number(velocity_limit) << ",\"acceleration_limit_mps2\":" << number(acceleration_limit)
            << ",\"feasibility_tolerance\":" << number(feasibility_tolerance)
            << ",\"control_points_m\":[";
        for(int i=0;i<controls.cols();++i) metadata << (i ? "," : "") << vector(controls.col(i));
        metadata << "],\"knots_s\":[";
        for(int i=0;i<knots.size();++i) metadata << (i ? "," : "") << number(knots[i]);
        metadata << "],\"guide_m\":[";
        for(size_t i=0;i<evidence.guide.size();++i) metadata << (i ? "," : "") << vector(evidence.guide[i]);
        metadata << "],\"guide_retention\":" << retention(evidence.guide_retention) << '}';
      }
      metadata << "],\n  \"final_check\": ";
      if(final_check) {
        metadata << "{\"execution_reason\":" << std::quoted(gridExecutionReasonName(final_check->execution_reason))
          << ",\"guide_retention\":" << retention(final_check->guide_retention)
          << ",\"generation\":" << final_check->evaluated_generation
          << ",\"evaluation_time_s\":" << number(final_check->evaluation_time_s)
          << ",\"first_position_m\":" << vector(final_check->first_execution_position)
          << ",\"required_clearance_m\":" << number(final_check->first_execution_cell.required_clearance_m)
          << ",\"map_available\":" << (final_snapshot ? "true" : "false");
        metadata << ",\"physical_check_scope\":" << std::quoted(final_check->physical_check_scope)
          << ",\"first_execution_section\":" << std::quoted(final_check->first_execution_section)
          << ",\"first_execution_time_s\":" << number(final_check->first_execution_time_s)
          << ",\"first_execution_stopping_distance_m\":" << number(final_check->first_execution_stopping_distance_m)
          << ",\"first_cell_generation\":" << final_check->first_execution_cell.occupancy_generation
          << ",\"sampled_points\":" << final_check->sampled_points
          << ",\"budget_exhausted\":" << (final_check->budget_exhausted ? "true" : "false")
          << ",\"nearest_raw_center_distance_m\":" << number(final_check->first_execution_cell.raw_center_clearance_m)
          << ",\"motion_stamp_s\":" << number(final_check->evaluated_motion.stamp_s)
          << ",\"motion_error_proxy_m\":" << number(final_check->evaluated_motion.error_proxy_m);
        if(final_snapshot) metadata << ",\"cloud_stamp_s\":" << number(final_snapshot->cloud_stamp_s)
          << ",\"origin_m\":" << vector(final_snapshot->origin)
          << ",\"dimensions\":[" << final_snapshot->dimensions.x() << ',' << final_snapshot->dimensions.y() << ',' << final_snapshot->dimensions.z() << ']'
          << ",\"resolution_m\":" << number(final_snapshot->resolution_m)
          << ",\"cell_flags_file\":" << std::quoted(final_snapshot->generation==snapshot->generation ? "cells.bin" : "final_check_cells.bin");
        metadata << '}';
      } else metadata << "null";
      metadata << ",\n  \"observation_evidence_available\": "
          << (snapshot->observation_evidence_available && snapshot->current_frame ? "true" : "false")
          << ",\n  \"observation_sources_file\": "
          << (snapshot->observation_sources.empty() ? "null" : "\"observation_sources.bin\"")
          << ",\n  \"observation_source_bits\": {\"current_hit\":1,\"current_free\":2,\"active_hit\":4,\"active_free\":8,\"unthinned_current_observed\":128},\n"
          << "  \"observation_loss_producer_mask\": 48,\n"
          << "  \"observation_loss_producers\": {\"1\":\"current_replace\",\"2\":\"active_delta\",\"3\":\"active_replace\"},\n"
          << "  \"active_window_generation\": " << snapshot->active_window_generation << ",\n"
          << "  \"sensor_position_m\": " << vector(snapshot->sensor_position) << ",\n"
          << "  \"vehicle_observed_radius_m\": " << number(snapshot->vehicle_observed_radius_m) << ",\n"
          << "  \"current_frame\": ";
      if (snapshot->current_frame) {
        const auto& frame = *snapshot->current_frame;
        metadata << "{\"frame_id\":" << frame.frame_id
                 << ",\"stamp_s\":" << number(frame.stamp_s)
                 << ",\"scan_end_stamp_s\":" << number(frame.scan_end_stamp_s)
                 << ",\"sensor_position_m\":" << vector(frame.T_map_lidar.translation())
                 << ",\"sensor_model_id\":" << std::quoted(frame.sensor_model_id)
                 << ",\"beam_content_hash\":" << std::quoted(frame.beam_content_hash)
                 << ",\"beam_binding_reason\":" << std::quoted(frame.beam_binding_reason)
                 << ",\"beam_received_count\":" << frame.beam_received_count
                 << ",\"beam_invalid_count\":" << frame.beam_invalid_count
                 << ",\"beam_evicted_count\":" << frame.beam_evicted_count
                 << ",\"beam_history_oldest_stamp_s\":" << number(frame.beam_history_oldest_stamp_s)
                 << ",\"beam_history_newest_stamp_s\":" << number(frame.beam_history_newest_stamp_s)
                 << ",\"beam_same_start_end_stamp_s\":" << number(frame.beam_same_start_end_stamp_s)
                 << ",\"min_range_m\":" << number(frame.min_range_m)
                 << ",\"beam_evidence_complete\":" << (frame.beam_evidence_complete ? "true" : "false")
                 << ",\"max_range_m\":" << number(frame.max_range_m)
                 << ",\"hits_file\":\"current_frame_hits.csv\",\"beams_file\":\"current_frame_beams.csv\"}";
      } else metadata << "null";
      metadata << "\n}\n";
      metadata.close();
      if (!metadata) throw std::runtime_error("snapshot.json write failed");
      std::filesystem::rename(pending, directory);
      std::filesystem::create_directories(manifest_path.parent_path());
      const auto manifest_pending = manifest_path.string() + ".pending";
      std::ofstream manifest(manifest_pending);
      manifest << "{\"schema_version\":\"iap_planner_failure_artifact_v3\","
          << "\"kind\":" << std::quoted(kind) << ","
          << "\"snapshot\":" << std::quoted(relative + "/snapshot.json")
          << ",\"cells\":" << std::quoted(relative + "/cells.bin")
          << ",\"risk_samples\":"
          << std::quoted(relative + "/queried_risk.csv");
      if(input_binding) manifest << ",\"planning_input\":"<<std::quoted(relative+"/planning_input.bin")
          <<",\"planning_attempt_id\":"<<input_binding->attempt_id<<",\"risk_version\":"<<input_binding->risk_version;
      if (!snapshot->observation_sources.empty())
        manifest << ",\"observation_sources\":"
                 << std::quoted(relative + "/observation_sources.bin");
      if(final_snapshot && final_snapshot->generation!=snapshot->generation)
        manifest << ",\"final_check_cells\":" << std::quoted(relative+"/final_check_cells.bin");
      if (snapshot->current_frame)
        manifest << ",\"current_frame_hits\":"
                 << std::quoted(relative + "/current_frame_hits.csv")
                 << ",\"current_frame_beams\":"
                 << std::quoted(relative + "/current_frame_beams.csv");
      if (!state_json.empty())
        manifest << ",\"state\":" << std::quoted(relative + "/state.json");
      manifest << "}\n";
      manifest.close();
      if (!manifest) throw std::runtime_error("subordinate manifest write failed");
      std::filesystem::rename(manifest_pending, manifest_path);
      RCLCPP_INFO(node->get_logger(),
          "planner failure map %s saved at %s generation=%lu voxels=%zu",
          kind.c_str(), directory.c_str(), static_cast<unsigned long>(snapshot->generation),
          snapshot->cell_flags.size());
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node->get_logger(),
          "planner failure map %s save failed: %s", kind.c_str(), error.what());
    }
    };
    // Intermediate rejections may be repaired in this same attempt. Only a
    // final disposition replaces terminal proof while a PlanningView is active.
    if(!planning_view_ || kind=="attempt_failure" || kind=="attempt_failure_curve") latest_failure_export_ = write;
    if (captured_failure_kinds_.insert(kind).second)
      queueFailureExport([write, kind]() mutable { write(kind); }, false);
  }

  void EGOPlannerManager::capturePlanningStall(
      const Eigen::Vector3d& start, const Eigen::Vector3d& target) {
    if (!capture_failure_map_) return;
    // Export the last rejected frozen attempt before capturing this later state.
    exportLatestFailure();
    if (captured_failure_kinds_.count("stall")) return;
    planning_time_s_ = node_->now().seconds();
    planning_motion_ = currentMotionContext();
    planning_risk_version_ = 0;
    const auto cell = grid_map_->queryPlanningCell(target, 0,
        planning_time_s_, planning_risk_policy_, planning_motion_, true);
    std::ostringstream state;
    state << "{\"schema_version\":\"iap_planner_stall_state_v1\",\"target_reason\":"
          << std::quoted(gridExecutionReasonName(cell.execution_reason)) << "}\n";
    failure_state_json_ = state.str();
    captureFailureMap("stall", target, start, cell);
  }

  std::optional<GridPlanningCell> EGOPlannerManager::queryAssessmentCell(
      const TrajectoryAssessment& assessment,const Eigen::Vector3d& position) const {
    if(!assessment.physical_epoch) return std::nullopt;
    const auto context=grid_map_->preparePlanningQuery(assessment.evaluation_time_s,
        assessment.evaluated_motion,assessment.physical_epoch);
    return grid_map_->queryPlanningCell(position,0,assessment.evaluation_time_s,
        planning_risk_policy_,assessment.evaluated_motion,true,&context);
  }

  void EGOPlannerManager::captureRemainingFailure(
      const std::string& kind, const Eigen::Vector3d& expected,
      const Eigen::Vector3d& actual, const double error_m,
      const int trajectory_id, const double command_time_s,
      const double odom_age_s, const double map_age_s,
      const GridExecutionReason reason, const TrajectoryAssessment* assessment) {
    if (!capture_failure_map_) return;
    auto* evidence_curve=&local_data_.position_traj_;
    TrajectoryAssessment curve_assessment;
    if(assessment && pending_trajectory_ && assessment->trajectory_id==pending_trajectory_->traj_id_) {
      // Supervision uses the executing start as a shared lead-time origin;
      // saved curve samples must use the owning pending curve's local time.
      curve_assessment=*assessment;
      const double offset=pending_trajectory_->start_time_.seconds()-local_data_.start_time_.seconds();
      curve_assessment.first_execution_time_s-=offset;
      curve_assessment.first_unobserved_time_s-=offset;
      curve_assessment.first_advisory_time_s-=offset;
      assessment=&curve_assessment;
      evidence_curve=&pending_trajectory_->position_traj_;
    }
    // A previously captured physical failure must not conceal a later hole
    // on the executing curve.
    if (assessment && std::isfinite(assessment->first_unobserved_time_s))
      captureFailureMap("curve_unobserved", assessment->first_unobserved_position,
          expected, assessment->first_unobserved_cell, nullptr, nullptr,
          evidence_curve, assessment);
    planning_time_s_ = node_->now().seconds();
    planning_motion_ = currentMotionContext();
    planning_risk_version_ = 0;
    const auto assessed_cell=assessment ? queryAssessmentCell(*assessment,actual) : std::nullopt;
    auto cell=assessed_cell ? *assessed_cell : grid_map_->queryPlanningCell(actual,0,
        planning_time_s_,planning_risk_policy_,planning_motion_,true);
    Eigen::Vector3d point = actual;
    if (assessment && assessment->first_execution_position.allFinite()) {
      point = assessment->first_execution_position;
      cell = assessment->first_execution_cell;
    }
    std::ostringstream state;
    state << std::setprecision(17)
          << "{\"schema_version\":\"iap_planner_stop_state_v1\","
          << "\"reason\":" << std::quoted(gridExecutionReasonName(reason))
          << ",\"time_s\":" << planning_time_s_
          << ",\"capture_ros_time_s\":" << planning_time_s_
          << ",\"assessment_time_s\":" << (assessment ? assessment->evaluation_time_s : planning_time_s_)
          << ",\"assessment_generation\":" << (assessment ? assessment->evaluated_generation : cell.occupancy_generation)
          << ",\"expected_position_m\":[" << expected.x() << ','
          << expected.y() << ',' << expected.z() << ']'
          << ",\"glio_position_m\":[" << actual.x() << ','
          << actual.y() << ',' << actual.z() << ']'
          << ",\"error_m\":" << error_m
          << ",\"trajectory_id\":" << trajectory_id
          << ",\"tracking_reference_trajectory_id\":" << trajectory_id
          << ",\"tracking_reference_error_m\":" << error_m
          << ",\"failed_curve_id\":" << (assessment ? assessment->trajectory_id : trajectory_id)
          << ",\"last_command_time_s\":"
          << (std::isfinite(command_time_s) ? command_time_s : -1.0)
          << ",\"command_age_s\":"
          << (std::isfinite(command_time_s)
              ? planning_time_s_ - command_time_s : -1.0)
          << ",\"glio_age_s\":" << odom_age_s
          << ",\"map_age_s\":" << map_age_s << "}\n";
    failure_state_json_ = state.str();
    captureFailureMap(kind, point, expected, cell, nullptr, nullptr,
        assessment ? evidence_curve : nullptr, assessment);
    exportLatestFailure();
  }

  EGOPlannerManager::EGOPlannerManager() {}

  void EGOPlannerManager::queueFailureExport(std::function<void()> job, bool terminal) {
    std::lock_guard<std::mutex> lock(failure_writer_mutex_);
    if (!failure_writer_.joinable()) {
      failure_writer_ = std::thread([this]() {
        std::unique_lock<std::mutex> lock(failure_writer_mutex_);
        while (true) {
          failure_writer_cv_.wait(lock, [this]() { return failure_writer_stopping_ || !failure_exports_.empty(); });
          if (failure_exports_.empty() && failure_writer_stopping_) break;
          auto work = std::move(failure_exports_.front()); failure_exports_.pop_front();
          failure_writer_busy_ = true; lock.unlock();
          try { work(); }
          catch (const std::exception& error) {
            RCLCPP_ERROR(node_->get_logger(), "planner failure export worker failed: %s", error.what());
          }
          lock.lock();
          failure_writer_busy_ = false; failure_writer_cv_.notify_all();
        }
      });
    }
    if (failure_exports_.size() == 2) {
      if (!terminal) {
        RCLCPP_WARN(node_->get_logger(), "failure export queue full: first-kind export omitted; latest failure retained");
        return;
      }
      failure_exports_.pop_front();
      RCLCPP_WARN(node_->get_logger(), "failure export queue full: terminal evidence supersedes queued export");
    }
    failure_exports_.push_back(std::move(job)); failure_writer_cv_.notify_one();
  }

  void EGOPlannerManager::exportLatestFailure(bool final) {
    if (!latest_failure_export_ || (!final && terminal_exports_ >= 3)) return;
    const auto label = final ? "terminal_final" : "terminal_" + std::to_string(++terminal_exports_);
    auto write = latest_failure_export_;
    queueFailureExport([write, label]() mutable { write(label); }, true);
  }

  void EGOPlannerManager::drainFailureExports() {
    std::unique_lock<std::mutex> lock(failure_writer_mutex_);
    failure_writer_cv_.wait(lock, [this]() { return failure_exports_.empty() && !failure_writer_busy_; });
  }

  EGOPlannerManager::~EGOPlannerManager() {
    exportLatestFailure(true);
    { std::lock_guard<std::mutex> lock(failure_writer_mutex_); failure_writer_stopping_ = true; }
    failure_writer_cv_.notify_one();
    if (failure_writer_.joinable()) failure_writer_.join();
  }

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);

    node->declare_parameter("manager/drone_id", -1);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    pp_.use_distinctive_trajs = false; // This rebuild has one route per attempt.
    node->get_parameter("manager/drone_id", pp_.drone_id);

    local_data_.traj_id_ = 0;
    local_data_.duration_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);
    node_ = node;
    initRiskInputs(node);
    initPredictionExport();
    capture_failure_map_ = node->declare_parameter(
        "planning/capture_failure_map", false);
    grid_map_->setFailureEvidenceCapture(capture_failure_map_);
    search_performance_diagnostics_ = node->declare_parameter(
        "planning/search_performance_diagnostics", false);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    const auto searcher_started = std::chrono::steady_clock::now();
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    planning_timings_.searcher_initialization_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - searcher_started).count();
    bspline_optimizer_->a_star_->setPerformanceDiagnostics(search_performance_diagnostics_);
    if (advisory_guidance_enabled_) bspline_optimizer_->a_star_->setAdvisoryQuery(
        [this](const Eigen::Vector3d& position) {
          return guidancePreference(queryPlanningViewAdvisory(position));
        }, [this]() {
          return planning_view_ ? planning_view_->advisory_stats : GridPlanningQueryStats{};
        });
    bspline_optimizer_->a_star_->setLiveGenerationProvider(
        [this]() { return grid_map_->occupancyGeneration(); });
    bspline_optimizer_->setSearchFailureObserver(
        [this](const AStar::Result& result,
               const BsplineOptimizer::SearchFailureContext& context) {
          if (capture_failure_map_) { failed_search_result_=result; failed_search_context_=context; }
          const auto diagnostic_map = grid_map_;
          const auto* diagnostic_context = planning_view_ ? &planning_view_->physical_context : nullptr;
          const double diagnostic_time = planning_view_ ? planning_view_->time_s : planning_time_s_;
          const auto& diagnostic_motion = planning_view_ ? planning_view_->motion : planning_motion_;
          const auto end = diagnostic_map->queryPlanningCell(
              result.requested_end, planning_risk_version_, diagnostic_time,
              planning_risk_policy_, diagnostic_motion, true, diagnostic_context);
          if (result.has_first_rejection && !capture_failure_map_) {
            const auto first = diagnostic_map->queryPlanningCell(
                result.first_rejection_position, 0, diagnostic_time,
                planning_risk_policy_, diagnostic_motion, true, diagnostic_context);
            RCLCPP_DEBUG(node_->get_logger(),
                "A* first rejection reason=%s advisory=%u required=%s nearest=%s generation=%lu",
                gridExecutionReasonName(result.first_rejection_cell.execution_reason),
                static_cast<unsigned>(result.first_rejection_cell.advisory_class),
                clearanceText(first.required_clearance_m).c_str(),
                clearanceText(first.raw_center_clearance_m).c_str(),
                static_cast<unsigned long>(first.occupancy_generation));
          }
          const auto start = diagnostic_map->queryPlanningCell(
              result.requested_start, planning_risk_version_, diagnostic_time,
              planning_risk_policy_, diagnostic_motion, true, diagnostic_context);
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
              "A* endpoints failure=%s start_reason=%s start_required=%s start_nearest=%s start_nearest_xyz=(%.3f %.3f %.3f) start_observed=%d end_reason=%s end_required=%s end_nearest=%s end_nearest_xyz=(%.3f %.3f %.3f) end_observed=%d end_advisory=%u generation=%lu cloud=%.3f",
              AStar::failureName(result.failure),
              gridExecutionReasonName(start.execution_reason),
              clearanceText(start.required_clearance_m).c_str(),
              clearanceText(start.raw_center_clearance_m).c_str(),
              start.nearest_raw_center.x(), start.nearest_raw_center.y(),
              start.nearest_raw_center.z(),
              start.observed,
              gridExecutionReasonName(end.execution_reason),
              clearanceText(end.required_clearance_m).c_str(),
              clearanceText(end.raw_center_clearance_m).c_str(),
              end.nearest_raw_center.x(), end.nearest_raw_center.y(),
              end.nearest_raw_center.z(), end.observed,
              static_cast<unsigned>(end.advisory.classification),
              static_cast<unsigned long>(end.occupancy_generation),
              end.cloud_stamp_s);
          std::string kind = "endpoint";
          if (result.failure == AStar::Failure::TIME_BUDGET)
            kind = "timeout";
          else if (result.failure == AStar::Failure::MAP_STALE)
            kind = "map_changed";
          else if (result.failure == AStar::Failure::NO_PATH ||
                   result.failure == AStar::Failure::NO_PATH_WITH_UNOBSERVED ||
                   result.failure == AStar::Failure::ADVISORY_NO_PATH)
            kind = "exhausted";
          if (capture_failure_map_)
            captureFailureMap(kind, result.requested_end,
                              result.requested_start, end, &result, &context);
          if (result.map_changed && kind != "map_changed")
            captureFailureMap("map_changed", result.requested_end,
                result.requested_start, end, &result, &context);
        });

    visualization_ = vis;
  }

  void EGOPlannerManager::setPlanningConnection(rclcpp::Time start_time, int predecessor_id) {
    connection_time_=start_time; connection_predecessor_=predecessor_id;
  }

  std::optional<int> EGOPlannerManager::requestPendingWithdrawal() {
    if(!pending_trajectory_) return std::nullopt;
    if(!pending_withdrawal_requested_s_) pending_withdrawal_requested_s_=node_->now().seconds();
    return pending_trajectory_->traj_id_;
  }

  void EGOPlannerManager::observeExecutingTrajectory(int trajectory_id,double command_time_s) {
    server_feedback_id_ = trajectory_id;
    if(pending_trajectory_ && pending_trajectory_->traj_id_==trajectory_id) {
      local_data_=*pending_trajectory_; pending_trajectory_.reset();
      pending_withdrawal_requested_s_.reset();
      RCLCPP_INFO(node_->get_logger(),"Trajectory %d executing at its scheduled connection",trajectory_id);
    } else if(pending_trajectory_ && pending_withdrawal_requested_s_ &&
        trajectory_id==local_data_.traj_id_ && std::isfinite(command_time_s) &&
        command_time_s<=node_->now().seconds() &&
        command_time_s>=*pending_withdrawal_requested_s_ &&
        command_time_s>=pending_trajectory_->start_time_.seconds()) {
      // Only a real predecessor command after both withdrawal and activation
      // time proves the server kept the old curve. Before-start/stale/foreign
      // IDs cannot acknowledge cancellation. This grants no motion authority.
      RCLCPP_INFO(node_->get_logger(),"Pending trajectory %d withdrawal confirmed by predecessor %d command at %.6f",
          pending_trajectory_->traj_id_,trajectory_id,command_time_s);
      pending_trajectory_.reset();pending_withdrawal_requested_s_.reset();
    }
  }

  bool EGOPlannerManager::publicationStillTimely() const {
    const double now=node_->now().seconds();
    return planning_view_ && planning_budget_ && !planning_budget_->expired() &&
        now>=planning_view_->time_s && (!connection_time_ || connection_time_->seconds()-now>=.1);
  }

  void EGOPlannerManager::discardUnpublishedTrajectory(const LocalTrajData& predecessor) {
    if(pending_trajectory_) pending_trajectory_.reset();
    else local_data_=predecessor;
    pending_withdrawal_requested_s_.reset();
    last_plan_failure_=planning_budget_ && planning_budget_->expired() ? PlanFailure::Budget : PlanFailure::Connection;
  }

  void EGOPlannerManager::recordTargetSelectionFailure(const Eigen::Vector3d& start,
      const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
      const Eigen::Vector3d& requested_target) {
    last_plan_failure_=planning_budget_ && (planning_budget_->expired() || planning_budget_->denied())
        ? PlanFailure::Budget : PlanFailure::Target;
    failure_start_p_=start;failure_start_v_=velocity;failure_start_a_=acceleration;
    if(capture_failure_map_ && planning_view_ && planning_view_->snapshot) {
      const auto cell=queryPlanningViewCell(requested_target);
      captureFailureMap("attempt_failure",requested_target,start,cell);
    }
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
      Eigen::Vector3d start_acc, Eigen::Vector3d target_pt, Eigen::Vector3d target_vel,
      bool /* polynomial_init */, bool /* random_polynomial */) {
    const bool own_view=!planning_view_;
    if(own_view && !beginPlanningView()) return false;
    struct EndView { EGOPlannerManager* manager; bool own;
      ~EndView() { if(own) manager->endPlanningView(); } } end_view{this,own_view};
    last_plan_failure_=PlanFailure::None;
    failure_start_p_=start_pt; failure_start_v_=start_vel; failure_start_a_=start_acc;
    const auto fail=[&](PlanFailure reason) {
      last_plan_failure_=planning_budget_->expired() || planning_budget_->denied() ? PlanFailure::Budget : reason;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Planner rebound rejected: phase=%s execution=%s budget_expired=%d repair_denied=%d",
          last_plan_failure_==PlanFailure::Budget ? "budget" : last_plan_failure_==PlanFailure::Target ? "target" :
          last_plan_failure_==PlanFailure::Search ? "search" : last_plan_failure_==PlanFailure::Curve ? "curve" :
          last_plan_failure_==PlanFailure::Release ? "release" : "connection", gridExecutionReasonName(last_release_assessment_ ? last_release_assessment_->execution_reason : last_candidate_assessment_.execution_reason),
          planning_budget_->expired(), planning_budget_->denied());
      if(capture_failure_map_ && planning_view_ && planning_view_->snapshot) {
        // Final disposition of this attempt, including backend/budget failures
        // that happen after an earlier search rejection. Do not read a later map.
        const auto& latest=bspline_optimizer_->a_star_->lastResult();
        const auto* search=latest.occupancy_generation==planning_view_->generation ? &latest : nullptr;
        // A failed-search observer context belongs only to that exact search.
        // A later successful guide must not inherit an earlier repair segment.
        const bool matches=search && failed_search_result_ &&
            search->failure==failed_search_result_->failure &&
            search->duration_s==failed_search_result_->duration_s &&
            search->requested_start.isApprox(failed_search_result_->requested_start,0.) &&
            search->requested_end.isApprox(failed_search_result_->requested_end,0.);
        const auto* context=matches && failed_search_context_ ? &*failed_search_context_ : nullptr;
        // Base input/search/guide evidence retains the original planning epoch.
        // The independent release assessment below owns final_check and, when
        // needed, final_check_cells.bin; its time/motion never replaces inputs.
        const auto cell=queryPlanningViewCell(start_pt);
        const auto* assessment=!last_release_assessment_ && last_candidate_assessment_.failure_snapshot &&
            last_candidate_assessment_.evaluated_generation==planning_view_->generation ? &last_candidate_assessment_ : nullptr;
        captureFailureMap(reason==PlanFailure::Curve ? "attempt_failure_curve" : "attempt_failure",start_pt,target_pt,cell,search,context,
            failed_candidate_curve_ ? &*failed_candidate_curve_ : nullptr,assessment);
      }
      ++continous_failures_count_; return false;
    };
    if(pending_trajectory_ || planning_budget_->expired()) return fail(PlanFailure::Connection);
    if(!start_pt.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
       !target_pt.allFinite() || !target_vel.allFinite()) return fail(PlanFailure::Target);
    if(planning_targets_.empty()) planning_targets_.push_back({target_pt,target_vel,Eigen::Vector3d::Zero(),0});
    planning_time_s_=planning_view_->time_s; planning_risk_version_=planning_view_->risk_version;
    planning_motion_=planning_view_->motion;
    auto& optimizer=*bspline_optimizer_;
    optimizer.a_star_->clearLastResult();
    optimizer.a_star_->setSearchMap(grid_map_);
    optimizer.a_star_->setFrozenEpoch(planning_view_->physical);
    const auto query=[this](const Eigen::Vector3d& point) { return queryGuidanceCell(point); };
    const auto start_cell=query(start_pt);
    if(!start_cell.executable()) {
      optimizer.a_star_->recordPresearchFailure(
          start_cell.execution_reason==GridExecutionReason::ENVIRONMENT_STALE ? AStar::Failure::MAP_STALE :
          start_cell.execution_reason==GridExecutionReason::CURRENT_MOTION_UNAVAILABLE ||
          start_cell.execution_reason==GridExecutionReason::CURRENT_MOTION_STALE ||
          start_cell.execution_reason==GridExecutionReason::CURRENT_MOTION_BUDGET ? AStar::Failure::CURRENT_MOTION :
          AStar::Failure::START_BLOCKED,start_pt,target_pt);
      if(capture_failure_map_) captureFailureMap("endpoint",start_pt,target_pt,start_cell);
      return fail(PlanFailure::Connection);
    }
    const double fitting_reserve=.5*grid_map_->getResolution();
    // Use the same epoch and physical query with additional fitting room.
    // Taper to the exact boundary states; legal starts/goals stay connectable.
    const auto guide_query=[this,start_pt,fitting_reserve](const Eigen::Vector3d& point) {
      double endpoint_distance=(point-start_pt).norm();
      for(const auto& target:planning_targets_) endpoint_distance=std::min(endpoint_distance,(point-target.position).norm());
      return queryGuidanceCell(point,fitting_reserve*std::clamp(endpoint_distance/.5,0.,1.));
    };
    optimizer.setPlanningQuery(query,false,guide_query);
    const auto& epoch=*planning_view_->physical;
    Eigen::Vector3d upper=epoch.lattice_origin+epoch.extent_m;
    if(epoch.virtual_ceiling_height_m>0) upper.z()=std::min(upper.z(),epoch.virtual_ceiling_height_m);
    optimizer.setCurvePhysicalBounds(epoch.lattice_origin+Eigen::Vector3d::Constant(1e-4),
        upper-Eigen::Vector3d::Constant(1e-4));
    optimizer.setPlanningBudget(planning_budget_);
    optimizer.setPlanningEndpoints(start_pt,target_pt);
    std::vector<Eigen::Vector3d> goals;
    std::vector<size_t> target_indices;
    for(size_t i=0;i<planning_targets_.size();++i) {
      goals.push_back(planning_targets_[i].position); target_indices.push_back(i);
    }
    const Eigen::Vector3d target_region_center=planning_target_center_.value_or((start_pt+target_pt)/2);
    optimizer.setPlanningGoals(goals,target_region_center);
    optimizer.a_star_->setTaskGoal(global_data_.global_traj_.getTimes().empty() ? target_pt :
        global_data_.getPosition(global_data_.global_duration_));
    LocalTarget selected{target_pt,target_vel,Eigen::Vector3d::Zero(),0};
    double interval=std::max(.05,pp_.ctrl_pt_dist/std::max(.1,pp_.max_vel_)*1.5);
    const double nominal_guide_interval=interval; // Fresh-guide time owner, independent of fit/retime outputs.
    std::vector<Eigen::Vector3d> points;
    if((target_pt-start_pt).norm()<.2) return fail(PlanFailure::Target);
    Eigen::MatrixXd control;
    const auto bind_boundaries=[&]() {
      UniformBspline::enforceBoundaryStates(control,interval,start_pt,start_vel,start_acc,
          selected.position,selected.velocity,selected.acceleration);
      optimizer.setLocalTargetPt(selected.position);
      optimizer.setBsplineInterval(interval);
    };
    // OFF and ON share one whole-route search on every local planning round.
    // Establish a valid control owner for failure evidence before the search.
    optimizer.initializeFromGuide(Eigen::MatrixXd::Zero(3,7));
    if(!optimizer.searchRecoveryGuide()) return fail(PlanFailure::Search);
    const auto initialize_guide=[&]() {
      const auto& guide=optimizer.recoveryGuide();
      if(guide.size()<2) return false;
      const auto index=optimizer.a_star_->lastResult().selected_goal;
      if(index<target_indices.size()) selected=planning_targets_[target_indices[index]];
      selected.position=guide.back();
      optimizer.setPlanningEndpoints(start_pt,selected.position);
      optimizer.setPlanningGoals(goals,target_region_center);
      const bool terminal_stop=!global_data_.global_traj_.getTimes().empty() ?
          (selected.position-global_data_.getPosition(global_data_.global_duration_)).norm()<1e-6 :
          selected.velocity.norm()<1e-9;
      if(!fitGuideCurve(guide,start_vel,start_acc,terminal_stop,selected,nominal_guide_interval,interval,points,control)) return false;
      recordCurveStage("guide_fit",control,interval,selected,NAN,nullptr,false,nominal_guide_interval);
      bind_boundaries(); recordCurveStage("guide_bound",control,interval,selected);
      optimizer.initializeFromGuide(control); return true;
    };
    if(optimizer.needsGuideReinitialization() && !initialize_guide()) return fail(PlanFailure::Search);
    const auto shorten_target=[&]() {
      if(target_indices.size()<2 || !planning_budget_->tryRepair(PlanningBudget::Repair::TargetShortening)) return false;
      const auto failed=std::find_if(target_indices.begin(),target_indices.end(),[&](size_t i) {
        return planning_targets_[i].position.isApprox(selected.position,1e-8);
      });
      if(failed==target_indices.end()) return false;
      target_indices.erase(failed); goals.clear();
      for(size_t i:target_indices) goals.push_back(planning_targets_[i].position);
      optimizer.setPlanningGoals(goals,target_region_center);
      // Existing ordered forward targets, one guide at a time, shared budget.
      return optimizer.searchRecoveryGuide() && initialize_guide();
    };
    TrajectoryAssessment assessment;
    UniformBspline curve;
    for(;;) {
      if(planning_budget_->expired()) return fail(PlanFailure::Budget);
      // Fit/retime can leave a legal guide. Constrain the actual initial curve
      // before either solver; the independent final/release checks still own authorization.
      const auto initial=assessTrajectory(UniformBspline(control,3,interval),0,planning_view_->time_s,
          false,0,std::numeric_limits<double>::infinity(),&planning_view_->physical_context);
      recordCurveStage("initial_check",control,interval,selected,NAN,&initial);
      if(initial.budget_exhausted) return fail(PlanFailure::Budget);
      optimizer.addCurveClearanceConstraints(control,interval,initial.curve_clearance_violations);
      const auto backend_start=PlanningBudget::Clock::now();
      const bool optimized=optimizer.BsplineOptimizeTrajRebound(control,interval);
      recordCurveStage(optimized ? "optimized" : "optimized_failed",control,interval,selected,NAN,nullptr,true);
      if(!optimized) {
        if(shorten_target()) continue;
        return fail(PlanFailure::Curve);
      }
      planning_timings_.backend_s+=std::chrono::duration<double>(PlanningBudget::Clock::now()-backend_start).count();
      bind_boundaries();
      recordCurveStage("optimized_bound",control,interval,selected);
      bool feasible=false;
      for(int retime=0;retime<4;++retime) {
        if(planning_budget_->expired()) return fail(PlanFailure::Budget);
        curve=UniformBspline(control,3,interval);
        curve.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
        double ratio=1;
        const bool dynamics_ok=curve.checkFeasibility(ratio,false);
        recordCurveStage(dynamics_ok ? "dynamics_pass" : "dynamics_fail",control,interval,selected,ratio);
        if(dynamics_ok) { feasible=true; break; }
        // Reconstruct a uniform spline and rebind physical derivatives after
        // stretching. A raw lengthenTime would silently change both endpoints.
        if(retime==3) break; // The last checked candidate remains the failure authority.
        interval*=std::max(1.1,ratio*1.05); bind_boundaries();
        recordCurveStage("retimed_bound",control,interval,selected);
        // Rebinding physical P/V/A moves endpoint controls; unchanged interior
        // controls can retain a super-limit boundary derivative indefinitely.
        // Rebind constraints to this parameterization and refine the same guide.
        optimizer.rebindAfterUniformRetime(control);
        const auto retimed=assessTrajectory(UniformBspline(control,3,interval),0,planning_view_->time_s,
            false,0,std::numeric_limits<double>::infinity(),&planning_view_->physical_context);
        if(retimed.budget_exhausted) return fail(PlanFailure::Budget);
        if(!retimed.curve_clearance_violations.empty() &&
            !planning_budget_->tryRepair(PlanningBudget::Repair::CurveCorrection)) return fail(PlanFailure::Budget);
        optimizer.addCurveClearanceConstraints(control,interval,retimed.curve_clearance_violations);
        const bool provisional=optimizer.BsplineOptimizeTrajRefine(control,interval,control);
        recordCurveStage(provisional ? "refined" : "refined_provisional_rejected",control,interval,selected,NAN,nullptr,true);
        if(planning_budget_->expired()) return fail(PlanFailure::Budget);
        if(!optimizer.lastOptimizationTerminatedNormally()) return fail(PlanFailure::Curve);
        // A provisional physical rejection is never authorization. The complete
        // independent final check below owns rejection and bounded correction.
        bind_boundaries(); recordCurveStage("refined_bound",control,interval,selected);
      }
      if(!feasible) return fail(PlanFailure::Curve);
      const auto check_start=PlanningBudget::Clock::now();
      assessment=assessTrajectory(curve,planning_view_->risk_version,planning_view_->time_s,
          false,0,std::numeric_limits<double>::infinity(),&planning_view_->physical_context);
      assessment.guide_retention=optimizer.assessGuideRetention(control,interval,
          [this](const Eigen::Vector3d& p) {return queryPlanningViewAdvisory(p);});
      assessment.budget_exhausted=assessment.budget_exhausted || assessment.guide_retention.budget_exhausted;
      recordCurveStage("route_checked",control,interval,selected,NAN,&assessment);
      planning_timings_.final_checks_s+=std::chrono::duration<double>(PlanningBudget::Clock::now()-check_start).count();
      last_candidate_assessment_=assessment;
      if(capture_failure_map_) failed_candidate_curve_=curve;
      if(assessment.budget_exhausted) return fail(PlanFailure::Budget);
      if(!assessment.guide_retention.checked) return fail(PlanFailure::Curve);
      // Terminal speed is rechecked after optimization against the same input.
      if(selected.velocity.norm()>terminalSpeedLimit(selected.position,selected.velocity)+1e-6)
        return fail(PlanFailure::Target);
      const bool advisory_violation=advisory_guidance_enabled_ && assessment.advisory_avoid_samples && !optimizer.advisoryFallbackUsed();
      const bool route_loss=assessment.guide_retention.route_lost ||
          (advisory_guidance_enabled_ && assessment.guide_retention.risk_preference_lost);
      if(assessment.executable() && !advisory_violation && !route_loss) break;
      if(!assessment.executable() && assessment.execution_reason!=GridExecutionReason::PHYSICAL_OBSTACLE &&
          assessment.execution_reason!=GridExecutionReason::INSUFFICIENT_CLEARANCE &&
          assessment.execution_reason!=GridExecutionReason::ENVIRONMENT_UNOBSERVED &&
          assessment.execution_reason!=GridExecutionReason::OUT_OF_MAP) return fail(PlanFailure::Curve);
      if(capture_failure_map_ && assessment.first_execution_position.allFinite())
        captureFailureMap("candidate",assessment.first_execution_position,selected.position,
            assessment.first_execution_cell,nullptr,nullptr,&curve,&assessment);
      if(optimizer.recoveryGuide().empty()) {
        if(!optimizer.searchRecoveryGuide() || !initialize_guide()) return fail(PlanFailure::Search);
      } else {
        const auto correction=correctCurveCandidate(optimizer,control,interval,assessment);
        if(correction!=PlanFailure::None) {
          if(correction==PlanFailure::Curve && shorten_target()) continue;
          return fail(correction);
        }
      }
    }
    const bool advisory_downgraded=!std::isfinite(planning_view_->advisory_valid_until_s) ||
        node_->now().seconds()>planning_view_->advisory_valid_until_s || grid_map_->occupancyGeneration()!=planning_view_->generation;
    if(assessment.advisory_unknown_samples || optimizer.advisoryFallbackUsed() || advisory_downgraded)
      RCLCPP_INFO(node_->get_logger(),"Trajectory advisory degraded: frozen_unknown=%zu fallback=%d historical_or_unavailable=%d",
          assessment.advisory_unknown_samples,optimizer.advisoryFallbackUsed(),
          advisory_downgraded);
    // Capture the latest relevant corridor once. Remote updates are harmless;
    // changes within this corridor get at most one budgeted recapture.
    bool committed=false;
    for(int capture=0;capture<2;++capture) {
      if(planning_budget_->expired()) return fail(PlanFailure::Budget);
      const double now=node_->now().seconds();
      if(now<planning_view_->time_s || !grid_map_->geometryMatches(*planning_view_->physical)) return fail(PlanFailure::Connection);
      if(connection_time_ && (connection_time_->seconds()-now<.1 || local_data_.traj_id_!=connection_predecessor_))
        return fail(PlanFailure::Connection);
      auto release=assessTrajectory(curve,0,now);
      recordCurveStage("release_curve_checked",control,interval,selected,NAN,&release,false,std::nullopt,false);
      last_release_assessment_=release;
      if(!release.executable() || !release.physical_epoch) {
        return fail(PlanFailure::Release);
      }
      if(selected.velocity.norm()>terminalSpeedLimit(selected.position,selected.velocity)+1e-6) return fail(PlanFailure::Target);
      {
        // One latest corridor owns all evidence needed through the switch and
        // terminal stopping space. Ordinary map updates outside it are allowed.
        std::vector<ReleasePathSample> samples;
        const double spacing=std::min(.01,grid_map_->getResolution()/(4*std::max(.1,pp_.max_vel_)));
        if(connection_time_) {
          const double from=std::max(0.0,now-local_data_.start_time_.seconds());
          const double to=connection_time_->seconds()-local_data_.start_time_.seconds();
          auto old=local_data_.position_traj_;
          for(double t=from;t<=to+spacing;t+=spacing) {
            if(planning_budget_->expired()) return fail(PlanFailure::Budget);
            samples.push_back({old.evaluateDeBoorT(std::min(t,to)),"predecessor_curve",std::min(t,to)});
          }
        }
        const double duration=curve.getTimeSum();
        for(double t=0;t<=duration+spacing;t+=spacing) {
          if(planning_budget_->expired()) return fail(PlanFailure::Budget);
          samples.push_back({curve.evaluateDeBoorT(std::min(t,duration)),"actual_curve",std::min(t,duration)});
        }
        if(selected.velocity.norm()>1e-9) {
          const double stopping=selected.velocity.squaredNorm()/(2*std::max(.1,pp_.max_acc_))+2*grid_map_->getResolution();
          for(double d=0;d<=stopping+grid_map_->getResolution()*.5;d+=grid_map_->getResolution()*.5) {
            if(planning_budget_->expired()) return fail(PlanFailure::Budget);
            samples.push_back({selected.position+selected.velocity.normalized()*std::min(d,stopping),"terminal_stopping_space",std::min(d,stopping)});
          }
        }
        release=assessReleaseCorridor(samples,now);
        recordCurveStage("release_corridor_checked",control,interval,selected,NAN,&release,false,std::nullopt,false);
        last_release_assessment_=release;
        if(!release.executable() || !release.physical_epoch) return fail(PlanFailure::Release);
      }
      const auto gate=grid_map_->commitFrozenCorridor(*release.physical_epoch,node_->now().seconds(),
          release.evaluated_motion.max_environment_age_s,[&]() {
        if(planning_budget_->expired()) return false;
        const auto current=currentMotionContext();
        const auto odom=latest_odom_provider_ ? latest_odom_provider_() : std::atomic_load(&risk_odom_);
        const double commit_time=node_->now().seconds();
        if(current.quality!=release.evaluated_motion_quality || !std::isfinite(current.error_proxy_m) ||
            current.error_proxy_m>release.evaluated_motion_error_proxy_m+1e-9 ||
            commit_time<current.stamp_s || commit_time-current.stamp_s>current.max_motion_age_s ||
            !odom || odom->header.frame_id!=release.physical_epoch->frame_id) return false;
        const double stamp=rclcpp::Time(odom->header.stamp).seconds();
        if(commit_time<stamp || commit_time-stamp>motion_max_age_s_) return false;
        const auto& p=odom->pose.pose.position;
        const Eigen::Vector3d actual(p.x,p.y,p.z);
        Eigen::Vector3d expected=start_pt;
        if(connection_time_) {
          if(local_data_.traj_id_!=connection_predecessor_ || connection_time_->seconds()-commit_time<.1 ||
              stamp<local_data_.start_time_.seconds()) return false;
          auto old=local_data_.position_traj_;
          expected=old.evaluateDeBoorT(std::clamp(stamp-local_data_.start_time_.seconds(),0.0,local_data_.duration_));
          const double t=connection_time_->seconds()-local_data_.start_time_.seconds();
          if(t<0 || t>local_data_.duration_) return false;
          auto velocity=old.getDerivative(); auto acceleration=velocity.getDerivative();
          if((old.evaluateDeBoorT(t)-start_pt).norm()>1e-6 ||
             (velocity.evaluateDeBoorT(t)-start_vel).norm()>1e-6 ||
             (acceleration.evaluateDeBoorT(t)-start_acc).norm()>1e-6) return false;
        }
        if(!actual.allFinite() || (actual-expected).norm()>motion_start_tolerance_m_) return false;
        if(connection_time_) {
          LocalTrajData candidate; candidate.start_time_=*connection_time_;
          candidate.position_traj_=curve; candidate.velocity_traj_=curve.getDerivative();
          candidate.acceleration_traj_=candidate.velocity_traj_.getDerivative();
          candidate.start_pos_=start_pt; candidate.duration_=curve.getTimeSum();
          next_trajectory_id_=std::max(next_trajectory_id_,local_data_.traj_id_)+1;
          candidate.traj_id_=next_trajectory_id_; pending_trajectory_=candidate;
          pending_withdrawal_requested_s_.reset();
        } else updateTrajInfo(curve,node_->now());
        return true;
      },planning_budget_);
      if(gate==GridMap::CorridorCommit::Committed) { committed=true; break; }
      if(gate!=GridMap::CorridorCommit::Changed || capture ||
          !planning_budget_->tryRepair(PlanningBudget::Repair::PublicationRecheck)) break;
    }
    if(!committed) return fail(PlanFailure::Release);
    continous_failures_count_=0;
    visualization_->displayInitPathList(points,.2,0);
    return true;
  }

  bool EGOPlannerManager::fitGuideCurve(const std::vector<Eigen::Vector3d>& guide,
      const Eigen::Vector3d& start_vel,const Eigen::Vector3d& start_acc,bool terminal_stop,
      LocalTarget& selected,double nominal_interval,double& interval,
      std::vector<Eigen::Vector3d>& points,Eigen::MatrixXd& control) {
    if(!std::isfinite(nominal_interval) || nominal_interval<=0 || guide.size()<2 || !std::isfinite(pp_.ctrl_pt_dist) || pp_.ctrl_pt_dist<=0 ||
        !std::all_of(guide.begin(),guide.end(),[](const Eigen::Vector3d& p){return p.allFinite();}) ||
        !guide.back().isApprox(selected.position,1e-9)) return false;
    std::vector<double> arc(guide.size(),0);
    for(size_t i=1;i<guide.size();++i) arc[i]=arc[i-1]+(guide[i]-guide[i-1]).norm();
    if(arc.back()<1e-6) return false;
    const size_t count=std::max<size_t>(7,std::ceil(arc.back()/pp_.ctrl_pt_dist)+1);
    const auto sample_guide=[&](size_t sample_count) {
      points.clear();size_t segment=1;
      for(size_t i=0;i<sample_count;++i) {
        if(planning_budget_ && planning_budget_->expired()) return false;
        const double d=arc.back()*i/(sample_count-1);
        while(segment+1<arc.size() && arc[segment]<d) ++segment;
        const double length=arc[segment]-arc[segment-1];
        const double alpha=length>1e-9 ? (d-arc[segment-1])/length : 0;
        points.push_back(guide[segment-1]*(1-alpha)+guide[segment]*alpha);
      }
      return true;
    };
    if(!sample_guide(count)) return false;
    selected.velocity.setZero();
    if(!terminal_stop) {
      // The nominal guide sampling owns the terminal approach window.
      // A tiny lattice-to-target connector can point backwards or vertically;
      // treating that connector as a full-speed approach contradicts the
      // resampled curve and forces a loop when the P/V/A triplets are bound.
      const Eigen::Vector3d tangent=points.back()-points[points.size()-2];
      if(tangent.norm()>1e-9) {
        const Eigen::Vector3d direction=tangent.normalized();
        selected.velocity=direction*terminalSpeedLimit(selected.position,direction*pp_.max_vel_);
      }
    }
    interval=std::max(nominal_interval,1.5*arc.back()/(std::max(.1,pp_.max_vel_)*(count-1)));
    // The nominal sampling still owns terminal approach and total duration.
    // Refine the initialization mesh to the physical guide's voxel scale:
    // coarse samples can erase an early turn needed by the exact start P/V/A.
    // Additional controls spend the same shared deadline, never extra repairs.
    if(!grid_map_) return false;
    const double voxel_diagonal=std::sqrt(3.)*grid_map_->getResolution();
    if(!std::isfinite(voxel_diagonal) || voxel_diagonal<=0) return false;
    const size_t subdivisions=std::max<size_t>(1,std::ceil(pp_.ctrl_pt_dist/voxel_diagonal));
    if(subdivisions>1) {
      if(!sample_guide(subdivisions*(count-1)+1)) return false;
      interval/=subdivisions;
    }
    UniformBspline::parameterizeToBspline(interval,points,
        {start_vel,selected.velocity,start_acc,selected.acceleration},control);
    return true;
  }

  EGOPlannerManager::PlanFailure EGOPlannerManager::correctCurveCandidate(
      BsplineOptimizer& optimizer,Eigen::MatrixXd& control,double interval,
      const TrajectoryAssessment& assessment) {
    if(!planning_budget_ || !planning_budget_->tryRepair(PlanningBudget::Repair::CurveCorrection))
      return PlanFailure::Budget;
    const bool route_loss=assessment.guide_retention.route_lost ||
        (advisory_guidance_enabled_ && assessment.guide_retention.risk_preference_lost);
    const bool advisory_violation=advisory_guidance_enabled_ && assessment.advisory_avoid_samples &&
        !optimizer.advisoryFallbackUsed();
    if(route_loss) optimizer.strengthenGuideTracking();
    if(!assessment.curve_clearance_violations.empty()) {
      if(!optimizer.addCurveClearanceConstraints(control,interval,assessment.curve_clearance_violations))
        return PlanFailure::Curve;
      RCLCPP_INFO(node_->get_logger(),"Curve correction: %zu actual clearance violations, fitting reserve=%.3fm",
          assessment.curve_clearance_violations.size(),.5*grid_map_->getResolution());
      optimizer.setControlPoints(control);
    } else if((route_loss || advisory_violation ||
        assessment.execution_reason==GridExecutionReason::ENVIRONMENT_UNOBSERVED ||
        assessment.execution_reason==GridExecutionReason::OUT_OF_MAP) &&
        optimizer.addCurveGuideConstraints(control,interval,assessment.guide_retention.route_lost,
            advisory_guidance_enabled_ && assessment.guide_retention.risk_preference_lost)) {
      optimizer.setControlPoints(control);
    } else {
      if(!route_loss) optimizer.strengthenGuideTracking();
      optimizer.initializeFromGuide(control);
    }
    return PlanFailure::None;
  }

  bool EGOPlannerManager::planCheckedBrake(
      const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
      const Eigen::Vector3d& acceleration)
  {
    if (!position.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite() || pp_.max_acc_ <= 0.0) {
      RCLCPP_WARN(node_->get_logger(),"Checked brake rejected: reason=INVALID_INPUT_OR_ACCELERATION trajectory=%d",local_data_.traj_id_);
      return false;
    }
    const double now = node_->now().seconds();
    const auto risk_version = beginRiskQuery();
    for (int attempt = 0; attempt < 3; ++attempt) {
      const double duration = std::max(0.5,
          2.0 * velocity.norm() / pp_.max_acc_) * std::pow(1.5, attempt);
      const Eigen::Vector3d end = position + velocity * duration * 0.5;
      auto polynomial = PolynomialTraj::one_segment_traj_gen(
          position, velocity, acceleration, end, Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(), duration);
      const double dt = duration / 10.0;
      std::vector<Eigen::Vector3d> samples;
      for (int i = 0; i <= 10; ++i)
        samples.push_back(polynomial.evaluate(i * dt));
      std::vector<Eigen::Vector3d> derivatives{
          velocity, Eigen::Vector3d::Zero(), acceleration,
          Eigen::Vector3d::Zero()};
      Eigen::MatrixXd controls;
      UniformBspline::parameterizeToBspline(dt, samples, derivatives, controls);
      UniformBspline candidate(controls, 3, dt);
      candidate.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                                  pp_.feasibility_tolerance_);
      double ratio = 1.0;
      if (!candidate.checkFeasibility(ratio, false)) {
        RCLCPP_WARN(node_->get_logger(),"Checked brake rejected: attempt=%d reason=DYNAMICS ratio=%.6f trajectory=%d",attempt,ratio,local_data_.traj_id_);
        continue;
      }
      const auto assessment = assessTrajectory(candidate, risk_version,
                                               now, true);
      if (!assessment.executable()) {
        RCLCPP_WARN(node_->get_logger(),
            "Checked brake rejected: attempt=%d reason=%s trajectory=%d generation=%lu evaluation_ros_time_s=%.9f violation_t=%.6f",
            attempt,gridExecutionReasonName(assessment.execution_reason),local_data_.traj_id_,
            assessment.evaluated_generation,assessment.evaluation_time_s,assessment.first_execution_time_s);
        continue;
      }
      updateTrajInfo(candidate, node_->now());
      RCLCPP_WARN(node_->get_logger(),
                  "Checked continuous braking trajectory committed");
      return true;
    }
    return false;
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

    auto time_now = node_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
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

    auto time_now = node_->now();

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
    pending_trajectory_.reset();
    pending_withdrawal_requested_s_.reset();
    next_trajectory_id_=std::max(next_trajectory_id_,local_data_.traj_id_)+1;
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ = next_trajectory_id_;
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
