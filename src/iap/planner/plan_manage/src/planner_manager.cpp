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

  void EGOPlannerManager::captureFailureMap(
      const std::string& kind, const Eigen::Vector3d& point,
      const Eigen::Vector3d& other, const GridPlanningCell& cell,
      const AStar::Result* search,
      const BsplineOptimizer::SearchFailureContext* context,
      const UniformBspline* trajectory,
      const TrajectoryAssessment* assessment)
  {
    if (!capture_failure_map_) return;
    if (captured_failure_kinds_.count(kind)) return;
    auto* artifacts = glim::RunLogManager::get_if_initialized();
    if (!artifacts) {
      RCLCPP_ERROR(node_->get_logger(),
                   "planner failure map capture requires IAP run artifacts");
      return;
    }
    auto snapshot = assessment ? assessment->failure_snapshot :
        std::shared_ptr<const GridMapFailureSnapshot>{};
    if (!snapshot && search && planning_view_ &&
        planning_view_->generation == search->occupancy_generation)
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
    const std::string relative = "planner/failure_map/" + kind;
    const auto directory = artifacts->export_path(relative);
    const auto manifest_path = artifacts->metadata_path(
        "manifests/planner_failure_map_" + kind + ".json");
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
        captured_failure_kinds_.insert(kind);
        return;
      }
      const auto pending = artifacts->export_path(relative + ".pending");
      if (std::filesystem::exists(pending))
        throw std::runtime_error("previous pending failure map artifact exists");
      std::filesystem::create_directories(pending);
      std::ofstream cells(pending / "cells.bin", std::ios::binary);
      cells.write(reinterpret_cast<const char*>(snapshot->cell_flags.data()),
                  snapshot->cell_flags.size());
      cells.close();
      if (!cells) throw std::runtime_error("cells.bin write failed");
      std::ofstream risk(pending / "queried_risk.csv");
      risk << "address,hpl_m,vpl_m,status,version\n";
      for (const auto& sample : snapshot->queried_risk)
        risk << sample.address << ',' << sample.value.hpl << ','
             << sample.value.vpl << ','
             << static_cast<unsigned>(sample.value.status) << ','
             << sample.value.version << '\n';
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
      std::ofstream metadata(pending / "snapshot.json");
      metadata << "{\n  \"schema_version\": \"iap_gridmap_failure_v3\",\n"
          << "  \"kind\": " << std::quoted(kind) << ",\n"
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
          << ",\n  \"cell_flags_file\": \"cells.bin\",\n"
          << "  \"cell_flag_bits\": {\"raw\": 1, \"inflated\": 2, \"observed\": 4},\n"
          << "  \"risk_version\": " << snapshot->risk_version << ",\n"
          << "  \"risk_context_matches_map\": "
          << (snapshot->risk_context_matches_map ? "true" : "false") << ",\n"
          << "  \"risk_reference_time_s\": "
          << number(snapshot->risk_reference_time_s) << ",\n"
          << "  \"risk_valid_until_s\": "
          << number(snapshot->risk_valid_until_s) << ",\n"
          << "  \"risk_samples_file\": \"queried_risk.csv\",\n"
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
          << ",\n  \"curve_checked_from_time_s\": "
          << number(assessment ? assessment->checked_from_time_s : NAN)
          << ",\n  \"curve_checked_to_time_s\": "
          << number(assessment ? assessment->checked_to_time_s : NAN)
          << ",\n  \"curve_sample_step_s\": "
          << number(assessment ? assessment->sample_step_s : NAN)
          << ",\n  \"actual_curve\": ";
      if (trajectory && assessment) {
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
      if (!snapshot->observation_sources.empty())
        manifest << ",\"observation_sources\":"
                 << std::quoted(relative + "/observation_sources.bin");
      if (snapshot->current_frame)
        manifest << ",\"current_frame_hits\":"
                 << std::quoted(relative + "/current_frame_hits.csv")
                 << ",\"current_frame_beams\":"
                 << std::quoted(relative + "/current_frame_beams.csv");
      if (kind == "stall" || kind == "tracking_error" ||
          kind == "remaining_failure" || kind == "remaining_stop")
        manifest << ",\"state\":" << std::quoted(relative + "/state.json");
      manifest << "}\n";
      manifest.close();
      if (!manifest) throw std::runtime_error("subordinate manifest write failed");
      std::filesystem::rename(manifest_pending, manifest_path);
      captured_failure_kinds_.insert(kind);
      RCLCPP_INFO(node_->get_logger(),
          "planner failure map %s saved at %s generation=%lu voxels=%zu",
          kind.c_str(), directory.c_str(), static_cast<unsigned long>(snapshot->generation),
          snapshot->cell_flags.size());
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(),
          "planner failure map %s save failed: %s", kind.c_str(), error.what());
    }
  }

  void EGOPlannerManager::capturePlanningStall(
      const Eigen::Vector3d& start, const Eigen::Vector3d& target) {
    if (!capture_failure_map_ || captured_failure_kinds_.count("stall")) return;
    planning_time_s_ = node_->now().seconds();
    planning_motion_ = currentMotionContext();
    planning_risk_version_ = 0;
    const auto cell = grid_map_->queryPlanningCell(target, 0,
        planning_time_s_, planning_risk_policy_, planning_motion_, true);
    captureFailureMap("stall", target, start, cell);
    if (!captured_failure_kinds_.count("stall")) return;
    auto* artifacts = glim::RunLogManager::get_if_initialized();
    if (!artifacts) return;
    const auto path = artifacts->export_path(
        "planner/failure_map/stall/state.json");
    const auto fingerprint = planningEvidenceFingerprint(start, target);
    std::ofstream state(path.string() + ".pending");
    state << std::setprecision(17)
          << "{\"schema_version\":\"iap_planner_stall_state_v1\","
          << "\"target_reason\":"
          << std::quoted(gridExecutionReasonName(cell.execution_reason))
          << ",\"evidence_hash\":" << (fingerprint ? *fingerprint : 0)
          << ",\"search_pool_margin_m\":5.0}\n";
    state.close();
    if (state) std::filesystem::rename(path.string() + ".pending", path);
    else {
      captured_failure_kinds_.erase("stall");
      RCLCPP_ERROR(node_->get_logger(), "planner stall state write failed");
    }
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
    if (captured_failure_kinds_.count(kind)) return;
    planning_time_s_ = node_->now().seconds();
    planning_motion_ = currentMotionContext();
    planning_risk_version_ = 0;
    auto cell = grid_map_->queryPlanningCell(actual, 0,
        planning_time_s_, planning_risk_policy_, planning_motion_, true);
    Eigen::Vector3d point = actual;
    if (assessment && assessment->first_execution_position.allFinite()) {
      point = assessment->first_execution_position;
      cell = assessment->first_execution_cell;
    }
    captureFailureMap(kind, point, expected, cell, nullptr, nullptr,
        assessment ? evidence_curve : nullptr, assessment);
    if (!captured_failure_kinds_.count(kind)) return;
    auto* artifacts = glim::RunLogManager::get_if_initialized();
    if (!artifacts) return;
    const auto path = artifacts->export_path(
        "planner/failure_map/" + kind + "/state.json");
    std::ofstream state(path.string() + ".pending");
    state << std::setprecision(17)
          << "{\"schema_version\":\"iap_planner_stop_state_v1\","
          << "\"reason\":" << std::quoted(gridExecutionReasonName(reason))
          << ",\"time_s\":" << planning_time_s_
          << ",\"expected_position_m\":[" << expected.x() << ','
          << expected.y() << ',' << expected.z() << ']'
          << ",\"glio_position_m\":[" << actual.x() << ','
          << actual.y() << ',' << actual.z() << ']'
          << ",\"error_m\":" << error_m
          << ",\"trajectory_id\":" << trajectory_id
          << ",\"failed_curve_id\":" << (assessment ? assessment->trajectory_id : trajectory_id)
          << ",\"last_command_time_s\":"
          << (std::isfinite(command_time_s) ? command_time_s : -1.0)
          << ",\"command_age_s\":"
          << (std::isfinite(command_time_s)
              ? planning_time_s_ - command_time_s : -1.0)
          << ",\"glio_age_s\":" << odom_age_s
          << ",\"map_age_s\":" << map_age_s << "}\n";
    state.close();
    if (state) std::filesystem::rename(path.string() + ".pending", path);
    else {
      captured_failure_kinds_.erase(kind);
      RCLCPP_ERROR(node_->get_logger(), "planner %s state write failed",
                   kind.c_str());
    }
  }

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

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
    bspline_optimizer_->a_star_->setAdvisoryQuery(
        [this](const Eigen::Vector3d& position) {
          return queryPlanningViewAdvisory(position);
        }, [this]() {
          return planning_view_ ? planning_view_->advisory_stats : GridPlanningQueryStats{};
        });
    bspline_optimizer_->a_star_->setLiveGenerationProvider(
        [this]() { return grid_map_->occupancyGeneration(); });
    bspline_optimizer_->setSearchFailureObserver(
        [this](const AStar::Result& result,
               const BsplineOptimizer::SearchFailureContext& context) {
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
          if (capture_failure_map_ && !captured_failure_kinds_.count(kind))
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

  void EGOPlannerManager::observeExecutingTrajectory(int trajectory_id) {
    if(pending_trajectory_ && pending_trajectory_->traj_id_==trajectory_id) {
      local_data_=*pending_trajectory_; pending_trajectory_.reset();
      RCLCPP_INFO(node_->get_logger(),"Trajectory %d executing at its scheduled connection",trajectory_id);
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
    last_plan_failure_=planning_budget_ && planning_budget_->expired() ? PlanFailure::Budget : PlanFailure::Connection;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
      Eigen::Vector3d start_acc, Eigen::Vector3d target_pt, Eigen::Vector3d target_vel,
      bool polynomial_init, bool /* random_polynomial */) {
    const bool own_view=!planning_view_;
    if(own_view && !beginPlanningView()) return false;
    struct EndView { EGOPlannerManager* manager; bool own;
      ~EndView() { if(own) manager->endPlanningView(); } } end_view{this,own_view};
    last_plan_failure_=PlanFailure::None;
    const auto fail=[&](PlanFailure reason) {
      last_plan_failure_=planning_budget_->expired() || planning_budget_->denied() ? PlanFailure::Budget : reason;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Planner rebound rejected: phase=%s execution=%s budget_expired=%d repair_denied=%d",
          last_plan_failure_==PlanFailure::Budget ? "budget" : last_plan_failure_==PlanFailure::Target ? "target" :
          last_plan_failure_==PlanFailure::Search ? "search" : last_plan_failure_==PlanFailure::Curve ? "curve" :
          last_plan_failure_==PlanFailure::Release ? "release" : "connection", gridExecutionReasonName(last_candidate_assessment_.execution_reason),
          planning_budget_->expired(), planning_budget_->denied());
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
    const auto query=[this](const Eigen::Vector3d& point) { return queryPlanningViewCell(point); };
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
      return queryPlanningViewCell(point,fitting_reserve*std::clamp(endpoint_distance/.5,0.,1.));
    };
    optimizer.setPlanningQuery(query,false,guide_query);
    optimizer.setPlanningBudget(planning_budget_);
    optimizer.setPlanningEndpoints(start_pt,target_pt);
    std::vector<Eigen::Vector3d> goals;
    for(const auto& target:planning_targets_) goals.push_back(target.position);
    optimizer.setPlanningGoals(goals);
    LocalTarget selected{target_pt,target_vel,Eigen::Vector3d::Zero(),0};
    std::vector<Eigen::Vector3d> derivatives{start_vel,selected.velocity,start_acc,selected.acceleration};
    double interval=std::max(.05,pp_.ctrl_pt_dist/std::max(.1,pp_.max_vel_)*1.5);
    std::vector<Eigen::Vector3d> points;
    const double distance=(target_pt-start_pt).norm();
    if(distance<.2) return fail(PlanFailure::Target);
    // Reuse starts at the bound connection time, never at a fresh now().
    if(!polynomial_init && local_data_.duration_>0 && connection_time_) {
      auto curve=local_data_.position_traj_;
      const double from=connection_time_->seconds()-local_data_.start_time_.seconds();
      for(double t=from;t<local_data_.duration_;t+=interval) {
        if(planning_budget_->expired()) return fail(PlanFailure::Budget);
        points.push_back(curve.evaluateDeBoorT(t));
      }
    }
    if(points.size()<7) {
      points.clear();
      const double duration=std::max(1.0,2*distance/std::max(.1,pp_.max_vel_));
      auto polynomial=PolynomialTraj::one_segment_traj_gen(start_pt,start_vel,start_acc,
          selected.position,selected.velocity,selected.acceleration,duration);
      const size_t count=std::max<size_t>(7,std::ceil(duration/interval)+1);
      interval=duration/(count-1);
      for(size_t i=0;i<count;++i) {
        if(planning_budget_->expired()) return fail(PlanFailure::Budget);
        points.push_back(polynomial.evaluate(i*interval));
      }
    } else {
      points.push_back(target_pt);
    }
    Eigen::MatrixXd control;
    const auto bind_boundaries=[&]() {
      UniformBspline::enforceBoundaryStates(control,interval,start_pt,start_vel,start_acc,
          selected.position,selected.velocity,selected.acceleration);
      optimizer.setLocalTargetPt(selected.position);
      optimizer.setBsplineInterval(interval);
    };
    UniformBspline::parameterizeToBspline(interval,points,derivatives,control);
    bind_boundaries();
    optimizer.initControlPoints(control,true);
    if(optimizer.initializationFailed()) return fail(PlanFailure::Search);
    const auto initialize_guide=[&]() {
      const auto& guide=optimizer.recoveryGuide();
      if(guide.size()<2) return false;
      const auto index=optimizer.a_star_->lastResult().selected_goal;
      if(index<planning_targets_.size()) selected=planning_targets_[index];
      derivatives={start_vel,selected.velocity,start_acc,selected.acceleration};
      std::vector<double> arc(guide.size(),0);
      for(size_t i=1;i<guide.size();++i) arc[i]=arc[i-1]+(guide[i]-guide[i-1]).norm();
      const size_t count=std::max<size_t>(7,std::ceil(arc.back()/pp_.ctrl_pt_dist)+1);
      points.clear(); size_t segment=1;
      for(size_t i=0;i<count;++i) {
        if(planning_budget_->expired()) return false;
        const double d=arc.back()*i/(count-1);
        while(segment+1<arc.size() && arc[segment]<d) ++segment;
        const double length=arc[segment]-arc[segment-1];
        const double alpha=length>1e-9 ? (d-arc[segment-1])/length : 0;
        points.push_back(guide[segment-1]*(1-alpha)+guide[segment]*alpha);
      }
      interval=std::max(interval,1.5*arc.back()/(std::max(.1,pp_.max_vel_)*(count-1)));
      UniformBspline::parameterizeToBspline(interval,points,derivatives,control);
      bind_boundaries(); optimizer.initializeFromGuide(control); return true;
    };
    if(optimizer.needsGuideReinitialization() && !initialize_guide()) return fail(PlanFailure::Search);
    TrajectoryAssessment assessment;
    UniformBspline curve;
    for(;;) {
      if(planning_budget_->expired()) return fail(PlanFailure::Budget);
      const auto backend_start=PlanningBudget::Clock::now();
      if(!optimizer.BsplineOptimizeTrajRebound(control,interval)) return fail(PlanFailure::Curve);
      planning_timings_.backend_s+=std::chrono::duration<double>(PlanningBudget::Clock::now()-backend_start).count();
      bind_boundaries();
      bool feasible=false;
      for(int retime=0;retime<4;++retime) {
        if(planning_budget_->expired()) return fail(PlanFailure::Budget);
        curve=UniformBspline(control,3,interval);
        curve.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
        double ratio=1;
        if(curve.checkFeasibility(ratio,false)) { feasible=true; break; }
        // Reconstruct a uniform spline and rebind physical derivatives after
        // stretching. A raw lengthenTime would silently change both endpoints.
        interval*=std::max(1.1,ratio*1.05); bind_boundaries();
      }
      if(!feasible) return fail(PlanFailure::Curve);
      const auto check_start=PlanningBudget::Clock::now();
      assessment=assessTrajectory(curve,planning_view_->risk_version,planning_view_->time_s,
          false,0,std::numeric_limits<double>::infinity(),&planning_view_->physical_context);
      planning_timings_.final_checks_s+=std::chrono::duration<double>(PlanningBudget::Clock::now()-check_start).count();
      last_candidate_assessment_=assessment;
      if(assessment.budget_exhausted) return fail(PlanFailure::Budget);
      // Terminal speed is rechecked after optimization against the same input.
      if(selected.velocity.norm()>terminalSpeedLimit(selected.position,selected.velocity)+1e-6)
        return fail(PlanFailure::Target);
      const bool advisory_violation=assessment.advisory_avoid_samples && !optimizer.advisoryFallbackUsed();
      if(assessment.executable() && !advisory_violation) break;
      if(!assessment.executable() && assessment.execution_reason!=GridExecutionReason::PHYSICAL_OBSTACLE &&
          assessment.execution_reason!=GridExecutionReason::INSUFFICIENT_CLEARANCE &&
          assessment.execution_reason!=GridExecutionReason::ENVIRONMENT_UNOBSERVED) return fail(PlanFailure::Curve);
      if(capture_failure_map_ && assessment.first_execution_position.allFinite())
        captureFailureMap("candidate",assessment.first_execution_position,selected.position,
            assessment.first_execution_cell,nullptr,nullptr,&curve,&assessment);
      if(optimizer.recoveryGuide().empty()) {
        if(!optimizer.searchRecoveryGuide() || !initialize_guide()) return fail(PlanFailure::Search);
      } else {
        if(!planning_budget_->tryRepair(PlanningBudget::Repair::CurveCorrection)) return fail(PlanFailure::Budget);
        if(!assessment.curve_clearance_violations.empty()) {
          if(!optimizer.addCurveClearanceConstraints(control,interval,assessment.curve_clearance_violations))
            return fail(PlanFailure::Curve);
          RCLCPP_INFO(node_->get_logger(),"Curve correction: %zu actual clearance violations, fitting reserve=%.3fm",
              assessment.curve_clearance_violations.size(),.5*grid_map_->getResolution());
          optimizer.setControlPoints(control);
        } else {
          optimizer.strengthenGuideTracking();
          optimizer.initializeFromGuide(control);
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
      if(!release.executable() || !release.physical_epoch) {
        last_candidate_assessment_=release; return fail(PlanFailure::Release);
      }
      if(selected.velocity.norm()>terminalSpeedLimit(selected.position,selected.velocity)+1e-6) return fail(PlanFailure::Target);
      {
        // One latest corridor owns all evidence needed through the switch and
        // terminal stopping space. Ordinary map updates outside it are allowed.
        std::vector<Eigen::Vector3d> positions;
        const double spacing=std::min(.01,grid_map_->getResolution()/(4*std::max(.1,pp_.max_vel_)));
        if(connection_time_) {
          const double from=std::max(0.0,now-local_data_.start_time_.seconds());
          const double to=connection_time_->seconds()-local_data_.start_time_.seconds();
          auto old=local_data_.position_traj_;
          for(double t=from;t<=to+spacing;t+=spacing) {
            if(planning_budget_->expired()) return fail(PlanFailure::Budget);
            positions.push_back(old.evaluateDeBoorT(std::min(t,to)));
          }
        }
        const double duration=curve.getTimeSum();
        for(double t=0;t<=duration+spacing;t+=spacing) {
          if(planning_budget_->expired()) return fail(PlanFailure::Budget);
          positions.push_back(curve.evaluateDeBoorT(std::min(t,duration)));
        }
        if(selected.velocity.norm()>1e-9) {
          const double stopping=selected.velocity.squaredNorm()/(2*std::max(.1,pp_.max_acc_))+2*grid_map_->getResolution();
          for(double d=0;d<=stopping+grid_map_->getResolution()*.5;d+=grid_map_->getResolution()*.5) {
            if(planning_budget_->expired()) return fail(PlanFailure::Budget);
            positions.push_back(selected.position+selected.velocity.normalized()*std::min(d,stopping));
          }
        }
        const auto view=captureExecutionView(positions,now,false,planning_budget_);
        if(!view.physical.epoch) {
          last_candidate_assessment_.execution_reason=view.physical.motion_reason!=GridExecutionReason::OK
              ? view.physical.motion_reason : GridExecutionReason::ENVIRONMENT_STALE;
          return fail(PlanFailure::Release);
        }
        release.physical_epoch=view.physical.epoch;
        release.evaluation_time_s=view.time_s;
        release.evaluated_motion=view.motion;
        release.evaluated_motion_quality=view.motion.quality;
        release.evaluated_motion_error_proxy_m=view.motion.error_proxy_m;
        for(const auto& point:positions) {
          if(planning_budget_->expired()) return fail(PlanFailure::Budget);
          const auto cell=grid_map_->queryPlanningCell(point,0,view.time_s,planning_risk_policy_,view.motion,false,&view.physical);
          if(!cell.executable()) {
            last_candidate_assessment_.execution_reason=cell.execution_reason;
            last_candidate_assessment_.first_execution_position=point;
            return fail(PlanFailure::Release);
          }
        }
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

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool EGOPlannerManager::planCheckedBrake(
      const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
      const Eigen::Vector3d& acceleration)
  {
    if (!position.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite() || pp_.max_acc_ <= 0.0)
      return false;
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
      if (!candidate.checkFeasibility(ratio, false)) continue;
      const auto assessment = assessTrajectory(candidate, risk_version,
                                               now, true);
      if (!assessment.executable()) continue;
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
