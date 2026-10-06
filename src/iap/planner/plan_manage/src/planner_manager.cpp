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
      first_rejection_detail = planning_view_->physical->queryPlanningCell(
          search->first_rejection_position, 0, planning_view_->time_s,
          planning_risk_policy_, planning_view_->motion, true);
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
    // A previously captured physical failure must not conceal a later hole
    // on the executing curve.
    if (assessment && std::isfinite(assessment->first_unobserved_time_s))
      captureFailureMap("curve_unobserved", assessment->first_unobserved_position,
          expected, assessment->first_unobserved_cell, nullptr, nullptr,
          &local_data_.position_traj_, assessment);
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
        assessment ? &local_data_.position_traj_ : nullptr, assessment);
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
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);
    node_ = node;
    initRiskInputs(node);
    initRiskVisualization(node);
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
          const auto diagnostic_map = planning_view_ ? planning_view_->physical : grid_map_;
          const double diagnostic_time = planning_view_ ? planning_view_->time_s : planning_time_s_;
          const auto& diagnostic_motion = planning_view_ ? planning_view_->motion : planning_motion_;
          const auto end = diagnostic_map->queryPlanningCell(
              result.requested_end, planning_risk_version_, diagnostic_time,
              planning_risk_policy_, diagnostic_motion, true);
          if (result.has_first_rejection && !capture_failure_map_) {
            const auto first = diagnostic_map->queryPlanningCell(
                result.first_rejection_position, 0, diagnostic_time,
                planning_risk_policy_, diagnostic_motion, true);
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
              planning_risk_policy_, diagnostic_motion, true);
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

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    planning_timings_.backend_s = 0;
    planning_timings_.final_checks_s = 0;
    bspline_optimizer_->a_star_->clearLastResult();
    const bool own_view = !planning_view_;
    if (own_view && !beginPlanningView()) return false;
    struct ViewReset {
      EGOPlannerManager* manager;
      bool own;
      ~ViewReset() { if (own) manager->endPlanningView(); }
    } reset{this, own_view};
    const auto risk_version = planning_view_->risk_version;
    const double planning_time_s = planning_view_->time_s;
    const auto motion = planning_view_->motion;
    bspline_optimizer_->a_star_->setSearchMap(planning_view_->physical);
    planning_risk_version_ = risk_version;
    planning_time_s_ = planning_time_s;
    planning_motion_ = motion;
    const auto planning_query = [this](
        const Eigen::Vector3d& position) {
      return queryPlanningViewCell(position);
    };
    const auto start_cell = planning_query(start_pt);
    if (!start_cell.executable()) {
      const auto detail = planning_view_->physical->queryPlanningCell(
          start_pt, risk_version, planning_time_s, planning_risk_policy_,
          motion, true);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Planning denied: start %s pos=(%.3f %.3f %.3f) required=%s nearest=%s generation=%lu cloud=%.3f",
          gridExecutionReasonName(detail.execution_reason), start_pt.x(),
          start_pt.y(), start_pt.z(),
          clearanceText(detail.required_clearance_m).c_str(),
          clearanceText(detail.raw_center_clearance_m).c_str(),
          static_cast<unsigned long>(detail.occupancy_generation),
          detail.cloud_stamp_s);
      return false;
    }
    const bool start_in_advisory =
        start_cell.advisory.classification == GridAdvisoryClass::AVOID ||
        start_cell.advisory.classification == GridAdvisoryClass::PREDICTED_DEGRADED;
    bspline_optimizer_->setPlanningQuery(planning_query, start_in_advisory);
    bspline_optimizer_->setPlanningBudget(planning_budget_);
    bspline_optimizer_->setPlanningEndpoints(start_pt, local_target_pt);
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    rclcpp::Time t_start = node_->now();
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
      if (planning_budget_->expired()) return false;
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
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

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

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    if (bspline_optimizer_->initializationFailed()) {
      ++continous_failures_count_;
      return false;
    }
    if (bspline_optimizer_->needsGuideReinitialization()) {
      if (!planning_budget_->tryRepair(PlanningBudget::Repair::Reinitialize)) return false;
      const auto& guide = bspline_optimizer_->recoveryGuide();
      std::vector<double> arc(guide.size(), 0.0);
      for (size_t i = 1; i < guide.size(); ++i)
        arc[i] = arc[i-1] + (guide[i] - guide[i-1]).norm();
      const size_t samples = std::max<size_t>(7, std::ceil(arc.back() / pp_.ctrl_pt_dist) + 1);
      point_set.clear(); size_t segment = 1;
      for (size_t i = 0; i < samples; ++i) {
        if (planning_budget_->expired()) return false;
        const double d = arc.back() * i / (samples - 1);
        while (segment + 1 < arc.size() && arc[segment] < d) ++segment;
        const double length = arc[segment] - arc[segment-1];
        const double alpha = length > 1e-9 ? (d - arc[segment-1]) / length : 0.0;
        point_set.push_back(guide[segment-1] * (1-alpha) + guide[segment] * alpha);
      }
      ts = std::max(ts, arc.back() / (pp_.max_vel_ * (samples - 1)));
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
      bspline_optimizer_->initializeFromGuide(ctrl_pts);
    }
    // 计算时间差并更新时间
    auto now = node_->now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    const auto backend_started = std::chrono::steady_clock::now();
    flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    planning_timings_.backend_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - backend_started).count();
    t_opt = node_->now() - t_start;
    visualization_->displayInitPathList(point_set, 0.2, 0);

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = node_->now();
    const auto refine_started = std::chrono::steady_clock::now();

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
        planning_timings_.backend_s += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - refine_started).count();
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
    t_refine = node_->now() - t_start;
    planning_timings_.backend_s += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - refine_started).count();

    // Assess the final time-adjusted curve before touching the active plan.
    double feasibility_ratio = 1.0;
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
    bool feasible = pos.checkFeasibility(feasibility_ratio, false);
    for (int retime = 0; !feasible && retime < 2; ++retime) {
      pos.lengthenTime(std::max(1.1, feasibility_ratio * 1.05));
      feasible = pos.checkFeasibility(feasibility_ratio, false);
    }
    if (!feasible) {
      RCLCPP_WARN(node_->get_logger(), "Candidate rejected: dynamic limits");
      ++continous_failures_count_;
      return false;
    }
    auto check_started = std::chrono::steady_clock::now();
    auto assessment = assessTrajectory(pos, risk_version,
                                       node_->now().seconds());
    planning_timings_.final_checks_s += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - check_started).count();
    if (!assessment.executable()) {
      auto detail = assessment.first_execution_cell;
      if (assessment.first_execution_position.allFinite() &&
          !std::isfinite(detail.raw_center_clearance_m))
        detail = grid_map_->queryPlanningCell(
            assessment.first_execution_position, risk_version,
            node_->now().seconds(), planning_risk_policy_,
            currentMotionContext(), true);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                  "Candidate rejected before commit: %s at t=%.3f pos=(%.3f %.3f %.3f) required=%s nearest=%s nearest_raw=(%.3f %.3f %.3f) generation=%lu cloud=%.3f advisory=%u",
                  gridExecutionReasonName(assessment.execution_reason),
                  assessment.first_execution_time_s,
                  assessment.first_execution_position.x(),
                  assessment.first_execution_position.y(),
                  assessment.first_execution_position.z(),
                  clearanceText(detail.required_clearance_m).c_str(),
                  clearanceText(detail.raw_center_clearance_m).c_str(),
                  detail.nearest_raw_center.x(), detail.nearest_raw_center.y(),
                  detail.nearest_raw_center.z(),
                  static_cast<unsigned long>(detail.occupancy_generation),
                  detail.cloud_stamp_s,
                  static_cast<unsigned>(detail.advisory.classification));
      if (capture_failure_map_ && !captured_failure_kinds_.count("candidate") &&
          assessment.first_execution_position.allFinite())
        captureFailureMap("candidate", assessment.first_execution_position,
                          local_target_pt, assessment.first_execution_cell,
                          nullptr, nullptr, &pos, &assessment);
      if (std::isfinite(assessment.first_unobserved_time_s))
        captureFailureMap("curve_unobserved", assessment.first_unobserved_position,
            local_target_pt, assessment.first_unobserved_cell,
            nullptr, nullptr, &pos, &assessment);
      ++continous_failures_count_;
      return false;
    }
    if (assessment.advisory_avoid_samples != 0 &&
        !bspline_optimizer_->advisoryFallbackUsed()) {
      // One bounded correction of a curve that cut across its guide. The
      // original executable candidate remains available if correction fails.
      Eigen::MatrixXd corrected_points = pos.getControlPoint();
      bspline_optimizer_->initControlPoints(corrected_points, true);
      bool correction_success = false;
      if (!bspline_optimizer_->initializationFailed()) {
        const auto correction_started = std::chrono::steady_clock::now();
        correction_success = bspline_optimizer_->BsplineOptimizeTrajRebound(
            corrected_points, pos.getInterval());
        planning_timings_.backend_s += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - correction_started).count();
      }
      if (correction_success) {
        UniformBspline corrected(corrected_points, 3, pos.getInterval());
        corrected.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                                    pp_.feasibility_tolerance_);
        double ratio = 1.0;
        check_started = std::chrono::steady_clock::now();
        const auto corrected_check = assessTrajectory(
            corrected, risk_version, node_->now().seconds());
        planning_timings_.final_checks_s += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - check_started).count();
        if (corrected.checkFeasibility(ratio, false) &&
            corrected_check.executable() &&
            corrected_check.advisory_avoid_samples <
                assessment.advisory_avoid_samples) {
          pos = corrected;
          assessment = corrected_check;
        }
      }
    }
    if (assessment.advisory_avoid_samples != 0)
      RCLCPP_WARN(node_->get_logger(),
                  "Candidate uses degraded advisory fallback: %zu warning samples",
                  assessment.advisory_avoid_samples);
    if (assessment.advisory_unknown_samples != 0)
      RCLCPP_INFO(node_->get_logger(),
                  "Candidate advisory coverage incomplete: %zu/%zu samples",
                  assessment.advisory_unknown_samples, assessment.sampled_points);
    // Recheck the actual curve against the newest map, motion report and GLIO
    // position after all optimization and advisory correction work.
    check_started = std::chrono::steady_clock::now();
    const auto release_check = assessTrajectory(pos, 0,
                                                 node_->now().seconds());
    planning_timings_.final_checks_s += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - check_started).count();
    const auto release_motion = currentMotionContext();
    if (planning_budget_->expired() || !release_check.executable() ||
        grid_map_->occupancyGeneration() !=
            release_check.evaluated_generation ||
        release_motion.quality !=
            release_check.evaluated_motion_quality ||
        !std::isfinite(release_motion.error_proxy_m) ||
        release_motion.error_proxy_m >
            release_check.evaluated_motion_error_proxy_m + 1e-9) {
      RCLCPP_WARN(node_->get_logger(),
          "Candidate withheld at publication gate: %s generation=%lu current=%lu",
          gridExecutionReasonName(release_check.execution_reason),
          static_cast<unsigned long>(release_check.evaluated_generation),
          static_cast<unsigned long>(grid_map_->occupancyGeneration()));
      if (release_check.first_execution_position.allFinite() &&
          !captured_failure_kinds_.count("candidate"))
        captureFailureMap("candidate",
            release_check.first_execution_position, local_target_pt,
            release_check.first_execution_cell, nullptr, nullptr, &pos, &release_check);
      if (std::isfinite(release_check.first_unobserved_time_s))
        captureFailureMap("curve_unobserved", release_check.first_unobserved_position,
            local_target_pt, release_check.first_unobserved_cell,
            nullptr, nullptr, &pos, &release_check);
      ++continous_failures_count_;
      return false;
    }
    // Commit is the only write to local_data_ on the success path.
    updateTrajInfo(pos, node_->now());

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
