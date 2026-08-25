#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <iap/planner/risk_grid_map.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <utility>
// using namespace std;

namespace ego_planner
{
  namespace
  {
    constexpr int kP1AcceptedProfileSampleCount = 200;
    constexpr const char *kP1AcceptedProfileCsvName =
        "planner_p1_accepted_trajectory_risk_profile.csv";
    constexpr const char *kP1AcceptedProfileContextCsvName =
        "planner_p1_accepted_trajectory_risk_profile_context.csv";
    constexpr const char *kP1ReplacementDecisionCsvName =
        "planner_p1_replacement_decision.csv";
    constexpr const char *kP1CandidateRetainedProfileCsvName =
        "planner_p1_candidate_retained_profile.csv";
    constexpr const char *kP1CandidateControlPointsCsvName =
        "planner_p1_candidate_control_points.csv";
    constexpr const char *kP1CandidateProfileCsvName =
        "planner_p1_candidate_profile.csv";
    constexpr const char *kP1CandidatePairwiseCsvName =
        "planner_p1_candidate_pairwise.csv";
    constexpr const char *kP1OptimizerCheckpointCsvName =
        "planner_p1_optimizer_checkpoint.csv";
    constexpr const char *kP0OccupancyQueryEvidenceCsvName =
        "planner_p0_occupancy_query_evidence.csv";

    // Candidate evidence must be measured against one lattice, rather than
    // against whichever adaptive samples happened to be visited by L-BFGS.
    // Keep this aligned with the accepted-profile lattice so the aggregate
    // record can be reconciled with the per-sample artifact.
    constexpr int kP1CandidateEvidenceSampleCount = 200;

    uint64_t fnv1aAppend(uint64_t hash, const std::string &value)
    {
      constexpr uint64_t kPrime = 1099511628211ULL;
      for (const unsigned char character : value)
      {
        hash ^= character;
        hash *= kPrime;
      }
      return hash;
    }

    std::string hashHex(uint64_t hash)
    {
      std::ostringstream stream;
      stream << std::hex << std::setfill('0') << std::setw(16) << hash;
      return stream.str();
    }

    template <typename Writer>
    std::string stableHash(Writer writer)
    {
      std::ostringstream stream;
      stream << std::setprecision(17);
      writer(stream);
      return hashHex(fnv1aAppend(1469598103934665603ULL, stream.str()));
    }

    std::string matrixHash(const Eigen::MatrixXd &points)
    {
      return stableHash([&points](std::ostream &stream) {
        stream << points.rows() << ':' << points.cols() << ':';
        for (int column = 0; column < points.cols(); ++column)
          for (int row = 0; row < points.rows(); ++row)
            stream << points(row, column) << ':';
      });
    }

    struct P1CandidateLatticeStats
    {
      int sample_count = kP1CandidateEvidenceSampleCount;
      int valid_count = 0;
      double mean_c_pi = std::numeric_limits<double>::quiet_NaN();
      double max_c_pi = std::numeric_limits<double>::quiet_NaN();
      std::string support_signature;

      bool fullValid() const
      {
        return sample_count == kP1CandidateEvidenceSampleCount &&
            valid_count == sample_count && std::isfinite(mean_c_pi) &&
            std::isfinite(max_c_pi);
      }
    };

    P1CandidateLatticeStats evaluateP1CandidateLattice(
        const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
        const Eigen::MatrixXd &control_points, const int order,
        const double interval_s, const double query_base_time_s)
    {
      P1CandidateLatticeStats stats;
      if (!snapshot || control_points.rows() != 3 ||
          control_points.cols() <= order || order != 3 ||
          !std::isfinite(interval_s) || interval_s <= 0.0 ||
          !std::isfinite(query_base_time_s))
      {
        return stats;
      }
      const auto &params = snapshot->params();
      stats.support_signature = stableHash(
          [&snapshot, &params, query_base_time_s](std::ostream &stream) {
            const Eigen::Vector3d origin = snapshot->origin();
            const Eigen::Vector3i voxel_num = snapshot->voxelNum();
            stream << snapshot->generation_id() << ':' << snapshot->stamp_s() << ':'
                   << query_base_time_s << ':' << params.frame_id << ':'
                   << params.resolution_m << ':' << origin.transpose() << ':'
                   << voxel_num.transpose() << ':'
                   << kP1CandidateEvidenceSampleCount << ':';
            for (const double horizon : params.horizons_s)
              stream << horizon << ':';
          });
      const double duration =
          static_cast<double>(control_points.cols() - order) * interval_s;
      if (!std::isfinite(duration) || duration <= 0.0)
        return stats;
      UniformBspline trajectory(control_points, order, interval_s);
      double total = 0.0;
      double maximum = -std::numeric_limits<double>::infinity();
      for (int sample_index = 0; sample_index < stats.sample_count;
           ++sample_index)
      {
        const double fraction = static_cast<double>(sample_index) /
            static_cast<double>(stats.sample_count - 1);
        iap::RiskCostSample sample;
        const Eigen::Vector3d position =
            trajectory.evaluateDeBoorT(duration * fraction);
        const bool hit = snapshot->queryCost(
            position, query_base_time_s + duration * fraction, &sample);
        if (!hit || !sample.valid || sample.stale || !std::isfinite(sample.cost))
          continue;
        ++stats.valid_count;
        total += sample.cost;
        maximum = std::max(maximum, sample.cost);
      }
      if (stats.valid_count > 0)
      {
        stats.mean_c_pi = total / static_cast<double>(stats.valid_count);
        stats.max_c_pi = maximum;
      }
      return stats;
    }

    std::string p1ConfigHash(const ego_planner::BsplineOptimizer::P1IntegrityConfig &config)
    {
      return stableHash([&config](std::ostream &stream) {
        stream << config.use_integrity_cost << ':' << config.metrics_only << ':'
               << config.lambda_integrity << ':' << config.sample_dt_min_s << ':'
               << config.sample_dt_scale << ':' << config.max_samples_per_eval << ':'
               << config.integrity_cost_max << ':' << config.integrity_grad_norm_max << ':'
               << config.unknown_policy << ':' << config.unknown_soft_penalty << ':'
               << config.max_candidates_per_attempt << ':'
               << config.objective_aggregation_mode << ':'
               << config.smooth_max_temperature << ':'
               << config.smooth_cvar_alpha;
      });
    }

    std::string siblingPath(const std::string &path, const std::string &filename)
    {
      const auto slash = path.find_last_of("/\\");
      if (slash == std::string::npos)
      {
        return filename;
      }
      return path.substr(0, slash + 1) + filename;
    }

    // The trilinear query domain is the cell-centre interior, not merely the
    // allocated voxel box.  This is derived from the immutable snapshot so a
    // P1 evaluation cannot be pulled toward bounds from a later refresh.
    bool snapshotInteriorBarrier(const iap::RiskGridSnapshot &snapshot,
                                 const Eigen::Vector3d &p,
                                 double penalty,
                                 double cost_max,
                                 double grad_max,
                                 double *cost,
                                 Eigen::Vector3d *grad)
    {
      if (!cost || !grad || !p.allFinite())
        return false;
      const double resolution = snapshot.params().resolution_m;
      const Eigen::Vector3i dims = snapshot.voxelNum();
      if (!(resolution > 0.0) || dims.minCoeff() < 2)
        return false;
      const Eigen::Vector3d lower = snapshot.origin() +
          Eigen::Vector3d::Constant(0.5 * resolution);
      const Eigen::Vector3d upper = snapshot.origin() +
          (dims.cast<double>().array() - 0.5).matrix() * resolution;
      Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
      for (int axis = 0; axis < 3; ++axis)
      {
        if (p(axis) < lower(axis))
          displacement(axis) = p(axis) - lower(axis);
        else if (p(axis) > upper(axis))
          displacement(axis) = p(axis) - upper(axis);
      }
      if (displacement.squaredNorm() <= 0.0)
        return false;
      const double bounded_penalty = std::max(1.0, penalty);
      *cost = std::min(cost_max, 0.5 * bounded_penalty * displacement.squaredNorm());
      *grad = bounded_penalty * displacement;
      const double norm = grad->norm();
      if (grad_max > 0.0 && norm > grad_max)
        *grad *= grad_max / norm;
      return true;
    }

    void writeP4Csv(const P4RiskAStarConfig &config,
                    const P4GuideDecision &decision,
                    double stamp)
    {
      if (!config.enable_risk_aware_astar || !config.debug_csv_enable || config.debug_csv_path.empty())
        return;
      std::ifstream existing(config.debug_csv_path);
      const bool write_header =
          !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
      existing.close();

      std::ofstream csv(config.debug_csv_path, std::ios::app);
      if (!csv.good())
        return;
      if (write_header)
      {
        csv << "schema_version,stamp,planning_attempt_id,collision_segment_id,"
               "request_hash,snapshot_generation_id,snapshot_stamp_s,snapshot_frame,"
               "query_base_time_s,occupancy_epoch,status,reason,selection_applied,"
               "original_hash,risk_hash,selected_hash,original_sample_count,"
               "original_valid_count,original_unknown_count,original_stale_count,"
               "original_non_finite_count,original_mean,original_max,risk_sample_count,"
               "risk_valid_count,risk_unknown_count,risk_stale_count,risk_non_finite_count,"
               "risk_mean,risk_max,original_path_length,risk_path_length,path_length_ratio,"
               "original_search_latency_ms,risk_search_latency_ms,total_search_latency_ms\n";
      }
      const auto &original = decision.original.risk_profile;
      const auto &risk = decision.risk.risk_profile;
      csv << decision.schema_version << ',' << stamp << ','
          << decision.planning_attempt_id << ','
          << decision.collision_segment_id << ',' << decision.request_hash << ','
          << decision.snapshot_generation << ',' << decision.snapshot_stamp_s << ','
          << decision.snapshot_frame << ',' << decision.query_base_time_s << ','
          << decision.occupancy_epoch << ','
          << p4GuideDecisionStatusName(decision.status) << ','
          << p4GuideDecisionReasonName(decision.reason) << ','
          << (decision.selection_applied ? 1 : 0) << ','
          << decision.original.canonical_hash << ','
          << decision.risk.canonical_hash << ','
          << decision.selected.canonical_hash << ','
          << original.sample_count << ',' << original.valid_count << ','
          << original.unknown_count << ',' << original.stale_count << ','
          << original.non_finite_count << ',' << original.mean << ','
          << original.max << ',' << risk.sample_count << ',' << risk.valid_count << ','
          << risk.unknown_count << ',' << risk.stale_count << ','
          << risk.non_finite_count << ',' << risk.mean << ',' << risk.max << ','
          << decision.original.length_m << ',' << decision.risk.length_m << ','
          << decision.risk_original_length_ratio << ','
          << decision.original_search_latency_ms << ','
          << decision.risk_search_latency_ms << ','
          << decision.total_search_latency_ms << '\n';
    }

    const char *p4OccupancyClass(
        const iap::RiskOccupancyDiagnostic &occupancy)
    {
      if (!occupancy.available)
        return "UNAVAILABLE";
      if (occupancy.raw_occupied)
        return "RAW_OCCUPIED";
      if (occupancy.inflated_occupied)
        return "INFLATED_OCCUPIED";
      return "FREE";
    }

    void writeP4ProfileTraceCsv(const P4RiskAStarConfig &config,
                                const P4GuideDecision &decision)
    {
      if (!config.profile_trace_enable || config.profile_trace_path.empty())
        return;
      std::ifstream existing(config.profile_trace_path);
      const bool write_header =
          !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
      existing.close();
      std::ofstream csv(config.profile_trace_path, std::ios::app);
      if (!csv.good())
        return;
      if (write_header)
        csv << "schema_version,planning_attempt_id,collision_segment_id,request_hash,"
               "arm,sample_index,point_x,point_y,point_z,query_time_s,query_tau_s,"
               "sample_valid,sample_stale,sample_cost,top_reason,risk_generation_id,"
               "frame_id,corner_id,temporal_layer,horizon_id,horizon_s,temporal_weight,"
               "voxel_x,voxel_y,voxel_z,voxel_position_x,voxel_position_y,voxel_position_z,"
               "spatial_weight,source_flags,corner_cost,corner_valid,corner_stale,"
               "corner_unknown,corner_reason,occupancy_class,occupancy_source\n";
      const auto emit_arm = [&](const char *arm, const P4GuideRecord &record) {
        for (const auto &sample_trace : record.sample_traces) {
          const auto emit = [&](const iap::RiskCostQueryCornerTrace *corner) {
            csv << "p4_equal_arc_profile_trace_v1," << decision.planning_attempt_id << ','
                << decision.collision_segment_id << ',' << decision.request_hash << ','
                << arm << ',' << sample_trace.sample_index << ','
                << sample_trace.point.x() << ',' << sample_trace.point.y() << ','
                << sample_trace.point.z() << ',' << sample_trace.query_time_s << ','
                << sample_trace.query.query_tau_s << ','
                << (sample_trace.sample.valid ? 1 : 0) << ','
                << (sample_trace.sample.stale ? 1 : 0) << ','
                << sample_trace.sample.cost << ',' << sample_trace.query.reason << ','
                << sample_trace.query.risk_generation_id << ','
                << sample_trace.query.frame_id << ',';
            if (!corner) {
              csv << "-1,-1,-1,nan,0,-1,-1,-1,nan,nan,nan,0,0,nan,0,1,1,not_evaluated,UNAVAILABLE,unavailable\n";
              return;
            }
            csv << corner->corner_id << ',' << corner->temporal_layer << ','
                << corner->horizon_id << ',' << corner->horizon_s << ','
                << corner->temporal_weight << ',' << corner->voxel_index.x() << ','
                << corner->voxel_index.y() << ',' << corner->voxel_index.z() << ','
                << corner->voxel_position.x() << ',' << corner->voxel_position.y() << ','
                << corner->voxel_position.z() << ',' << corner->spatial_weight << ','
                << corner->source_flags << ',' << corner->c_pi << ','
                << (corner->valid ? 1 : 0) << ',' << (corner->stale ? 1 : 0) << ','
                << (corner->unknown ? 1 : 0) << ',' << corner->invalid_reason << ','
                << p4OccupancyClass(corner->occupancy) << ','
                << corner->occupancy.source << '\n';
          };
          if (sample_trace.query.corners.empty())
            emit(nullptr);
          else
            for (const auto &corner : sample_trace.query.corners)
              emit(&corner);
        }
      };
      emit_arm("original", decision.original);
      emit_arm("risk", decision.risk);
    }
  } // namespace


  void BsplineOptimizer::setParam(rclcpp::Node::SharedPtr node)
  {

    node->declare_parameter("optimization/lambda_smooth", -1.0);
    node->declare_parameter("optimization/lambda_collision", -1.0);
    node->declare_parameter("optimization/lambda_feasibility", -1.0);
    node->declare_parameter("optimization/lambda_fitness", -1.0);

    node->declare_parameter("optimization/dist0", -1.0);
    node->declare_parameter("optimization/swarm_clearance", -1.0);
    node->declare_parameter("optimization/max_vel", -1.0);
    node->declare_parameter("optimization/max_acc", -1.0);

    node->declare_parameter("optimization/order", 3);
    node->declare_parameter("p1.use_integrity_cost", false);
    node->declare_parameter("p1.metrics_only", true);
    node->declare_parameter("p1.lambda_integrity", 0.0);
    node->declare_parameter("p1.sample_dt_min_s", 0.1);
    node->declare_parameter("p1.sample_dt_scale", 1.0);
    node->declare_parameter("p1.max_samples_per_eval", 30);
    node->declare_parameter("p1.integrity_cost_max", 100.0);
    node->declare_parameter("p1.integrity_grad_norm_max", 0.1);
    node->declare_parameter("p1.unknown_policy", std::string("skip"));
    node->declare_parameter("p1.unknown_soft_penalty", 1.0);
    node->declare_parameter("p1.debug_csv_enable", false);
    node->declare_parameter("p1.debug_csv_path", std::string(""));
    node->declare_parameter("p1.evidence_schema_version", std::string(""));
    node->declare_parameter("p1.evidence_run_id", std::string(""));
    node->declare_parameter("p1.evidence_manifest_path", std::string(""));
    node->declare_parameter("p1.max_candidates_per_attempt", 8);
    node->declare_parameter("p1.objective_aggregation_mode",
                            std::string("fixed_200_smooth_cvar"));
    node->declare_parameter("p1.smooth_max_temperature", 0.01);
    node->declare_parameter("p1.smooth_cvar_alpha", 0.90);
    node->declare_parameter("p1.normalization_budget_fraction", 0.30);
    node->declare_parameter("p4.enable_risk_aware_astar", false);
    node->declare_parameter("p4.metrics_only", false);
    node->declare_parameter("p4.lambda_p4_risk", 0.05);
    node->declare_parameter("p4.risk_cost_max", 100.0);
    node->declare_parameter("p4.unknown_edge_penalty", 1.0);
    node->declare_parameter("p4.max_extra_path_ratio", 1.3);
    node->declare_parameter("p4.fallback_to_original_when_risk_not_ready", true);
    node->declare_parameter("p4.debug_csv_enable", false);
    node->declare_parameter("p4.debug_csv_path", std::string(""));
    node->declare_parameter("p4.profile_trace_enable", false);
    node->declare_parameter("p4.profile_trace_path", std::string(""));
    node->declare_parameter(
      "p4.cost_query_policy", std::string("LEGACY_STRICT"));

    node->get_parameter("optimization/lambda_smooth", lambda1_);
    node->get_parameter("optimization/lambda_collision", lambda2_);
    node->get_parameter("optimization/lambda_feasibility", lambda3_);
    node->get_parameter("optimization/lambda_fitness", lambda4_);

    node->get_parameter("optimization/dist0", dist0_);
    node->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node->get_parameter("optimization/max_vel", max_vel_);
    node->get_parameter("optimization/max_acc", max_acc_);

    node->get_parameter("optimization/order", order_);
    node->get_parameter("p1.use_integrity_cost", p1_config_.use_integrity_cost);
    node->get_parameter("p1.metrics_only", p1_config_.metrics_only);
    node->get_parameter("p1.lambda_integrity", p1_config_.lambda_integrity);
    node->get_parameter("p1.sample_dt_min_s", p1_config_.sample_dt_min_s);
    node->get_parameter("p1.sample_dt_scale", p1_config_.sample_dt_scale);
    node->get_parameter("p1.max_samples_per_eval", p1_config_.max_samples_per_eval);
    node->get_parameter("p1.integrity_cost_max", p1_config_.integrity_cost_max);
    node->get_parameter("p1.integrity_grad_norm_max", p1_config_.integrity_grad_norm_max);
    node->get_parameter("p1.unknown_policy", p1_config_.unknown_policy);
    node->get_parameter("p1.unknown_soft_penalty", p1_config_.unknown_soft_penalty);
    node->get_parameter("p1.debug_csv_enable", p1_config_.debug_csv_enable);
    node->get_parameter("p1.debug_csv_path", p1_config_.debug_csv_path);
    node->get_parameter("p1.evidence_schema_version", p1_config_.evidence_schema_version);
    node->get_parameter("p1.evidence_run_id", p1_config_.evidence_run_id);
    node->get_parameter("p1.evidence_manifest_path", p1_config_.evidence_manifest_path);
    node->get_parameter("p1.max_candidates_per_attempt", p1_config_.max_candidates_per_attempt);
    node->get_parameter("p1.objective_aggregation_mode",
                        p1_config_.objective_aggregation_mode);
    node->get_parameter("p1.smooth_max_temperature",
                        p1_config_.smooth_max_temperature);
    node->get_parameter("p1.smooth_cvar_alpha",
                        p1_config_.smooth_cvar_alpha);
    node->get_parameter("p1.normalization_budget_fraction",
                        p1_config_.normalization_budget_fraction);
    if (!std::isfinite(p1_config_.normalization_budget_fraction) ||
        p1_config_.normalization_budget_fraction <= 0.0 ||
        p1_config_.normalization_budget_fraction > 1.0)
    {
      RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"),
                  "invalid p1.normalization_budget_fraction %.17g; using 0.30",
                  p1_config_.normalization_budget_fraction);
      p1_config_.normalization_budget_fraction = 0.30;
    }
    p1_config_.max_candidates_per_attempt = std::clamp(
        p1_config_.max_candidates_per_attempt, 1, 8);
    if (p1_config_.objective_aggregation_mode != "adaptive_mean" &&
        p1_config_.objective_aggregation_mode != "fixed_200_mean" &&
        p1_config_.objective_aggregation_mode != "fixed_200_lse" &&
        p1_config_.objective_aggregation_mode != "fixed_200_smooth_cvar")
    {
      RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"),
                  "unsupported p1.objective_aggregation_mode '%s'; using fixed_200_smooth_cvar",
                  p1_config_.objective_aggregation_mode.c_str());
      p1_config_.objective_aggregation_mode = "fixed_200_smooth_cvar";
    }
    const bool smooth_fixed_aggregation =
        p1_config_.objective_aggregation_mode == "fixed_200_lse" ||
        p1_config_.objective_aggregation_mode == "fixed_200_smooth_cvar";
    if (smooth_fixed_aggregation &&
        (!(p1_config_.smooth_max_temperature > 0.0) ||
         !std::isfinite(p1_config_.smooth_max_temperature)))
    {
      RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"),
                  "invalid p1.smooth_max_temperature %.17g; using fixed_200_mean",
                  p1_config_.smooth_max_temperature);
      p1_config_.objective_aggregation_mode = "fixed_200_mean";
    }
    if (p1_config_.objective_aggregation_mode == "fixed_200_smooth_cvar" &&
        (!(p1_config_.smooth_cvar_alpha > 0.0) ||
         !(p1_config_.smooth_cvar_alpha < 1.0) ||
         !std::isfinite(p1_config_.smooth_cvar_alpha)))
    {
      RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"),
                  "invalid p1.smooth_cvar_alpha %.17g; using fixed_200_mean",
                  p1_config_.smooth_cvar_alpha);
      p1_config_.objective_aggregation_mode = "fixed_200_mean";
    }
    node->get_parameter("p4.enable_risk_aware_astar", p4_config_.enable_risk_aware_astar);
    node->get_parameter("p4.metrics_only", p4_config_.metrics_only);
    node->get_parameter("p4.lambda_p4_risk", p4_config_.lambda_p4_risk);
    node->get_parameter("p4.risk_cost_max", p4_config_.risk_cost_max);
    node->get_parameter("p4.unknown_edge_penalty", p4_config_.unknown_edge_penalty);
    node->get_parameter("p4.max_extra_path_ratio", p4_config_.max_extra_path_ratio);
    node->get_parameter("p4.fallback_to_original_when_risk_not_ready", p4_config_.fallback_to_original_when_risk_not_ready);
    node->get_parameter("p4.debug_csv_enable", p4_config_.debug_csv_enable);
    node->get_parameter("p4.debug_csv_path", p4_config_.debug_csv_path);
    node->get_parameter("p4.profile_trace_enable", p4_config_.profile_trace_enable);
    node->get_parameter("p4.profile_trace_path", p4_config_.profile_trace_path);
    std::string p4_cost_query_policy;
    node->get_parameter("p4.cost_query_policy", p4_cost_query_policy);
    if (p4_cost_query_policy == "CONSERVATIVE_OCCUPIED_COST_SUPPORT") {
      p4_config_.cost_query_policy =
        iap::RiskCostQueryPolicy::CONSERVATIVE_OCCUPIED_COST_SUPPORT;
    } else {
      if (p4_cost_query_policy != "LEGACY_STRICT") {
        RCLCPP_WARN(
          rclcpp::get_logger("BsplineOptimizer"),
          "unsupported p4.cost_query_policy '%s'; using LEGACY_STRICT",
          p4_cost_query_policy.c_str());
      }
      p4_config_.cost_query_policy = iap::RiskCostQueryPolicy::LEGACY_STRICT;
    }
    p4_config_.query_speed_mps = std::isfinite(max_vel_) && max_vel_ > 1.0e-3 ? max_vel_ : 1.0;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    this->grid_map_ = map;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj)
  {
    this->grid_map_ = map;
    this->moving_objs_ = mov_obj;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  void BsplineOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void BsplineOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  void BsplineOptimizer::setRiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                                         double query_base_time_s)
  {
    risk_snapshot_ = std::move(snapshot);
    risk_query_base_time_s_ = query_base_time_s;
    p1_risk_context_.snapshot = risk_snapshot_;
    p1_risk_context_.query_base_time_s = risk_query_base_time_s_;
  }

  void BsplineOptimizer::setP1PlanningRiskContext(P1PlanningRiskContext context)
  {
    if (!p1_pre_optimization_traces_.empty() &&
        p1_pre_optimization_traces_.begin()->first.first !=
            context.planning_attempt_id)
    {
      p1_pre_optimization_traces_.clear();
      p1_candidate_artifacts_.clear();
    }
    p1_risk_context_ = std::move(context);
    risk_snapshot_ = p1_risk_context_.snapshot;
    risk_query_base_time_s_ = p1_risk_context_.query_base_time_s;
  }

  void BsplineOptimizer::clearRiskSnapshot()
  {
    risk_snapshot_.reset();
    risk_query_base_time_s_ = 0.0;
    p1_risk_context_ = P1PlanningRiskContext{};
  }

  void BsplineOptimizer::setP4RiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                                           double query_base_time_s,
                                           uint64_t planning_attempt_id)
  {
    p4_config_.query_speed_mps = std::isfinite(max_vel_) && max_vel_ > 1.0e-3 ? max_vel_ : 1.0;
    p4_risk_snapshot_ = std::move(snapshot);
    p4_query_base_time_s_ = query_base_time_s;
    p4_occupancy_epoch_ = grid_map_ ? grid_map_->occupancyGeneration() : 0;
    active_p4_attempt_id_ = planning_attempt_id;
    if (!a_star_)
      return;
    a_star_->setP4Config(p4_config_);
    a_star_->setRiskSnapshot(p4_risk_snapshot_, query_base_time_s);
  }

  void BsplineOptimizer::clearP4RiskSnapshot()
  {
    p4_risk_snapshot_.reset();
    p4_query_base_time_s_ = 0.0;
    p4_occupancy_epoch_ = 0;
    active_p4_attempt_id_ = 0;
    if (a_star_)
      a_star_->clearRiskSnapshot();
  }

  uint64_t BsplineOptimizer::p4SegmentId(const std::pair<int, int> &segment) const
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(segment.first + 1)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(segment.second + 1));
  }

  P4GuideRequest BsplineOptimizer::makeP4GuideRequest(
      const Eigen::MatrixXd &points,
      const std::pair<int, int> &segment) const
  {
    return P4GuideRequest(
        active_p4_attempt_id_, p4SegmentId(segment),
        points.col(segment.first), points.col(segment.second), true,
        p4_risk_snapshot_, p4_query_base_time_s_, p4_occupancy_epoch_,
        [map = grid_map_]() { return map ? map->occupancyGeneration() : 0; },
        p4_config_);
  }

  P4GuideDecision BsplineOptimizer::planCollisionGuideForSegment(
      const Eigen::MatrixXd &points,
      const std::pair<int, int> &segment)
  {
    const P4GuideRequest request = makeP4GuideRequest(points, segment);
    P4AStarGuideSearch search(a_star_);
    P4CollisionGuidePlanner planner(search);
    return planner.planCollisionGuide(request);
  }

  bool BsplineOptimizer::p4DecisionReadyForInjection(
      P4GuideDecision *decision,
      const Eigen::MatrixXd &points,
      const std::pair<int, int> &segment) const
  {
    if (!decision)
      return false;
    P4GuideDecisionReason reason = P4GuideDecisionReason::NOT_EVALUATED;
    const bool ready = ego_planner::p4GuideDecisionReadyForInjection(
        *decision, makeP4GuideRequest(points, segment), &reason);
    if (!ready)
    {
      decision->status = P4GuideDecisionStatus::DECISION_INVALID_REPLAN_REQUIRED;
      decision->reason = reason;
      decision->selected = P4GuideRecord{};
      decision->selection_applied = false;
    }
    return ready;
  }

  bool BsplineOptimizer::collectP4GuidesForSegments(
      const Eigen::MatrixXd &points,
      const std::vector<std::pair<int, int>> &segments,
      std::vector<std::vector<Eigen::Vector3d>> *guide_paths,
      const char *logger_name)
  {
    if (!guide_paths)
      return false;
    guide_paths->clear();
    guide_paths->reserve(segments.size());
    for (const auto &segment : segments)
    {
      P4GuideDecision decision = planCollisionGuideForSegment(points, segment);
      writeP4Csv(p4_config_, decision, rclcpp::Clock().now().seconds());
      writeP4ProfileTraceCsv(p4_config_, decision);
      last_p4_guides_.push_back(std::move(decision));
      const auto &stored = last_p4_guides_.back();
      if (stored.status != P4GuideDecisionStatus::ORIGINAL_SELECTED ||
          !stored.selected.returned)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger(logger_name),
            "P4 guide decision failed closed: status=%s reason=%s",
            p4GuideDecisionStatusName(stored.status),
            p4GuideDecisionReasonName(stored.reason));
        return false;
      }
      guide_paths->push_back(stored.selected.complete_path);
    }
    for (size_t index = 0; index < segments.size(); ++index)
    {
      if (!p4DecisionReadyForInjection(
          &last_p4_guides_[index], points, segments[index]))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger(logger_name),
            "P4 guide identity changed before constraint injection: reason=%s",
            p4GuideDecisionReasonName(last_p4_guides_[index].reason));
        guide_paths->clear();
        return false;
      }
    }
    return true;
  }

  // 返回多个安全的控制点集
  std::vector<ControlPoints> BsplineOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    last_p1_fanout_diagnostics_ = P1FanoutDiagnostics{};
    last_p1_fanout_diagnostics_.input_topology_segments =
        static_cast<int>(segments.size());
    last_p1_fanout_diagnostics_.configured_cap =
        std::clamp(p1_config_.max_candidates_per_attempt, 1, 8);
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      last_p1_fanout_diagnostics_.returned_candidate_count = 1;
      last_p1_fanout_diagnostics_.singleton_due_to_empty_segments = true;
      return oneSeg;
    }

    constexpr int VARIS = 2;                                                                                // 允许的变化种类数
    const int max_trajs = std::clamp(p1_config_.max_candidates_per_attempt, 1, 8);
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(max_trajs) / log(VARIS)))); // 允许变换的片段数量上限
    std::vector<ControlPoints> control_pts_buf;
    control_pts_buf.reserve(max_trajs);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1); // 计算控制点间的平均距离

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ControlPoints, ControlPoints>> RichInfoSegs;
    // 初始化两套控制点信息
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ControlPoints, ControlPoints> RichInfoOneSeg;
      ControlPoints RichInfoOneSeg_temp;
      // 获取指定片段的控制点信息
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);

      // cout << "RichInfoOneSeg_temp, out" << endl;
      // cout << "RichInfoSegs[" << i << "].first" << endl;
      // for ( int k=0; k<RichInfoOneSeg_temp.size; k++ )
      //   if ( RichInfoOneSeg_temp.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << RichInfoOneSeg_temp.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < RichInfoOneSeg_temp.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << RichInfoOneSeg_temp.base_point[k][k2].transpose() << " @ " << RichInfoOneSeg_temp.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.size - 1; j++)
        {
          // cout << "A *" << j << "*" << endl;
          //  遍历每个控制点及其后一个点，在两点间通过线性插值生成采样点
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            // cout << " " << grid_map_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            //  如果检测到在障碍物内，则存储对应数据
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        // 查找片段与最后一个障碍物的交点
        for (int j = RichInfoSegs[i].first.size - 1; j >= 1; j--)
        {
          // cout << "j=" << j << endl;
          // cout << "B *" << j << "*" << endl;
          ;
          // 和上面同样的采样，然后检测是否在障碍物中
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            // cout << " " << grid_map_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            ;
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        // 如果片段的起点或者终点在障碍物中，将会被移除
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;

          // cout << "RichInfoSegs[" << i << "].first" << endl;
          // for (int k = 0; k < RichInfoSegs[i].first.size; k++)
          // {
          //   if (RichInfoSegs[i].first.base_point.size() > 0)
          //   {
          //     cout << "###" << RichInfoSegs[i].first.points.col(k).transpose() << endl;
          //     for (int k2 = 0; k2 < RichInfoSegs[i].first.base_point[k].size(); k2++)
          //     {
          //       cout << "      " << RichInfoSegs[i].first.base_point[k][k2].transpose() << " @ " << RichInfoSegs[i].first.direction[k][k2].transpose() << endl;
          //     }
          //   }
          // }
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          // 检查控制点的base point是否为1
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << cps_.clearance << " cps.size=" << cps_.size << endl;
            // 输出错误信息
            for (int temp_i = 0; temp_i < cps_.size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            last_p1_fanout_diagnostics_.singleton_due_to_degenerate_segments = true;
            std::vector<ControlPoints> blank;
            return blank;
          }

          // 通过取反获得相反方向的向量
          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          // 若当前控制点为片段的起始点 occ_start_id，则将障碍物交点 occ_start_pt 直接设为 base_pt_reverse
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          // 若当前控制点为片段的终止点 occ_end_id，则将终点交点 occ_end_pt 设为 base_pt_reverse
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          // 对于片段中的中间控制点，将基准点 base_pt_reverse 设置为当前控制点 points.col(j) 沿反向向量 base_vec_reverse 方向延伸的某一距离位置
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          // 检查base_pt_reverse是否在障碍物中
          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            // 最大搜索范围
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              // 不断将控制点向外移动，寻找不在障碍物中的控制点
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              // cout << base_pt_temp.transpose() << endl;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            // 如果找不到则删除这一段
            if (l > l_upbound)
            {
              RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          // 如果距离控制点足够远且不再障碍物中则无需继续搜索
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          // 基点和控制点太近则删除这一段
          else
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.size)
        {
          // 为片段起点之前和终点之后的控制点设置统一的基准点和方向，使得这些控制点在障碍物影响范围外时能够保持一致的路径属性
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      // 片段只有一个控制点的情况
      else if (RichInfoSegs[i].first.size == 1)
      {
        cout << "i=" << i << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        cout << "RichInfoSegs[i].first.size=" << RichInfoSegs[i].first.size << endl;
        cout << "RichInfoSegs[i].first.direction.size()=" << RichInfoSegs[i].first.direction.size() << endl;
        cout << "RichInfoSegs[i].first.direction[0].size()=" << RichInfoSegs[i].first.direction[0].size() << endl;
        cout << "RichInfoSegs[i].first.points.cols()=" << RichInfoSegs[i].first.points.cols() << endl;
        cout << "RichInfoSegs[i].first.base_point.size()=" << RichInfoSegs[i].first.base_point.size() << endl;
        cout << "RichInfoSegs[i].first.base_point[0].size()=" << RichInfoSegs[i].first.base_point[0].size() << endl;
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            // cout << base_pt_temp.transpose() << endl;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), 
                        "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), 
                        "base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
      else
      {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
      }
    }
    // cout << "A3" << endl;

    // Step 2. Assemble each segment to make up the new control point sequence.
    // 将每个分段组合起来，组成新的控制点序列
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      last_p1_fanout_diagnostics_.surviving_topology_segments = 0;
      last_p1_fanout_diagnostics_.returned_candidate_count = 1;
      last_p1_fanout_diagnostics_.singleton_due_to_opposite_direction_unavailable = true;
      return oneSeg;
    }

    // 初始化选择向量
    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    // 计算最大组合数
    int max_traj_nums = std::min(max_trajs,
        static_cast<int>(pow(VARIS, seg_upbound)));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      // 生成一个选择表
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), 
                        "Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
          
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ControlPoints cpsOneSample;
      cpsOneSample.resize(cps_.size);
      cpsOneSample.clearance = cps_.clearance;
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      // 遍历所有控制点
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.size)
      {
        // cout << "A ";
        //  if ( seg_id >= RichInfoSegs.size() )
        //  {
        //    cout << "seg_id=" << seg_id << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        //  }
        //  if ( cp_id >= cps_.base_point.size() )
        //  {
        //    cout << "cp_id=" << cp_id << " cps_.base_point.size()=" << cps_.base_point.size() << endl;
        //  }
        //  if ( cp_of_seg_id >= RichInfoSegs[seg_id].first.base_point.size() )
        //  {
        //    cout << "cp_of_seg_id=" << cp_of_seg_id << " RichInfoSegs[seg_id].first.base_point.size()=" << RichInfoSegs[seg_id].first.base_point.size() << endl;
        //  }
        //  判断控制点是否在当前控制范围内
        //  如果不在则直接从原始控制点集中复制数据
        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        // 如果 cp_id 位于当前片段范围内，根据 selection[seg_id] 的值选择片段的第一套或第二套基准点和方向
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          // 当遍历到片段的最后一个控制点时，将 cp_of_seg_id 重置为 0，并将 seg_id 指向下一个片段
          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), 
                    "Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    last_p1_fanout_diagnostics_.surviving_topology_segments = seg_upbound;
    last_p1_fanout_diagnostics_.returned_candidate_count =
        static_cast<int>(control_pts_buf.size());
    last_p1_fanout_diagnostics_.truncated =
        static_cast<int>(control_pts_buf.size()) >
        last_p1_fanout_diagnostics_.configured_cap;
    if (control_pts_buf.size() == 1 && seg_upbound <
        last_p1_fanout_diagnostics_.input_topology_segments)
    {
      last_p1_fanout_diagnostics_.singleton_due_to_degenerate_segments = true;
    }
    return control_pts_buf;
  } // namespace ego_planner

  std::vector<ControlPoints> BsplineOptimizer::supplementP1RiskGradientCandidates(
      const ControlPoints &base,
      const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
      const double query_base_time_s, const int remaining_capacity)
  {
    std::vector<ControlPoints> supplemental;
    if (!snapshot || remaining_capacity <= 0 || base.points.rows() != 3 ||
        base.size <= order_ || base.points.cols() != base.size ||
        !std::isfinite(query_base_time_s))
      return supplemental;

    const auto previous_snapshot = risk_snapshot_;
    const double previous_query_base = risk_query_base_time_s_;
    risk_snapshot_ = snapshot;
    risk_query_base_time_s_ = query_base_time_s;
    double raw_cost = 0.0;
    Eigen::MatrixXd projected_gradient = Eigen::MatrixXd::Zero(
        3, base.size);
    P1IntegrityMetrics metrics;
    calcIntegrityTrajectoryCost(
        base.points, raw_cost, projected_gradient, metrics);
    risk_snapshot_ = previous_snapshot;
    risk_query_base_time_s_ = previous_query_base;
    const int first = std::max(order_, 0);
    projected_gradient.leftCols(first).setZero();
    const double max_column_norm =
        projected_gradient.colwise().norm().maxCoeff();
    if (!projected_gradient.allFinite() ||
        !std::isfinite(max_column_norm) || max_column_norm <= 1.0e-12 ||
        metrics.sample_count != kP1CandidateEvidenceSampleCount ||
        metrics.hit_count != kP1CandidateEvidenceSampleCount)
      return supplemental;

    constexpr double kDisplacementsM[] = {0.025, 0.05, 0.10};
    for (const double displacement : kDisplacementsM)
    {
      if (static_cast<int>(supplemental.size()) >= remaining_capacity)
        break;
      ControlPoints candidate = base;
      candidate.points -=
          (displacement / max_column_norm) * projected_gradient;
      supplemental.push_back(std::move(candidate));
    }
    last_p1_fanout_diagnostics_.supplemental_candidate_count =
        static_cast<int>(supplemental.size());
    last_p1_fanout_diagnostics_.returned_candidate_count +=
        static_cast<int>(supplemental.size());
    return supplemental;
  }

  const char *collisionScanStatusName(const CollisionScanStatus status)
  {
    switch (status)
    {
      case CollisionScanStatus::NO_COLLISION:
        return "NO_COLLISION";
      case CollisionScanStatus::CLOSED_SEGMENTS:
        return "CLOSED_SEGMENTS";
      case CollisionScanStatus::OPEN_ENDED_COLLISION:
        return "OPEN_ENDED_COLLISION";
      case CollisionScanStatus::INVALID_INPUT:
        return "INVALID_INPUT";
    }
    return "INVALID_INPUT";
  }

  bool collisionScanFailsClosed(const CollisionScanStatus status)
  {
    return status == CollisionScanStatus::OPEN_ENDED_COLLISION ||
           status == CollisionScanStatus::INVALID_INPUT;
  }

  CollisionScanResult BsplineOptimizer::scanCollisionSegments(
      const Eigen::MatrixXd &points) const
  {
    CollisionScanResult result;
    if (!grid_map_ || points.rows() != 3 || points.cols() <= 2 * order_ ||
        !points.allFinite())
      return result;

    const double resolution = grid_map_->getResolution();
    const double average_spacing =
        (points.col(0) - points.col(points.cols() - 1)).norm() /
        static_cast<double>(points.cols() - 1);
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
        !std::isfinite(average_spacing) || average_spacing <= 0.0)
      return result;

    const double step_size = resolution / average_spacing / 1.5;
    if (!std::isfinite(step_size) || step_size <= 0.0)
      return result;

    constexpr int kEnoughStableSamples = 2;
    const int trigger_end = static_cast<int>(points.cols()) - order_ -
        (static_cast<int>(points.cols()) - 2 * order_) / 3;
    const int tail_end = static_cast<int>(points.cols()) - 1;
    int stable_sample_count = kEnoughStableSamples + 1;
    int free_start_index = -1;
    int free_end_index = -1;
    bool last_occupied = false;
    bool active_entry = false;
    bool possible_exit = false;

    for (int index = order_; index <= tail_end; ++index)
    {
      const bool entry_allowed = index <= trigger_end;
      for (double alpha = 1.0; alpha > 0.0; alpha -= step_size)
      {
        const Eigen::Vector3d sample =
            alpha * points.col(index - 1) +
            (1.0 - alpha) * points.col(index);
        const int occupancy = grid_map_->getInflateOccupancy(sample);
        if (occupancy < 0)
          return CollisionScanResult{};
        const bool occupied = occupancy != 0;

        if (occupied && !last_occupied)
        {
          if (!active_entry && entry_allowed &&
              (stable_sample_count > kEnoughStableSamples || index == order_))
          {
            free_start_index = index - 1;
            active_entry = true;
          }
          stable_sample_count = 0;
          possible_exit = false;
        }
        else if (!occupied && last_occupied)
        {
          if (active_entry)
          {
            free_end_index = index;
            possible_exit = true;
          }
          stable_sample_count = 0;
        }
        else
        {
          ++stable_sample_count;
        }

        if (active_entry && possible_exit &&
            (stable_sample_count > kEnoughStableSamples || index == tail_end))
        {
          const int start_occupancy = grid_map_->getInflateOccupancy(
              points.col(free_start_index));
          const int end_occupancy = grid_map_->getInflateOccupancy(
              points.col(free_end_index));
          if (start_occupancy != 0 || end_occupancy != 0)
            return CollisionScanResult{};
          if (!result.closed_segments.empty() &&
              free_start_index <= result.closed_segments.back().second)
          {
            result.closed_segments.back().second = std::max(
                result.closed_segments.back().second, free_end_index);
          }
          else
          {
            result.closed_segments.emplace_back(
                free_start_index, free_end_index);
          }
          active_entry = false;
          possible_exit = false;
        }

        last_occupied = occupied;
      }
    }

    if (active_entry)
    {
      result.status = CollisionScanStatus::OPEN_ENDED_COLLISION;
      result.closed_segments.clear();
      return result;
    }
    result.status = result.closed_segments.empty()
        ? CollisionScanStatus::NO_COLLISION
        : CollisionScanStatus::CLOSED_SEGMENTS;
    return result;
  }

  /* This function is very similar to check_collision_and_rebound().
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  // 初始化控制点
  CollisionScanResult BsplineOptimizer::initControlPoints(
      Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {
    last_p4_guides_.clear();

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    last_collision_scan_result_ = scanCollisionSegments(init_points);
    if (last_collision_scan_result_.status !=
        CollisionScanStatus::CLOSED_SEGMENTS)
      return last_collision_scan_result_;
    vector<std::pair<int, int>> segment_ids =
        last_collision_scan_result_.closed_segments;
    bool occ = false;

    /*** one immutable P4 decision per scanner-closed segment ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    if (!collectP4GuidesForSegments(
        init_points, segment_ids, &a_star_pathes, "initControlPoints"))
    {
      last_collision_scan_result_ = CollisionScanResult{};
      return last_collision_scan_result_;
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    // 遍历每一段，设置边界
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          // 取当前片段结束点 segment_ids[0].second 和下一个片段起始点 segment_ids[1].first 的中间位置，向下取整后作为高边界
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          // 距末尾 order_ + 1 个点的位置
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      // 末尾的边界
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        // 尾段的低边界为当前片段起点和上一个片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 距末尾点 order_ + 1 个点
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        // 低边界：为当前片段起点和前一片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 高边界：为当前片段终点和下一片段起点的中间位置，向下取整
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    // 控制点的最小比例
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    // 控制点的最小数量
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      // 获取当前片段的点数
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      // cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        // 如果点数不够则在两侧扩展点，确保不能超过边界
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }

      // cout << "final:" << "i = " << i << " first = " << adjusted_segment_ids[i].first << " second = " << adjusted_segment_ids[i].second << endl;
    }
    // 避免重叠
    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      // 遍历该段所有控制点的id，并将标志位置为false
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      // 初始化交点标记
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        // 计算控制点的方向向量
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        // A*路径的中点
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        // 路径点和控制点之间的点积，判断当前路径点在控制点方向 ctrl_pts_law 的哪一侧
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          // 根据 val 的正负决定 Astar_id 的移动方向
          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 寻找可能的交点，TODO:还没搞懂
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            // cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          // 计算交点到控制点之间的距离
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            // 逐步进行采样
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              // 通过线性插值计算采样点位置
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                // 记录基点并计算到控制点的方向
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                // cout << "A " << j << endl;
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      // 当轨迹片段只有两个控制点（即长度过短）时，A* 路径的方向可能与控制点的方向不一致，导致梯度方向相反
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        // 计算控制点方向向量和中点
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        // 计算A*中点并判断方向，同上
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          // 和上面不一样，这里减去的是中点
          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 计算交点
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            // 满足距离要求则存储相关信息
            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      // step 3
      if (got_intersection_id >= 0)
      {
        // 遍历交点之后的控制点
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          // 如果没被标记则将前一个控制点的信息给该点
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
            // cout << "AAA " << j << endl;
          }

        // 遍历交点前的控制点，如果没被标记用后一个控制点的数据赋值
        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
            // cout << "AAAA " << j << endl;
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    last_collision_scan_result_.status = final_segment_ids.empty()
        ? CollisionScanStatus::INVALID_INPUT
        : CollisionScanStatus::CLOSED_SEGMENTS;
    last_collision_scan_result_.closed_segments = std::move(final_segment_ids);
    return last_collision_scan_result_;
  }

  // 急停情况下提前退出
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    if (k > 0)
    {
      const bool first_accepted = std::none_of(
          opt->current_p1_checkpoints_.begin(),
          opt->current_p1_checkpoints_.end(), [](const auto &item) {
            return item.checkpoint == "first_accepted_step";
          });
      P1OptimizerCheckpoint checkpoint;
      checkpoint.stage = opt->p1_base_prepass_active_
          ? "base_prepass" : "p1_stage";
      checkpoint.checkpoint = first_accepted
          ? "first_accepted_step" : "accepted_iteration";
      checkpoint.restart_index = opt->current_p1_restart_index_;
      checkpoint.iteration = k;
      checkpoint.line_search_count = ls;
      checkpoint.step = step;
      checkpoint.objective = fx;
      checkpoint.base_objective =
          opt->last_optimizer_cost_breakdown_.original_cost;
      checkpoint.raw_p1_objective = opt->last_p1_metrics_.f_integrity;
      checkpoint.normalized_p1_objective =
          opt->last_optimizer_cost_breakdown_.normalized_integrity_cost;
      checkpoint.anchor_objective =
          opt->last_optimizer_cost_breakdown_.anchor_cost;
      checkpoint.x_norm = xnorm;
      checkpoint.gradient_norm = gnorm;
      if (opt->current_p1_last_accepted_x_.size() == n)
      {
        const Eigen::Map<const Eigen::VectorXd> accepted_x(x, n);
        const Eigen::Map<const Eigen::VectorXd> accepted_gradient(g, n);
        checkpoint.directional_derivative = accepted_gradient.dot(
            accepted_x - opt->current_p1_last_accepted_x_);
        opt->current_p1_last_accepted_x_ = accepted_x;
      }
      if (first_accepted || k % 10 == 0)
        opt->current_p1_checkpoints_.push_back(std::move(checkpoint));
    }
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // 利用combineCostRebound计算损失
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);
    opt->last_rebound_total_gradient_norm_ =
        Eigen::Map<const Eigen::VectorXd>(grad, n).norm();

    opt->iter_num_ += 1;
    return cost;
  }

  // 利用combineCostRefine计算优化后的损失
  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 几个计算损失的函数
  void BsplineOptimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_ - (double)(q.cols() - 2 * order_) * 1.0 / 3.0; // Only check the first 2/3 points
    const double CLEARANCE = swarm_clearance_ * 2;
    double t_now = rclcpp::Clock().now().seconds();
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (int i = order_; i < end_idx; i++)
    {
      double glb_time = t_now + ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ((swarm_trajs_->at(id).drone_id != (int)id) || swarm_trajs_->at(id).drone_id == drone_id_)
        {
          continue;
        }

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.seconds();
        if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ - 0.1)
        {
          /* def cost=(c-sqrt([Q-O]'D[Q-O]))^2, D=[1/b^2,0,0;0,1/b^2,0;0,0,1/a^2] */
          Eigen::Vector3d swarm_prid = swarm_trajs_->at(id).position_traj_.evaluateDeBoorT(glb_time - traj_i_satrt_time);
          Eigen::Vector3d dist_vec = cps_.points.col(i) - swarm_prid;
          double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
          double dist_err = CLEARANCE - ellip_dist;

          Eigen::Vector3d dist_grad = cps_.points.col(i) - swarm_prid;
          Eigen::Vector3d Coeff;
          Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
          Coeff(1) = Coeff(0);
          Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;

          if (dist_err < 0)
          {
            /* do nothing */
          }
          else
          {
            cost += pow(dist_err, 2);
            gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
          }

          if (min_ellip_dist_ > dist_err)
          {
            min_ellip_dist_ = dist_err;
          }
        }
      }
    }
  }

  void BsplineOptimizer::calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 1.5;
    double t_now = rclcpp::Clock().now().seconds();

    for (int i = order_; i < end_idx; i++)
    {
      double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (int id = 0; id < moving_objs_->getObjNums(); id++)
      {
        Eigen::Vector3d obj_prid = moving_objs_->evaluateConstVel(id, t_now + time);
        double dist = (cps_.points.col(i) - obj_prid).norm();
        // cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
        double dist_err = CLEARANCE - dist;
        Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid).normalized();

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else
        {
          cost += pow(dist_err, 2);
          gradient.col(i) += -2.0 * dist_err * dist_grad;
        }
      }
      // cout << "time=" << time << " i=" << i << " order_=" << order_ << " end_idx=" << end_idx << endl;
      // cout << "--" << endl;
    }
    // cout << "---------------" << endl;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (!suppress_rebound_collision_for_test_ && iter_num > 3 &&
        smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;

    // zero cost and gradient in hard constraints
    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q.col(q.cols() - 3);
    q_2 = q.col(q.cols() - 2);
    q_1 = q.col(q.cols() - 1);

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
    cost += dq.squaredNorm();

    gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
    gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
    gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    // #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      // cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "zx-todo VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      // cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "zx-todo ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      // cout << endl;
    }

#endif
  }
  /*                    上面七个都是计算损失的函数           */

  // 检查是否有障碍物？
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {
    last_p4_guides_.clear();
    last_collision_scan_result_ = scanCollisionSegments(cps_.points);
    if (collisionScanFailsClosed(last_collision_scan_result_.status))
    {
      force_stop_type_ = STOP_FOR_ERROR;
      return false;
    }
    if (last_collision_scan_result_.status ==
        CollisionScanStatus::NO_COLLISION)
      return false;

    vector<std::pair<int, int>> segment_ids;
    bool has_unclassifiable_segment = false;
    for (const auto &segment : last_collision_scan_result_.closed_segments)
    {
      bool has_occupied_interior_sample = false;
      bool new_collision = false;
      for (int index = segment.first + 1;
           index < segment.second && !new_collision; ++index)
      {
        bool occupied =
            grid_map_->getInflateOccupancy(cps_.points.col(index)) > 0;
        has_occupied_interior_sample =
            has_occupied_interior_sample || occupied;
        for (size_t direction_index = 0;
             occupied && direction_index < cps_.direction[index].size();
             ++direction_index)
        {
          cout.precision(2);
          if ((cps_.points.col(index) -
               cps_.base_point[index][direction_index])
                  .dot(cps_.direction[index][direction_index]) <
              grid_map_->getResolution())
            occupied = false;
        }
        new_collision = occupied;
      }
      if (!has_occupied_interior_sample)
      {
        has_unclassifiable_segment = true;
        break;
      }
      if (new_collision)
        segment_ids.push_back(segment);
    }

    if (has_unclassifiable_segment)
    {
      force_stop_type_ = STOP_FOR_ERROR;
      return false;
    }

    if (segment_ids.empty())
    {
      last_collision_scan_result_.status =
          CollisionScanStatus::NO_COLLISION;
      last_collision_scan_result_.closed_segments.clear();
      return false;
    }
    last_collision_scan_result_.closed_segments = segment_ids;

    if (!segment_ids.empty())
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      if (!collectP4GuidesForSegments(
          cps_.points, segment_ids, &a_star_pathes,
          "check_collision_and_rebound"))
      {
        force_stop_type_ = STOP_FOR_ERROR;
        return false;
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        // step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("check_collision_and_rebound"), "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  // 设置时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    last_p1_optimization_trace_ = P1OptimizationTrace{};
    captureP1PreOptimizationTrajectory(cps_.points, ts);
    setBsplineInterval(ts);

    double final_cost;
    bool flag_success = rebound_optimize(final_cost);
    captureP1PostOptimizationTrajectory(cps_.points, ts);

    optimal_points = cps_.points;
    last_p1_optimization_trace_.optimization_success = flag_success;
    if (!flag_success)
      last_p1_optimization_trace_.selection_reason = "optimizer_failure";

    return flag_success;
  }

  // 设置初始控制点control_points、时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，
  // 得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts)
  {
    last_p1_optimization_trace_ = P1OptimizationTrace{};
    captureP1PreOptimizationTrajectory(control_points.points, ts);
    // 将时间间隔存储到成员变量
    setBsplineInterval(ts);

    cps_ = control_points;

    bool flag_success = rebound_optimize(final_cost);
    captureP1PostOptimizationTrajectory(cps_.points, ts);

    optimal_points = cps_.points;
    last_p1_optimization_trace_.optimization_success = flag_success;
    if (!flag_success)
      last_p1_optimization_trace_.selection_reason = "optimizer_failure";

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajBasePrepass(
      Eigen::MatrixXd &optimal_points, double &final_cost,
      const ControlPoints &control_points, const double ts)
  {
    clearP1NormalizedStage();
    p1_normalized_stage_.budget_fraction =
        p1_config_.normalization_budget_fraction;
    p1_base_prepass_active_ = true;
    const auto started = std::chrono::steady_clock::now();
    const bool success = BsplineOptimizeTrajRebound(
        optimal_points, final_cost, control_points, ts);
    p1_base_prepass_active_ = false;
    last_p1_base_prepass_trace_.pre_objective =
        last_p1_optimization_trace_.pre_base_objective;
    last_p1_base_prepass_trace_.post_objective =
        last_p1_optimization_trace_.post_base_objective;
    last_p1_base_prepass_trace_.duration_ms = 1000.0 *
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
    last_p1_base_prepass_trace_.solver_result =
        last_p1_optimization_trace_.solver_result;
    last_p1_base_prepass_trace_.iteration_count =
        last_p1_optimization_trace_.iteration_count;
    last_p1_base_prepass_trace_.success = success;
    last_p1_base_prepass_trace_.termination_reason =
        last_p1_optimization_trace_.termination_reason;
    return success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajNormalizedP1(
      Eigen::MatrixXd &optimal_points, double &final_cost,
      const ControlPoints &seed_control_points, const double ts,
      const P1BasePrepassTrace &base_prepass, std::string *reason)
  {
    if (!prepareP1NormalizedStage(
            seed_control_points.points, ts, base_prepass, reason))
      return false;
    const bool success = BsplineOptimizeTrajRebound(
        optimal_points, final_cost, seed_control_points, ts);
    clearP1NormalizedStage();
    return success;
  }

  // 设置初始控制点init_points、时间间隔ts，调用refine_optimize()重新分配时间，得到最优的动力学可行轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    // 将控制点数据存储到成员变量
    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到光滑、无碰撞、动力学可行、与其他无人机碰撞、结束项的轨迹。
  bool BsplineOptimizer::rebound_optimize(double &final_cost)
  {
    iter_num_ = 0;
    int start_id = order_;
    // int end_id = this->cps_.size - order_; //Fixed end
    int end_id = this->cps_.size; // Free end
    // 变量个数
    variable_num_ = 3 * (end_id - start_id);

    rclcpp::Time t0 = rclcpp::Clock().now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    current_p1_checkpoints_.clear();
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      min_ellip_dist_ = INIT_min_ellip_dist_;
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      // 控制点数组初始化
      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      // Record a complete, fixed-lattice pre-state before L-BFGS can mutate
      // the candidate.  This is intentionally separate from the adaptive
      // objective sampling used by the optimizer itself.
      const Eigen::MatrixXd initial_control_points = cps_.points;
      const auto pre_lattice = evaluateP1CandidateLattice(
          risk_snapshot_, initial_control_points, order_, bspline_interval_,
          risk_query_base_time_s_);
      std::vector<double> initial_gradient(
          static_cast<std::size_t>(variable_num_), 0.0);
      double initial_cost = 0.0;
      combineCostRebound(q, initial_gradient.data(), initial_cost, variable_num_);
      last_rebound_total_gradient_norm_ = Eigen::Map<const Eigen::VectorXd>(
          initial_gradient.data(), variable_num_).norm();
      current_p1_restart_index_ = restart_nums;
      if (restart_nums > 0)
      {
        P1OptimizerCheckpoint restart;
        restart.stage = p1_base_prepass_active_ ? "base_prepass" : "p1_stage";
        restart.checkpoint = "restart";
        restart.restart_index = restart_nums;
        restart.reason = "collision_or_swarm_recheck";
        current_p1_checkpoints_.push_back(std::move(restart));
      }
      P1OptimizerCheckpoint first_direction;
      first_direction.stage = p1_base_prepass_active_ ? "base_prepass" : "p1_stage";
      first_direction.checkpoint = "first_direction";
      first_direction.restart_index = restart_nums;
      first_direction.objective = initial_cost;
      first_direction.base_objective = last_optimizer_cost_breakdown_.original_cost;
      first_direction.raw_p1_objective = last_p1_metrics_.f_integrity;
      first_direction.normalized_p1_objective =
          last_optimizer_cost_breakdown_.normalized_integrity_cost;
      first_direction.anchor_objective = last_optimizer_cost_breakdown_.anchor_cost;
      first_direction.x_norm = Eigen::Map<const Eigen::VectorXd>(
          q, variable_num_).norm();
      first_direction.gradient_norm = last_rebound_total_gradient_norm_;
      first_direction.directional_derivative =
          -last_rebound_total_gradient_norm_ * last_rebound_total_gradient_norm_;
      current_p1_checkpoints_.push_back(std::move(first_direction));
      P1OptimizationTrace &trace = last_p1_optimization_trace_;
      trace.planning_attempt_id = p1_risk_context_.planning_attempt_id;
      trace.candidate_id = p1_risk_context_.candidate_id;
      trace.snapshot_generation_id = risk_snapshot_ ? risk_snapshot_->generation_id() : 0;
      trace.query_base_time_s = risk_query_base_time_s_;
      trace.pre_base_objective = last_optimizer_cost_breakdown_.original_cost;
      trace.pre_total_objective = initial_cost;
      trace.pre_raw_p1_cost = last_p1_metrics_.f_integrity;
      trace.pre_weighted_p1_cost = last_p1_metrics_.weighted_f_integrity;
      trace.raw_p1_cost = trace.pre_raw_p1_cost;
      trace.weighted_p1_cost = trace.pre_weighted_p1_cost;
      trace.pre_base_gradient_norm = last_p1_metrics_.grad_norm_original;
      trace.pre_full_base_gradient_norm =
          last_p1_metrics_.full_grad_norm_original;
      trace.pre_raw_p1_gradient_norm = last_p1_metrics_.grad_norm_integrity;
      trace.pre_full_raw_p1_gradient_norm =
          last_p1_metrics_.full_grad_norm_integrity;
      trace.pre_weighted_p1_gradient_norm =
          last_p1_metrics_.weighted_grad_integrity_norm;
      trace.pre_normalized_weighted_p1_gradient_norm =
          last_p1_metrics_.normalized_weighted_grad_integrity_norm;
      trace.pre_full_normalized_weighted_p1_gradient_norm =
          last_p1_metrics_.full_normalized_weighted_grad_integrity_norm;
      trace.pre_normalized_p1_cost =
          last_p1_metrics_.normalized_weighted_f_integrity;
      trace.pre_anchor_cost = last_p1_metrics_.anchor_cost;
      trace.pre_base_p1_cosine = last_p1_metrics_.base_p1_cosine;
      trace.pre_total_gradient_norm = last_rebound_total_gradient_norm_;
      trace.pre_full_total_gradient_norm =
          last_p1_metrics_.full_total_gradient_norm;
      trace.base_gradient_norm = trace.pre_base_gradient_norm;
      trace.p1_gradient_norm = trace.pre_raw_p1_gradient_norm;
      trace.total_gradient_norm = trace.pre_total_gradient_norm;
      trace.pre_mean_c_pi = pre_lattice.mean_c_pi;
      trace.pre_max_c_pi = pre_lattice.max_c_pi;
      trace.support_sample_count = pre_lattice.sample_count;
      trace.pre_support_valid_count = pre_lattice.valid_count;
      trace.pre_support_coverage = static_cast<double>(pre_lattice.valid_count) /
          static_cast<double>(std::max(1, pre_lattice.sample_count));
      trace.support_signature = pre_lattice.support_signature;
      trace.initial_control_points_hash = matrixHash(initial_control_points);
      trace.p1_config_hash = p1ConfigHash(p1_config_);
      trace.objective_allowed = p1_risk_context_.objective_allowed;
      trace.objective_applied = last_p1_metrics_.applied_to_objective;
      trace.fallback_reason = p1_risk_context_.fallback_reason;
      if (p1_normalized_stage_.enabled)
      {
        trace.normalization_mode = "base_improvement_budget_v1";
        trace.normalization_reference_lambda =
            p1_normalized_stage_.reference_lambda;
        trace.normalization_scale = p1_normalized_stage_.scale;
        trace.normalization_budget_fraction =
            p1_normalized_stage_.budget_fraction;
        trace.normalization_base_improvement_budget =
            p1_normalized_stage_.base_improvement_budget;
        trace.normalization_reference_displacement_m =
            p1_normalized_stage_.reference_displacement_m;
        trace.base_prepass_pre_objective =
            p1_normalized_stage_.base_prepass.pre_objective;
        trace.base_prepass_post_objective =
            p1_normalized_stage_.base_prepass.post_objective;
        trace.base_prepass_duration_ms =
            p1_normalized_stage_.base_prepass.duration_ms;
        trace.base_prepass_solver_result =
            p1_normalized_stage_.base_prepass.solver_result;
        trace.base_prepass_iteration_count =
            p1_normalized_stage_.base_prepass.iteration_count;
        trace.base_prepass_success =
            p1_normalized_stage_.base_prepass.success;
        trace.base_prepass_termination_reason =
            p1_normalized_stage_.base_prepass.termination_reason;
      }
      trace.aggregation_mode = p1_config_.objective_aggregation_mode;
      trace.aggregation_temperature = p1_config_.smooth_max_temperature;
      trace.aggregation_tail_fraction =
          p1_config_.objective_aggregation_mode == "fixed_200_smooth_cvar"
              ? p1_config_.smooth_cvar_alpha : 0.0;
      const double adaptive_dt = std::max(p1_config_.sample_dt_min_s,
          bspline_interval_ * std::max(0.0, p1_config_.sample_dt_scale));
      trace.adaptive_sample_count = adaptive_dt > 0.0 && std::isfinite(adaptive_dt)
          ? std::max(1, static_cast<int>(std::ceil(
                static_cast<double>(initial_control_points.cols() - order_) *
                bspline_interval_ / adaptive_dt)))
          : 0;
      if (p1_config_.max_samples_per_eval > 0)
        trace.adaptive_sample_count = std::min(
            trace.adaptive_sample_count, p1_config_.max_samples_per_eval);
      trace.fixed_sample_count = kP1CandidateEvidenceSampleCount;
      trace.peak_contribution = last_p1_metrics_.peak_contribution;

      // 初始化L-BFGS算法的参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16; // 算法保存的历史优化步数
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;
      if (p1_config_.use_integrity_cost && !p1_config_.metrics_only &&
          p1_risk_context_.objective_allowed &&
          p1_config_.lambda_integrity != 0.0)
      {
        const double variable_norm = Eigen::Map<const Eigen::VectorXd>(
            q, variable_num_).norm();
        lbfgs_params.g_epsilon = p1LbfgsGradientEpsilon(
            p1_normalized_stage_.enabled
                ? last_p1_metrics_.normalized_weighted_grad_integrity_norm
                : last_p1_metrics_.weighted_grad_integrity_norm,
            variable_norm);
      }

      /* ---------- optimize ---------- */
      t1 = rclcpp::Clock().now();
      current_p1_last_accepted_x_ = Eigen::Map<const Eigen::VectorXd>(
          q, variable_num_);
      // 执行优化
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      P1OptimizerCheckpoint terminal;
      terminal.stage = p1_base_prepass_active_ ? "base_prepass" : "p1_stage";
      terminal.checkpoint = "terminal";
      terminal.restart_index = restart_nums;
      terminal.iteration = iter_num_;
      terminal.objective = final_cost;
      terminal.base_objective = last_optimizer_cost_breakdown_.original_cost;
      terminal.raw_p1_objective = last_p1_metrics_.f_integrity;
      terminal.normalized_p1_objective =
          last_optimizer_cost_breakdown_.normalized_integrity_cost;
      terminal.anchor_objective = last_optimizer_cost_breakdown_.anchor_cost;
      terminal.x_norm = Eigen::Map<const Eigen::VectorXd>(q, variable_num_).norm();
      terminal.gradient_norm = last_rebound_total_gradient_norm_;
      terminal.solver_result = result;
      terminal.reason = lbfgs::lbfgs_strerror(result);
      current_p1_checkpoints_.push_back(std::move(terminal));
      // The final callback supplied the terminal metrics and gradient.  Copy
      // q explicitly before querying the fixed lattice, because a solver
      // error/restart must not leave evidence attached to an older callback.
      memcpy(cps_.points.data() + 3 * start_id, q,
             variable_num_ * sizeof(q[0]));
      const auto post_lattice = evaluateP1CandidateLattice(
          risk_snapshot_, cps_.points, order_, bspline_interval_,
          risk_query_base_time_s_);
      trace.post_total_objective = final_cost;
      trace.post_base_objective = last_optimizer_cost_breakdown_.original_cost;
      trace.post_raw_p1_cost = last_p1_metrics_.f_integrity;
      trace.post_weighted_p1_cost = last_p1_metrics_.weighted_f_integrity;
      trace.raw_p1_cost = trace.post_raw_p1_cost;
      trace.weighted_p1_cost = trace.post_weighted_p1_cost;
      trace.post_base_gradient_norm = last_p1_metrics_.grad_norm_original;
      trace.post_full_base_gradient_norm =
          last_p1_metrics_.full_grad_norm_original;
      trace.post_raw_p1_gradient_norm = last_p1_metrics_.grad_norm_integrity;
      trace.post_full_raw_p1_gradient_norm =
          last_p1_metrics_.full_grad_norm_integrity;
      trace.post_weighted_p1_gradient_norm =
          last_p1_metrics_.weighted_grad_integrity_norm;
      trace.post_normalized_weighted_p1_gradient_norm =
          last_p1_metrics_.normalized_weighted_grad_integrity_norm;
      trace.post_full_normalized_weighted_p1_gradient_norm =
          last_p1_metrics_.full_normalized_weighted_grad_integrity_norm;
      trace.post_normalized_p1_cost =
          last_p1_metrics_.normalized_weighted_f_integrity;
      trace.post_anchor_cost = last_p1_metrics_.anchor_cost;
      trace.post_base_p1_cosine = last_p1_metrics_.base_p1_cosine;
      trace.post_total_gradient_norm = last_rebound_total_gradient_norm_;
      trace.post_full_total_gradient_norm =
          last_p1_metrics_.full_total_gradient_norm;
      trace.total_gradient_norm = trace.post_total_gradient_norm;
      trace.displacement_norm = (cps_.points - initial_control_points).norm();
      double raw_integrity_cost = 0.0;
      Eigen::MatrixXd raw_integrity_gradient = Eigen::MatrixXd::Zero(
          3, initial_control_points.cols());
      P1IntegrityMetrics raw_integrity_metrics;
      const P1IntegrityMetrics terminal_metrics = last_p1_metrics_;
      trace.peak_contribution = terminal_metrics.peak_contribution;
      const auto terminal_viz_samples = last_p1_viz_samples_;
      calcIntegrityTrajectoryCost(initial_control_points, raw_integrity_cost,
                                  raw_integrity_gradient, raw_integrity_metrics);
      last_p1_metrics_ = terminal_metrics;
      last_p1_viz_samples_ = terminal_viz_samples;
      trace.grad_integrity_dot_displacement =
          (raw_integrity_gradient.array() *
           (cps_.points - initial_control_points).array()).sum();
      trace.weighted_p1_gradient_dot_displacement =
          p1_config_.lambda_integrity * trace.grad_integrity_dot_displacement;
      const Eigen::MatrixXd total_displacement = cps_.points - initial_control_points;
      trace.total_gradient_dot_displacement =
          Eigen::Map<const Eigen::VectorXd>(initial_gradient.data(), variable_num_).dot(
              Eigen::Map<const Eigen::VectorXd>(
                  total_displacement.data() + 3 * start_id, variable_num_));
      trace.final_control_points_hash = matrixHash(cps_.points);
      trace.post_mean_c_pi = post_lattice.mean_c_pi;
      trace.post_max_c_pi = post_lattice.max_c_pi;
      trace.post_support_valid_count = post_lattice.valid_count;
      trace.post_support_coverage = static_cast<double>(post_lattice.valid_count) /
          static_cast<double>(std::max(1, post_lattice.sample_count));
      trace.support_full_valid = pre_lattice.fullValid() && post_lattice.fullValid() &&
          pre_lattice.support_signature == post_lattice.support_signature &&
          !pre_lattice.support_signature.empty();
      trace.solver_result = result;
      trace.iteration_count = iter_num_;
      trace.termination_reason = lbfgs::lbfgs_strerror(result);
      t2 = rclcpp::Clock().now();
      double time_ms = (t2 - t1).seconds() * 1000;
      double total_time_ms = (t2 - t0).seconds() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      // 收敛、达到最大迭代次数、已达到最小值或被停止，则进入碰撞检测阶段
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        /*** collision check, phase 1 ***/
        if ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))
        {
          success = false;
          restart_nums++;
          const auto collision_scan = initControlPoints(cps_.points, false);
          if (collisionScanFailsClosed(collision_scan.status))
            return false;
          new_lambda2_ *= 2; // 提高规避权重

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);

          continue;
        }

        /*** collision check, phase 2 ***/
        // 创建均匀的B样条曲线
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        // 开始时间，结束时间
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        // 计算时间步长
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        // 遍历轨迹的前2/3部分进行障碍物检测
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ)
          {
            // cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            // 如果在前三个控制点范围内检测到了碰撞则视为不可行
            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              // cout << cps_.points.col(1).transpose() << "\n"
              //      << cps_.points.col(2).transpose() << "\n"
              //      << cps_.points.col(3).transpose() << "\n"
              //      << cps_.points.col(4).transpose() << endl;
              RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"), "First 3 control points in obstacles! return false, t=%f", t);
              return false;
            }

            break;
          }
        }

        // cout << "XXXXXX" << ((cps_.points.col(cps_.points.cols()-1) + 4*cps_.points.col(cps_.points.cols()-2) + cps_.points.col(cps_.points.cols()-3))/6 - local_target_pt_).norm() << endl;

        /*** collision check, phase 3 ***/
// #define USE_SECOND_CLEARENCE_CHECK
#ifdef USE_SECOND_CLEARENCE_CHECK
        bool flag_cls_xyp, flag_cls_xyn, flag_cls_zp, flag_cls_zn;
        Eigen::Vector3d start_end_vec = traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm);
        Eigen::Vector3d offset_xy(-start_end_vec(0), start_end_vec(1), 0);
        offset_xy.normalize();
        Eigen::Vector3d offset_z = start_end_vec.cross(offset_xy);
        offset_z.normalize();
        offset_xy *= cps_.clearance / 2;
        offset_z *= cps_.clearance / 2;

        Eigen::MatrixXd check_pts(cps_.points.rows(), cps_.points.cols());
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts.col(i) = cps_.points.col(i);
          check_pts(0, i) += offset_xy(0);
          check_pts(1, i) += offset_xy(1);
          check_pts(2, i) += offset_xy(2);
        }
        flag_cls_xyp = !initControlPoints(
            check_pts, false).closed_segments.empty();
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_xy(0);
          check_pts(1, i) -= 2 * offset_xy(1);
          check_pts(2, i) -= 2 * offset_xy(2);
        }
        flag_cls_xyn = !initControlPoints(
            check_pts, false).closed_segments.empty();
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) += offset_xy(0) + offset_z(0);
          check_pts(1, i) += offset_xy(1) + offset_z(1);
          check_pts(2, i) += offset_xy(2) + offset_z(2);
        }
        flag_cls_zp = !initControlPoints(
            check_pts, false).closed_segments.empty();
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_z(0);
          check_pts(1, i) -= 2 * offset_z(1);
          check_pts(2, i) -= 2 * offset_z(2);
        }
        flag_cls_zn = !initControlPoints(
            check_pts, false).closed_segments.empty();
        if ((flag_cls_xyp ^ flag_cls_xyn) || (flag_cls_zp ^ flag_cls_zn))
          flag_occ = true;
#endif

        // 如果没有检测到碰撞视为优化成功
        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        // 如果有碰撞则重新初始化
        else // restart
        {
          restart_nums++;
          const auto collision_scan = initControlPoints(cps_.points, false);
          if (collisionScanFailsClosed(collision_scan.status))
            return false;
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      // 如果优化被强制取消
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"), 
                                        "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (rclcpp::ok());
      }

    } while (
        ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到重新分配时间后，光滑、拟合较好、动力学可行的轨迹。
  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("refine_optimize"), 
                                        "Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      // 使用优化后的控制点创建新的轨迹
      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          // 将ref_pts存储为矩阵形式
          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      // 如果存在碰撞则调整参数并重新迭代
      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    // cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  bool BsplineOptimizer::cubicBasisForTime(double t, int control_point_count,
                                           int &first_control_point,
                                           double weights[4]) const
  {
    if (order_ != 3 || control_point_count < 4 || bspline_interval_ <= 0.0 ||
        !std::isfinite(t))
    {
      return false;
    }

    const double duration = static_cast<double>(control_point_count - order_) * bspline_interval_;
    if (duration <= 0.0)
    {
      return false;
    }

    const double clamped_t = std::min(std::max(0.0, t), duration);
    int k = order_;
    double s = 0.0;
    if (clamped_t >= duration)
    {
      k = control_point_count - 1;
      s = 1.0;
    }
    else
    {
      const double span = std::floor(clamped_t / bspline_interval_);
      k = order_ + static_cast<int>(span);
      k = std::min(std::max(order_, k), control_point_count - 1);
      s = (clamped_t - static_cast<double>(k - order_) * bspline_interval_) / bspline_interval_;
      s = std::min(std::max(0.0, s), 1.0);
    }

    first_control_point = k - order_;
    if (first_control_point < 0 || first_control_point + 3 >= control_point_count)
    {
      return false;
    }

    const double s2 = s * s;
    const double s3 = s2 * s;
    weights[0] = (1.0 - 3.0 * s + 3.0 * s2 - s3) / 6.0;
    weights[1] = (4.0 - 6.0 * s2 + 3.0 * s3) / 6.0;
    weights[2] = (1.0 + 3.0 * s + 3.0 * s2 - 3.0 * s3) / 6.0;
    weights[3] = s3 / 6.0;
    return true;
  }

  void BsplineOptimizer::calcIntegrityTrajectoryCost(const Eigen::MatrixXd &q, double &cost,
                                                     Eigen::MatrixXd &gradient,
                                                     P1IntegrityMetrics &metrics)
  {
    cost = 0.0;
    gradient.setZero();
    metrics = P1IntegrityMetrics{};
    metrics.planning_attempt_id = p1_risk_context_.planning_attempt_id;
    metrics.candidate_id = p1_risk_context_.candidate_id;
    last_p1_viz_samples_.clear();

    if (!risk_snapshot_)
    {
      metrics.fallback_reason = "snapshot_unavailable";
      return;
    }

    metrics.snapshot_generation_id = risk_snapshot_->generation_id();

    if (order_ != 3)
    {
      metrics.fallback_reason = "unsupported_order";
      return;
    }

    if (q.cols() < order_ + 1 || bspline_interval_ <= 0.0)
    {
      metrics.fallback_reason = "invalid_trajectory";
      return;
    }

    const double duration = static_cast<double>(q.cols() - order_) * bspline_interval_;
    if (duration <= 0.0 || !std::isfinite(duration))
    {
      metrics.fallback_reason = "invalid_duration";
      return;
    }

    const double raw_dt = std::max(p1_config_.sample_dt_min_s,
                                   bspline_interval_ * std::max(0.0, p1_config_.sample_dt_scale));
    if (!(raw_dt > 0.0) || !std::isfinite(raw_dt))
    {
      metrics.fallback_reason = "invalid_sample_dt";
      return;
    }

    int adaptive_sample_count = static_cast<int>(std::ceil(duration / raw_dt));
    adaptive_sample_count = std::max(1, adaptive_sample_count);
    if (p1_config_.max_samples_per_eval > 0)
    {
      adaptive_sample_count = std::min(
          adaptive_sample_count, p1_config_.max_samples_per_eval);
    }
    const bool fixed_lattice =
        p1_config_.objective_aggregation_mode == "fixed_200_mean" ||
        p1_config_.objective_aggregation_mode == "fixed_200_lse" ||
        p1_config_.objective_aggregation_mode == "fixed_200_smooth_cvar";
    const int sample_count = fixed_lattice
        ? kP1CandidateEvidenceSampleCount : adaptive_sample_count;
    metrics.sample_count = sample_count;

    struct ObjectiveSample
    {
      double cost = 0.0;
      Eigen::Vector3d spatial_gradient = Eigen::Vector3d::Zero();
      int first_control_point = -1;
      double basis[4] = {0.0, 0.0, 0.0, 0.0};
    };
    std::vector<ObjectiveSample> objective_samples(
        static_cast<std::size_t>(sample_count));

    UniformBspline traj(q, order_, bspline_interval_);
    const double denom = sample_count > 1 ? static_cast<double>(sample_count - 1) : 1.0;
    const bool small_penalty = p1_config_.unknown_policy == "small_penalty";
    const double cost_max = std::max(0.0, p1_config_.integrity_cost_max);
    const double grad_max = std::max(0.0, p1_config_.integrity_grad_norm_max);

    for (int sample_id = 0; sample_id < sample_count; ++sample_id)
    {
      const double t = sample_count > 1 ? duration * static_cast<double>(sample_id) / denom : 0.0;
      const Eigen::Vector3d p = traj.evaluateDeBoorT(t);
      P1IntegrityVizSample viz;
      viz.position = p;
      viz.t_s = t;

      iap::RiskCostSample sample;
      const bool hit = risk_snapshot_->queryCost(p, risk_query_base_time_s_ + t, &sample);
      if (!hit || !sample.valid || !std::isfinite(sample.cost))
      {
        metrics.miss_count++;
        if (sample.stale)
        {
          metrics.stale_count++;
        }
        viz.hit = false;
        viz.stale = sample.stale;
        viz.unknown = true;
        viz.reason = sample.reason.empty() || sample.reason == "not_evaluated"
            ? "query_miss" : sample.reason;
        const bool spatial_boundary_miss =
            viz.reason == "position_out_of_map" ||
            viz.reason == "position_out_of_interpolation_bounds";
        const bool conservative_scalar_miss =
            viz.reason.find("stale") != std::string::npos ||
            viz.reason.find("invalid") != std::string::npos ||
            viz.reason == "time_out_of_range" ||
            viz.reason == "time_out_of_horizon" ||
            viz.reason == "mixed";
        Eigen::Vector3d barrier_grad = Eigen::Vector3d::Zero();
        double barrier_cost = 0.0;
        const bool has_barrier = spatial_boundary_miss &&
            snapshotInteriorBarrier(*risk_snapshot_, p,
                                    p1_config_.unknown_soft_penalty,
                                    cost_max, grad_max,
                                    &barrier_cost, &barrier_grad);
        if (has_barrier)
        {
          viz.cost = barrier_cost;
          viz.grad = barrier_grad;
          viz.push = -p1_config_.lambda_integrity * barrier_grad;
        }
        if (has_barrier)
        {
          ObjectiveSample &objective_sample = objective_samples[sample_id];
          objective_sample.cost = barrier_cost;
          objective_sample.spatial_gradient = barrier_grad;
          int first_control_point = 0;
          double weights[4] = {0.0, 0.0, 0.0, 0.0};
          if (cubicBasisForTime(t, static_cast<int>(q.cols()), first_control_point, weights))
          {
            for (int i = 0; i < 4; ++i)
              objective_sample.basis[i] = weights[i];
            objective_sample.first_control_point = first_control_point;
          }
        }
        else if (small_penalty || conservative_scalar_miss)
        {
          // Stale/invalid/time misses must not silently look like low risk.
          // They have no trustworthy spatial direction, so retain zero grad.
          const double penalty = std::max(1.0, p1_config_.unknown_soft_penalty);
          viz.cost = penalty;
          objective_samples[sample_id].cost = penalty;
        }
        last_p1_viz_samples_.push_back(viz);
        continue;
      }

      metrics.hit_count++;
      if (sample.stale)
      {
        metrics.stale_count++;
      }

      double sample_cost = std::min(std::max(0.0, sample.cost), cost_max);
      Eigen::Vector3d sample_grad = sample.grad;
      if (!sample_grad.allFinite())
      {
        sample_grad.setZero();
      }
      const double grad_norm = sample_grad.norm();
      if (grad_max > 0.0 && grad_norm > grad_max)
      {
        sample_grad *= grad_max / grad_norm;
        metrics.clipped_grad_count++;
      }

      ObjectiveSample &objective_sample = objective_samples[sample_id];
      objective_sample.cost = sample_cost;
      objective_sample.spatial_gradient = sample_grad;
      viz.hit = true;
      viz.stale = sample.stale;
      viz.unknown = false;
      viz.cost = sample_cost;
      viz.grad = sample_grad;
      viz.push = -p1_config_.lambda_integrity * sample_grad;
      viz.reason = sample.reason;
      last_p1_viz_samples_.push_back(viz);

      int first_control_point = 0;
      double weights[4] = {0.0, 0.0, 0.0, 0.0};
      if (cubicBasisForTime(t, static_cast<int>(q.cols()), first_control_point, weights))
      {
        for (int i = 0; i < 4; ++i)
          objective_sample.basis[i] = weights[i];
        objective_sample.first_control_point = first_control_point;
      }
      else
      {
        metrics.miss_count++;
      }
    }

    if (sample_count > 0)
    {
      std::vector<double> aggregation_weights(
          static_cast<std::size_t>(sample_count),
          1.0 / static_cast<double>(sample_count));
      if (p1_config_.objective_aggregation_mode == "fixed_200_lse")
      {
        const double temperature = p1_config_.smooth_max_temperature;
        double maximum_cost = -std::numeric_limits<double>::infinity();
        for (const auto &sample : objective_samples)
          maximum_cost = std::max(maximum_cost, sample.cost);
        double exponential_sum = 0.0;
        for (int index = 0; index < sample_count; ++index)
        {
          aggregation_weights[index] = std::exp(
              (objective_samples[index].cost - maximum_cost) / temperature);
          exponential_sum += aggregation_weights[index];
        }
        if (exponential_sum > 0.0 && std::isfinite(exponential_sum))
        {
          cost = maximum_cost + temperature *
              (std::log(exponential_sum) -
               std::log(static_cast<double>(sample_count)));
          for (double &weight : aggregation_weights)
            weight /= exponential_sum;
        }
        else
        {
          cost = maximum_cost;
          std::fill(aggregation_weights.begin(), aggregation_weights.end(),
                    1.0 / static_cast<double>(sample_count));
        }
      }
      else if (p1_config_.objective_aggregation_mode ==
               "fixed_200_smooth_cvar")
      {
        const double temperature = p1_config_.smooth_max_temperature;
        const double tail_probability = 1.0 - p1_config_.smooth_cvar_alpha;
        const double target_tail_mass =
            tail_probability * static_cast<double>(sample_count);
        double minimum_cost = std::numeric_limits<double>::infinity();
        double maximum_cost = -std::numeric_limits<double>::infinity();
        for (const auto &sample : objective_samples)
        {
          minimum_cost = std::min(minimum_cost, sample.cost);
          maximum_cost = std::max(maximum_cost, sample.cost);
        }
        const auto stable_sigmoid = [](const double value) {
          if (value >= 0.0)
            return 1.0 / (1.0 + std::exp(-value));
          const double exponential = std::exp(value);
          return exponential / (1.0 + exponential);
        };
        const auto stable_softplus = [](const double value) {
          return value > 0.0
              ? value + std::log1p(std::exp(-value))
              : std::log1p(std::exp(value));
        };

        // The eta derivative is monotone.  Fixed-count bisection keeps the
        // auxiliary threshold deterministic; the 64*T guard reaches sigmoid
        // saturation without evaluating an overflowing exponential.
        double eta_lower = minimum_cost - 64.0 * temperature;
        double eta_upper = maximum_cost + 64.0 * temperature;
        for (int iteration = 0; iteration < 100; ++iteration)
        {
          const double eta_midpoint = 0.5 * (eta_lower + eta_upper);
          double sigmoid_sum = 0.0;
          for (const auto &sample : objective_samples)
            sigmoid_sum += stable_sigmoid(
                (sample.cost - eta_midpoint) / temperature);
          if (sigmoid_sum > target_tail_mass)
            eta_lower = eta_midpoint;
          else
            eta_upper = eta_midpoint;
        }
        const double eta = 0.5 * (eta_lower + eta_upper);
        const double tail_scale = 1.0 / target_tail_mass;
        // Normalize the smooth approximation so a tied profile returns its
        // common c_pi exactly.  This q-independent entropy bias preserves the
        // envelope gradient and keeps unknown-skip zero-cost semantics intact.
        const double binary_entropy =
            -tail_probability * std::log(tail_probability) -
            p1_config_.smooth_cvar_alpha *
                std::log(p1_config_.smooth_cvar_alpha);
        cost = eta - temperature * binary_entropy / tail_probability;
        for (int index = 0; index < sample_count; ++index)
        {
          const double standardized =
              (objective_samples[index].cost - eta) / temperature;
          cost += tail_scale * temperature * stable_softplus(standardized);
          // By the envelope theorem, eta's implicit derivative vanishes at
          // its stationary point; only these smooth tail weights remain.
          aggregation_weights[index] =
              tail_scale * stable_sigmoid(standardized);
        }
        if (minimum_cost == maximum_cost)
        {
          cost = minimum_cost;
          std::fill(aggregation_weights.begin(), aggregation_weights.end(),
                    1.0 / static_cast<double>(sample_count));
        }
      }
      else
      {
        for (const auto &sample : objective_samples)
          cost += sample.cost;
        cost /= static_cast<double>(sample_count);
      }

      for (int index = 0; index < sample_count; ++index)
      {
        const auto &sample = objective_samples[index];
        if (sample.first_control_point < 0)
          continue;
        for (int basis_index = 0; basis_index < 4; ++basis_index)
          gradient.col(sample.first_control_point + basis_index) +=
              aggregation_weights[index] * sample.basis[basis_index] *
              sample.spatial_gradient;
      }
      metrics.peak_contribution = *std::max_element(
          aggregation_weights.begin(), aggregation_weights.end());
      metrics.miss_ratio = static_cast<double>(metrics.miss_count) / static_cast<double>(sample_count);
      metrics.stale_ratio = static_cast<double>(metrics.stale_count) / static_cast<double>(sample_count);
    }

    metrics.f_integrity = cost;
    metrics.weighted_f_integrity = p1_config_.lambda_integrity * cost;
    metrics.full_grad_norm_integrity = gradient.norm();
    metrics.grad_norm_integrity = gradient.cols() > order_
        ? gradient.rightCols(gradient.cols() - order_).norm()
        : 0.0;
    metrics.weighted_grad_integrity_norm = std::abs(p1_config_.lambda_integrity) * metrics.grad_norm_integrity;
    metrics.fallback_reason = metrics.hit_count > 0 || small_penalty ||
                                      metrics.miss_count > 0 ? "ok" : "no_valid_samples";
  }

  void BsplineOptimizer::writeP1DebugCsv(const P1IntegrityMetrics &metrics) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty())
    {
      return;
    }

    std::ifstream existing(p1_config_.debug_csv_path);
    const bool write_header = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    existing.close();

    std::ofstream out(p1_config_.debug_csv_path, std::ios::app);
    if (!out.good())
    {
      return;
    }
    // Candidate/context identity is compared with the accepted-profile
    // sidecar.  Preserve enough precision for snapshot-relative time bases
    // instead of collapsing epoch seconds to the default stream precision.
    out << std::setprecision(17);
    if (write_header)
    {
      out << "schema_version,run_id,manifest_path,stamp,lbfgs_iter,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,"
             "sample_count,hit_count,miss_count,stale_count,miss_ratio,stale_ratio,"
             "f_integrity,weighted_f_integrity,grad_norm_integrity,grad_norm_original,"
             "grad_ratio,clipped_grad_count,fallback_reason,applied_to_objective\n";
    }

    out << p1_config_.evidence_schema_version << ','
        << p1_config_.evidence_run_id << ','
        << p1_config_.evidence_manifest_path << ','
        << rclcpp::Clock().now().seconds() << ','
        << iter_num_ << ','
        << metrics.planning_attempt_id << ','
        << metrics.candidate_id << ','
        << metrics.snapshot_generation_id << ','
        << risk_query_base_time_s_ << ','
        << metrics.sample_count << ','
        << metrics.hit_count << ','
        << metrics.miss_count << ','
        << metrics.stale_count << ','
        << metrics.miss_ratio << ','
        << metrics.stale_ratio << ','
        << metrics.f_integrity << ','
        << metrics.weighted_f_integrity << ','
        << metrics.grad_norm_integrity << ','
        << metrics.grad_norm_original << ','
        << metrics.grad_ratio << ','
        << metrics.clipped_grad_count << ','
        << metrics.fallback_reason << ','
        << (metrics.applied_to_objective ? 1 : 0) << '\n';
  }

  std::string BsplineOptimizer::p1CandidateOptimizationPath() const
  {
    if (p1_config_.debug_csv_path.empty())
      return "planner_p1_candidate_optimization.csv";
    return siblingPath(p1_config_.debug_csv_path,
                       "planner_p1_candidate_optimization.csv");
  }

  std::string BsplineOptimizer::p1ReplacementDecisionPath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP1ReplacementDecisionCsvName
        : siblingPath(p1_config_.debug_csv_path, kP1ReplacementDecisionCsvName);
  }

  std::string BsplineOptimizer::p1CandidateRetainedProfilePath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP1CandidateRetainedProfileCsvName
        : siblingPath(p1_config_.debug_csv_path, kP1CandidateRetainedProfileCsvName);
  }

  std::string BsplineOptimizer::p1CandidateControlPointsPath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP1CandidateControlPointsCsvName
        : siblingPath(p1_config_.debug_csv_path, kP1CandidateControlPointsCsvName);
  }

  std::string BsplineOptimizer::p1CandidateProfilePath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP1CandidateProfileCsvName
        : siblingPath(p1_config_.debug_csv_path, kP1CandidateProfileCsvName);
  }

  std::string BsplineOptimizer::p1PrequalificationCandidateProfilePath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? "planner_p1_prequalification_candidate_profile.csv"
        : siblingPath(p1_config_.debug_csv_path,
                      "planner_p1_prequalification_candidate_profile.csv");
  }

  bool BsplineOptimizer::writeP1PrequalificationCandidateProfile(
      UniformBspline candidate, const bool selected, const std::string &phase)
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty() ||
        !risk_snapshot_ || !std::isfinite(risk_query_base_time_s_))
      return false;
    const auto summary = evaluateP1FixedLatticeRisk(candidate);
    const std::string path = p1PrequalificationCandidateProfilePath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return false;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,phase,sample_index,t_s,x,y,z,valid,stale,c_pi,invalid_reason,collision_free,generation_success,selected\n";
    }
    const double duration = candidate.getTimeSum();
    for (int index = 0; index < kP1CandidateEvidenceSampleCount; ++index) {
      const double fraction = static_cast<double>(index) /
          static_cast<double>(kP1CandidateEvidenceSampleCount - 1);
      const double t_s = duration * fraction;
      const Eigen::Vector3d point = candidate.evaluateDeBoorT(t_s);
      iap::RiskCostSample sample;
      const bool hit = risk_snapshot_->queryCost(
          point, risk_query_base_time_s_ + t_s, &sample);
      out << p1_config_.evidence_schema_version << ','
          << p1_config_.evidence_run_id << ','
          << p1_config_.evidence_manifest_path << ','
          << p1_risk_context_.planning_attempt_id << ','
          << p1_risk_context_.candidate_id << ','
          << risk_snapshot_->generation_id() << ','
          << risk_query_base_time_s_ << ',' << phase << ',' << index << ',' << t_s << ','
          << point.x() << ',' << point.y() << ',' << point.z() << ','
          << (hit && sample.valid ? 1 : 0) << ','
          << (hit && sample.stale ? 1 : 0) << ',';
      if (hit && sample.valid) out << sample.cost;
      out << ',' << (hit ? sample.reason : "query_failed") << ','
          << (summary.full_support ? 1 : 0) << ",1,"
          << (selected ? 1 : 0) << '\n';
    }
    return out.good();
  }

  std::string BsplineOptimizer::p1CandidatePairwisePath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP1CandidatePairwiseCsvName
        : siblingPath(p1_config_.debug_csv_path, kP1CandidatePairwiseCsvName);
  }

  std::string BsplineOptimizer::p1OptimizerCheckpointPath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP1OptimizerCheckpointCsvName
        : siblingPath(p1_config_.debug_csv_path, kP1OptimizerCheckpointCsvName);
  }

  std::string BsplineOptimizer::p0OccupancyQueryEvidencePath() const
  {
    return p1_config_.debug_csv_path.empty()
        ? kP0OccupancyQueryEvidenceCsvName
        : siblingPath(p1_config_.debug_csv_path, kP0OccupancyQueryEvidenceCsvName);
  }

  void BsplineOptimizer::writeP1CandidateOptimizationCsv(
      const P1OptimizationTrace &trace) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty())
      return;
    const std::string path = p1CandidateOptimizationPath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,stamp,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,"
             "pre_base_objective,post_base_objective,pre_total_objective,post_total_objective,"
             "raw_p1_cost,weighted_p1_cost,base_gradient_norm,p1_gradient_norm,total_gradient_norm,"
             "displacement_norm,grad_integrity_dot_displacement,weighted_p1_gradient_dot_displacement,total_gradient_dot_displacement,pre_mean_c_pi,pre_max_c_pi,"
             "post_mean_c_pi,post_max_c_pi,selected,solver_result,termination_reason,iteration_count,"
             "objective_allowed,objective_applied,fallback_reason,max_candidates_per_attempt,"
             "pre_raw_p1_cost,post_raw_p1_cost,pre_weighted_p1_cost,post_weighted_p1_cost,"
             "pre_base_gradient_norm,post_base_gradient_norm,"
             "pre_raw_p1_gradient_norm,post_raw_p1_gradient_norm,"
             "pre_weighted_p1_gradient_norm,post_weighted_p1_gradient_norm,"
             "pre_total_gradient_norm,post_total_gradient_norm,"
             "support_sample_count,pre_support_valid_count,post_support_valid_count,"
             "pre_support_coverage,post_support_coverage,support_full_valid,"
             "support_signature,initial_control_points_hash,final_control_points_hash,p1_config_hash,"
             "optimization_success,selection_score,selection_reason,candidate_rank,p1_descent,rank_eligible,replacement_accepted,replacement_reason,incumbent_available,incumbent_mean_c_pi,incumbent_max_c_pi,"
             "replacement_comparison_mode,replacement_comparison_duration_s,replacement_candidate_mean_c_pi,replacement_candidate_max_c_pi,"
             "aggregation_mode,aggregation_temperature,aggregation_tail_fraction,adaptive_sample_count,fixed_sample_count,peak_contribution,"
             "pre_full_base_gradient_norm,post_full_base_gradient_norm,pre_full_raw_p1_gradient_norm,post_full_raw_p1_gradient_norm,"
             "pre_normalized_weighted_p1_gradient_norm,post_normalized_weighted_p1_gradient_norm,pre_base_p1_cosine,post_base_p1_cosine,"
             "pre_full_normalized_weighted_p1_gradient_norm,post_full_normalized_weighted_p1_gradient_norm,pre_full_total_gradient_norm,post_full_total_gradient_norm,"
             "pre_normalized_p1_cost,post_normalized_p1_cost,pre_anchor_cost,post_anchor_cost,"
             "normalization_mode,normalization_reference_lambda,normalization_scale,normalization_budget_fraction,normalization_base_improvement_budget,normalization_reference_displacement_m,"
             "base_prepass_pre_objective,base_prepass_post_objective,base_prepass_duration_ms,base_prepass_solver_result,base_prepass_iteration_count,base_prepass_success,base_prepass_termination_reason,"
             "fanout_input_segments,fanout_surviving_segments,fanout_returned_count,fanout_configured_cap,fanout_truncated,fanout_optimizer_successes,fanout_full_support,fanout_p1_descent_eligible,fanout_supplemental_count,fanout_singleton_reason\n";
    }
    out << p1_config_.evidence_schema_version << ',' << p1_config_.evidence_run_id << ','
        << p1_config_.evidence_manifest_path << ',' << rclcpp::Clock().now().seconds() << ',' << trace.planning_attempt_id << ','
        << trace.candidate_id << ',' << trace.snapshot_generation_id << ','
        << trace.query_base_time_s << ',' << trace.pre_base_objective << ','
        << trace.post_base_objective << ',' << trace.pre_total_objective << ','
        << trace.post_total_objective << ',' << trace.raw_p1_cost << ','
        << trace.weighted_p1_cost << ',' << trace.base_gradient_norm << ','
        << trace.p1_gradient_norm << ',' << trace.total_gradient_norm << ','
        << trace.displacement_norm << ',' << trace.grad_integrity_dot_displacement << ','
        << trace.weighted_p1_gradient_dot_displacement << ','
        << trace.total_gradient_dot_displacement << ','
        << trace.pre_mean_c_pi << ',' << trace.pre_max_c_pi << ','
        << trace.post_mean_c_pi << ',' << trace.post_max_c_pi << ','
        << (trace.selected ? 1 : 0) << ',' << trace.solver_result << ','
        << trace.termination_reason << ',' << trace.iteration_count << ','
        << (trace.objective_allowed ? 1 : 0) << ','
        << (trace.objective_applied ? 1 : 0) << ','
        << trace.fallback_reason << ','
        << p1_config_.max_candidates_per_attempt << ','
        << trace.pre_raw_p1_cost << ',' << trace.post_raw_p1_cost << ','
        << trace.pre_weighted_p1_cost << ',' << trace.post_weighted_p1_cost << ','
        << trace.pre_base_gradient_norm << ',' << trace.post_base_gradient_norm << ','
        << trace.pre_raw_p1_gradient_norm << ',' << trace.post_raw_p1_gradient_norm << ','
        << trace.pre_weighted_p1_gradient_norm << ','
        << trace.post_weighted_p1_gradient_norm << ','
        << trace.pre_total_gradient_norm << ',' << trace.post_total_gradient_norm << ','
        << trace.support_sample_count << ',' << trace.pre_support_valid_count << ','
        << trace.post_support_valid_count << ',' << trace.pre_support_coverage << ','
        << trace.post_support_coverage << ',' << (trace.support_full_valid ? 1 : 0) << ','
        << trace.support_signature << ',' << trace.initial_control_points_hash << ','
        << trace.final_control_points_hash << ','
        << trace.p1_config_hash << ',' << (trace.optimization_success ? 1 : 0) << ','
        << trace.selection_score << ',' << trace.selection_reason << ','
        << trace.candidate_rank << ',' << (trace.p1_descent ? 1 : 0) << ','
        << (trace.rank_eligible ? 1 : 0) << ','
        << (trace.replacement_accepted ? 1 : 0) << ','
        << trace.replacement_reason << ',' << (trace.incumbent_available ? 1 : 0) << ','
        << trace.incumbent_mean_c_pi << ','
        << trace.incumbent_max_c_pi << ','
        << trace.replacement_comparison_mode << ','
        << trace.replacement_comparison_duration_s << ','
        << trace.replacement_candidate_mean_c_pi << ','
        << trace.replacement_candidate_max_c_pi << ','
        << trace.aggregation_mode << ',' << trace.aggregation_temperature << ','
        << trace.aggregation_tail_fraction << ','
        << trace.adaptive_sample_count << ',' << trace.fixed_sample_count << ','
        << trace.peak_contribution << ','
        << trace.pre_full_base_gradient_norm << ','
        << trace.post_full_base_gradient_norm << ','
        << trace.pre_full_raw_p1_gradient_norm << ','
        << trace.post_full_raw_p1_gradient_norm << ','
        << trace.pre_normalized_weighted_p1_gradient_norm << ','
        << trace.post_normalized_weighted_p1_gradient_norm << ','
        << trace.pre_base_p1_cosine << ',' << trace.post_base_p1_cosine << ','
        << trace.pre_full_normalized_weighted_p1_gradient_norm << ','
        << trace.post_full_normalized_weighted_p1_gradient_norm << ','
        << trace.pre_full_total_gradient_norm << ','
        << trace.post_full_total_gradient_norm << ','
        << trace.pre_normalized_p1_cost << ','
        << trace.post_normalized_p1_cost << ','
        << trace.pre_anchor_cost << ',' << trace.post_anchor_cost << ','
        << trace.normalization_mode << ','
        << trace.normalization_reference_lambda << ','
        << trace.normalization_scale << ','
        << trace.normalization_budget_fraction << ','
        << trace.normalization_base_improvement_budget << ','
        << trace.normalization_reference_displacement_m << ','
        << trace.base_prepass_pre_objective << ','
        << trace.base_prepass_post_objective << ','
        << trace.base_prepass_duration_ms << ','
        << trace.base_prepass_solver_result << ','
        << trace.base_prepass_iteration_count << ','
        << (trace.base_prepass_success ? 1 : 0) << ','
        << trace.base_prepass_termination_reason << ','
        << trace.fanout.input_topology_segments << ','
        << trace.fanout.surviving_topology_segments << ','
        << trace.fanout.returned_candidate_count << ','
        << trace.fanout.configured_cap << ',' << (trace.fanout.truncated ? 1 : 0) << ','
        << trace.fanout.optimizer_success_count << ','
        << trace.fanout.full_support_count << ','
        << trace.fanout.p1_descent_eligible_count << ','
        << trace.fanout.supplemental_candidate_count << ','
        << (trace.fanout.singleton_due_to_empty_segments ? "empty_segments" :
            trace.fanout.singleton_due_to_degenerate_segments ? "degenerate_segments" :
            trace.fanout.singleton_due_to_opposite_direction_unavailable ? "opposite_direction_unavailable" : "none")
        << '\n';
  }

  void BsplineOptimizer::setLastP1OptimizationSelected(const bool selected)
  {
    last_p1_optimization_trace_.selected = selected;
  }

  void BsplineOptimizer::writeP1CandidateSidecars(
      const P1OptimizationTrace &trace) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty())
      return;
    const auto key = std::make_pair(
        trace.planning_attempt_id, trace.candidate_id);
    const auto artifact_it = p1_candidate_artifacts_.find(key);
    if (artifact_it == p1_candidate_artifacts_.end())
      return;
    const auto &artifact = artifact_it->second;
    const auto append_stream = [](const std::string &path,
                                  const std::string &header) {
      std::ifstream existing(path);
      const bool write_header = !existing.good() ||
          existing.peek() == std::ifstream::traits_type::eof();
      existing.close();
      std::ofstream out(path, std::ios::app);
      out << std::setprecision(17);
      if (out.good() && write_header)
        out << header << '\n';
      return out;
    };
    const auto prefix = [&](std::ostream &out, const uint64_t candidate_id) {
      out << p1_config_.evidence_schema_version << ','
          << p1_config_.evidence_run_id << ','
          << p1_config_.evidence_manifest_path << ','
          << trace.planning_attempt_id << ',' << candidate_id << ','
          << trace.snapshot_generation_id << ',' << trace.query_base_time_s;
    };

    auto control_points = append_stream(
        p1CandidateControlPointsPath(),
        "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,phase,control_point_index,x,y,z,control_points_hash");
    if (control_points.good())
    {
      const auto write_points = [&](const char *phase,
                                    const Eigen::MatrixXd &points) {
        const std::string hash = matrixHash(points);
        for (int column = 0; column < points.cols(); ++column)
        {
          prefix(control_points, trace.candidate_id);
          control_points << ',' << phase << ',' << column << ','
              << points(0, column) << ',' << points(1, column) << ','
              << points(2, column) << ',' << hash << '\n';
        }
      };
      write_points("initial", artifact.initial_control_points);
      write_points("final", artifact.final_control_points);
    }

    std::ofstream profile = append_stream(
        p1CandidateProfilePath(),
        "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,phase,sample_index,t_s,x,y,z,valid,stale,c_pi,invalid_reason");
    std::ofstream occupancy = append_stream(
        p0OccupancyQueryEvidencePath(),
        "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,phase,sample_index,query_x,query_y,query_z,query_time_s,query_tau_s,query_reason,temporal_layer,horizon_id,horizon_s,temporal_weight,corner_id,corner_weight,corner_ix,corner_iy,corner_iz,corner_x,corner_y,corner_z,source_flags,c_pi,invalid_reason,occupancy_available,raw_occupied,inflated_occupied,occupancy_ix,occupancy_iy,occupancy_iz,occupancy_x,occupancy_y,occupancy_z,resolution_m,inflation_m,frame_id,cloud_stamp_s,occupancy_generation,occupancy_source");
    const auto sample_points = [&](const P1CandidateArtifact &source_artifact,
                                   const uint64_t candidate_id,
                                   const char *phase,
                                   const Eigen::MatrixXd &points,
                                   std::vector<double> *costs,
                                   std::string *failure,
                                   const bool emit_rows) {
      if (!source_artifact.snapshot || points.rows() != 3 || points.cols() <= order_ ||
          !std::isfinite(source_artifact.interval_s) || source_artifact.interval_s <= 0.0)
      {
        if (failure) *failure = "artifact_context_unavailable";
        return false;
      }
      UniformBspline trajectory(points, 3, source_artifact.interval_s);
      const double duration = trajectory.getTimeSum();
      bool all_valid = true;
      if (costs) costs->clear();
      for (int sample_index = 0;
           sample_index < kP1CandidateEvidenceSampleCount; ++sample_index)
      {
        const double fraction = static_cast<double>(sample_index) /
            static_cast<double>(kP1CandidateEvidenceSampleCount - 1);
        const double t_s = duration * fraction;
        const Eigen::Vector3d point = trajectory.evaluateDeBoorT(t_s);
        iap::RiskCostSample sample;
        iap::RiskCostQueryTrace query_trace;
        const bool valid = source_artifact.snapshot->queryCost(
            point, source_artifact.query_base_time_s + t_s, &sample, &query_trace) &&
            sample.valid;
        all_valid = all_valid && valid;
        if (costs)
          costs->push_back(valid ? sample.cost :
              std::numeric_limits<double>::quiet_NaN());
        if (emit_rows && profile.good())
        {
          prefix(profile, candidate_id);
          profile << ',' << phase << ',' << sample_index << ',' << t_s << ','
              << point.x() << ',' << point.y() << ',' << point.z() << ','
              << (valid ? 1 : 0) << ',' << (sample.stale ? 1 : 0) << ',';
          if (valid) profile << sample.cost;
          profile << ',' << (valid ? "none" : sample.reason) << '\n';
        }
        if (emit_rows && occupancy.good())
        {
          for (const auto &corner : query_trace.corners)
          {
            prefix(occupancy, candidate_id);
            occupancy << ',' << phase << ',' << sample_index << ','
                << point.x() << ',' << point.y() << ',' << point.z() << ','
                << source_artifact.query_base_time_s + t_s << ','
                << query_trace.query_tau_s << ',' << query_trace.reason << ','
                << corner.temporal_layer << ',' << corner.horizon_id << ','
                << corner.horizon_s << ',' << corner.temporal_weight << ','
                << corner.corner_id << ',' << corner.spatial_weight << ','
                << corner.voxel_index.x() << ',' << corner.voxel_index.y() << ','
                << corner.voxel_index.z() << ',' << corner.voxel_position.x() << ','
                << corner.voxel_position.y() << ',' << corner.voxel_position.z() << ','
                << corner.source_flags << ',';
            if (std::isfinite(corner.c_pi)) occupancy << corner.c_pi;
            occupancy << ',' << corner.invalid_reason << ','
                << (corner.occupancy.available ? 1 : 0) << ','
                << (corner.occupancy.raw_occupied ? 1 : 0) << ','
                << (corner.occupancy.inflated_occupied ? 1 : 0) << ','
                << corner.occupancy.voxel_index.x() << ','
                << corner.occupancy.voxel_index.y() << ','
                << corner.occupancy.voxel_index.z() << ','
                << corner.occupancy.voxel_center.x() << ','
                << corner.occupancy.voxel_center.y() << ','
                << corner.occupancy.voxel_center.z() << ','
                << corner.occupancy.resolution_m << ','
                << corner.occupancy.inflation_m << ','
                << corner.occupancy.frame_id << ','
                << corner.occupancy.cloud_stamp_s << ','
                << corner.occupancy.occupancy_generation << ','
                << corner.occupancy.source << '\n';
          }
        }
      }
      if (!all_valid && failure) *failure = "fixed_support_not_full";
      return all_valid;
    };
    std::vector<double> ignored_costs;
    std::string ignored_failure;
    sample_points(artifact, trace.candidate_id, "initial",
                  artifact.initial_control_points, &ignored_costs,
                  &ignored_failure, true);
    sample_points(artifact, trace.candidate_id, "final",
                  artifact.final_control_points, &ignored_costs,
                  &ignored_failure, true);

    auto checkpoints = append_stream(
        p1OptimizerCheckpointPath(),
        "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,stage,checkpoint,restart_index,iteration,line_search_count,step,objective,base_objective,raw_p1_objective,normalized_p1_objective,anchor_objective,x_norm,gradient_norm,directional_derivative,solver_result,reason");
    if (checkpoints.good())
    {
      const auto write_checkpoint = [&](const P1OptimizerCheckpoint &item) {
        prefix(checkpoints, trace.candidate_id);
        checkpoints << ',' << item.stage << ',' << item.checkpoint << ','
            << item.restart_index << ',' << item.iteration << ','
            << item.line_search_count << ',' << item.step << ','
            << item.objective << ',' << item.base_objective << ','
            << item.raw_p1_objective << ',' << item.normalized_p1_objective << ','
            << item.anchor_objective << ',' << item.x_norm << ','
            << item.gradient_norm << ',' << item.directional_derivative << ','
            << item.solver_result << ',' << item.reason << '\n';
      };
      if (std::isfinite(trace.base_prepass_pre_objective) &&
          std::isfinite(trace.base_prepass_post_objective))
      {
        P1OptimizerCheckpoint base_start;
        base_start.stage = "base_prepass";
        base_start.checkpoint = "start";
        base_start.objective = trace.base_prepass_pre_objective;
        base_start.base_objective = trace.base_prepass_pre_objective;
        write_checkpoint(base_start);
        P1OptimizerCheckpoint base_end;
        base_end.stage = "base_prepass";
        base_end.checkpoint = "terminal";
        base_end.iteration = trace.base_prepass_iteration_count;
        base_end.objective = trace.base_prepass_post_objective;
        base_end.base_objective = trace.base_prepass_post_objective;
        base_end.solver_result = trace.base_prepass_solver_result;
        base_end.reason = trace.base_prepass_termination_reason;
        write_checkpoint(base_end);
      }
      for (const auto &item : artifact.checkpoints)
        write_checkpoint(item);
    }

    if (!trace.selected)
      return;
    auto pairwise = append_stream(
        p1CandidatePairwisePath(),
        "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id_a,candidate_id_b,snapshot_generation_id,query_base_time_s,phase,control_point_distance,risk_profile_distance,profile_valid,invalid_reason");
    if (!pairwise.good())
      return;
    std::vector<std::pair<std::pair<uint64_t, uint64_t>, const P1CandidateArtifact *>> attempt_artifacts;
    for (const auto &item : p1_candidate_artifacts_)
      if (item.first.first == trace.planning_attempt_id)
        attempt_artifacts.push_back({item.first, &item.second});
    for (std::size_t a = 0; a < attempt_artifacts.size(); ++a)
      for (std::size_t b = a; b < attempt_artifacts.size(); ++b)
        for (const char *phase : {"initial", "final"})
        {
          const auto &left = *attempt_artifacts[a].second;
          const auto &right = *attempt_artifacts[b].second;
          const Eigen::MatrixXd &left_points = std::string(phase) == "initial"
              ? left.initial_control_points : left.final_control_points;
          const Eigen::MatrixXd &right_points = std::string(phase) == "initial"
              ? right.initial_control_points : right.final_control_points;
          const bool point_shapes_match =
              left_points.rows() == right_points.rows() &&
              left_points.cols() == right_points.cols();
          std::vector<double> left_costs, right_costs;
          std::string left_failure, right_failure;
          const bool left_valid = sample_points(
              left, attempt_artifacts[a].first.second, phase, left_points,
              &left_costs, &left_failure, false);
          const bool right_valid = sample_points(
              right, attempt_artifacts[b].first.second, phase, right_points,
              &right_costs, &right_failure, false);
          double profile_distance = 0.0;
          if (left_valid && right_valid && left_costs.size() == right_costs.size())
          {
            for (std::size_t index = 0; index < left_costs.size(); ++index)
              profile_distance += std::pow(
                  left_costs[index] - right_costs[index], 2);
            profile_distance = std::sqrt(profile_distance /
                static_cast<double>(std::max<std::size_t>(1, left_costs.size())));
          }
          pairwise << p1_config_.evidence_schema_version << ','
              << p1_config_.evidence_run_id << ','
              << p1_config_.evidence_manifest_path << ','
              << trace.planning_attempt_id << ','
              << attempt_artifacts[a].first.second << ','
              << attempt_artifacts[b].first.second << ','
              << trace.snapshot_generation_id << ',' << trace.query_base_time_s << ','
              << phase << ',';
          if (point_shapes_match)
            pairwise << (left_points - right_points).norm();
          pairwise << ',';
          if (left_valid && right_valid) pairwise << profile_distance;
          const bool pair_valid = point_shapes_match && left_valid && right_valid;
          pairwise << ',' << (pair_valid ? 1 : 0) << ','
              << (!point_shapes_match ? "control_point_shape_mismatch" :
                  pair_valid ? "none" :
                  !left_valid ? left_failure : right_failure) << '\n';
        }
  }

  void BsplineOptimizer::writeP1OptimizationTrace(
      const P1OptimizationTrace &trace) const
  {
    writeP1CandidateOptimizationCsv(trace);
    writeP1CandidateSidecars(trace);
  }

  void BsplineOptimizer::writeP1ReplacementDecision(
      const P1OptimizationTrace &trace, const uint64_t incumbent_trajectory_id,
      const double incumbent_start_stamp_s,
      const std::string &final_trajectory_source,
      const std::string &publish_identity) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty())
      return;
    const std::string path = p1ReplacementDecisionPath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,stamp,planning_attempt_id,optimizer_selected_candidate_id,rejected_candidate_id,snapshot_generation_id,query_base_time_s,"
             "replacement_accepted,replacement_reason,incumbent_trajectory_id,incumbent_start_stamp_s,incumbent_mean_c_pi,incumbent_max_c_pi,"
             "candidate_mean_c_pi,candidate_max_c_pi,replacement_comparison_mode,replacement_comparison_duration_s,replacement_candidate_mean_c_pi,replacement_candidate_max_c_pi,final_trajectory_source,publish_identity\n";
    }
    out << p1_config_.evidence_schema_version << ',' << p1_config_.evidence_run_id << ','
        << p1_config_.evidence_manifest_path << ',' << rclcpp::Clock().now().seconds() << ','
        << trace.planning_attempt_id << ',' << trace.candidate_id << ','
        << (trace.replacement_accepted ? 0 : trace.candidate_id) << ','
        << trace.snapshot_generation_id << ',' << trace.query_base_time_s << ','
        << (trace.replacement_accepted ? 1 : 0) << ',' << trace.replacement_reason << ','
        << incumbent_trajectory_id << ',' << incumbent_start_stamp_s << ','
        << trace.incumbent_mean_c_pi << ',' << trace.incumbent_max_c_pi << ','
        << trace.post_mean_c_pi << ',' << trace.post_max_c_pi << ','
        << trace.replacement_comparison_mode << ','
        << trace.replacement_comparison_duration_s << ','
        << trace.replacement_candidate_mean_c_pi << ','
        << trace.replacement_candidate_max_c_pi << ','
        << final_trajectory_source << ',' << publish_identity << '\n';
  }

  bool BsplineOptimizer::writeP1CandidateRetainedProfile(
      UniformBspline candidate, const uint64_t planning_attempt_id,
      const uint64_t candidate_id,
      const UniformBspline *incumbent, const uint64_t incumbent_trajectory_id,
      const std::string &final_trajectory_source,
      const double incumbent_start_t_s) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty() ||
        !risk_snapshot_ || !std::isfinite(risk_query_base_time_s_))
      return false;
    const std::string path = p1CandidateRetainedProfilePath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return false;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,trajectory_role,trajectory_id,snapshot_generation_id,query_base_time_s,final_trajectory_source,sample_index,t_s,x,y,z,c_pi,valid,stale,reason\n";
    }
    const auto write_trajectory = [&](UniformBspline trajectory,
                                      const char *role, uint64_t trajectory_id,
                                      double source_start_t_s) {
      const double total_duration = trajectory.getTimeSum();
      if (!std::isfinite(total_duration) || total_duration < 0.0 ||
          !std::isfinite(source_start_t_s))
        return false;
      source_start_t_s = std::clamp(source_start_t_s, 0.0, total_duration);
      const double duration = total_duration - source_start_t_s;
      for (int index = 0; index < kP1CandidateEvidenceSampleCount; ++index) {
        const double fraction = static_cast<double>(index) /
            static_cast<double>(kP1CandidateEvidenceSampleCount - 1);
        const double t_s = duration * fraction;
        const Eigen::Vector3d point = trajectory.evaluateDeBoorT(
            source_start_t_s + t_s);
        iap::RiskCostSample sample;
        const bool hit = risk_snapshot_->queryCost(
            point, risk_query_base_time_s_ + t_s, &sample);
        out << p1_config_.evidence_schema_version << ',' << p1_config_.evidence_run_id << ','
            << p1_config_.evidence_manifest_path << ','
            << planning_attempt_id << ',' << candidate_id << ','
            << role << ',' << trajectory_id << ',' << risk_snapshot_->generation_id() << ','
            << risk_query_base_time_s_ << ',' << final_trajectory_source << ','
            << index << ',' << t_s << ',' << point.x() << ',' << point.y() << ','
            << point.z() << ',' << sample.cost << ','
            << (hit && sample.valid ? 1 : 0) << ',' << (sample.stale ? 1 : 0) << ','
            << sample.reason << '\n';
      }
      return true;
    };
    if (!write_trajectory(
            candidate, "optimizer_selected_candidate", candidate_id, 0.0))
      return false;
    return !incumbent || write_trajectory(
        *incumbent, "retained_incumbent", incumbent_trajectory_id,
        incumbent_start_t_s);
  }

  std::string BsplineOptimizer::p1AcceptedTrajectoryRiskProfilePath() const
  {
    if (p1_config_.debug_csv_path.empty())
    {
      return kP1AcceptedProfileCsvName;
    }
    return siblingPath(p1_config_.debug_csv_path, kP1AcceptedProfileCsvName);
  }

  std::string BsplineOptimizer::p1AcceptedTrajectoryRiskProfileContextPath() const
  {
    if (p1_config_.debug_csv_path.empty())
    {
      return kP1AcceptedProfileContextCsvName;
    }
    return siblingPath(p1_config_.debug_csv_path, kP1AcceptedProfileContextCsvName);
  }

  void BsplineOptimizer::captureP1PreOptimizationTrajectory(
      const Eigen::MatrixXd &control_points, const double interval_s)
  {
    if (control_points.rows() != 3 || control_points.cols() <= order_ ||
        !std::isfinite(interval_s) || interval_s <= 0.0)
      return;
    P1PreOptimizationTrace trace;
    trace.control_points = control_points;
    trace.interval_s = interval_s;
    p1_pre_optimization_traces_[
        {p1_risk_context_.planning_attempt_id, p1_risk_context_.candidate_id}] =
        std::move(trace);
  }

  void BsplineOptimizer::captureP1PostOptimizationTrajectory(
      const Eigen::MatrixXd &control_points, const double interval_s)
  {
    const auto key = std::make_pair(
        p1_risk_context_.planning_attempt_id, p1_risk_context_.candidate_id);
    auto &artifact = p1_candidate_artifacts_[key];
    const auto pre = p1_pre_optimization_traces_.find(key);
    artifact.initial_control_points =
        pre != p1_pre_optimization_traces_.end()
            ? pre->second.control_points : control_points;
    artifact.final_control_points = control_points;
    artifact.interval_s = interval_s;
    artifact.snapshot = risk_snapshot_;
    artifact.query_base_time_s = risk_query_base_time_s_;
    artifact.checkpoints = current_p1_checkpoints_;
  }

  void BsplineOptimizer::setP1PreOptimizationTrajectoryForTest(
      const Eigen::MatrixXd &control_points, const double interval_s)
  {
    captureP1PreOptimizationTrajectory(control_points, interval_s);
  }

  bool BsplineOptimizer::writeP1AcceptedTrajectoryRiskProfile(
      UniformBspline trajectory,
      const uint64_t profile_seq,
      const uint64_t trajectory_id,
      const double stamp_s,
      const double planning_start_s,
      const std::string &trajectory_frame_id,
      const double trajectory_start_stamp_s,
      const double trajectory_start_t_s,
      const double window_duration_s) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty())
    {
      return false;
    }

    const double full_duration = trajectory.getTimeSum();
    if (!std::isfinite(full_duration) || full_duration < 0.0 ||
        !std::isfinite(trajectory_start_t_s) || trajectory_start_t_s < 0.0 ||
        std::isnan(window_duration_s) || window_duration_s < 0.0)
    {
      return false;
    }
    const double segment_start_t_s = std::clamp(
        trajectory_start_t_s, 0.0, full_duration);
    const double remaining_duration = full_duration - segment_start_t_s;
    const double duration = std::isfinite(window_duration_s)
        ? std::min(window_duration_s, remaining_duration)
        : remaining_duration;

    const std::string profile_path = p1AcceptedTrajectoryRiskProfilePath();
    std::ifstream existing(profile_path);
    const bool write_header =
        !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    existing.close();

    std::ofstream out(profile_path, std::ios::app);
    if (!out.good())
    {
      return false;
    }
    out << std::setprecision(17);
    if (write_header)
    {
      out << "schema_version,run_id,manifest_path,profile_seq,stamp,trajectory_id,planning_attempt_id,candidate_id,applied_to_objective,metrics_only,"
             "lambda_integrity,snapshot_generation_id,query_base_time_s,"
             "sample_index,arc_fraction,t_s,x,y,z,hit,valid,stale,base_collision_occupied,c_pi,reason,"
             "trace_available,grad_x,grad_y,grad_z,neg_grad_x,neg_grad_y,neg_grad_z,"
             "pre_x,pre_y,pre_z,disp_x,disp_y,disp_z,grad_dot_displacement,delta_c_pi,"
             "objective_requested,objective_applied,p1_fallback,fallback_reason\n";
    }

    // Candidate sidecars normally emit this trace during optimizer starts.
    // A strict-support fallback has no candidate artifact, but its occupied
    // interpolation corners are exactly the evidence needed to distinguish a
    // legitimate P0 support miss from a point-wise collision/frame mismatch.
    // Keep the same v4 schema and append accepted-profile queries under an
    // explicit phase rather than fabricating an optimizer candidate.
    std::ofstream occupancy;
    if (risk_snapshot_ &&
        p1_config_.evidence_schema_version == "p1_evidence_provenance_v4")
    {
      const std::string occupancy_path = p0OccupancyQueryEvidencePath();
      std::ifstream occupancy_existing(occupancy_path);
      const bool occupancy_header = !occupancy_existing.good() ||
          occupancy_existing.peek() == std::ifstream::traits_type::eof();
      occupancy_existing.close();
      occupancy.open(occupancy_path, std::ios::app);
      if (occupancy.good())
      {
        occupancy << std::setprecision(17);
        if (occupancy_header)
        {
          occupancy << "schema_version,run_id,manifest_path,planning_attempt_id,candidate_id,snapshot_generation_id,query_base_time_s,phase,sample_index,query_x,query_y,query_z,query_time_s,query_tau_s,query_reason,temporal_layer,horizon_id,horizon_s,temporal_weight,corner_id,corner_weight,corner_ix,corner_iy,corner_iz,corner_x,corner_y,corner_z,source_flags,c_pi,invalid_reason,occupancy_available,raw_occupied,inflated_occupied,occupancy_ix,occupancy_iy,occupancy_iz,occupancy_x,occupancy_y,occupancy_z,resolution_m,inflation_m,frame_id,cloud_stamp_s,occupancy_generation,occupancy_source\n";
        }
      }
    }

    const bool applied_to_objective =
        p1_config_.use_integrity_cost && !p1_config_.metrics_only &&
        p1_risk_context_.objective_allowed &&
        p1_config_.lambda_integrity != 0.0;
    const bool objective_requested =
        p1_config_.use_integrity_cost && !p1_config_.metrics_only &&
        p1_config_.lambda_integrity != 0.0;
    const int denom = std::max(1, kP1AcceptedProfileSampleCount - 1);
    int matched_count = 0;
    int query_miss_count = 0;
    int stale_count = 0;
    int invalid_count = 0;
    Eigen::Vector3d trajectory_min = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d trajectory_max = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    std::vector<iap::P1AcceptedContextSample> validation_samples;
    validation_samples.reserve(kP1AcceptedProfileSampleCount);
    std::map<std::string, int> miss_reasons;
    const auto trace_it = p1_pre_optimization_traces_.find(
        {p1_risk_context_.planning_attempt_id, p1_risk_context_.candidate_id});
    const bool trace_available =
        trace_it != p1_pre_optimization_traces_.end() &&
        trace_it->second.control_points.rows() == 3 &&
        trace_it->second.control_points.cols() > order_ &&
        std::isfinite(trace_it->second.interval_s) &&
        trace_it->second.interval_s > 0.0;
    std::unique_ptr<UniformBspline> pre_trajectory;
    double pre_duration = std::numeric_limits<double>::quiet_NaN();
    if (trace_available)
    {
      pre_trajectory = std::make_unique<UniformBspline>(
          trace_it->second.control_points, order_, trace_it->second.interval_s);
      pre_duration = pre_trajectory->getTimeSum();
    }
    for (int sample_index = 0; sample_index < kP1AcceptedProfileSampleCount;
         ++sample_index)
    {
      const double arc_fraction =
          static_cast<double>(sample_index) / static_cast<double>(denom);
      const double t_s = duration * arc_fraction;
      const Eigen::Vector3d p = trajectory.evaluateDeBoorT(
          segment_start_t_s + t_s);
      trajectory_min = trajectory_min.cwiseMin(p);
      trajectory_max = trajectory_max.cwiseMax(p);
      iap::RiskCostSample sample;
      iap::RiskCostQueryTrace query_trace;
      const bool hit = risk_snapshot_ &&
          risk_snapshot_->queryCost(
              p, risk_query_base_time_s_ + t_s, &sample, &query_trace);
      const bool c_pi_finite =
          hit && sample.valid && !sample.stale && std::isfinite(sample.cost);
      // Rebound's collision phase queries this same inflated occupancy map.
      // Persist the point-wise predicate beside the P0 query so an occupied
      // strict-support miss can be distinguished from a recorder/frame issue.
      const bool base_collision_occupied = grid_map_ &&
          grid_map_->getInflateOccupancy(p);
      Eigen::Vector3d evidence_grad = sample.grad;
      Eigen::Vector3d pre_position = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      Eigen::Vector3d displacement = pre_position;
      double grad_dot_displacement = std::numeric_limits<double>::quiet_NaN();
      double delta_c_pi = std::numeric_limits<double>::quiet_NaN();
      if (pre_trajectory && std::isfinite(pre_duration))
      {
        const double pre_segment_start_t_s = std::clamp(
            segment_start_t_s, 0.0, pre_duration);
        const double pre_window_duration = std::min(
            duration, pre_duration - pre_segment_start_t_s);
        const double pre_t_s = pre_segment_start_t_s +
            pre_window_duration * arc_fraction;
        pre_position = pre_trajectory->evaluateDeBoorT(pre_t_s);
        iap::RiskCostSample pre_sample;
        const bool pre_hit = risk_snapshot_ && risk_snapshot_->queryCost(
            pre_position, risk_query_base_time_s_ + pre_t_s, &pre_sample);
        const bool pre_cost_finite = pre_hit && pre_sample.valid &&
            !pre_sample.stale && std::isfinite(pre_sample.cost) &&
            pre_sample.grad.allFinite();
        if (pre_cost_finite && c_pi_finite)
        {
          evidence_grad = pre_sample.grad;
          displacement = p - pre_position;
          grad_dot_displacement = evidence_grad.dot(displacement);
          delta_c_pi = sample.cost - pre_sample.cost;
        }
      }
      const double grad_norm = evidence_grad.norm();
      Eigen::Vector3d negative_gradient = Eigen::Vector3d::Zero();
      if (evidence_grad.allFinite() && std::isfinite(grad_norm) &&
          grad_norm > 1.0e-12)
        negative_gradient = -evidence_grad / grad_norm;
      std::string reason = !risk_snapshot_
          ? "snapshot_unavailable"
          : (sample.reason.empty() ? "query_miss" : sample.reason);
      if (!hit && reason == "not_evaluated")
      {
        reason = "query_miss";
      }
      if (c_pi_finite)
        ++matched_count;
      else if (sample.stale)
        ++stale_count;
      else if (!hit)
        ++query_miss_count;
      else
        ++invalid_count;
      if (!c_pi_finite)
        ++miss_reasons[reason];

      iap::P1AcceptedContextSample validation_sample;
      validation_sample.position_w = p;
      validation_sample.trajectory_time_s = t_s;
      validation_sample.query_hit = hit;
      validation_sample.query_valid = sample.valid;
      validation_sample.query_stale = sample.stale;
      validation_sample.query_reason = reason;
      validation_samples.push_back(std::move(validation_sample));

      if (occupancy.good())
      {
        for (const auto &corner : query_trace.corners)
        {
          occupancy << p1_config_.evidence_schema_version << ','
              << p1_config_.evidence_run_id << ','
              << p1_config_.evidence_manifest_path << ','
              << p1_risk_context_.planning_attempt_id << ','
              << p1_risk_context_.candidate_id << ','
              << risk_snapshot_->generation_id() << ','
              << risk_query_base_time_s_ << ",accepted," << sample_index << ','
              << p.x() << ',' << p.y() << ',' << p.z() << ','
              << risk_query_base_time_s_ + t_s << ','
              << query_trace.query_tau_s << ',' << query_trace.reason << ','
              << corner.temporal_layer << ',' << corner.horizon_id << ','
              << corner.horizon_s << ',' << corner.temporal_weight << ','
              << corner.corner_id << ',' << corner.spatial_weight << ','
              << corner.voxel_index.x() << ',' << corner.voxel_index.y() << ','
              << corner.voxel_index.z() << ',' << corner.voxel_position.x() << ','
              << corner.voxel_position.y() << ',' << corner.voxel_position.z() << ','
              << corner.source_flags << ',';
          if (std::isfinite(corner.c_pi))
            occupancy << corner.c_pi;
          occupancy << ',' << corner.invalid_reason << ','
              << (corner.occupancy.available ? 1 : 0) << ','
              << (corner.occupancy.raw_occupied ? 1 : 0) << ','
              << (corner.occupancy.inflated_occupied ? 1 : 0) << ','
              << corner.occupancy.voxel_index.x() << ','
              << corner.occupancy.voxel_index.y() << ','
              << corner.occupancy.voxel_index.z() << ','
              << corner.occupancy.voxel_center.x() << ','
              << corner.occupancy.voxel_center.y() << ','
              << corner.occupancy.voxel_center.z() << ','
              << corner.occupancy.resolution_m << ','
              << corner.occupancy.inflation_m << ','
              << corner.occupancy.frame_id << ','
              << corner.occupancy.cloud_stamp_s << ','
              << corner.occupancy.occupancy_generation << ','
              << corner.occupancy.source << '\n';
        }
      }

      out << p1_config_.evidence_schema_version << ','
          << p1_config_.evidence_run_id << ','
          << p1_config_.evidence_manifest_path << ','
          << profile_seq << ','
          << stamp_s << ','
          << trajectory_id << ','
          << p1_risk_context_.planning_attempt_id << ','
          << p1_risk_context_.candidate_id << ','
          << (applied_to_objective ? 1 : 0) << ','
          << (p1_config_.metrics_only ? 1 : 0) << ','
          << p1_config_.lambda_integrity << ','
          << (risk_snapshot_ ? risk_snapshot_->generation_id() : 0) << ','
          << risk_query_base_time_s_ << ','
          << sample_index << ','
          << arc_fraction << ','
          << t_s << ','
          << p.x() << ','
          << p.y() << ','
          << p.z() << ','
          << (hit ? 1 : 0) << ','
          << (sample.valid ? 1 : 0) << ','
          << (sample.stale ? 1 : 0) << ','
          << (base_collision_occupied ? 1 : 0) << ',';
      if (c_pi_finite)
      {
        out << sample.cost;
      }
      out << ',' << reason << ',' << (trace_available ? 1 : 0) << ','
          << evidence_grad.x() << ',' << evidence_grad.y() << ','
          << evidence_grad.z() << ',' << negative_gradient.x() << ','
          << negative_gradient.y() << ',' << negative_gradient.z();
      for (int axis = 0; axis < 3; ++axis)
      {
        out << ',';
        if (pre_position.allFinite())
          out << pre_position(axis);
      }
      for (int axis = 0; axis < 3; ++axis)
      {
        out << ',';
        if (displacement.allFinite())
          out << displacement(axis);
      }
      out << ',';
      if (std::isfinite(grad_dot_displacement))
        out << grad_dot_displacement;
      out << ',';
      if (std::isfinite(delta_c_pi))
        out << delta_c_pi;
      out << ',' << (objective_requested ? 1 : 0)
          << ',' << (applied_to_objective ? 1 : 0)
          << ',' << (applied_to_objective ? 0 : 1)
          << ',' << p1_risk_context_.fallback_reason;
      out << '\n';
    }
    out.close();

    const std::string context_path = p1AcceptedTrajectoryRiskProfileContextPath();
    // Keep every completed accepted-profile binding.  The analyzer needs the
    // history to select the latest trajectory that actually crossed the DDS
    // recorder boundary; launch shutdown can otherwise leave one newer CSV
    // profile whose already-published message did not reach rosbag.  Rewrite
    // the complete history atomically so an interrupted writer can expose
    // neither a partial row nor a false binding.
    std::string existing_context;
    {
      std::ifstream context_file(context_path);
      if (context_file.good())
      {
        std::ostringstream buffer;
        buffer << context_file.rdbuf();
        existing_context = buffer.str();
      }
    }
    std::ostringstream context;
    iap::P1AcceptedContextValidationInput validation_input;
    validation_input.snapshot = risk_snapshot_;
    validation_input.snapshot_frame_id = risk_snapshot_
        ? risk_snapshot_->params().frame_id : "";
    validation_input.trajectory_frame_id = trajectory_frame_id;
    validation_input.expected_generation_id = risk_snapshot_
        ? risk_snapshot_->generation_id() : 0;
    validation_input.query_base_time_s = risk_query_base_time_s_;
    validation_input.accepted_stamp_s = stamp_s;
    validation_input.samples = validation_samples;
    const auto validation = iap::validateP1AcceptedContext(validation_input);
    const Eigen::Vector3d min_bound = validation.interpolation_min;
    const Eigen::Vector3d max_bound = validation.interpolation_max;
    const std::vector<double> empty_horizons;
    const auto &horizons = risk_snapshot_
        ? risk_snapshot_->params().horizons_s : empty_horizons;
    const auto horizon_minmax = std::minmax_element(horizons.begin(), horizons.end());
    const double tau_min = horizon_minmax.first == horizons.end() ? 0.0 : *horizon_minmax.first;
    const double tau_max = horizon_minmax.second == horizons.end() ? 0.0 : *horizon_minmax.second;
    const Eigen::Vector3d center = 0.5 * (min_bound + max_bound);
    const double immutable_planning_start_s =
        std::isfinite(p1_risk_context_.planning_start_s)
            ? p1_risk_context_.planning_start_s
            : planning_start_s;
    context << std::setprecision(17);
    if (existing_context.empty())
    {
      context << "schema_version,run_id,manifest_path,profile_seq,trajectory_id,planning_attempt_id,candidate_id,planning_start_s,accepted_stamp_s,planning_duration_s,snapshot_generation_id,snapshot_stamp_s,query_base_time_s,snapshot_center_x,snapshot_center_y,snapshot_center_z,snapshot_x_min,snapshot_x_max,snapshot_y_min,snapshot_y_max,snapshot_z_min,snapshot_z_max,snapshot_time_min_s,snapshot_time_max_s,trajectory_x_min,trajectory_x_max,trajectory_y_min,trajectory_y_max,trajectory_z_min,trajectory_z_max,trajectory_time_min_s,trajectory_time_max_s,expected_sample_count,matched_sample_count,match_ratio,query_miss_count,stale_count,invalid_count,miss_reason_histogram,snapshot_frame_id,trajectory_frame_id,spatial_in_bounds,temporal_in_horizon,frame_match,generation_match,query_time_match,fresh,coverage_ok,spatial_miss_count,temporal_miss_count,occupied_miss_count,stale_miss_count,invalid_miss_count,trajectory_start_stamp_s,objective_requested,objective_applied,p1_fallback,fallback_reason\n";
    }
    else
    {
      context << existing_context;
      if (existing_context.back() != '\n') context << '\n';
    }
    std::ostringstream miss_reason_histogram;
    bool first_reason = true;
    for (const auto &entry : miss_reasons)
    {
      if (!first_reason) miss_reason_histogram << '|';
      miss_reason_histogram << entry.first << ':' << entry.second;
      first_reason = false;
    }
    context << p1_config_.evidence_schema_version << ','
            << p1_config_.evidence_run_id << ','
            << p1_config_.evidence_manifest_path << ','
            << profile_seq << ',' << trajectory_id << ','
            << p1_risk_context_.planning_attempt_id << ',' << p1_risk_context_.candidate_id << ','
            << immutable_planning_start_s << ',' << stamp_s << ','
            << (std::isfinite(immutable_planning_start_s) ? stamp_s - immutable_planning_start_s : std::numeric_limits<double>::quiet_NaN()) << ','
            << (risk_snapshot_ ? risk_snapshot_->generation_id() : 0) << ','
            << (risk_snapshot_ ? risk_snapshot_->stamp_s() : std::numeric_limits<double>::quiet_NaN()) << ',' << risk_query_base_time_s_ << ','
            << center.x() << ',' << center.y() << ',' << center.z() << ','
            << min_bound.x() << ',' << max_bound.x() << ',' << min_bound.y() << ',' << max_bound.y() << ',' << min_bound.z() << ',' << max_bound.z() << ','
            << risk_query_base_time_s_ + tau_min << ',' << risk_query_base_time_s_ + tau_max << ','
            << trajectory_min.x() << ',' << trajectory_max.x() << ',' << trajectory_min.y() << ',' << trajectory_max.y() << ',' << trajectory_min.z() << ',' << trajectory_max.z() << ','
            << 0.0 << ',' << duration << ','
            << kP1AcceptedProfileSampleCount << ',' << matched_count << ',' << static_cast<double>(matched_count) / kP1AcceptedProfileSampleCount << ','
            << query_miss_count << ',' << stale_count << ',' << invalid_count << ','
            << miss_reason_histogram.str() << ','
            << (risk_snapshot_ ? risk_snapshot_->params().frame_id : "") << ',' << trajectory_frame_id << ','
            << (validation.spatial_in_bounds ? 1 : 0) << ','
            << (validation.temporal_in_horizon ? 1 : 0) << ','
            << (validation.frame_match ? 1 : 0) << ','
            << (validation.generation_match ? 1 : 0) << ','
            << (validation.query_time_match ? 1 : 0) << ','
            << (validation.fresh ? 1 : 0) << ','
            << (validation.coverage_ok ? 1 : 0) << ','
            << validation.spatial_miss_count << ','
            << validation.temporal_miss_count << ','
            << validation.occupied_miss_count << ','
            << validation.stale_miss_count << ','
            << validation.invalid_miss_count << ','
            << trajectory_start_stamp_s << ','
            << (objective_requested ? 1 : 0) << ','
            << (applied_to_objective ? 1 : 0) << ','
            << (applied_to_objective ? 0 : 1) << ','
            << p1_risk_context_.fallback_reason << '\n';
    const std::string temp_context_path = context_path + ".tmp";
    {
      std::ofstream context_file(temp_context_path, std::ios::trunc);
      if (!context_file.good())
        return false;
      context_file << context.str();
      context_file.flush();
      if (!context_file.good())
        return false;
    }
    if (std::rename(temp_context_path.c_str(), context_path.c_str()) != 0)
    {
      std::remove(temp_context_path.c_str());
      return false;
    }
    return true;
  }

  iap::P1AcceptedContextValidation
  BsplineOptimizer::validateP1AcceptedTrajectoryRiskContext(
      UniformBspline trajectory, const double accepted_stamp_s,
      const std::string &trajectory_frame_id) const
  {
    iap::P1AcceptedContextValidationInput input;
    input.snapshot = risk_snapshot_;
    input.snapshot_frame_id = risk_snapshot_
        ? risk_snapshot_->params().frame_id : "";
    input.trajectory_frame_id = trajectory_frame_id;
    input.expected_generation_id = p1_risk_context_.snapshot
        ? p1_risk_context_.snapshot->generation_id() : 0;
    input.query_base_time_s = risk_query_base_time_s_;
    input.accepted_stamp_s = accepted_stamp_s;
    // Candidate optimization evidence is evaluated on exactly the same
    // 200-sample lattice as the accepted-profile contract.  A partial
    // profile is useful diagnostic evidence but cannot admit an enabled P1
    // candidate, because it cannot prove pre/post risk on common support.
    input.minimum_covered_samples = kP1AcceptedProfileSampleCount;
    input.minimum_coverage_ratio = 1.0;
    if (!risk_snapshot_)
      return iap::validateP1AcceptedContext(input);
    const double duration = trajectory.getTimeSum();
    if (!std::isfinite(duration) || duration < 0.0)
      return iap::validateP1AcceptedContext(input);
    const int denominator = std::max(1, kP1AcceptedProfileSampleCount - 1);
    input.samples.reserve(kP1AcceptedProfileSampleCount);
    for (int sample_index = 0; sample_index < kP1AcceptedProfileSampleCount;
         ++sample_index)
    {
      const double t_s = duration * static_cast<double>(sample_index) /
          static_cast<double>(denominator);
      iap::RiskCostSample query;
      iap::P1AcceptedContextSample sample;
      sample.position_w = trajectory.evaluateDeBoorT(t_s);
      sample.trajectory_time_s = t_s;
      sample.query_hit = risk_snapshot_->queryCost(
          sample.position_w, risk_query_base_time_s_ + t_s, &query);
      sample.query_valid = query.valid;
      sample.query_stale = query.stale;
      sample.query_reason = query.reason;
      input.samples.push_back(std::move(sample));
    }
    return iap::validateP1AcceptedContext(input);
  }

  BsplineOptimizer::P1FixedLatticeRiskSummary
  BsplineOptimizer::evaluateP1FixedLatticeRisk(
      UniformBspline trajectory, double trajectory_start_t_s,
      double window_duration_s) const
  {
    P1FixedLatticeRiskSummary summary;
    if (!risk_snapshot_)
      return summary;
    const double total_duration = trajectory.getTimeSum();
    if (!std::isfinite(total_duration) || total_duration < 0.0 ||
        !std::isfinite(trajectory_start_t_s))
      return summary;
    trajectory_start_t_s = std::clamp(
        trajectory_start_t_s, 0.0, total_duration);
    const double remaining_duration = total_duration - trajectory_start_t_s;
    const double duration = std::isfinite(window_duration_s)
        ? std::min(remaining_duration, std::max(0.0, window_duration_s))
        : remaining_duration;
    const int denominator = std::max(1, kP1AcceptedProfileSampleCount - 1);
    double sum = 0.0;
    double maximum = -std::numeric_limits<double>::infinity();
    for (int index = 0; index < kP1AcceptedProfileSampleCount; ++index)
    {
      const double t_s = duration * static_cast<double>(index) /
          static_cast<double>(denominator);
      iap::RiskCostSample sample;
      const bool hit = risk_snapshot_->queryCost(
          trajectory.evaluateDeBoorT(trajectory_start_t_s + t_s),
          risk_query_base_time_s_ + t_s, &sample);
      if (!hit || !sample.valid || sample.stale || !std::isfinite(sample.cost))
      {
        if (sample.reason == "occupied")
          ++summary.occupied_sample_count;
        else
          ++summary.evidence_miss_count;
        continue;
      }
      sum += sample.cost;
      maximum = std::max(maximum, sample.cost);
      ++summary.valid_sample_count;
    }
    summary.full_support = summary.valid_sample_count == kP1AcceptedProfileSampleCount;
    if (summary.full_support)
    {
      summary.mean_c_pi = sum / static_cast<double>(summary.valid_sample_count);
      summary.max_c_pi = maximum;
    }
    return summary;
  }

  bool BsplineOptimizer::evaluateReboundCostForTest(const Eigen::MatrixXd &control_points,
                                                    double ts, double &cost,
                                                    Eigen::MatrixXd &gradient)
  {
    if (control_points.rows() != 3 || control_points.cols() <= order_)
    {
      return false;
    }

    setBsplineInterval(ts);
    cps_.resize(static_cast<int>(control_points.cols()));
    cps_.points = control_points;
    cps_.clearance = dist0_;
    min_ellip_dist_ = INIT_min_ellip_dist_;
    iter_num_ = 0;
    new_lambda2_ = lambda2_;
    variable_num_ = 3 * (cps_.size - order_);

    std::vector<double> x(variable_num_, 0.0);
    std::vector<double> grad(variable_num_, 0.0);
    memcpy(x.data(), cps_.points.data() + 3 * order_, variable_num_ * sizeof(double));
    combineCostRebound(x.data(), grad.data(), cost, variable_num_);

    gradient = Eigen::MatrixXd::Zero(3, cps_.size);
    memcpy(gradient.data() + 3 * order_, grad.data(), variable_num_ * sizeof(double));
    return true;
  }

  bool BsplineOptimizer::evaluateP1RawCostForTest(
      const Eigen::MatrixXd &control_points, const double ts, double &cost,
      Eigen::MatrixXd &gradient)
  {
    if (control_points.rows() != 3 || control_points.cols() <= order_ ||
        !std::isfinite(ts) || ts <= 0.0 || !risk_snapshot_)
      return false;
    setBsplineInterval(ts);
    P1IntegrityMetrics metrics;
    gradient = Eigen::MatrixXd::Zero(3, control_points.cols());
    calcIntegrityTrajectoryCost(control_points, cost, gradient, metrics);
    last_p1_metrics_ = metrics;
    return std::isfinite(cost) && gradient.allFinite();
  }

  bool BsplineOptimizer::optimizeP1BasePrepassForTest(
      Eigen::MatrixXd &control_points, const double ts,
      const int max_iterations, double &final_cost, int &iterations)
  {
    clearP1NormalizedStage();
    p1_base_prepass_active_ = true;
    const auto started = std::chrono::steady_clock::now();
    const bool success = optimizeReboundCostForTest(
        control_points, ts, max_iterations, final_cost, iterations);
    p1_base_prepass_active_ = false;
    last_p1_base_prepass_trace_.pre_objective =
        last_p1_optimization_trace_.pre_base_objective;
    last_p1_base_prepass_trace_.post_objective =
        last_p1_optimization_trace_.post_base_objective;
    last_p1_base_prepass_trace_.duration_ms = 1000.0 *
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
    last_p1_base_prepass_trace_.solver_result =
        last_p1_optimization_trace_.solver_result;
    last_p1_base_prepass_trace_.iteration_count = iterations;
    last_p1_base_prepass_trace_.success = success;
    last_p1_base_prepass_trace_.termination_reason =
        last_p1_optimization_trace_.termination_reason;
    return success;
  }

  bool BsplineOptimizer::prepareP1NormalizedStage(
      const Eigen::MatrixXd &seed_control_points, const double ts,
      const P1BasePrepassTrace &base_prepass, std::string *reason)
  {
    clearP1NormalizedStage();
    p1_normalized_stage_.budget_fraction =
        p1_config_.normalization_budget_fraction;
    const auto fail = [&](const char *message) {
      if (reason)
        *reason = message;
      return false;
    };
    if (!base_prepass.success ||
        !std::isfinite(base_prepass.pre_objective) ||
        !std::isfinite(base_prepass.post_objective))
      return fail("base_prepass_failed");
    if (seed_control_points.rows() != 3 ||
        seed_control_points.cols() <= order_ ||
        !std::isfinite(ts) || ts <= 0.0 || !risk_snapshot_)
      return fail("invalid_normalization_seed");

    double raw_cost = 0.0;
    Eigen::MatrixXd raw_gradient;
    if (!evaluateP1RawCostForTest(
            seed_control_points, ts, raw_cost, raw_gradient))
      return fail("raw_p1_evaluation_failed");
    const auto metrics = last_p1_metrics_;
    if (metrics.sample_count != kP1CandidateEvidenceSampleCount ||
        metrics.hit_count != kP1CandidateEvidenceSampleCount ||
        metrics.miss_count != 0 || metrics.stale_count != 0)
      return fail("fixed_support_not_full");

    const double active_gradient_norm = raw_gradient.rightCols(
        seed_control_points.cols() - order_).norm();
    if (!std::isfinite(active_gradient_norm) ||
        active_gradient_norm <= 1.0e-12)
      return fail("raw_p1_gradient_too_small");

    constexpr double kBaseBudgetEpsilon = 1.0e-9;
    const double budget = std::max(
        base_prepass.pre_objective - base_prepass.post_objective,
        kBaseBudgetEpsilon);
    p1_normalized_stage_.enabled = true;
    p1_normalized_stage_.base_improvement_budget = budget;
    p1_normalized_stage_.raw_gradient_norm = active_gradient_norm;
    p1_normalized_stage_.seed_raw_cost = raw_cost;
    p1_normalized_stage_.seed_control_points = seed_control_points;
    p1_normalized_stage_.base_prepass = base_prepass;
    p1_normalized_stage_.scale =
        p1_normalized_stage_.budget_fraction * budget /
        (p1_normalized_stage_.reference_lambda *
         p1_normalized_stage_.reference_displacement_m *
         active_gradient_norm);
    if (!std::isfinite(p1_normalized_stage_.scale) ||
        p1_normalized_stage_.scale <= 0.0)
    {
      clearP1NormalizedStage();
      return fail("normalization_scale_invalid");
    }
    if (reason)
      *reason = "ok";
    return true;
  }

  void BsplineOptimizer::clearP1NormalizedStage()
  {
    p1_normalized_stage_ = P1NormalizedStage{};
  }

  bool BsplineOptimizer::optimizeReboundCostForTest(
      Eigen::MatrixXd &control_points, const double ts,
      const int max_iterations, double &final_cost, int &iterations)
  {
    if (control_points.rows() != 3 || control_points.cols() <= order_ ||
        !std::isfinite(ts) || ts <= 0.0 || max_iterations <= 0)
      return false;
    captureP1PreOptimizationTrajectory(control_points, ts);
    current_p1_checkpoints_.clear();
    current_p1_restart_index_ = 0;
    setBsplineInterval(ts);
    cps_.resize(static_cast<int>(control_points.cols()));
    cps_.points = control_points;
    cps_.clearance = dist0_;
    min_ellip_dist_ = INIT_min_ellip_dist_;
    iter_num_ = 0;
    new_lambda2_ = lambda2_;
    variable_num_ = 3 * (cps_.size - order_);
    std::vector<double> x(static_cast<std::size_t>(variable_num_), 0.0);
    std::vector<double> initial_gradient(
        static_cast<std::size_t>(variable_num_), 0.0);
    memcpy(x.data(), cps_.points.data() + 3 * order_,
           variable_num_ * sizeof(double));
    const Eigen::MatrixXd initial_control_points = control_points;
    const auto pre_lattice = evaluateP1CandidateLattice(
        risk_snapshot_, initial_control_points, order_, bspline_interval_,
        risk_query_base_time_s_);
    double initial_cost = 0.0;
    combineCostRebound(x.data(), initial_gradient.data(), initial_cost,
                       variable_num_);
    P1OptimizerCheckpoint first_direction;
    first_direction.stage = p1_base_prepass_active_ ? "base_prepass" : "p1_stage";
    first_direction.checkpoint = "first_direction";
    first_direction.objective = initial_cost;
    first_direction.base_objective = last_optimizer_cost_breakdown_.original_cost;
    first_direction.raw_p1_objective = last_p1_metrics_.f_integrity;
    first_direction.normalized_p1_objective =
        last_optimizer_cost_breakdown_.normalized_integrity_cost;
    first_direction.anchor_objective = last_optimizer_cost_breakdown_.anchor_cost;
    first_direction.x_norm = Eigen::Map<const Eigen::VectorXd>(
        x.data(), variable_num_).norm();
    first_direction.gradient_norm = Eigen::Map<const Eigen::VectorXd>(
        initial_gradient.data(), variable_num_).norm();
    first_direction.directional_derivative =
        -first_direction.gradient_norm * first_direction.gradient_norm;
    current_p1_checkpoints_.push_back(std::move(first_direction));
    P1OptimizationTrace trace;
    trace.planning_attempt_id = p1_risk_context_.planning_attempt_id;
    trace.candidate_id = p1_risk_context_.candidate_id;
    trace.snapshot_generation_id = risk_snapshot_ ? risk_snapshot_->generation_id() : 0;
    trace.query_base_time_s = risk_query_base_time_s_;
    trace.pre_total_objective = initial_cost;
    trace.pre_base_objective = last_optimizer_cost_breakdown_.original_cost;
    trace.raw_p1_cost = last_p1_metrics_.f_integrity;
    trace.weighted_p1_cost = last_p1_metrics_.weighted_f_integrity;
    trace.base_gradient_norm = last_p1_metrics_.grad_norm_original;
    trace.p1_gradient_norm = last_p1_metrics_.grad_norm_integrity;
    trace.pre_raw_p1_cost = last_p1_metrics_.f_integrity;
    trace.pre_weighted_p1_cost = last_p1_metrics_.weighted_f_integrity;
    trace.pre_base_gradient_norm = last_p1_metrics_.grad_norm_original;
    trace.pre_full_base_gradient_norm = last_p1_metrics_.full_grad_norm_original;
    trace.pre_raw_p1_gradient_norm = last_p1_metrics_.grad_norm_integrity;
    trace.pre_full_raw_p1_gradient_norm =
        last_p1_metrics_.full_grad_norm_integrity;
    trace.pre_weighted_p1_gradient_norm =
        last_p1_metrics_.weighted_grad_integrity_norm;
    trace.pre_normalized_weighted_p1_gradient_norm =
        last_p1_metrics_.normalized_weighted_grad_integrity_norm;
    trace.pre_full_normalized_weighted_p1_gradient_norm =
        last_p1_metrics_.full_normalized_weighted_grad_integrity_norm;
    trace.pre_normalized_p1_cost =
        last_p1_metrics_.normalized_weighted_f_integrity;
    trace.pre_anchor_cost = last_p1_metrics_.anchor_cost;
    trace.pre_base_p1_cosine = last_p1_metrics_.base_p1_cosine;
    trace.pre_total_gradient_norm = Eigen::Map<const Eigen::VectorXd>(
        initial_gradient.data(), variable_num_).norm();
    trace.pre_full_total_gradient_norm =
        last_p1_metrics_.full_total_gradient_norm;
    trace.pre_mean_c_pi = pre_lattice.mean_c_pi;
    trace.pre_max_c_pi = pre_lattice.max_c_pi;
    trace.support_sample_count = pre_lattice.sample_count;
    trace.pre_support_valid_count = pre_lattice.valid_count;
    trace.pre_support_coverage = static_cast<double>(pre_lattice.valid_count) /
        static_cast<double>(std::max(1, pre_lattice.sample_count));
    trace.support_signature = pre_lattice.support_signature;
    trace.initial_control_points_hash = matrixHash(initial_control_points);
    trace.p1_config_hash = p1ConfigHash(p1_config_);
    trace.objective_allowed = p1_risk_context_.objective_allowed;
    trace.objective_applied = last_p1_metrics_.applied_to_objective;
    trace.fallback_reason = p1_risk_context_.fallback_reason;
    if (p1_normalized_stage_.enabled)
    {
      trace.normalization_mode = "base_improvement_budget_v1";
      trace.normalization_reference_lambda =
          p1_normalized_stage_.reference_lambda;
      trace.normalization_scale = p1_normalized_stage_.scale;
      trace.normalization_budget_fraction =
          p1_normalized_stage_.budget_fraction;
      trace.normalization_base_improvement_budget =
          p1_normalized_stage_.base_improvement_budget;
      trace.normalization_reference_displacement_m =
          p1_normalized_stage_.reference_displacement_m;
      trace.base_prepass_pre_objective =
          p1_normalized_stage_.base_prepass.pre_objective;
      trace.base_prepass_post_objective =
          p1_normalized_stage_.base_prepass.post_objective;
      trace.base_prepass_duration_ms =
          p1_normalized_stage_.base_prepass.duration_ms;
      trace.base_prepass_solver_result =
          p1_normalized_stage_.base_prepass.solver_result;
      trace.base_prepass_iteration_count =
          p1_normalized_stage_.base_prepass.iteration_count;
      trace.base_prepass_success = p1_normalized_stage_.base_prepass.success;
      trace.base_prepass_termination_reason =
          p1_normalized_stage_.base_prepass.termination_reason;
    }
    trace.aggregation_mode = p1_config_.objective_aggregation_mode;
    trace.aggregation_temperature = p1_config_.smooth_max_temperature;
    trace.aggregation_tail_fraction =
        p1_config_.objective_aggregation_mode == "fixed_200_smooth_cvar"
            ? p1_config_.smooth_cvar_alpha : 0.0;
    const double adaptive_dt = std::max(
        p1_config_.sample_dt_min_s,
        bspline_interval_ * std::max(0.0, p1_config_.sample_dt_scale));
    trace.adaptive_sample_count =
        adaptive_dt > 0.0 && std::isfinite(adaptive_dt)
            ? std::max(1, static_cast<int>(std::ceil(
                  static_cast<double>(initial_control_points.cols() - order_) *
                  bspline_interval_ / adaptive_dt)))
            : 0;
    if (p1_config_.max_samples_per_eval > 0)
      trace.adaptive_sample_count = std::min(
          trace.adaptive_sample_count, p1_config_.max_samples_per_eval);
    trace.fixed_sample_count = kP1CandidateEvidenceSampleCount;
    trace.peak_contribution = last_p1_metrics_.peak_contribution;

    lbfgs::lbfgs_parameter_t parameters;
    lbfgs::lbfgs_load_default_parameters(&parameters);
    parameters.mem_size = 16;
    parameters.max_iterations = max_iterations;
    parameters.g_epsilon = p1LbfgsGradientEpsilon(
        p1_normalized_stage_.enabled
            ? last_p1_metrics_.normalized_weighted_grad_integrity_norm
            : last_p1_metrics_.weighted_grad_integrity_norm,
        Eigen::Map<const Eigen::VectorXd>(x.data(), variable_num_).norm());
    suppress_rebound_collision_for_test_ = true;
    current_p1_last_accepted_x_ = Eigen::Map<const Eigen::VectorXd>(
        x.data(), variable_num_);
    const int result = lbfgs::lbfgs_optimize(
        variable_num_, x.data(), &final_cost,
        BsplineOptimizer::costFunctionRebound, nullptr,
        BsplineOptimizer::earlyExit, this,
        &parameters);
    suppress_rebound_collision_for_test_ = false;
    iterations = iter_num_;
    // L-BFGS has already evaluated the terminal point.  Do not issue another
    // callback here: the rebound solver's collision/restart state is not a
    // pure postcondition and a second callback can re-enter it after cleanup.
    trace.post_total_objective = final_cost;
    trace.post_base_objective = last_optimizer_cost_breakdown_.original_cost;
    trace.raw_p1_cost = last_p1_metrics_.f_integrity;
    trace.weighted_p1_cost = last_p1_metrics_.weighted_f_integrity;
    trace.post_raw_p1_cost = last_p1_metrics_.f_integrity;
    trace.post_weighted_p1_cost = last_p1_metrics_.weighted_f_integrity;
    trace.post_base_gradient_norm = last_p1_metrics_.grad_norm_original;
    trace.post_full_base_gradient_norm = last_p1_metrics_.full_grad_norm_original;
    trace.post_raw_p1_gradient_norm = last_p1_metrics_.grad_norm_integrity;
    trace.post_full_raw_p1_gradient_norm =
        last_p1_metrics_.full_grad_norm_integrity;
    trace.post_weighted_p1_gradient_norm =
        last_p1_metrics_.weighted_grad_integrity_norm;
    trace.post_normalized_weighted_p1_gradient_norm =
        last_p1_metrics_.normalized_weighted_grad_integrity_norm;
    trace.post_full_normalized_weighted_p1_gradient_norm =
        last_p1_metrics_.full_normalized_weighted_grad_integrity_norm;
    trace.post_normalized_p1_cost =
        last_p1_metrics_.normalized_weighted_f_integrity;
    trace.post_anchor_cost = last_p1_metrics_.anchor_cost;
    trace.post_base_p1_cosine = last_p1_metrics_.base_p1_cosine;
    trace.post_total_gradient_norm = last_rebound_total_gradient_norm_;
    trace.post_full_total_gradient_norm =
        last_p1_metrics_.full_total_gradient_norm;
    trace.total_gradient_norm = trace.post_total_gradient_norm;
    trace.displacement_norm = (Eigen::Map<const Eigen::VectorXd>(
        x.data(), variable_num_) - Eigen::Map<const Eigen::VectorXd>(
        control_points.data() + 3 * order_, variable_num_)).norm();
    trace.solver_result = result;
    trace.iteration_count = iterations;
    trace.termination_reason = lbfgs::lbfgs_strerror(result);
    P1OptimizerCheckpoint terminal;
    terminal.stage = p1_base_prepass_active_ ? "base_prepass" : "p1_stage";
    terminal.checkpoint = "terminal";
    terminal.iteration = iterations;
    terminal.objective = final_cost;
    terminal.base_objective = last_optimizer_cost_breakdown_.original_cost;
    terminal.raw_p1_objective = last_p1_metrics_.f_integrity;
    terminal.normalized_p1_objective =
        last_optimizer_cost_breakdown_.normalized_integrity_cost;
    terminal.anchor_objective = last_optimizer_cost_breakdown_.anchor_cost;
    terminal.x_norm = Eigen::Map<const Eigen::VectorXd>(
        x.data(), variable_num_).norm();
    terminal.gradient_norm = last_rebound_total_gradient_norm_;
    terminal.solver_result = result;
    terminal.reason = trace.termination_reason;
    current_p1_checkpoints_.push_back(std::move(terminal));
    memcpy(control_points.data() + 3 * order_, x.data(),
           variable_num_ * sizeof(double));
    const auto post_lattice = evaluateP1CandidateLattice(
        risk_snapshot_, control_points, order_, bspline_interval_,
        risk_query_base_time_s_);
    trace.displacement_norm = (control_points - initial_control_points).norm();
    Eigen::MatrixXd raw_integrity_gradient = Eigen::MatrixXd::Zero(
        3, initial_control_points.cols());
    double raw_integrity_cost = 0.0;
    P1IntegrityMetrics raw_integrity_metrics;
    const P1IntegrityMetrics terminal_metrics = last_p1_metrics_;
    trace.peak_contribution = terminal_metrics.peak_contribution;
    const auto terminal_viz_samples = last_p1_viz_samples_;
    calcIntegrityTrajectoryCost(initial_control_points, raw_integrity_cost,
                                raw_integrity_gradient, raw_integrity_metrics);
    last_p1_metrics_ = terminal_metrics;
    last_p1_viz_samples_ = terminal_viz_samples;
    trace.grad_integrity_dot_displacement =
        (raw_integrity_gradient.array() *
         (control_points - initial_control_points).array()).sum();
    trace.weighted_p1_gradient_dot_displacement =
        p1_config_.lambda_integrity * trace.grad_integrity_dot_displacement;
    const Eigen::MatrixXd total_displacement = control_points - initial_control_points;
    trace.total_gradient_dot_displacement = Eigen::Map<const Eigen::VectorXd>(
        initial_gradient.data(), variable_num_).dot(Eigen::Map<const Eigen::VectorXd>(
            total_displacement.data() + 3 * order_, variable_num_));
    trace.final_control_points_hash = matrixHash(control_points);
    trace.post_mean_c_pi = post_lattice.mean_c_pi;
    trace.post_max_c_pi = post_lattice.max_c_pi;
    trace.post_support_valid_count = post_lattice.valid_count;
    trace.post_support_coverage = static_cast<double>(post_lattice.valid_count) /
        static_cast<double>(std::max(1, post_lattice.sample_count));
    trace.support_full_valid = pre_lattice.fullValid() && post_lattice.fullValid() &&
        pre_lattice.support_signature == post_lattice.support_signature &&
        !pre_lattice.support_signature.empty();
    trace.optimization_success = std::isfinite(final_cost) &&
        (result == lbfgs::LBFGS_CONVERGENCE ||
         result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
         result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
         result == lbfgs::LBFGS_STOP);
    trace.selection_score = final_cost;
    trace.selection_reason = trace.optimization_success
        ? "deterministic_single_candidate" : "optimizer_failure";
    last_p1_optimization_trace_ = trace;
    captureP1PostOptimizationTrajectory(control_points, ts);
    return std::isfinite(final_cost) &&
        (result == lbfgs::LBFGS_CONVERGENCE ||
         result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
         result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
         result == lbfgs::LBFGS_STOP);
  }

  double BsplineOptimizer::p1LbfgsGradientEpsilon(
      const double initial_weighted_p1_gradient_norm,
      const double variable_norm) const
  {
    constexpr double kHistoricalEpsilon = 0.01;
    constexpr double kMinimumObjectiveAppliedEpsilon = 1.0e-8;
    if (!p1_config_.use_integrity_cost || p1_config_.metrics_only ||
        !p1_risk_context_.objective_allowed ||
        p1_config_.lambda_integrity == 0.0 ||
        !std::isfinite(initial_weighted_p1_gradient_norm) ||
        initial_weighted_p1_gradient_norm <= 0.0)
      return kHistoricalEpsilon;
    const double scale = std::max(1.0, std::abs(variable_norm));
    const double relative_p1_gradient =
        0.1 * initial_weighted_p1_gradient_norm / scale;
    return std::min(kHistoricalEpsilon,
                    std::max(kMinimumObjectiveAppliedEpsilon,
                             relative_p1_gradient));
  }

  // 计算损失
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // cout << "drone_id_=" << drone_id_ << endl;
    // cout << "cps_.points.size()=" << cps_.points.size() << endl;
    // cout << "n=" << n << endl;
    // cout << "sizeof(x[0])=" << sizeof(x[0]) << endl;

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility /*, f_mov_objs*/, f_swarm, f_terminal;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    // Eigen::MatrixXd g_mov_objs = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    // calcMovingObjCost(cps_.points, f_mov_objs, g_mov_objs);
    calcSwarmCost(cps_.points, f_swarm, g_swarm);
    calcTerminalCost(cps_.points, f_terminal, g_terminal);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_swarm + lambda2_ * f_terminal;
    const double f_original = f_combine;
    // f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_mov_objs;
    // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_swarm + lambda2_ * g_terminal;
    // Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_mov_objs;

    last_p1_metrics_ = P1IntegrityMetrics{};
    last_p1_metrics_.full_grad_norm_original = grad_3D.norm();
    last_p1_metrics_.grad_norm_original = grad_3D.rightCols(
        std::max(0, cps_.size - order_)).norm();
    last_p1_viz_samples_.clear();
    last_optimizer_cost_breakdown_ = OptimizerCostBreakdown{};
    last_optimizer_cost_breakdown_.original_cost = f_original;
    last_optimizer_cost_breakdown_.total_cost = f_original;
    const bool p1_should_evaluate =
        !p1_base_prepass_active_ && risk_snapshot_ &&
        (p1_config_.metrics_only || p1_config_.use_integrity_cost ||
         p1_config_.debug_csv_enable);
    if (p1_should_evaluate)
    {
      double f_integrity = 0.0;
      Eigen::MatrixXd g_integrity = Eigen::MatrixXd::Zero(3, cps_.size);
      P1IntegrityMetrics metrics = last_p1_metrics_;
      calcIntegrityTrajectoryCost(cps_.points, f_integrity, g_integrity, metrics);
      metrics.full_grad_norm_original = grad_3D.norm();
      metrics.grad_norm_original = grad_3D.rightCols(
          std::max(0, cps_.size - order_)).norm();
      metrics.full_grad_norm_integrity = g_integrity.norm();
      metrics.grad_norm_integrity = g_integrity.rightCols(
          std::max(0, cps_.size - order_)).norm();
      metrics.weighted_f_integrity = p1_config_.lambda_integrity * f_integrity;
      metrics.weighted_grad_integrity_norm =
          std::abs(p1_config_.lambda_integrity) * metrics.grad_norm_integrity;
      if (metrics.grad_norm_original > 1.0e-12)
      {
        metrics.grad_ratio = metrics.weighted_grad_integrity_norm / metrics.grad_norm_original;
      }
      if (metrics.grad_norm_original > 1.0e-12 &&
          metrics.grad_norm_integrity > 1.0e-12)
      {
        const auto base_active = grad_3D.rightCols(
            std::max(0, cps_.size - order_));
        const auto p1_active = g_integrity.rightCols(
            std::max(0, cps_.size - order_));
        metrics.base_p1_cosine =
            base_active.cwiseProduct(p1_active).sum() /
            (metrics.grad_norm_original * metrics.grad_norm_integrity);
      }
      metrics.applied_to_objective =
          p1_config_.use_integrity_cost && !p1_config_.metrics_only &&
          p1_risk_context_.objective_allowed &&
          p1_config_.lambda_integrity != 0.0 &&
          !p1_base_prepass_active_;

      if (metrics.applied_to_objective)
      {
        if (p1_normalized_stage_.enabled)
        {
          const double normalized_p1 = p1_config_.lambda_integrity *
              p1_normalized_stage_.scale *
              (f_integrity - p1_normalized_stage_.seed_raw_cost);
          const Eigen::MatrixXd displacement =
              cps_.points - p1_normalized_stage_.seed_control_points;
          const double anchor_coefficient =
              (p1_config_.lambda_integrity /
               p1_normalized_stage_.reference_lambda) *
              p1_normalized_stage_.budget_fraction *
              p1_normalized_stage_.base_improvement_budget /
              (2.0 * p1_normalized_stage_.reference_displacement_m *
               p1_normalized_stage_.reference_displacement_m);
          const double anchor = anchor_coefficient * displacement.squaredNorm();
          const Eigen::MatrixXd normalized_gradient =
              p1_config_.lambda_integrity * p1_normalized_stage_.scale *
                  g_integrity +
              2.0 * anchor_coefficient * displacement;
          f_combine += normalized_p1 + anchor;
          grad_3D += normalized_gradient;
          metrics.normalized_weighted_f_integrity = normalized_p1;
          metrics.anchor_cost = anchor;
          metrics.normalized_weighted_grad_integrity_norm =
              std::abs(p1_config_.lambda_integrity *
                       p1_normalized_stage_.scale) *
              metrics.grad_norm_integrity;
          metrics.full_normalized_weighted_grad_integrity_norm =
              std::abs(p1_config_.lambda_integrity *
                       p1_normalized_stage_.scale) *
              metrics.full_grad_norm_integrity;
          last_optimizer_cost_breakdown_.normalized_integrity_cost = normalized_p1;
          last_optimizer_cost_breakdown_.anchor_cost = anchor;
          last_optimizer_cost_breakdown_.integrity_cost = normalized_p1 + anchor;
        }
        else
        {
          f_combine += p1_config_.lambda_integrity * f_integrity;
          grad_3D += p1_config_.lambda_integrity * g_integrity;
          metrics.normalized_weighted_f_integrity = metrics.weighted_f_integrity;
          metrics.normalized_weighted_grad_integrity_norm =
              metrics.weighted_grad_integrity_norm;
          metrics.full_normalized_weighted_grad_integrity_norm =
              std::abs(p1_config_.lambda_integrity) *
              metrics.full_grad_norm_integrity;
          last_optimizer_cost_breakdown_.normalized_integrity_cost =
              metrics.weighted_f_integrity;
          last_optimizer_cost_breakdown_.integrity_cost =
              metrics.weighted_f_integrity;
        }
      }

      last_p1_metrics_ = metrics;
    }
    last_p1_metrics_.full_total_gradient_norm = grad_3D.norm();
    if (p1_should_evaluate)
      writeP1DebugCsv(last_p1_metrics_);
    last_optimizer_cost_breakdown_.total_cost = f_combine;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  // 计算优化后的损失
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // time_satrt = rclcpp::Clock().now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace ego_planner
