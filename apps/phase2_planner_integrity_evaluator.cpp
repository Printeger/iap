#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_ionosphere_parameter.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <iap/map/local_occupancy.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/planner/future_pl_field_predictor.hpp>
#include <iap/planner/integrity_snapshot.hpp>
#include <iap/planner/pi_cost_adapter.hpp>
#include <iap/planner/predicted_araim.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

constexpr double kLightSpeed = 2.99792458e8;

const std::vector<std::string> kCsvFields = {
    "stamp",
    "traj_id",
    "sample_index",
    "sample_t_from_now",
    "sample_abs_time",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "ax",
    "ay",
    "az",
    "yaw",
    "dist_to_obstacle",
    "dist_to_vertical_lower",
    "dist_to_vertical_upper",
    "AL_H_pred",
    "AL_V_pred",
    "AL_pred",
    "current_HPL",
    "current_VPL",
    "current_PL",
    "PL_H_pred",
    "PL_V_pred",
    "PL_pred",
    "hpl_pred",
    "vpl_pred",
    "pl_pred_scalar",
    "pl_ff_h",
    "pl_ff_v",
    "sigma_h",
    "sigma_v",
    "n_vis",
    "pdop",
    "n_hypotheses",
    "valid",
    "fallback",
    "fallback_reason",
    "query_source",
    "grid_enabled",
    "grid_generation",
    "grid_age_s",
    "grid_build_time_ms",
    "gnss_hpl",
    "gnss_vpl",
    "fused_hpl",
    "fused_vpl",
    "lidar_valid",
    "lidar_alpha",
    "lidar_tdop",
    "lidar_condition",
    "lidar_n_primitives",
    "lidar_bias_h",
    "lidar_bias_v",
    "lidar_fallback_reason",
    "IM_H_pred",
    "IM_V_pred",
    "IM_pred_axis_min",
    "IM_pred_scalar",
    "IM_pred",
    "risk_state_pred",
    "pi_cost_h",
    "pi_cost_v",
    "pi_cost_total",
    "pi_risk_band",
    "pi_margin_h",
    "pi_margin_v",
    "pi_dominant_axis",
    "pi_risk_band_code",
    "pi_grad_x",
    "pi_grad_y",
    "pi_grad_z",
    "pl_model",
    "al_model",
    "odom_source",
    "map_source",
};

const std::vector<std::string> kSnapshotCsvFields = {
    "stamp",
    "snapshot_valid",
    "has_pose",
    "p_x",
    "p_y",
    "p_z",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "current_valid",
    "current_integrity_state",
    "current_HPL",
    "current_VPL",
    "current_PL",
    "current_PL_E",
    "current_PL_N",
    "current_PL_U",
    "current_HAL",
    "current_VAL",
    "current_IM",
    "n_sv_used",
    "pdop",
    "n_hypotheses",
    "n_detected",
    "excluded_prns",
    "n_trunks_observed",
    "current_tdop",
    "lidar_modulation_alpha",
    "has_epoch",
    "epoch_sat_count",
    "has_lambda_base",
    "has_lidar_snapshot",
    "has_lidar_araim_result",
    "pred_now_hpl",
    "pred_now_vpl",
    "pred_now_pl",
    "pred_now_raw_hpl",
    "pred_now_raw_vpl",
    "pred_now_raw_pl",
    "pred_now_n_vis",
    "pred_now_pdop",
    "pred_now_valid",
    "pred_now_fallback",
    "pred_now_fallback_reason",
    "consistency_pl_ratio",
    "consistency_hpl_error",
    "consistency_vpl_error",
    "raw_consistency_pl_ratio",
    "raw_consistency_hpl_error",
    "raw_consistency_vpl_error",
};

  const std::vector<std::string> kGridConsistencyCsvFields = {
    "stamp",
    "grid_generation",
    "sample_index",
    "p_x",
    "p_y",
    "p_z",
    "hpl_direct",
    "hpl_grid",
    "vpl_direct",
    "vpl_grid",
    "abs_err_h",
    "rel_err_h",
    "abs_err_v",
    "rel_err_v",
    "n_vis_direct",
    "n_vis_grid",
    "query_source",
  };

  const std::vector<std::string> kActualExecCsvFields = {
    "stamp",
    "odom_x",
    "odom_y",
    "odom_z",
    "actual_HPL_pred",
    "actual_VPL_pred",
    "actual_HAL",
    "actual_VAL",
    "actual_IM_H",
    "actual_IM_V",
    "actual_IM_min",
    "current_HPL",
    "current_VPL",
    "current_IM",
    "tracking_error_to_plan",
    "query_source",
    "pl_model",
  };

bool is_finite(const double value) {
  return std::isfinite(value);
}

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::string fmt_num(const double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << value;
  return oss.str();
}

std::string csv_escape(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }
  std::string out = "\"";
  for (const char c : value) {
    if (c == '"') {
      out += "\"\"";
    } else {
      out += c;
    }
  }
  out += '"';
  return out;
}

std::string bool_str(const bool value) {
  return value ? "true" : "false";
}

std::optional<std::string> read_command_output(const std::string& command) {
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return std::nullopt;
  }

  std::string output;
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  const int rc = pclose(pipe);
  if (rc != 0 || output.empty()) {
    return std::nullopt;
  }

  while (!output.empty() &&
         (output.back() == '\n' || output.back() == '\r' || output.back() == ' ')) {
    output.pop_back();
  }
  return output.empty() ? std::nullopt : std::optional<std::string>(output);
}

std::string git_commit() {
  if (const auto commit =
          read_command_output("git -C \"" + std::string(IAP_SOURCE_ROOT) +
                              "\" rev-parse HEAD 2>/dev/null")) {
    return *commit;
  }
  return "unknown";
}

std::string join_ints(const std::vector<int>& values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) {
      oss << ';';
    }
    oss << values[i];
  }
  return oss.str();
}

std::optional<double> finite_or_none(const std::string& value) {
  try {
    std::size_t pos = 0;
    const double out = std::stod(value, &pos);
    if (pos != value.size() || !std::isfinite(out)) {
      return std::nullopt;
    }
    return out;
  } catch (...) {
    return std::nullopt;
  }
}

double relative_pl_error(const double pred_pl, const double current_pl) {
  return is_finite(pred_pl) && is_finite(current_pl)
             ? std::abs(pred_pl - current_pl) /
                   std::max(std::abs(current_pl), 1.0e-9)
             : std::numeric_limits<double>::quiet_NaN();
}

double signed_error(const double pred, const double current) {
  return is_finite(pred) && is_finite(current)
             ? pred - current
             : std::numeric_limits<double>::quiet_NaN();
}

double quantile(std::vector<double> values, const double q) {
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](double v) { return !std::isfinite(v); }),
               values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  if (values.size() == 1) {
    return values.front();
  }
  const double pos = static_cast<double>(values.size() - 1) * q;
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = static_cast<std::size_t>(std::ceil(pos));
  if (lo == hi) {
    return values[lo];
  }
  return values[lo] * (static_cast<double>(hi) - pos) +
         values[hi] * (pos - static_cast<double>(lo));
}

nlohmann::json json_or_null(const double value) {
  return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
}

class BsplineTrajectory {
 public:
  std::vector<Eigen::Vector3d> control_points;
  int order = 0;
  std::vector<double> knots;

  static BsplineTrajectory from_msg(const traj_utils::msg::Bspline& msg) {
    BsplineTrajectory out;
    out.order = msg.order;
    out.knots.assign(msg.knots.begin(), msg.knots.end());
    out.control_points.reserve(msg.pos_pts.size());
    for (const auto& p : msg.pos_pts) {
      out.control_points.emplace_back(p.x, p.y, p.z);
    }
    return out;
  }

  bool valid() const {
    const auto ncp = static_cast<int>(control_points.size());
    return ncp > order && order >= 0 &&
           static_cast<int>(knots.size()) >= ncp + order + 1;
  }

  double duration() const {
    if (!valid()) {
      return 0.0;
    }
    const int ncp = static_cast<int>(control_points.size());
    const double start = knots[order];
    const double end = knots[ncp];
    return std::isfinite(start) && std::isfinite(end) && end >= start
               ? end - start
               : 0.0;
  }

  Eigen::Vector3d evaluate(const double u) const {
    if (!valid()) {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    const int p = order;
    const int ncp = static_cast<int>(control_points.size());
    const int m = ncp + p;
    const double lo = knots[p];
    const double hi = knots[m - p];
    const double ub = std::min(std::max(u, lo), hi);

    int k = p;
    while (k + 1 < static_cast<int>(knots.size()) && knots[k + 1] < ub) {
      ++k;
    }

    std::vector<Eigen::Vector3d> d;
    d.reserve(static_cast<std::size_t>(p + 1));
    for (int i = 0; i <= p; ++i) {
      d.push_back(control_points[k - p + i]);
    }

    for (int r = 1; r <= p; ++r) {
      for (int i = p; i >= r; --i) {
        const double denom = knots[i + 1 + k - r] - knots[i + k - p];
        const double alpha =
            std::abs(denom) < 1.0e-12 ? 0.0 : (ub - knots[i + k - p]) / denom;
        d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
      }
    }
    return d[p];
  }

  Eigen::Vector3d evaluate_t(const double t) const {
    if (!valid()) {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return evaluate(t + knots[order]);
  }

  std::optional<BsplineTrajectory> derivative() const {
    if (!valid() || order <= 0 || control_points.size() < 2) {
      return std::nullopt;
    }
    BsplineTrajectory out;
    out.order = order - 1;
    out.knots.assign(knots.begin() + 1, knots.end() - 1);
    out.control_points.reserve(control_points.size() - 1);
    for (std::size_t i = 0; i + 1 < control_points.size(); ++i) {
      const double denom = knots[i + order + 1] - knots[i + 1];
      if (std::abs(denom) < 1.0e-12) {
        out.control_points.push_back(
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
      } else {
        out.control_points.push_back(
            static_cast<double>(order) * (control_points[i + 1] - control_points[i]) /
            denom);
      }
    }
    return out;
  }
};

struct PlFields {
  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double pl = std::numeric_limits<double>::quiet_NaN();
  double pl_ff_h = std::numeric_limits<double>::quiet_NaN();
  double pl_ff_v = std::numeric_limits<double>::quiet_NaN();
  double sigma_h = std::numeric_limits<double>::quiet_NaN();
  double sigma_v = std::numeric_limits<double>::quiet_NaN();
  double pdop = std::numeric_limits<double>::quiet_NaN();
  int n_vis = -1;
  int n_hypotheses = -1;
  bool valid = false;
  bool fallback = false;
  std::string fallback_reason;
  std::string query_source = "current";
  bool grid_enabled = false;
  int grid_generation = -1;
  double grid_age_s = std::numeric_limits<double>::quiet_NaN();
  double grid_build_time_ms = std::numeric_limits<double>::quiet_NaN();
  double gnss_hpl = std::numeric_limits<double>::quiet_NaN();
  double gnss_vpl = std::numeric_limits<double>::quiet_NaN();
  double fused_hpl = std::numeric_limits<double>::quiet_NaN();
  double fused_vpl = std::numeric_limits<double>::quiet_NaN();
  bool lidar_valid = false;
  double lidar_alpha = std::numeric_limits<double>::quiet_NaN();
  double lidar_tdop = std::numeric_limits<double>::quiet_NaN();
  double lidar_condition = std::numeric_limits<double>::quiet_NaN();
  int lidar_n_primitives = -1;
  double lidar_bias_h = std::numeric_limits<double>::quiet_NaN();
  double lidar_bias_v = std::numeric_limits<double>::quiet_NaN();
  std::string lidar_fallback_reason = "lidar_disabled";
};

using Row = std::map<std::string, std::string>;

}  // namespace

class Phase2PlannerIntegrityEvaluator : public rclcpp::Node {
 public:
  Phase2PlannerIntegrityEvaluator()
      : rclcpp::Node("phase2_planner_integrity_evaluator"),
        predictor_(predictor_params_) {
    declare_parameter<std::string>("log_root", "/home/dev/ws_iap/src/iap/log");
    declare_parameter<std::string>("run_dir", "");
    declare_parameter<std::string>("map_source", "unknown");
    declare_parameter<std::string>("odom_topic", "/drone_0_visual_slam/odom");
    declare_parameter<std::string>("bspline_topic", "/drone_0_planning/bspline");
    declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
    declare_parameter<std::string>("map_topic", "/sim/drone_0/lidar");
    declare_parameter<std::string>("integrity_topic", "/iap/integrity");
    declare_parameter<std::string>("gnss_scenario_file", "");
    declare_parameter<std::string>("range_meas_topic", "/ublox_driver/range_meas");
    declare_parameter<std::string>("ephem_topic", "/ublox_driver/ephem");
    declare_parameter<std::string>("glo_ephem_topic", "/ublox_driver/glo_ephem");
    declare_parameter<std::string>("receiver_lla_topic", "/ublox_driver/receiver_lla");
    declare_parameter<std::string>("iono_topic", "/ublox_driver/iono_params");
    declare_parameter<double>("eval_horizon_s", 5.0);
    declare_parameter<double>("eval_dt_s", 0.2);
    declare_parameter<int>("max_samples_per_traj", 30);
    declare_parameter<std::string>("pl_model", "constant_current");
    declare_parameter<std::string>("al_model", "cloud_clearance");
    declare_parameter<double>("fallback_pl_m", 20.0);
    declare_parameter<bool>("use_pl_grid", false);
    declare_parameter<double>("pl_grid_resolution_m", 1.0);
    declare_parameter<double>("pl_grid_size_x_m", 30.0);
    declare_parameter<double>("pl_grid_size_y_m", 30.0);
    declare_parameter<double>("pl_grid_size_z_m", 8.0);
    declare_parameter<double>("pl_grid_update_hz", 2.0);
    declare_parameter<bool>("use_lidar_observability", false);
    declare_parameter<double>("lidar_search_radius_m", 8.0);
    declare_parameter<int>("lidar_min_points", 12);
    declare_parameter<int>("lidar_good_points", 80);
    declare_parameter<double>("lidar_sigma_m", 0.5);
    declare_parameter<double>("lidar_info_scale", 1.0);
    declare_parameter<double>("lidar_alpha_min", 0.02);
    declare_parameter<double>("lidar_alpha_max", 1.0);
    declare_parameter<double>("lidar_condition_ref", 30.0);
    declare_parameter<double>("lidar_condition_max", 1.0e6);
    declare_parameter<double>("lidar_tdop_ref", 2.0);
    declare_parameter<double>("lidar_tdop_max", 20.0);
    declare_parameter<double>("lidar_bias_h_m", 0.0);
    declare_parameter<double>("lidar_bias_v_m", 0.0);
    declare_parameter<int>("lidar_map_max_points", 2500);
    declare_parameter<bool>("visibility_hard_occlusion", false);
    declare_parameter<double>("visibility_occ_range_m", 20.0);
    declare_parameter<double>("visibility_occ_l_m", 5.0);
    declare_parameter<double>("visibility_ray_start_offset_m", 1.0);
    declare_parameter<double>("drone_radius", 0.35);
    declare_parameter<double>("safety_buffer", 0.20);
    declare_parameter<double>("gamma_h", 0.8);
    declare_parameter<double>("gamma_v", 0.8);
    declare_parameter<double>("pi_cost_weight_h", 1.0);
    declare_parameter<double>("pi_cost_weight_v", 1.0);
    declare_parameter<double>("pi_cost_marginal_margin_m", 1.0);
    declare_parameter<double>("pi_cost_gradient_step_m", 0.5);
    declare_parameter<bool>("snapshot_anchor_current_integrity", true);
    declare_parameter<bool>("publish_integrity_cost_field", false);
    declare_parameter<bool>("planner_use_integrity_cost", false);
    declare_parameter<std::string>("integrity_cost_field_topic",
                                   "/iap/integrity_cost_field");
    declare_parameter<double>("z_min", 0.5);
    declare_parameter<double>("z_max", 5.0);
    declare_parameter<double>("safe_margin", 0.0);
    declare_parameter<bool>("publish_markers", true);

    log_root_ = get_parameter("log_root").as_string();
    explicit_run_dir_ = get_parameter("run_dir").as_string();
    map_source_ = get_parameter("map_source").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    bspline_topic_ = get_parameter("bspline_topic").as_string();
    pos_cmd_topic_ = get_parameter("pos_cmd_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    integrity_topic_ = get_parameter("integrity_topic").as_string();
    gnss_scenario_file_ = get_parameter("gnss_scenario_file").as_string();
    range_meas_topic_ = get_parameter("range_meas_topic").as_string();
    ephem_topic_ = get_parameter("ephem_topic").as_string();
    glo_ephem_topic_ = get_parameter("glo_ephem_topic").as_string();
    receiver_lla_topic_ = get_parameter("receiver_lla_topic").as_string();
    iono_topic_ = get_parameter("iono_topic").as_string();
    horizon_s_ = get_parameter("eval_horizon_s").as_double();
    dt_s_ = get_parameter("eval_dt_s").as_double();
    max_samples_ = get_parameter("max_samples_per_traj").as_int();
    pl_model_ = get_parameter("pl_model").as_string();
    al_model_ = get_parameter("al_model").as_string();
    fallback_pl_m_ = get_parameter("fallback_pl_m").as_double();
    use_pl_grid_ = get_parameter("use_pl_grid").as_bool();
    pl_grid_update_hz_ = get_parameter("pl_grid_update_hz").as_double();
    field_predictor_params_.use_grid = use_pl_grid_;
    field_predictor_params_.grid_resolution_m =
        get_parameter("pl_grid_resolution_m").as_double();
    field_predictor_params_.grid_size_x_m =
        get_parameter("pl_grid_size_x_m").as_double();
    field_predictor_params_.grid_size_y_m =
        get_parameter("pl_grid_size_y_m").as_double();
    field_predictor_params_.grid_size_z_m =
        get_parameter("pl_grid_size_z_m").as_double();
    use_lidar_observability_ =
        get_parameter("use_lidar_observability").as_bool();
    field_predictor_params_.use_fused_fim_grid =
        pl_model_ == "fused_fim_grid";
    field_predictor_params_.use_lidar_observability =
        use_lidar_observability_;
    field_predictor_params_.lidar_search_radius_m =
        get_parameter("lidar_search_radius_m").as_double();
    field_predictor_params_.lidar_min_points =
        get_parameter("lidar_min_points").as_int();
    field_predictor_params_.lidar_good_points =
        get_parameter("lidar_good_points").as_int();
    field_predictor_params_.lidar_sigma_m =
        get_parameter("lidar_sigma_m").as_double();
    field_predictor_params_.lidar_info_scale =
        get_parameter("lidar_info_scale").as_double();
    field_predictor_params_.lidar_alpha_min =
        get_parameter("lidar_alpha_min").as_double();
    field_predictor_params_.lidar_alpha_max =
        get_parameter("lidar_alpha_max").as_double();
    field_predictor_params_.lidar_condition_ref =
        get_parameter("lidar_condition_ref").as_double();
    field_predictor_params_.lidar_condition_max =
        get_parameter("lidar_condition_max").as_double();
    field_predictor_params_.lidar_tdop_ref =
        get_parameter("lidar_tdop_ref").as_double();
    field_predictor_params_.lidar_tdop_max =
        get_parameter("lidar_tdop_max").as_double();
    field_predictor_params_.lidar_bias_h_m =
        get_parameter("lidar_bias_h_m").as_double();
    field_predictor_params_.lidar_bias_v_m =
        get_parameter("lidar_bias_v_m").as_double();
    lidar_map_max_points_ = get_parameter("lidar_map_max_points").as_int();
    predictor_params_.vis_params.hard_occlusion =
        get_parameter("visibility_hard_occlusion").as_bool();
    predictor_params_.vis_params.occ_range =
        get_parameter("visibility_occ_range_m").as_double();
    predictor_params_.vis_params.occ_L =
        get_parameter("visibility_occ_l_m").as_double();
    predictor_params_.vis_params.ray_start_offset =
        get_parameter("visibility_ray_start_offset_m").as_double();
    drone_radius_ = get_parameter("drone_radius").as_double();
    safety_buffer_ = get_parameter("safety_buffer").as_double();
    gamma_h_ = get_parameter("gamma_h").as_double();
    gamma_v_ = get_parameter("gamma_v").as_double();
    pi_cost_params_.weight_h = get_parameter("pi_cost_weight_h").as_double();
    pi_cost_params_.weight_v = get_parameter("pi_cost_weight_v").as_double();
    pi_cost_params_.marginal_margin_m =
        get_parameter("pi_cost_marginal_margin_m").as_double();
    pi_cost_adapter_ = iap::PICostAdapter(pi_cost_params_);
    pi_cost_gradient_step_m_ =
        get_parameter("pi_cost_gradient_step_m").as_double();
    snapshot_anchor_current_integrity_ =
        get_parameter("snapshot_anchor_current_integrity").as_bool();
    publish_integrity_cost_field_ =
        get_parameter("publish_integrity_cost_field").as_bool();
    planner_use_integrity_cost_ =
      get_parameter("planner_use_integrity_cost").as_bool();
    integrity_cost_field_topic_ =
        get_parameter("integrity_cost_field_topic").as_string();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    safe_margin_ = get_parameter("safe_margin").as_double();
    publish_markers_ = get_parameter("publish_markers").as_bool();

    predictor_params_.fallback_pl = fallback_pl_m_;
    predictor_ = iap::PredictedAraimComputer(predictor_params_);
    predictor_.set_occupancy(&occupancy_);
    field_predictor_params_.araim_params = predictor_params_;
    field_predictor_params_.grid_max_age_s =
        std::max(1.0, 2.0 / std::max(0.1, pl_grid_update_hz_));
    field_predictor_.set_params(field_predictor_params_);
    field_predictor_.set_occupancy(&occupancy_);

    start_wall_ = std::chrono::system_clock::now();
    initial_latest_target_ = current_latest_target();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          on_odom(*msg);
        });
    bspline_sub_ = create_subscription<traj_utils::msg::Bspline>(
        bspline_topic_, qos,
        [this](const traj_utils::msg::Bspline::ConstSharedPtr msg) {
          on_bspline(*msg);
        });
    pos_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        pos_cmd_topic_, qos,
        [this](const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
          on_pos_cmd(*msg);
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        map_topic_, qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          on_cloud(*msg);
        });
    integrity_sub_ = create_subscription<iap::msg::IntegrityReport>(
        integrity_topic_, qos,
        [this](const iap::msg::IntegrityReport::ConstSharedPtr msg) {
          on_integrity(*msg);
        });
    range_sub_ = create_subscription<gnss_comm::msg::GnssMeasMsg>(
        range_meas_topic_, qos,
        [this](const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
          on_range_meas(msg);
        });
    ephem_sub_ = create_subscription<gnss_comm::msg::GnssEphemMsg>(
        ephem_topic_, qos,
        [this](const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
          auto ephem = gnss_comm::msg2ephem(msg);
          if (ephem) {
            ephem_cache_[ephem->sat] = ephem;
          }
        });
    glo_ephem_sub_ = create_subscription<gnss_comm::msg::GnssGloEphemMsg>(
        glo_ephem_topic_, qos,
        [this](const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
          auto ephem = gnss_comm::msg2glo_ephem(msg);
          if (ephem) {
            glo_ephem_cache_[ephem->sat] = ephem;
          }
        });
    receiver_lla_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        receiver_lla_topic_, qos,
        [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
          if (origin_set_) {
            return;
          }
          // gnss_comm::geo2ecef uses degrees for lat/lon.
          const Eigen::Vector3d lla(msg->latitude,
                                    msg->longitude,
                                    msg->altitude);
          if (lla.allFinite()) {
            origin_ecef_ = gnss_comm::geo2ecef(lla);
            origin_set_ = true;
          }
        });
    iono_sub_ = create_subscription<gnss_comm::msg::GnssIonosphereParameter>(
        iono_topic_, qos,
        [this](const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
          if (msg->type == 0 && msg->parameters.size() >= 8) {
            iono_params_.assign(msg->parameters.begin(), msg->parameters.begin() + 8);
          }
        });

    if (publish_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "/iap/planner_integrity_markers", 10);
    }
    if (publish_integrity_cost_field_) {
      integrity_cost_field_pub_ =
          create_publisher<sensor_msgs::msg::PointCloud2>(
              integrity_cost_field_topic_, rclcpp::QoS(1).best_effort());
    }

    open_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
      open_outputs_if_ready();
    });
    summary_timer_ = create_wall_timer(std::chrono::seconds(2), [this] {
      write_summary();
    });
    if (use_pl_grid_) {
      const auto period = std::chrono::duration<double>(
          1.0 / std::max(0.1, pl_grid_update_hz_));
      pl_grid_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this] { schedule_pl_grid_rebuild(); });
    }

    RCLCPP_INFO(get_logger(),
                "phase2 C++ evaluator started; odom=%s bspline=%s map=%s pl_model=%s pl_grid=%s lidar_obs=%s",
                odom_topic_.c_str(), bspline_topic_.c_str(), map_topic_.c_str(),
                pl_model_.c_str(), use_pl_grid_ ? "enabled" : "disabled",
                use_lidar_observability_ ? "enabled" : "disabled");
  }

  ~Phase2PlannerIntegrityEvaluator() override {
    finalize();
  }

 private:
  std::optional<std::filesystem::path> current_latest_target() const {
    const auto latest = std::filesystem::path(log_root_) / "latest";
    std::error_code ec;
    if (!std::filesystem::exists(latest, ec)) {
      return std::nullopt;
    }
    const auto target = std::filesystem::canonical(latest, ec);
    if (ec) {
      return std::nullopt;
    }
    return target;
  }

  std::optional<std::filesystem::path> resolve_run_dir() const {
    if (!explicit_run_dir_.empty()) {
      return std::filesystem::absolute(std::filesystem::path(explicit_run_dir_));
    }

    const auto target = current_latest_target();
    if (!target || !std::filesystem::is_directory(*target)) {
      return std::nullopt;
    }
    if (!initial_latest_target_ || *target != *initial_latest_target_) {
      return target;
    }

    std::error_code ec;
    const auto mtime = std::filesystem::last_write_time(*target, ec);
    if (ec) {
      return std::nullopt;
    }
    const auto now_fs = std::filesystem::file_time_type::clock::now();
    const auto now_sys = std::chrono::system_clock::now();
    const auto start_fs = now_fs + (start_wall_ - now_sys);
    if (mtime >= start_fs - std::chrono::seconds(2)) {
      return target;
    }
    return std::nullopt;
  }

  void open_outputs_if_ready() {
    if (outputs_open_) {
      return;
    }
    const auto run_dir = resolve_run_dir();
    if (!run_dir) {
      return;
    }
    run_dir_ = *run_dir;
    export_dir_ = run_dir_.value() / "export";
    std::filesystem::create_directories(*export_dir_);
    csv_file_.open(*export_dir_ / "integrity_along_planner_traj.csv",
                   std::ios::out | std::ios::trunc);
    snapshot_csv_file_.open(*export_dir_ / "future_integrity_snapshot.csv",
                            std::ios::out | std::ios::trunc);
    actual_exec_csv_file_.open(*export_dir_ / "actual_integrity_along_odom.csv",
                               std::ios::out | std::ios::trunc);
    grid_consistency_csv_file_.open(*export_dir_ / "pl_grid_consistency.csv",
                                    std::ios::out | std::ios::trunc);
    if (!csv_file_ || !snapshot_csv_file_) {
      warn_once("failed to open Phase 2 evaluator CSV outputs");
      return;
    }
    for (std::size_t i = 0; i < kCsvFields.size(); ++i) {
      if (i) {
        csv_file_ << ',';
      }
      csv_file_ << kCsvFields[i];
    }
    csv_file_ << '\n';
    for (std::size_t i = 0; i < kSnapshotCsvFields.size(); ++i) {
      if (i) {
        snapshot_csv_file_ << ',';
      }
      snapshot_csv_file_ << kSnapshotCsvFields[i];
    }
    snapshot_csv_file_ << '\n';
    if (actual_exec_csv_file_) {
      for (std::size_t i = 0; i < kActualExecCsvFields.size(); ++i) {
        if (i) {
          actual_exec_csv_file_ << ',';
        }
        actual_exec_csv_file_ << kActualExecCsvFields[i];
      }
      actual_exec_csv_file_ << '\n';
    }
    if (grid_consistency_csv_file_) {
      for (std::size_t i = 0; i < kGridConsistencyCsvFields.size(); ++i) {
        if (i) {
          grid_consistency_csv_file_ << ',';
        }
        grid_consistency_csv_file_ << kGridConsistencyCsvFields[i];
      }
      grid_consistency_csv_file_ << '\n';
    }
    outputs_open_ = true;
    write_summary();
    RCLCPP_INFO(get_logger(), "phase2 C++ evaluator writing export files under %s",
                export_dir_->string().c_str());
  }

  void warn_once(const std::string& text) {
    if (std::find(warnings_.begin(), warnings_.end(), text) == warnings_.end()) {
      warnings_.push_back(text);
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    }
  }

  void schedule_pl_grid_rebuild() {
    if (!use_pl_grid_ || finalized_) {
      return;
    }
    bool expected = false;
    if (!pl_grid_building_.compare_exchange_strong(expected, true)) {
      return;
    }

    std::lock_guard<std::mutex> worker_lock(pl_grid_worker_mutex_);
    if (pl_grid_worker_.joinable()) {
      pl_grid_worker_.join();
    }
    const double rebuild_stamp = now().seconds();
    pl_grid_worker_ = std::thread([this, rebuild_stamp] {
      try {
        std::lock_guard<std::mutex> occupancy_lock(occupancy_mutex_);
        const bool rebuilt = field_predictor_.rebuild_grid(rebuild_stamp);
        if (rebuilt) {
          write_grid_consistency_samples(rebuild_stamp);
        }
      } catch (...) {
        // Keep the evaluator alive; the next timer tick can try again.
      }
      pl_grid_building_.store(false);
    });
  }

  void join_pl_grid_worker() {
    std::lock_guard<std::mutex> worker_lock(pl_grid_worker_mutex_);
    if (pl_grid_worker_.joinable()) {
      pl_grid_worker_.join();
    }
    pl_grid_building_.store(false);
  }

  void on_odom(const nav_msgs::msg::Odometry& msg) {
    latest_odom_stamp_ = stamp_to_sec(msg.header.stamp);
    latest_odom_p_ = Eigen::Vector3d(msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z);
    latest_odom_q_ = Eigen::Quaterniond(msg.pose.pose.orientation.w,
                                        msg.pose.pose.orientation.x,
                                        msg.pose.pose.orientation.y,
                                        msg.pose.pose.orientation.z);
    latest_odom_pose_valid_ =
        latest_odom_p_.allFinite() && is_finite(latest_odom_q_.w()) &&
        is_finite(latest_odom_q_.x()) && is_finite(latest_odom_q_.y()) &&
        is_finite(latest_odom_q_.z());
  }

  iap::CurrentIntegrityState current_from_msg(
      const iap::msg::IntegrityReport& msg) const {
    iap::CurrentIntegrityState current;
    current.stamp = stamp_to_sec(msg.header.stamp);
    current.integrity_state = msg.integrity_state;
    current.hpl = msg.hpl;
    current.vpl = msg.vpl;
    current.pl_e = msg.pl_e;
    current.pl_n = msg.pl_n;
    current.pl_u = msg.pl_u;
    current.pl = iap::current_pl_scalar(msg.hpl, msg.vpl);
    current.hal = msg.hal;
    current.val = msg.val;
    current.im = msg.im;
    current.pl_ff = msg.pl_ff;
    current.pl_ff_v = msg.pl_ff_v;
    current.k_ff_used = msg.k_ff_used;
    current.k_fa_used = msg.k_fa_used;
    current.n_sv_used = msg.n_sv_used;
    current.n_constellations = msg.n_constellations;
    current.pdop = msg.pdop;
    current.sigma_h = msg.sigma_h;
    current.n_hypotheses = msg.n_hypotheses;
    current.n_detected = msg.n_detected;
    current.excluded_prns.assign(msg.excluded_prns.begin(),
                                 msg.excluded_prns.end());
    current.excluded_trunk_ids.assign(msg.excluded_trunk_ids.begin(),
                                      msg.excluded_trunk_ids.end());
    current.n_trunks_observed = msg.n_trunks_observed;
    current.tdop = msg.tdop;
    current.valid = is_finite(current.hpl) && is_finite(current.vpl) &&
                    is_finite(current.hal) && is_finite(current.val) &&
                    is_finite(current.im);
    return current;
  }

  void on_integrity(const iap::msg::IntegrityReport& msg) {
    current_integrity_stamp_ = stamp_to_sec(msg.header.stamp);
    current_hpl_ = msg.hpl;
    current_vpl_ = msg.vpl;

    open_outputs_if_ready();
    if (!outputs_open_) {
      return;
    }
    if (!latest_odom_pose_valid_) {
      warn_once("skipping integrity snapshot until IAP odom pose is available");
      return;
    }

    const auto current = current_from_msg(msg);
    write_snapshot(current);
    write_actual_exec_row(current);
    write_summary();
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2& msg) {
    latest_cloud_stamp_ = stamp_to_sec(msg.header.stamp);
    auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const Eigen::Vector3d p(*iter_x, *iter_y, *iter_z);
        if (p.allFinite()) {
          points->push_back(p);
        }
      }
    } catch (const std::exception& e) {
      latest_cloud_points_.clear();
      {
        std::lock_guard<std::mutex> lock(occupancy_mutex_);
        occupancy_.reset();
      }
      field_predictor_.set_lidar_map_points(nullptr);
      warn_once(std::string("failed to parse map cloud ") + map_topic_ + ": " + e.what());
      return;
    }
    latest_cloud_points_ = *points;
    auto predictor_points = points;
    if (lidar_map_max_points_ > 0 &&
        static_cast<int>(points->size()) > lidar_map_max_points_) {
      predictor_points = std::make_shared<std::vector<Eigen::Vector3d>>();
      predictor_points->reserve(lidar_map_max_points_);
      const double stride =
          static_cast<double>(points->size()) /
          static_cast<double>(lidar_map_max_points_);
      for (int i = 0; i < lidar_map_max_points_; ++i) {
        const std::size_t idx = std::min<std::size_t>(
            points->size() - 1, static_cast<std::size_t>(std::floor(i * stride)));
        predictor_points->push_back((*points)[idx]);
      }
    }
    {
      std::lock_guard<std::mutex> lock(occupancy_mutex_);
      occupancy_.reset();
      occupancy_.insert_points(latest_cloud_points_);
    }
    field_predictor_.set_lidar_map_points(predictor_points);
  }

  void on_range_meas(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr& msg) {
    if (!origin_set_) {
      return;
    }
    const auto obs_list = gnss_comm::msg2meas(msg);
    if (obs_list.empty()) {
      return;
    }

    iap::GnssEpoch epoch;
    const auto utc_t = gnss_comm::gpst2utc(obs_list.front()->time);
    epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
    epoch.gps_sec = static_cast<double>(obs_list.front()->time.time) +
                    obs_list.front()->time.sec;
    epoch.iono_params = iono_params_;

    for (const auto& obs : obs_list) {
      if (!obs) {
        continue;
      }
      int l1_idx = -1;
      const double freq = gnss_comm::L1_freq(obs, &l1_idx);
      if (l1_idx < 0 || freq < 0.0 ||
          static_cast<int>(obs->psr.size()) <= l1_idx) {
        continue;
      }
      const double pr = obs->psr[l1_idx];
      if (pr <= 0.0 || !std::isfinite(pr)) {
        continue;
      }

      const uint32_t sat_id = obs->sat;
      const uint32_t sys = gnss_comm::satsys(sat_id, nullptr);
      Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
      double svdt = 0.0;
      double svddt = 0.0;
      double tgd = 0.0;
      const auto t_tx = gnss_comm::time_add(obs->time, -pr / kLightSpeed);

      if (sys == SYS_GLO) {
        const auto it = glo_ephem_cache_.find(sat_id);
        if (it == glo_ephem_cache_.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
      } else {
        const auto it = ephem_cache_.find(sat_id);
        if (it == ephem_cache_.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::eph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::eph2vel(t_tx, it->second, &svddt);
        tgd = it->second->tgd[0];
      }
      if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) {
        continue;
      }

      double azel[2] = {0.0, M_PI / 2.0};
      gnss_comm::sat_azel(origin_ecef_, sat_ecef_pos, azel);
      const double elevation = azel[1];
      const double azimuth = azel[0];
      if (elevation < predictor_params_.vis_params.min_elevation) {
        continue;
      }

      double dop_meas = 0.0;
      if (static_cast<int>(obs->dopp.size()) > l1_idx && freq > 0.0 &&
          std::isfinite(obs->dopp[l1_idx])) {
        dop_meas = -obs->dopp[l1_idx] * (kLightSpeed / freq);
      }

      const double pr_sigma =
          static_cast<int>(obs->psr_std.size()) > l1_idx && obs->psr_std[l1_idx] > 0.05
              ? obs->psr_std[l1_idx]
              : 5.0;
      const double dop_sigma =
          static_cast<int>(obs->dopp_std.size()) > l1_idx &&
                  obs->dopp_std[l1_idx] > 0.01 && freq > 0.0
              ? obs->dopp_std[l1_idx] * (kLightSpeed / freq)
              : 0.5;

      iap::SatObs sat;
      sat.sat_id = static_cast<int>(sat_id);
      sat.constellation = (sys == SYS_GLO) ? 'R'
                          : (sys == SYS_GAL) ? 'E'
                          : (sys == SYS_BDS) ? 'C'
                                             : 'G';
      sat.pr_meas = pr + svdt * kLightSpeed;
      sat.dop_meas = dop_meas + svddt * kLightSpeed;
      sat.pr_sigma = pr_sigma;
      sat.dop_sigma = dop_sigma;
      sat.sat_pos = sat_ecef_pos;
      sat.sat_vel = sat_ecef_vel;
      sat.elevation = elevation;
      sat.azimuth = azimuth;
      sat.tgd = tgd;
      sat.svddt = svddt;
      epoch.sats.push_back(sat);
    }

    if (!epoch.sats.empty()) {
      latest_epoch_ = std::move(epoch);
      predictor_.set_epoch(&*latest_epoch_);
    }
  }

  double nearest_obstacle_distance(const Eigen::Vector3d& pos) {
    if (al_model_ != "cloud_clearance") {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (latest_cloud_points_.empty()) {
      warn_once("map/cloud not available yet; AL_H_pred is NaN");
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (!pos.allFinite()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& p : latest_cloud_points_) {
      best = std::min(best, (p - pos).squaredNorm());
    }
    return std::sqrt(best);
  }

  void al_values(const Eigen::Vector3d& pos,
                 double* dist,
                 double* lower,
                 double* upper,
                 double* al_h,
                 double* al_v,
                 double* al) {
    *dist = nearest_obstacle_distance(pos);
    if (is_finite(*dist)) {
      const double clearance_h = *dist - drone_radius_ - safety_buffer_;
      *al_h = gamma_h_ * std::max(clearance_h, 0.0);
    } else {
      *al_h = std::numeric_limits<double>::quiet_NaN();
    }

    *lower = is_finite(pos.z()) ? pos.z() - z_min_ : std::numeric_limits<double>::quiet_NaN();
    *upper = is_finite(pos.z()) ? z_max_ - pos.z() : std::numeric_limits<double>::quiet_NaN();
    if (is_finite(*lower) && is_finite(*upper)) {
      *al_v = gamma_v_ * std::max(std::min(*lower, *upper), 0.0);
    } else {
      *al_v = std::numeric_limits<double>::quiet_NaN();
    }
    *al = is_finite(*al_h) && is_finite(*al_v)
              ? std::min(*al_h, *al_v)
              : std::numeric_limits<double>::quiet_NaN();
  }

  PlFields pl_values(const Eigen::Vector3d& pos) {
    PlFields out;
    out.grid_enabled = use_pl_grid_;
    if (pl_model_ == "constant_current") {
      out.hpl = current_hpl_;
      out.vpl = current_vpl_;
      out.pl = is_finite(out.hpl) && is_finite(out.vpl)
                   ? std::max(out.hpl, out.vpl)
                   : std::numeric_limits<double>::quiet_NaN();
      out.valid = is_finite(out.pl);
      out.fallback = false;
      out.query_source = "current";
      out.gnss_hpl = out.hpl;
      out.gnss_vpl = out.vpl;
      out.fused_hpl = out.hpl;
      out.fused_vpl = out.vpl;
      return out;
    }

    const bool use_field_predictor =
        use_pl_grid_ || pl_model_ == "fused_fim_grid" ||
        use_lidar_observability_;
    if (use_field_predictor) {
      const iap::FuturePLQueryResult query =
          field_predictor_.query(pos, now().seconds());
      out.valid = query.valid;
      out.fallback = query.fallback;
      out.fallback_reason = query.fallback_reason;
      out.pl_ff_h = query.pl_ff_h;
      out.pl_ff_v = query.pl_ff_v;
      out.sigma_h = query.sigma_h;
      out.sigma_v = query.sigma_v;
      out.pdop = query.pdop;
      out.n_vis = query.n_vis;
      out.n_hypotheses = query.n_hypotheses;
      out.grid_generation = query.grid_generation;
      out.grid_age_s = query.grid_age_s;
      out.grid_build_time_ms = query.grid_build_time_ms;
      out.gnss_hpl = query.gnss_hpl;
      out.gnss_vpl = query.gnss_vpl;
      out.fused_hpl = query.fused_hpl;
      out.fused_vpl = query.fused_vpl;
      out.lidar_valid = query.lidar_valid;
      out.lidar_alpha = query.lidar_alpha;
      out.lidar_tdop = query.lidar_tdop;
      out.lidar_condition = query.lidar_condition;
      out.lidar_n_primitives = query.lidar_n_primitives;
      out.lidar_bias_h = query.lidar_bias_h;
      out.lidar_bias_v = query.lidar_bias_v;
      out.lidar_fallback_reason = query.lidar_fallback_reason;
      if (query.valid) {
        out.hpl = query.hpl;
        out.vpl = query.vpl;
        out.pl = query.pl_scalar;
        out.query_source = query.query_source;
        return out;
      }
    } else {
      const auto pred = predictor_.predict_araim_result(pos);
      out.valid = pred.valid;
      out.fallback = pred.fallback;
      out.fallback_reason = pred.fallback_reason;
      out.pl_ff_h = pred.pl_ff_h;
      out.pl_ff_v = pred.pl_ff_v;
      out.sigma_h = pred.sigma_h;
      out.sigma_v = pred.sigma_v;
      out.pdop = pred.pdop;
      out.n_vis = pred.n_vis;
      out.n_hypotheses = pred.n_hypotheses;
      out.gnss_hpl = pred.hpl;
      out.gnss_vpl = pred.vpl;
      out.fused_hpl = pred.hpl;
      out.fused_vpl = pred.vpl;

      if (pred.valid) {
        out.hpl = pred.hpl;
        out.vpl = pred.vpl;
        out.pl = pred.pl_scalar;
        out.query_source = "direct";
        return out;
      }
    }

    if (out.fallback_reason.empty()) {
      out.fallback_reason = "prediction_failed";
    }
    if (out.valid) {
      return out;
    }

    out.fallback = true;
    out.query_source = "fallback";
    if (pl_model_ == "gnss_geometry_araim_fallback_current" &&
        is_finite(current_hpl_) && is_finite(current_vpl_)) {
      out.hpl = current_hpl_;
      out.vpl = current_vpl_;
      out.pl = std::max(out.hpl, out.vpl);
    } else {
      out.hpl = fallback_pl_m_;
      out.vpl = fallback_pl_m_;
      out.pl = fallback_pl_m_;
    }
    return out;
  }

  void im_values(const double al_h,
                 const double al_v,
                 const double al,
                 const double pl_h,
                 const double pl_v,
                 const double pl,
                 double* im_h,
                 double* im_v,
                 double* im_axis_min,
                 double* im_scalar,
                 double* im,
                 std::string* state) const {
    if (!is_finite(al)) {
      *im_h = *im_v = *im_axis_min = *im_scalar = *im =
          std::numeric_limits<double>::quiet_NaN();
      *state = "UNKNOWN_AL";
      return;
    }
    if (!is_finite(pl)) {
      *im_h = *im_v = *im_axis_min = *im_scalar = *im =
          std::numeric_limits<double>::quiet_NaN();
      *state = "UNKNOWN_PL";
      return;
    }
    *im_h = is_finite(al_h) && is_finite(pl_h) ? al_h - pl_h
                                         : std::numeric_limits<double>::quiet_NaN();
    *im_v = is_finite(al_v) && is_finite(pl_v) ? al_v - pl_v
                                         : std::numeric_limits<double>::quiet_NaN();
    std::vector<double> axis_vals;
    if (is_finite(*im_h)) {
      axis_vals.push_back(*im_h);
    }
    if (is_finite(*im_v)) {
      axis_vals.push_back(*im_v);
    }
    *im_axis_min = axis_vals.empty()
                       ? std::numeric_limits<double>::quiet_NaN()
                       : *std::min_element(axis_vals.begin(), axis_vals.end());
    *im_scalar = al - pl;
    std::vector<double> candidates;
    if (is_finite(*im_axis_min)) {
      candidates.push_back(*im_axis_min);
    }
    if (is_finite(*im_scalar)) {
      candidates.push_back(*im_scalar);
    }
    *im = candidates.empty()
              ? std::numeric_limits<double>::quiet_NaN()
              : *std::min_element(candidates.begin(), candidates.end());
    if (!is_finite(*im)) {
      *state = "UNKNOWN_PL";
    } else if (*im > safe_margin_) {
      *state = "SAFE_PRED";
    } else if (std::abs(*im) <= safe_margin_) {
      *state = "MARGINAL_PRED";
    } else {
      *state = "UNSAFE_PRED";
    }
  }

  iap::PICostResult pi_cost_at(const Eigen::Vector3d& pos) {
    double dist, lower, upper, al_h, al_v, al;
    al_values(pos, &dist, &lower, &upper, &al_h, &al_v, &al);
    const PlFields pl = pl_values(pos);
    return pi_cost_adapter_.evaluate(al_h, al_v, pl.hpl, pl.vpl);
  }

  Eigen::Vector3d pi_cost_gradient_at(const Eigen::Vector3d& pos) {
    const double step =
        std::isfinite(pi_cost_gradient_step_m_) && pi_cost_gradient_step_m_ > 0.0
            ? pi_cost_gradient_step_m_
            : 0.5;
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(axis) = step;
      const auto plus = pi_cost_at(pos + delta);
      const auto minus = pi_cost_at(pos - delta);
      if (!plus.valid || !minus.valid || !std::isfinite(plus.cost_total) ||
          !std::isfinite(minus.cost_total)) {
        return Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
      }
      grad(axis) = (plus.cost_total - minus.cost_total) / (2.0 * step);
    }
    return grad;
  }

  Row make_sample_row(const double stamp,
                      const int64_t traj_id,
                      const int sample_index,
                      const double sample_t_from_now,
                      const double sample_abs_time,
                      const Eigen::Vector3d& pos,
                      const Eigen::Vector3d& vel,
                      const Eigen::Vector3d& acc,
                      const double yaw) {
    double dist, lower, upper, al_h, al_v, al;
    al_values(pos, &dist, &lower, &upper, &al_h, &al_v, &al);
    const PlFields pl = pl_values(pos);
    const double current_pl = is_finite(current_hpl_) && is_finite(current_vpl_)
                                  ? std::max(current_hpl_, current_vpl_)
                                  : std::numeric_limits<double>::quiet_NaN();
    double im_h, im_v, im_axis_min, im_scalar, im;
    std::string state;
    im_values(al_h, al_v, al, pl.hpl, pl.vpl, pl.pl, &im_h, &im_v,
              &im_axis_min, &im_scalar, &im, &state);
    const Eigen::Vector3d pi_grad = pi_cost_gradient_at(pos);
    const iap::PICostResult pi = pi_cost_adapter_.evaluate_with_gradient(
        al_h, al_v, pl.hpl, pl.vpl, pi_grad.x(), pi_grad.y(), pi_grad.z());

    Row row;
    row["stamp"] = fmt_num(stamp);
    row["traj_id"] = std::to_string(traj_id);
    row["sample_index"] = std::to_string(sample_index);
    row["sample_t_from_now"] = fmt_num(sample_t_from_now);
    row["sample_abs_time"] = fmt_num(sample_abs_time);
    row["x"] = fmt_num(pos.x());
    row["y"] = fmt_num(pos.y());
    row["z"] = fmt_num(pos.z());
    row["vx"] = fmt_num(vel.x());
    row["vy"] = fmt_num(vel.y());
    row["vz"] = fmt_num(vel.z());
    row["ax"] = fmt_num(acc.x());
    row["ay"] = fmt_num(acc.y());
    row["az"] = fmt_num(acc.z());
    row["yaw"] = fmt_num(yaw);
    row["dist_to_obstacle"] = fmt_num(dist);
    row["dist_to_vertical_lower"] = fmt_num(lower);
    row["dist_to_vertical_upper"] = fmt_num(upper);
    row["AL_H_pred"] = fmt_num(al_h);
    row["AL_V_pred"] = fmt_num(al_v);
    row["AL_pred"] = fmt_num(al);
    row["current_HPL"] = fmt_num(current_hpl_);
    row["current_VPL"] = fmt_num(current_vpl_);
    row["current_PL"] = fmt_num(current_pl);
    row["PL_H_pred"] = fmt_num(pl.hpl);
    row["PL_V_pred"] = fmt_num(pl.vpl);
    row["PL_pred"] = fmt_num(pl.pl);
    row["hpl_pred"] = fmt_num(pl.hpl);
    row["vpl_pred"] = fmt_num(pl.vpl);
    row["pl_pred_scalar"] = fmt_num(pl.pl);
    row["pl_ff_h"] = fmt_num(pl.pl_ff_h);
    row["pl_ff_v"] = fmt_num(pl.pl_ff_v);
    row["sigma_h"] = fmt_num(pl.sigma_h);
    row["sigma_v"] = fmt_num(pl.sigma_v);
    row["n_vis"] = pl.n_vis >= 0 ? std::to_string(pl.n_vis) : "nan";
    row["pdop"] = fmt_num(pl.pdop);
    row["n_hypotheses"] =
        pl.n_hypotheses >= 0 ? std::to_string(pl.n_hypotheses) : "nan";
    row["valid"] = pl.valid ? "true" : "false";
    row["fallback"] = pl.fallback ? "true" : "false";
    row["fallback_reason"] = pl.fallback_reason;
    row["query_source"] = pl.query_source;
    row["grid_enabled"] = bool_str(pl.grid_enabled);
    row["grid_generation"] = std::to_string(pl.grid_generation);
    row["grid_age_s"] = fmt_num(pl.grid_age_s);
    row["grid_build_time_ms"] = fmt_num(pl.grid_build_time_ms);
    row["gnss_hpl"] = fmt_num(pl.gnss_hpl);
    row["gnss_vpl"] = fmt_num(pl.gnss_vpl);
    row["fused_hpl"] = fmt_num(pl.fused_hpl);
    row["fused_vpl"] = fmt_num(pl.fused_vpl);
    row["lidar_valid"] = bool_str(pl.lidar_valid);
    row["lidar_alpha"] = fmt_num(pl.lidar_alpha);
    row["lidar_tdop"] = fmt_num(pl.lidar_tdop);
    row["lidar_condition"] = fmt_num(pl.lidar_condition);
    row["lidar_n_primitives"] =
        pl.lidar_n_primitives >= 0 ? std::to_string(pl.lidar_n_primitives)
                                   : "nan";
    row["lidar_bias_h"] = fmt_num(pl.lidar_bias_h);
    row["lidar_bias_v"] = fmt_num(pl.lidar_bias_v);
    row["lidar_fallback_reason"] = pl.lidar_fallback_reason;
    row["IM_H_pred"] = fmt_num(im_h);
    row["IM_V_pred"] = fmt_num(im_v);
    row["IM_pred_axis_min"] = fmt_num(im_axis_min);
    row["IM_pred_scalar"] = fmt_num(im_scalar);
    row["IM_pred"] = fmt_num(im);
    row["risk_state_pred"] = state;
    row["pi_cost_h"] = fmt_num(pi.cost_h);
    row["pi_cost_v"] = fmt_num(pi.cost_v);
    row["pi_cost_total"] = fmt_num(pi.cost_total);
    row["pi_risk_band"] = pi.risk_band;
    row["pi_margin_h"] = fmt_num(pi.margin_h);
    row["pi_margin_v"] = fmt_num(pi.margin_v);
    row["pi_dominant_axis"] = pi.dominant_axis;
    row["pi_risk_band_code"] = std::to_string(pi.risk_band_code);
    row["pi_grad_x"] = fmt_num(pi.grad_x);
    row["pi_grad_y"] = fmt_num(pi.grad_y);
    row["pi_grad_z"] = fmt_num(pi.grad_z);
    row["pl_model"] = pl_model_;
    row["al_model"] = al_model_;
    row["odom_source"] = odom_topic_;
    row["map_source"] = map_topic_;
    return row;
  }

  void write_sample(const Row& row) {
    if (!csv_file_) {
      return;
    }
    std::lock_guard<std::mutex> lock(csv_mutex_);
    for (std::size_t i = 0; i < kCsvFields.size(); ++i) {
      if (i) {
        csv_file_ << ',';
      }
      const auto it = row.find(kCsvFields[i]);
      csv_file_ << csv_escape(it == row.end() ? "" : it->second);
    }
    csv_file_ << '\n';
    csv_file_.flush();

    ++sample_count_;
    const std::string state = row.count("risk_state_pred") ? row.at("risk_state_pred")
                                                           : "UNKNOWN_PL";
    ++risk_counts_[state];
    const std::string pi_band = row.count("pi_risk_band") ? row.at("pi_risk_band")
                                                          : "UNKNOWN_PI";
    ++pi_risk_band_counts_[pi_band];
    const std::string pi_axis = row.count("pi_dominant_axis")
                                    ? row.at("pi_dominant_axis")
                                    : "unknown";
    ++pi_dominant_axis_counts_[pi_axis];
    if (const auto pi_cost = finite_or_none(row.at("pi_cost_total"))) {
      pi_cost_values_.push_back(*pi_cost);
    }
    if (const auto im = finite_or_none(row.at("IM_pred"))) {
      im_values_.push_back(*im);
    }
    if (const auto pl = finite_or_none(row.at("PL_pred"))) {
      pl_values_.push_back(*pl);
    }
    if (row.at("fallback") == "true") {
      ++fallback_count_;
      ++fallback_reason_counts_[row.at("fallback_reason")];
    }
    if (row.at("valid") == "true" &&
        (row.at("query_source") == "direct" ||
         row.at("query_source") == "grid")) {
      ++finite_gnss_prediction_count_;
    }
  }

  void write_snapshot(const iap::CurrentIntegrityState& current) {
    if (!snapshot_csv_file_) {
      return;
    }
    std::lock_guard<std::mutex> lock(csv_mutex_);

    iap::IntegritySnapshotBuilderInput input;
    input.stamp = current.stamp;
    input.has_pose = latest_odom_pose_valid_;
    input.p_wb = latest_odom_p_;
    input.q_wb = latest_odom_q_;
    input.current = current;
    input.gnss_epoch = latest_epoch_ ? &*latest_epoch_ : nullptr;

    const iap::IntegritySnapshot snapshot =
        snapshot_builder_.build_from_latest(input);
    field_predictor_.update_snapshot(snapshot);

    PlFields pred_now_raw;
    PlFields pred_now;
    if (snapshot.has_pose) {
      pred_now_raw = pl_values(snapshot.p_wb);
      pred_now = pred_now_raw;
      if (snapshot_anchor_current_integrity_ && snapshot.current.valid &&
          is_finite(snapshot.current.hpl) && is_finite(snapshot.current.vpl)) {
        pred_now.hpl = snapshot.current.hpl;
        pred_now.vpl = snapshot.current.vpl;
        pred_now.pl = std::max(pred_now.hpl, pred_now.vpl);
        pred_now.valid = true;
        pred_now.fallback = false;
        pred_now.fallback_reason = "anchored_current_integrity";
      }
    }

    const double consistency_pl_ratio =
        relative_pl_error(pred_now.pl, snapshot.current.pl);
    const double consistency_hpl_error =
        signed_error(pred_now.hpl, snapshot.current.hpl);
    const double consistency_vpl_error =
        signed_error(pred_now.vpl, snapshot.current.vpl);
    const double raw_consistency_pl_ratio =
        relative_pl_error(pred_now_raw.pl, snapshot.current.pl);
    const double raw_consistency_hpl_error =
        signed_error(pred_now_raw.hpl, snapshot.current.hpl);
    const double raw_consistency_vpl_error =
        signed_error(pred_now_raw.vpl, snapshot.current.vpl);

    Row row;
    row["stamp"] = fmt_num(snapshot.stamp);
    row["snapshot_valid"] = bool_str(snapshot.valid);
    row["has_pose"] = bool_str(snapshot.has_pose);
    row["p_x"] = fmt_num(snapshot.p_wb.x());
    row["p_y"] = fmt_num(snapshot.p_wb.y());
    row["p_z"] = fmt_num(snapshot.p_wb.z());
    row["q_w"] = fmt_num(snapshot.q_wb.w());
    row["q_x"] = fmt_num(snapshot.q_wb.x());
    row["q_y"] = fmt_num(snapshot.q_wb.y());
    row["q_z"] = fmt_num(snapshot.q_wb.z());
    row["current_valid"] = bool_str(snapshot.current.valid);
    row["current_integrity_state"] =
        std::to_string(snapshot.current.integrity_state);
    row["current_HPL"] = fmt_num(snapshot.current.hpl);
    row["current_VPL"] = fmt_num(snapshot.current.vpl);
    row["current_PL"] = fmt_num(snapshot.current.pl);
    row["current_PL_E"] = fmt_num(snapshot.current.pl_e);
    row["current_PL_N"] = fmt_num(snapshot.current.pl_n);
    row["current_PL_U"] = fmt_num(snapshot.current.pl_u);
    row["current_HAL"] = fmt_num(snapshot.current.hal);
    row["current_VAL"] = fmt_num(snapshot.current.val);
    row["current_IM"] = fmt_num(snapshot.current.im);
    row["n_sv_used"] = std::to_string(snapshot.current.n_sv_used);
    row["pdop"] = fmt_num(snapshot.current.pdop);
    row["n_hypotheses"] = std::to_string(snapshot.current.n_hypotheses);
    row["n_detected"] = std::to_string(snapshot.current.n_detected);
    row["excluded_prns"] = join_ints(snapshot.current.excluded_prns);
    row["n_trunks_observed"] =
        std::to_string(snapshot.current.n_trunks_observed);
    row["current_tdop"] = fmt_num(snapshot.current.tdop);
    row["lidar_modulation_alpha"] = fmt_num(pred_now.lidar_alpha);
    row["has_epoch"] = bool_str(snapshot.has_epoch);
    row["epoch_sat_count"] =
        std::to_string(snapshot.has_epoch
                           ? static_cast<int>(snapshot.gnss_epoch.sats.size())
                           : 0);
    row["has_lambda_base"] = bool_str(snapshot.has_lambda_base);
    row["has_lidar_snapshot"] = bool_str(snapshot.has_lidar_snapshot);
    row["has_lidar_araim_result"] =
        bool_str(snapshot.has_lidar_araim_result);
    row["pred_now_hpl"] = fmt_num(pred_now.hpl);
    row["pred_now_vpl"] = fmt_num(pred_now.vpl);
    row["pred_now_pl"] = fmt_num(pred_now.pl);
    row["pred_now_raw_hpl"] = fmt_num(pred_now_raw.hpl);
    row["pred_now_raw_vpl"] = fmt_num(pred_now_raw.vpl);
    row["pred_now_raw_pl"] = fmt_num(pred_now_raw.pl);
    row["pred_now_n_vis"] =
        pred_now_raw.n_vis >= 0 ? std::to_string(pred_now_raw.n_vis) : "nan";
    row["pred_now_pdop"] = fmt_num(pred_now_raw.pdop);
    row["pred_now_valid"] = bool_str(pred_now_raw.valid);
    row["pred_now_fallback"] = bool_str(pred_now_raw.fallback);
    row["pred_now_fallback_reason"] = pred_now_raw.fallback_reason;
    row["consistency_pl_ratio"] = fmt_num(consistency_pl_ratio);
    row["consistency_hpl_error"] = fmt_num(consistency_hpl_error);
    row["consistency_vpl_error"] = fmt_num(consistency_vpl_error);
    row["raw_consistency_pl_ratio"] = fmt_num(raw_consistency_pl_ratio);
    row["raw_consistency_hpl_error"] = fmt_num(raw_consistency_hpl_error);
    row["raw_consistency_vpl_error"] = fmt_num(raw_consistency_vpl_error);

    for (std::size_t i = 0; i < kSnapshotCsvFields.size(); ++i) {
      if (i) {
        snapshot_csv_file_ << ',';
      }
      const auto it = row.find(kSnapshotCsvFields[i]);
      snapshot_csv_file_ << csv_escape(it == row.end() ? "" : it->second);
    }
    snapshot_csv_file_ << '\n';
    snapshot_csv_file_.flush();

    ++snapshot_count_;
    if (snapshot.valid) {
      ++snapshot_valid_count_;
    }
    if (snapshot.has_epoch) {
      ++snapshot_with_epoch_count_;
    }
    if (pred_now_raw.valid && is_finite(pred_now_raw.pl)) {
      ++snapshot_pred_now_finite_count_;
    }
    if (pred_now_raw.fallback) {
      ++snapshot_pred_now_fallback_count_;
    }
    if (is_finite(consistency_pl_ratio)) {
      consistency_anchored_pl_ratios_.push_back(consistency_pl_ratio);
    }
    if (is_finite(consistency_hpl_error)) {
      consistency_anchored_hpl_errors_.push_back(consistency_hpl_error);
    }
    if (is_finite(consistency_vpl_error)) {
      consistency_anchored_vpl_errors_.push_back(consistency_vpl_error);
    }
    if (is_finite(raw_consistency_pl_ratio)) {
      consistency_raw_pl_ratios_.push_back(raw_consistency_pl_ratio);
    }
    if (is_finite(raw_consistency_hpl_error)) {
      consistency_raw_hpl_errors_.push_back(raw_consistency_hpl_error);
    }
    if (is_finite(raw_consistency_vpl_error)) {
      consistency_raw_vpl_errors_.push_back(raw_consistency_vpl_error);
    }
  }

  void write_actual_exec_row(const iap::CurrentIntegrityState& current) {
    if (!actual_exec_csv_file_ || !latest_odom_pose_valid_) {
      return;
    }
    std::lock_guard<std::mutex> lock(csv_mutex_);

    double dist = std::numeric_limits<double>::quiet_NaN();
    double lower = std::numeric_limits<double>::quiet_NaN();
    double upper = std::numeric_limits<double>::quiet_NaN();
    double al_h = std::numeric_limits<double>::quiet_NaN();
    double al_v = std::numeric_limits<double>::quiet_NaN();
    double al = std::numeric_limits<double>::quiet_NaN();
    al_values(latest_odom_p_, &dist, &lower, &upper, &al_h, &al_v, &al);
    const PlFields actual_pl = pl_values(latest_odom_p_);
    const double tracking_error_to_plan =
        latest_plan_pose_valid_ ? (latest_odom_p_ - latest_plan_pose_).norm()
                                : std::numeric_limits<double>::quiet_NaN();

    Row row;
    row["stamp"] = fmt_num(current.stamp);
    row["odom_x"] = fmt_num(latest_odom_p_.x());
    row["odom_y"] = fmt_num(latest_odom_p_.y());
    row["odom_z"] = fmt_num(latest_odom_p_.z());
    row["actual_HPL_pred"] = fmt_num(actual_pl.hpl);
    row["actual_VPL_pred"] = fmt_num(actual_pl.vpl);
    row["actual_HAL"] = fmt_num(al_h);
    row["actual_VAL"] = fmt_num(al_v);
    row["actual_IM_H"] = fmt_num(is_finite(al_h) && is_finite(actual_pl.hpl)
                                      ? al_h - actual_pl.hpl
                                      : std::numeric_limits<double>::quiet_NaN());
    row["actual_IM_V"] = fmt_num(is_finite(al_v) && is_finite(actual_pl.vpl)
                                      ? al_v - actual_pl.vpl
                                      : std::numeric_limits<double>::quiet_NaN());
    row["actual_IM_min"] = fmt_num(is_finite(al) && is_finite(actual_pl.pl)
                                        ? al - actual_pl.pl
                                        : std::numeric_limits<double>::quiet_NaN());
    row["current_HPL"] = fmt_num(current.hpl);
    row["current_VPL"] = fmt_num(current.vpl);
    row["current_IM"] = fmt_num(current.im);
    row["tracking_error_to_plan"] = fmt_num(tracking_error_to_plan);
    row["query_source"] = actual_pl.query_source;
    row["pl_model"] = pl_model_;

    for (std::size_t i = 0; i < kActualExecCsvFields.size(); ++i) {
      if (i) {
        actual_exec_csv_file_ << ',';
      }
      const auto it = row.find(kActualExecCsvFields[i]);
      actual_exec_csv_file_ << csv_escape(it == row.end() ? "" : it->second);
    }
    actual_exec_csv_file_ << '\n';
    actual_exec_csv_file_.flush();
  }

  void write_grid_consistency_samples(const double stamp_s) {
    if (!grid_consistency_csv_file_ || !use_pl_grid_) {
      return;
    }
    std::lock_guard<std::mutex> lock(csv_mutex_);
    if (!latest_odom_pose_valid_ || !latest_odom_p_.allFinite()) {
      return;
    }

    const auto grid_stats = field_predictor_.stats();
    const int generation = grid_stats.generation;
    const Eigen::Vector3d center = latest_odom_p_;
    const uint32_t seed = static_cast<uint32_t>(std::llround(stamp_s * 1000.0)) ^
                          static_cast<uint32_t>(generation < 0 ? 0 : generation);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dx(-0.5 * field_predictor_params_.grid_size_x_m,
                                              0.5 * field_predictor_params_.grid_size_x_m);
    std::uniform_real_distribution<double> dy(-0.5 * field_predictor_params_.grid_size_y_m,
                                              0.5 * field_predictor_params_.grid_size_y_m);
    std::uniform_real_distribution<double> dz(-0.5 * field_predictor_params_.grid_size_z_m,
                                              0.5 * field_predictor_params_.grid_size_z_m);

    for (int sample_idx = 0; sample_idx < 20; ++sample_idx) {
      const Eigen::Vector3d p = center + Eigen::Vector3d(dx(rng), dy(rng), dz(rng));
      const PlFields direct = pl_values(p);
      const iap::FuturePLQueryResult grid = field_predictor_.query(p, stamp_s);
      const double abs_err_h = (is_finite(direct.hpl) && is_finite(grid.hpl))
                                   ? std::abs(direct.hpl - grid.hpl)
                                   : std::numeric_limits<double>::quiet_NaN();
      const double rel_err_h = (is_finite(abs_err_h) && is_finite(direct.hpl) &&
                                std::abs(direct.hpl) > 1.0e-9)
                                   ? abs_err_h / std::abs(direct.hpl)
                                   : std::numeric_limits<double>::quiet_NaN();
      const double abs_err_v = (is_finite(direct.vpl) && is_finite(grid.vpl))
                                   ? std::abs(direct.vpl - grid.vpl)
                                   : std::numeric_limits<double>::quiet_NaN();
      const double rel_err_v = (is_finite(abs_err_v) && is_finite(direct.vpl) &&
                                std::abs(direct.vpl) > 1.0e-9)
                                   ? abs_err_v / std::abs(direct.vpl)
                                   : std::numeric_limits<double>::quiet_NaN();

      Row row;
      row["stamp"] = fmt_num(stamp_s);
      row["grid_generation"] = std::to_string(generation);
      row["sample_index"] = std::to_string(sample_idx);
      row["p_x"] = fmt_num(p.x());
      row["p_y"] = fmt_num(p.y());
      row["p_z"] = fmt_num(p.z());
      row["hpl_direct"] = fmt_num(direct.hpl);
      row["hpl_grid"] = fmt_num(grid.hpl);
      row["vpl_direct"] = fmt_num(direct.vpl);
      row["vpl_grid"] = fmt_num(grid.vpl);
      row["abs_err_h"] = fmt_num(abs_err_h);
      row["rel_err_h"] = fmt_num(rel_err_h);
      row["abs_err_v"] = fmt_num(abs_err_v);
      row["rel_err_v"] = fmt_num(rel_err_v);
      row["n_vis_direct"] = direct.n_vis >= 0 ? std::to_string(direct.n_vis) : "nan";
      row["n_vis_grid"] = grid.n_vis >= 0 ? std::to_string(grid.n_vis) : "nan";
      row["query_source"] = grid.query_source;

      for (std::size_t i = 0; i < kGridConsistencyCsvFields.size(); ++i) {
        if (i) {
          grid_consistency_csv_file_ << ',';
        }
        const auto it = row.find(kGridConsistencyCsvFields[i]);
        grid_consistency_csv_file_ << csv_escape(it == row.end() ? "" : it->second);
      }
      grid_consistency_csv_file_ << '\n';
    }
    grid_consistency_csv_file_.flush();
  }

  void on_bspline(const traj_utils::msg::Bspline& msg) {
    open_outputs_if_ready();
    seen_bspline_ = true;
    last_bspline_wall_ = std::chrono::steady_clock::now();
    if (!outputs_open_) {
      return;
    }
    if (!is_finite(latest_odom_stamp_)) {
      warn_once("skipping bspline evaluation until IAP odom is available");
      return;
    }

    const BsplineTrajectory traj = BsplineTrajectory::from_msg(msg);
    if (!traj.valid()) {
      warn_once("received invalid planner bspline; skipping trajectory");
      return;
    }

    const double planner_now = now().seconds();
    const double start_time = stamp_to_sec(msg.start_time);
    const double t_cur = is_finite(start_time) ? std::max(0.0, planner_now - start_time) : 0.0;
    const double duration = traj.duration();
    if (duration <= 0.0) {
      warn_once("received zero-duration planner bspline; skipping trajectory");
      return;
    }
    if (t_cur > duration) {
      warn_once("received planner bspline that is already expired; skipping trajectory");
      return;
    }

    const auto vel_traj = traj.derivative();
    const auto acc_traj = vel_traj ? vel_traj->derivative() : std::nullopt;
    std::vector<Row> rows;
    const double sample_limit = std::max(0.0, std::min(horizon_s_, duration - t_cur));
    int count = std::min(max_samples_,
                         static_cast<int>(std::floor(sample_limit / std::max(dt_s_, 1.0e-6))) + 1);
    count = std::max(1, count);

    for (int idx = 0; idx < count; ++idx) {
      const double sample_t = std::min(static_cast<double>(idx) * dt_s_, sample_limit);
      const double eval_t = std::min(t_cur + sample_t, duration);
      const Eigen::Vector3d pos = traj.evaluate_t(eval_t);
      const Eigen::Vector3d vel = vel_traj ? vel_traj->evaluate_t(eval_t)
                                           : Eigen::Vector3d::Constant(
                                                 std::numeric_limits<double>::quiet_NaN());
      const Eigen::Vector3d acc = acc_traj ? acc_traj->evaluate_t(eval_t)
                                           : Eigen::Vector3d::Constant(
                                                 std::numeric_limits<double>::quiet_NaN());
      Row row = make_sample_row(planner_now, msg.traj_id, idx, sample_t,
                                latest_odom_stamp_ + sample_t, pos, vel, acc,
                                std::numeric_limits<double>::quiet_NaN());
      if (idx == 0 && pos.allFinite()) {
        latest_plan_pose_ = pos;
        latest_plan_pose_valid_ = true;
      }
      write_sample(row);
      rows.push_back(std::move(row));
    }
    ++traj_count_;
    publish_markers(rows);
    publish_integrity_cost_field(rows);
    write_summary();
  }

  void on_pos_cmd(const quadrotor_msgs::msg::PositionCommand& msg) {
    if (seen_bspline_ &&
        std::chrono::steady_clock::now() - last_bspline_wall_ < std::chrono::seconds(2)) {
      return;
    }
    const auto now_wall = std::chrono::steady_clock::now();
    if (now_wall - last_fallback_wall_ <
        std::chrono::duration<double>(std::max(dt_s_, 0.2))) {
      return;
    }
    last_fallback_wall_ = now_wall;
    open_outputs_if_ready();
    if (!outputs_open_) {
      return;
    }
    if (!is_finite(latest_odom_stamp_)) {
      warn_once("skipping pos_cmd fallback until IAP odom is available");
      return;
    }
    warn_once("using pos_cmd fallback; future B-spline trajectory is unavailable");

    ++fallback_traj_id_;
    double planner_stamp = stamp_to_sec(msg.header.stamp);
    if (!is_finite(planner_stamp)) {
      planner_stamp = now().seconds();
    }
    const Eigen::Vector3d pos(msg.position.x, msg.position.y, msg.position.z);
    const Eigen::Vector3d vel(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    const Eigen::Vector3d acc(msg.acceleration.x, msg.acceleration.y,
                              msg.acceleration.z);
    Row row = make_sample_row(planner_stamp, -fallback_traj_id_, 0, 0.0,
                              latest_odom_stamp_, pos, vel, acc, msg.yaw);
    write_sample(row);
    ++traj_count_;
    publish_markers({row});
    write_summary();
  }

  nlohmann::json summary_data() const {
    const auto sum = [](const std::vector<double>& values) {
      return std::accumulate(values.begin(), values.end(), 0.0);
    };
    const auto mean_or_null = [&](const std::vector<double>& values) {
      return values.empty() ? nlohmann::json(nullptr)
                            : nlohmann::json(sum(values) / values.size());
    };
    const auto max_abs_or_null = [](const std::vector<double>& values) {
      if (values.empty()) {
        return nlohmann::json(nullptr);
      }
      double out = 0.0;
      for (const double value : values) {
        out = std::max(out, std::abs(value));
      }
      return nlohmann::json(out);
    };
    const double fallback_rate =
        sample_count_ > 0 ? static_cast<double>(fallback_count_) /
                                static_cast<double>(sample_count_)
                          : 0.0;

    nlohmann::json reason_hist = nlohmann::json::object();
    for (const auto& [reason, count] : fallback_reason_counts_) {
      reason_hist[reason.empty() ? "unknown" : reason] = count;
    }
    const auto counts_json = [](const std::map<std::string, int>& counts) {
      nlohmann::json out = nlohmann::json::object();
      for (const auto& [key, count] : counts) {
        out[key.empty() ? "unknown" : key] = count;
      }
      return out;
    };

    nlohmann::json summary;
    summary["available"] = true;
    summary["run_dir"] = run_dir_ ? run_dir_->string() : "";
    summary["traj_count"] = traj_count_;
    summary["sample_count"] = sample_count_;
    summary["aligned_sample_count"] = 0;
    summary["online_truth_used"] = false;
    summary["odom_source"] = odom_topic_;
    summary["map_source"] = map_source_;
    summary["pl_model"] = pl_model_;
    summary["al_model"] = al_model_;
    summary["sampling"] = {
        {"horizon_s", horizon_s_},
        {"dt_s", dt_s_},
        {"max_samples_per_traj", max_samples_},
    };
    summary["stage1_predictor_config"] = {
      {"git_commit", git_commit()},
      {"source_root", IAP_SOURCE_ROOT},
      {"phase2_pl_model", pl_model_},
      {"phase2_use_pl_grid", use_pl_grid_},
      {"phase2_use_lidar_observability", use_lidar_observability_},
      {"planner_use_integrity_cost", planner_use_integrity_cost_},
      {"gnss_scenario_file", gnss_scenario_file_},
      {"map_source", map_source_},
      {"phase2_al_model", al_model_},
      {"phase2_publish_integrity_cost_field",
       publish_integrity_cost_field_},
      {"phase2_fallback_pl_m", fallback_pl_m_},
      {"phase2_pl_grid_resolution_m",
       field_predictor_params_.grid_resolution_m},
      {"phase2_pl_grid_size_x_m", field_predictor_params_.grid_size_x_m},
      {"phase2_pl_grid_size_y_m", field_predictor_params_.grid_size_y_m},
      {"phase2_pl_grid_size_z_m", field_predictor_params_.grid_size_z_m},
      {"phase2_pl_grid_update_hz", pl_grid_update_hz_},
      {"phase2_lidar_search_radius_m",
       field_predictor_params_.lidar_search_radius_m},
      {"phase2_lidar_min_points", field_predictor_params_.lidar_min_points},
      {"phase2_lidar_good_points", field_predictor_params_.lidar_good_points},
      {"phase2_lidar_sigma_m", field_predictor_params_.lidar_sigma_m},
      {"phase2_lidar_info_scale", field_predictor_params_.lidar_info_scale},
      {"phase2_lidar_alpha_min", field_predictor_params_.lidar_alpha_min},
      {"phase2_lidar_alpha_max", field_predictor_params_.lidar_alpha_max},
    };
    summary["fallback_count"] = fallback_count_;
    summary["fallback_rate"] = fallback_rate;
    summary["fallback_reason_histogram"] = reason_hist;
    summary["finite_gnss_prediction_count"] = finite_gnss_prediction_count_;
    summary["integrity_snapshot"] = {
        {"available", snapshot_count_ > 0},
        {"sample_count", snapshot_count_},
        {"valid_count", snapshot_valid_count_},
        {"has_epoch_count", snapshot_with_epoch_count_},
        {"missing_epoch_count", snapshot_count_ - snapshot_with_epoch_count_},
        {"pred_now_finite_count", snapshot_pred_now_finite_count_},
        {"pred_now_fallback_count", snapshot_pred_now_fallback_count_},
        {"csv", "future_integrity_snapshot.csv"},
    };
    const auto consistency_summary =
        [&](const std::vector<double>& ratios,
            const std::vector<double>& hpl_errors,
            const std::vector<double>& vpl_errors) {
          return nlohmann::json{
              {"available", !ratios.empty()},
              {"finite_count", static_cast<int>(ratios.size())},
              {"warning_threshold_ratio", 0.10},
              {"mean_pl_ratio", mean_or_null(ratios)},
              {"max_pl_ratio",
               ratios.empty()
                   ? nlohmann::json(nullptr)
                   : nlohmann::json(
                         *std::max_element(ratios.begin(), ratios.end()))},
              {"mean_hpl_error", mean_or_null(hpl_errors)},
              {"mean_vpl_error", mean_or_null(vpl_errors)},
              {"max_abs_hpl_error", max_abs_or_null(hpl_errors)},
              {"max_abs_vpl_error", max_abs_or_null(vpl_errors)},
          };
        };
    summary["current_consistency_raw"] = consistency_summary(
        consistency_raw_pl_ratios_, consistency_raw_hpl_errors_,
        consistency_raw_vpl_errors_);
    summary["current_consistency_anchored"] = consistency_summary(
        consistency_anchored_pl_ratios_, consistency_anchored_hpl_errors_,
        consistency_anchored_vpl_errors_);
    summary["current_consistency"] = summary["current_consistency_raw"];
    summary["stage1_capabilities"] = {
        {"fused_araim_style", "deferred_after_rc"},
        {"self_consistency_rc_metric", "current_consistency_raw"},
        {"snapshot_anchor_current_integrity",
         snapshot_anchor_current_integrity_},
        {"planner_integrity_cost",
         planner_use_integrity_cost_ ? "enabled_experimental"
                                     : "disabled_by_default"},
    };
    const auto grid_stats = field_predictor_.stats();
    summary["pl_grid"] = {
        {"enabled", grid_stats.enabled},
        {"active", grid_stats.active},
        {"resolution_m", grid_stats.resolution_m},
        {"dimensions_m",
         {{"x", grid_stats.size_x_m},
          {"y", grid_stats.size_y_m},
          {"z", grid_stats.size_z_m}}},
        {"generation", grid_stats.generation},
        {"update_count", grid_stats.update_count},
        {"skip_count", grid_stats.skip_count},
        {"query_counts",
         {{"grid", grid_stats.query_grid_count},
          {"direct", grid_stats.query_direct_count},
          {"fallback", grid_stats.query_fallback_count}}},
        {"build_time_ms",
         {{"last", json_or_null(grid_stats.last_build_time_ms)},
          {"mean", json_or_null(grid_stats.mean_build_time_ms)},
          {"max", json_or_null(grid_stats.max_build_time_ms)}}},
        {"last_grid_age_s", json_or_null(grid_stats.last_grid_age_s)},
        {"grid_vs_direct_self_check",
         {{"last_pl_ratio",
           json_or_null(grid_stats.last_self_check_pl_ratio)},
          {"warning_threshold_ratio", 0.10}}},
    };
    nlohmann::json lidar_hist = nlohmann::json::object();
    for (const auto& [reason, count] :
         grid_stats.lidar_fallback_reason_histogram) {
      lidar_hist[reason.empty() ? "unknown" : reason] = count;
    }
    const double lidar_valid_rate =
        grid_stats.lidar_query_count > 0
            ? static_cast<double>(grid_stats.lidar_valid_count) /
                  static_cast<double>(grid_stats.lidar_query_count)
            : 0.0;
    summary["lidar_observability"] = {
        {"enabled", grid_stats.lidar_enabled},
        {"use_lidar_observability", use_lidar_observability_},
        {"fused_fim_grid", pl_model_ == "fused_fim_grid"},
        {"query_count", grid_stats.lidar_query_count},
        {"valid_count", grid_stats.lidar_valid_count},
        {"valid_rate", lidar_valid_rate},
        {"fallback_count", grid_stats.lidar_fallback_count},
        {"fallback_reason_histogram", lidar_hist},
        {"mean_alpha", json_or_null(grid_stats.mean_lidar_alpha)},
        {"max_alpha", json_or_null(grid_stats.max_lidar_alpha)},
        {"mean_tdop", json_or_null(grid_stats.mean_lidar_tdop)},
        {"mean_condition", json_or_null(grid_stats.mean_lidar_condition)},
        {"conservative_fusion_check_count",
         grid_stats.lidar_conservative_check_count},
        {"conservative_fusion_violation_count",
         grid_stats.lidar_conservative_violation_count},
        {"nonfinite_debug_count", grid_stats.lidar_nonfinite_debug_count},
    };
    summary["phase_h_lite"] = {
        {"grid_update_timing", grid_stats.enabled ? "available"
                                                  : "skipped_not_applicable"},
        {"lidar_observability",
         grid_stats.lidar_enabled ? "available" : "skipped_not_applicable"},
        {"fused_fim_grid", (pl_model_ == "fused_fim_grid") ? "available"
                                                           : "skipped_not_applicable"},
    };
    summary["predicted_integrity"] = {
        {"safe_count", risk_counts_.count("SAFE_PRED") ? risk_counts_.at("SAFE_PRED") : 0},
        {"marginal_count",
         risk_counts_.count("MARGINAL_PRED") ? risk_counts_.at("MARGINAL_PRED") : 0},
        {"unsafe_count",
         risk_counts_.count("UNSAFE_PRED") ? risk_counts_.at("UNSAFE_PRED") : 0},
        {"unknown_count",
         (risk_counts_.count("UNKNOWN_PL") ? risk_counts_.at("UNKNOWN_PL") : 0) +
             (risk_counts_.count("UNKNOWN_AL") ? risk_counts_.at("UNKNOWN_AL") : 0)},
        {"min_IM", im_values_.empty()
                       ? nlohmann::json(nullptr)
                       : nlohmann::json(*std::min_element(im_values_.begin(), im_values_.end()))},
        {"mean_IM", im_values_.empty()
                        ? nlohmann::json(nullptr)
                        : nlohmann::json(sum(im_values_) / im_values_.size())},
        {"p05_IM", json_or_null(quantile(im_values_, 0.05))},
        {"p50_IM", json_or_null(quantile(im_values_, 0.50))},
        {"p95_PL", json_or_null(quantile(pl_values_, 0.95))},
        {"max_PL", pl_values_.empty()
                       ? nlohmann::json(nullptr)
                       : nlohmann::json(*std::max_element(pl_values_.begin(), pl_values_.end()))},
        {"fallback_count", fallback_count_},
        {"fallback_rate", fallback_rate},
        {"fallback_reason_histogram", reason_hist},
        {"finite_gnss_prediction_count", finite_gnss_prediction_count_},
    };
    summary["pi_cost"] = {
        {"available", !pi_cost_values_.empty()},
        {"count", static_cast<int>(pi_cost_values_.size())},
        {"weight_h", pi_cost_params_.weight_h},
        {"weight_v", pi_cost_params_.weight_v},
        {"marginal_margin_m", pi_cost_params_.marginal_margin_m},
        {"mean", pi_cost_values_.empty()
                     ? nlohmann::json(nullptr)
                     : nlohmann::json(sum(pi_cost_values_) /
                                      static_cast<double>(pi_cost_values_.size()))},
        {"max", pi_cost_values_.empty()
                    ? nlohmann::json(nullptr)
                    : nlohmann::json(*std::max_element(pi_cost_values_.begin(),
                                                       pi_cost_values_.end()))},
        {"p05", json_or_null(quantile(pi_cost_values_, 0.05))},
        {"p50", json_or_null(quantile(pi_cost_values_, 0.50))},
        {"p95", json_or_null(quantile(pi_cost_values_, 0.95))},
        {"risk_band_histogram", counts_json(pi_risk_band_counts_)},
        {"dominant_axis_histogram", counts_json(pi_dominant_axis_counts_)},
    };
    summary["actual_alignment"] = {
        {"matched_count", 0},
        {"match_ratio", 0.0},
        {"mean_time_alignment_error_s", nullptr},
        {"mean_spatial_tracking_error", nullptr},
        {"mean_estimation_error", nullptr},
        {"mean_pred_actual_PL_error", nullptr},
        {"mean_pred_actual_IM_error", nullptr},
        {"safe_unsafe_label_agreement_ratio", nullptr},
    };
    summary["warnings"] = warnings_;
    summary["errors"] = errors_;
    return summary;
  }

  void write_summary() {
    if (!outputs_open_ || !export_dir_) {
      return;
    }
    std::ofstream out(*export_dir_ / "phase2_summary.json",
                      std::ios::out | std::ios::trunc);
    out << summary_data().dump(2) << '\n';
  }

  void publish_markers(const std::vector<Row>& rows) {
    if (!marker_pub_) {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    for (std::size_t idx = 0; idx < rows.size(); ++idx) {
      const auto& row = rows[idx];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.ns = "phase2_pi_lite";
      marker.id = static_cast<int>(idx);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = finite_or_none(row.at("x")).value_or(0.0);
      marker.pose.position.y = finite_or_none(row.at("y")).value_or(0.0);
      marker.pose.position.z = finite_or_none(row.at("z")).value_or(0.0);
      marker.scale.x = 0.12;
      marker.scale.y = 0.12;
      marker.scale.z = 0.12;
      marker.color.r = 0.5f;
      marker.color.g = 0.5f;
      marker.color.b = 0.5f;
      marker.color.a = 0.65f;
      const std::string pi_state =
          row.count("pi_risk_band") ? row.at("pi_risk_band") : "UNKNOWN_PI";
      const std::string state = row.at("risk_state_pred");
      if (pi_state == "SAFE_PI" || state == "SAFE_PRED") {
        marker.color.r = 0.1f;
        marker.color.g = 0.8f;
        marker.color.b = 0.2f;
        marker.color.a = 0.75f;
      } else if (pi_state == "MARGINAL_PI" || state == "MARGINAL_PRED") {
        marker.scale.x = marker.scale.y = marker.scale.z = 0.18;
        marker.color.r = 1.0f;
        marker.color.g = 0.75f;
        marker.color.b = 0.05f;
        marker.color.a = 0.8f;
      } else if (pi_state == "UNSAFE_PI" || state == "UNSAFE_PRED") {
        marker.scale.x = marker.scale.y = marker.scale.z = 0.24;
        marker.color.r = 1.0f;
        marker.color.g = 0.05f;
        marker.color.b = 0.05f;
        marker.color.a = 0.9f;
      }
      arr.markers.push_back(marker);
    }
    marker_pub_->publish(arr);
  }

  void publish_integrity_cost_field(const std::vector<Row>& rows) {
    if (!integrity_cost_field_pub_ || rows.empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        16,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
        "hal", 1, sensor_msgs::msg::PointField::FLOAT32,
        "val", 1, sensor_msgs::msg::PointField::FLOAT32,
        "im_h", 1, sensor_msgs::msg::PointField::FLOAT32,
        "im_v", 1, sensor_msgs::msg::PointField::FLOAT32,
        "im_min", 1, sensor_msgs::msg::PointField::FLOAT32,
        "cost", 1, sensor_msgs::msg::PointField::FLOAT32,
        "grad_x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "grad_y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "grad_z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "risk_band", 1, sensor_msgs::msg::PointField::FLOAT32,
        "risk_band_code", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(rows.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl");
    sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl");
    sensor_msgs::PointCloud2Iterator<float> hal(cloud, "hal");
    sensor_msgs::PointCloud2Iterator<float> val(cloud, "val");
    sensor_msgs::PointCloud2Iterator<float> im_h(cloud, "im_h");
    sensor_msgs::PointCloud2Iterator<float> im_v(cloud, "im_v");
    sensor_msgs::PointCloud2Iterator<float> im_min(cloud, "im_min");
    sensor_msgs::PointCloud2Iterator<float> cost(cloud, "cost");
    sensor_msgs::PointCloud2Iterator<float> grad_x(cloud, "grad_x");
    sensor_msgs::PointCloud2Iterator<float> grad_y(cloud, "grad_y");
    sensor_msgs::PointCloud2Iterator<float> grad_z(cloud, "grad_z");
    sensor_msgs::PointCloud2Iterator<float> risk_band(cloud, "risk_band");
    sensor_msgs::PointCloud2Iterator<float> risk_band_code(cloud, "risk_band_code");
    for (const auto& row : rows) {
      const auto f = [&row](const std::string& key) {
        return static_cast<float>(finite_or_none(row.at(key)).value_or(0.0));
      };
      *x = f("x");
      *y = f("y");
      *z = f("z");
      *hpl = f("PL_H_pred");
      *vpl = f("PL_V_pred");
      *hal = f("AL_H_pred");
      *val = f("AL_V_pred");
      *im_h = f("IM_H_pred");
      *im_v = f("IM_V_pred");
      *im_min = f("IM_pred_axis_min");
      *cost = f("pi_cost_total");
      *grad_x = f("pi_grad_x");
      *grad_y = f("pi_grad_y");
      *grad_z = f("pi_grad_z");
      *risk_band = f("pi_risk_band_code");
      *risk_band_code = f("pi_risk_band_code");
      ++x;
      ++y;
      ++z;
      ++hpl;
      ++vpl;
      ++hal;
      ++val;
      ++im_h;
      ++im_v;
      ++im_min;
      ++cost;
      ++grad_x;
      ++grad_y;
      ++grad_z;
      ++risk_band;
      ++risk_band_code;
    }
    integrity_cost_field_pub_->publish(cloud);
  }

  void finalize() {
    if (finalized_) {
      return;
    }
    finalized_ = true;
    join_pl_grid_worker();
    if (outputs_open_) {
      write_summary();
    }
    if (csv_file_) {
      csv_file_.flush();
      csv_file_.close();
    }
    if (snapshot_csv_file_) {
      snapshot_csv_file_.flush();
      snapshot_csv_file_.close();
    }
    if (actual_exec_csv_file_) {
      actual_exec_csv_file_.flush();
      actual_exec_csv_file_.close();
    }
    if (grid_consistency_csv_file_) {
      grid_consistency_csv_file_.flush();
      grid_consistency_csv_file_.close();
    }
  }

  iap::PredictedAraimComputer::Params predictor_params_{};
  iap::PredictedAraimComputer predictor_;
  iap::FuturePLFieldPredictor::Params field_predictor_params_{};
  iap::FuturePLFieldPredictor field_predictor_;
  iap::PICostAdapter::Params pi_cost_params_{};
  iap::PICostAdapter pi_cost_adapter_;
  iap::IntegritySnapshotBuilder snapshot_builder_;
  iap::LocalOccupancyGrid occupancy_;
  mutable std::mutex occupancy_mutex_;

  std::string log_root_;
  std::string explicit_run_dir_;
  std::string map_source_;
  std::string odom_topic_;
  std::string bspline_topic_;
  std::string pos_cmd_topic_;
  std::string map_topic_;
  std::string integrity_topic_;
  std::string range_meas_topic_;
  std::string ephem_topic_;
  std::string glo_ephem_topic_;
  std::string receiver_lla_topic_;
  std::string iono_topic_;
  double horizon_s_ = 5.0;
  double dt_s_ = 0.2;
  int max_samples_ = 30;
  std::string pl_model_ = "constant_current";
  std::string al_model_ = "cloud_clearance";
  double fallback_pl_m_ = 20.0;
  bool use_pl_grid_ = false;
  bool use_lidar_observability_ = false;
  double pl_grid_update_hz_ = 2.0;
  int lidar_map_max_points_ = 2500;
  double drone_radius_ = 0.35;
  double safety_buffer_ = 0.20;
  double gamma_h_ = 0.8;
  double gamma_v_ = 0.8;
  double z_min_ = 0.5;
  double z_max_ = 5.0;
  double safe_margin_ = 0.0;
  bool publish_markers_ = true;
  double pi_cost_gradient_step_m_ = 0.5;
  bool snapshot_anchor_current_integrity_ = true;
  bool publish_integrity_cost_field_ = false;
  bool planner_use_integrity_cost_ = false;
  std::string integrity_cost_field_topic_ = "/iap/integrity_cost_field";
  std::string gnss_scenario_file_;

  std::chrono::system_clock::time_point start_wall_;
  std::optional<std::filesystem::path> initial_latest_target_;
  std::optional<std::filesystem::path> run_dir_;
  std::optional<std::filesystem::path> export_dir_;
  std::ofstream csv_file_;
  std::ofstream snapshot_csv_file_;
  std::ofstream actual_exec_csv_file_;
  std::ofstream grid_consistency_csv_file_;
  mutable std::mutex csv_mutex_;
  bool outputs_open_ = false;
  bool finalized_ = false;

  double latest_odom_stamp_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d latest_odom_p_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond latest_odom_q_ = Eigen::Quaterniond::Identity();
  bool latest_odom_pose_valid_ = false;
    Eigen::Vector3d latest_plan_pose_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    bool latest_plan_pose_valid_ = false;
  double latest_cloud_stamp_ = std::numeric_limits<double>::quiet_NaN();
  double current_hpl_ = std::numeric_limits<double>::quiet_NaN();
  double current_vpl_ = std::numeric_limits<double>::quiet_NaN();
  double current_integrity_stamp_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Vector3d> latest_cloud_points_;
  bool seen_bspline_ = false;
  std::chrono::steady_clock::time_point last_bspline_wall_{};
  std::chrono::steady_clock::time_point last_fallback_wall_{};
  int64_t fallback_traj_id_ = 0;

  int traj_count_ = 0;
  int sample_count_ = 0;
  std::map<std::string, int> risk_counts_;
  std::map<std::string, int> pi_risk_band_counts_;
  std::map<std::string, int> pi_dominant_axis_counts_;
  std::vector<double> im_values_;
  std::vector<double> pl_values_;
  std::vector<double> pi_cost_values_;
  std::vector<std::string> warnings_;
  std::vector<std::string> errors_;
  int fallback_count_ = 0;
  int finite_gnss_prediction_count_ = 0;
  std::map<std::string, int> fallback_reason_counts_;
  int snapshot_count_ = 0;
  int snapshot_valid_count_ = 0;
  int snapshot_with_epoch_count_ = 0;
  int snapshot_pred_now_finite_count_ = 0;
  int snapshot_pred_now_fallback_count_ = 0;
  std::vector<double> consistency_raw_pl_ratios_;
  std::vector<double> consistency_raw_hpl_errors_;
  std::vector<double> consistency_raw_vpl_errors_;
  std::vector<double> consistency_anchored_pl_ratios_;
  std::vector<double> consistency_anchored_hpl_errors_;
  std::vector<double> consistency_anchored_vpl_errors_;

  bool origin_set_ = false;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
  std::vector<double> iono_params_;
  std::optional<iap::GnssEpoch> latest_epoch_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssMeasMsg>::SharedPtr range_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr glo_ephem_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr receiver_lla_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr iono_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      integrity_cost_field_pub_;
  rclcpp::TimerBase::SharedPtr open_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr pl_grid_timer_;
  std::atomic_bool pl_grid_building_{false};
  std::mutex pl_grid_worker_mutex_;
  std::thread pl_grid_worker_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Phase2PlannerIntegrityEvaluator>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "phase2 evaluator exception: %s", e.what());
  }
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
