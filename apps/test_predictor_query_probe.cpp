#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_ionosphere_parameter.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <iap/map/local_occupancy.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/planner/integrity_snapshot.hpp>
#include <iap/predictor/fusion_advisory_predictor.hpp>
#include <iap/predictor/gnss_advisory_predictor.hpp>
#include <iap/predictor/lidar_advisory_predictor.hpp>
#include <iap/predictor/predictor_module.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {

constexpr double kLightSpeed = 2.99792458e8;

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool is_finite(const double value) {
  return std::isfinite(value);
}

std::string bool_str(const bool value) {
  return value ? "1" : "0";
}

std::string fmt_num(const double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream oss;
  oss << std::setprecision(12) << value;
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

std::string join_ints(const std::vector<int>& values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << '|';
    }
    oss << values[i];
  }
  return oss.str();
}

std::vector<int> parse_batch_sizes(const std::string& value) {
  std::vector<int> out;
  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(),
                               [](unsigned char c) { return std::isspace(c); }),
                token.end());
    if (token.empty()) {
      continue;
    }
    try {
      const int parsed = std::stoi(token);
      if (parsed > 0) {
        out.push_back(parsed);
      }
    } catch (const std::exception&) {
      // Invalid tokens are ignored; launch/analyzer checks enforce E12 content.
    }
  }
  return out;
}

void create_parent_directories(const std::string& path) {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

std::string matrix_header(const std::string& prefix) {
  std::ostringstream oss;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (r != 0 || c != 0) {
        oss << ',';
      }
      oss << prefix << r << c;
    }
  }
  return oss.str();
}

std::string matrix_values(const Eigen::Matrix3d& matrix) {
  std::ostringstream oss;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (r != 0 || c != 0) {
        oss << ',';
      }
      oss << fmt_num(matrix(r, c));
    }
  }
  return oss.str();
}

double percentile(std::vector<double> values, const double q) {
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](double v) { return !std::isfinite(v); }),
               values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double pos = std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
  if (lo == hi) {
    return values[lo];
  }
  const double w = pos - static_cast<double>(lo);
  return values[lo] * (1.0 - w) + values[hi] * w;
}

double us_between(const std::chrono::steady_clock::time_point& a,
                  const std::chrono::steady_clock::time_point& b) {
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count()) /
         1000.0;
}

std::string mode_or_default(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (mode == "gnss_only" || mode == "lidar_only" || mode == "fusion") {
    return mode;
  }
  return "fusion";
}

const char* flag_name(const iap::PredictorResultFlags flag) {
  switch (flag) {
    case iap::PREDICTOR_RESULT_VALID:
      return "valid";
    case iap::PREDICTOR_RESULT_FALLBACK:
      return "fallback";
    case iap::PREDICTOR_RESULT_GNSS_VALID:
      return "gnss_valid";
    case iap::PREDICTOR_RESULT_LIDAR_VALID:
      return "lidar_valid";
    case iap::PREDICTOR_RESULT_FUSION_VALID:
      return "fusion_valid";
    case iap::PREDICTOR_RESULT_PRIOR_VALID:
      return "prior_valid";
    case iap::PREDICTOR_RESULT_GNSS_USED:
      return "gnss_used";
    case iap::PREDICTOR_RESULT_LIDAR_USED:
      return "lidar_used";
    case iap::PREDICTOR_RESULT_REGULARIZED:
      return "regularized";
    case iap::PREDICTOR_RESULT_CONSERVATIVE_MAX:
      return "conservative_max";
    case iap::PREDICTOR_RESULT_AVAILABLE:
      return "available";
  }
  return "unknown";
}

bool flag_set(const uint32_t flags, const iap::PredictorResultFlags flag) {
  return (flags & static_cast<uint32_t>(flag)) != 0u;
}

struct SelectedOutput {
  std::string source = "NONE";
  bool available = false;
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double pl = std::numeric_limits<double>::quiet_NaN();
};

struct QuerySpec {
  std::string label;
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
};

struct BatchStats {
  int queries_attempted = 0;
  int queries_recorded = 0;
  int selected_valid_count = 0;
  int selected_fallback_count = 0;
  int gnss_valid_count = 0;
  int lidar_valid_count = 0;
  int fused_valid_count = 0;
  int fusion_source_count = 0;
};

struct CurrentVariantSpec {
  std::string label;
  bool override_active = false;
  iap::CurrentIntegrityState current;
};

struct StaleVariantSpec {
  std::string label = "normal";
  std::string stale_source;
};

}  // namespace

class TestPredictorQueryProbe final : public rclcpp::Node {
 public:
  TestPredictorQueryProbe() : rclcpp::Node("test_predictor_query_probe") {
    output_mode_ = mode_or_default(declare_parameter<std::string>(
        "predictor_output_mode", "fusion"));
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/drone_0_visual_slam/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map_generator/global_cloud");
    integrity_topic_ = declare_parameter<std::string>("integrity_topic", "/iap/integrity");
    range_meas_topic_ =
        declare_parameter<std::string>("range_meas_topic", "/ublox_driver/range_meas");
    ephem_topic_ = declare_parameter<std::string>("ephem_topic", "/ublox_driver/ephem");
    glo_ephem_topic_ =
        declare_parameter<std::string>("glo_ephem_topic", "/ublox_driver/glo_ephem");
    receiver_lla_topic_ =
        declare_parameter<std::string>("receiver_lla_topic", "/ublox_driver/receiver_lla");
    iono_topic_ =
        declare_parameter<std::string>("iono_topic", "/ublox_driver/iono_params");
    csv_path_ = declare_parameter<std::string>(
        "csv_path", "/tmp/test_predictor_query_probe.csv");
    summary_path_ = declare_parameter<std::string>(
        "summary_path", "/tmp/test_predictor_query_probe_summary.json");
    query_set_ = declare_parameter<std::string>("query_set", "current_pose");
    enable_debug_log_ =
        declare_parameter<bool>("enable_debug_log", false);
    query_debug_csv_path_ = declare_parameter<std::string>(
        "query_debug_csv_path", "/tmp/source_selection_debug.csv");
    gnss_debug_csv_path_ = declare_parameter<std::string>(
        "gnss_debug_csv_path", "/tmp/gnss_epoch_debug.csv");
    gnss_visibility_csv_path_ = declare_parameter<std::string>(
        "gnss_visibility_csv_path", "/tmp/gnss_visibility_by_query.csv");
    lidar_debug_csv_path_ = declare_parameter<std::string>(
        "lidar_debug_csv_path", "/tmp/predictor_lidar_debug.csv");
    lidar_primitives_debug_csv_path_ = declare_parameter<std::string>(
        "lidar_primitives_debug_csv_path",
        "/tmp/predictor_lidar_primitives_debug.csv");
    fusion_debug_csv_path_ = declare_parameter<std::string>(
        "fusion_debug_csv_path", "/tmp/predictor_fusion_debug.csv");
    timing_csv_path_ = declare_parameter<std::string>(
        "timing_csv_path", "/tmp/latency_debug.csv");
    latency_stress_tick_csv_path_ = declare_parameter<std::string>(
        "latency_stress_tick_csv_path", "/tmp/latency_stress_tick_debug.csv");
    probe_metadata_path_ = declare_parameter<std::string>(
        "probe_metadata_path", "/tmp/predictor_probe_config.json");
    fallback_reason_csv_path_ = declare_parameter<std::string>(
        "fallback_reason_csv_path", "/tmp/fallback_reason_by_time.csv");
    stale_debug_csv_path_ = declare_parameter<std::string>(
        "stale_debug_csv_path", "/tmp/stale_snapshot_debug.csv");
    map_snapshot_csv_path_ = declare_parameter<std::string>(
        "map_snapshot_csv_path", "/tmp/downsampled_map.csv");
    debug_max_lidar_primitives_ =
        declare_parameter<int>("debug_max_lidar_primitives", 500);
    use_lidar_primitives_ =
        declare_parameter<bool>("use_lidar_primitives", true);
    enable_current_prior_ =
        declare_parameter<bool>("enable_current_prior", true);
    require_gnss_for_selected_output_ =
        declare_parameter<bool>("require_gnss_for_selected_output", false);
    current_variant_set_ =
        declare_parameter<std::string>("current_variant_set", "observed");
    current_high_hpl_m_ =
        declare_parameter<double>("current_high_hpl_m", 1000.0);
    current_high_vpl_m_ =
        declare_parameter<double>("current_high_vpl_m", 1000.0);
    current_unsafe_hpl_m_ =
        declare_parameter<double>("current_unsafe_hpl_m", 500.0);
    current_unsafe_vpl_m_ =
        declare_parameter<double>("current_unsafe_vpl_m", 600.0);
    current_unsafe_state_ =
        declare_parameter<int>("current_unsafe_state", 2);
    stale_variant_set_ =
        declare_parameter<std::string>("stale_variant_set", "observed");
    stale_age_s_ =
        declare_parameter<double>("stale_age_s", 2.0);
    enable_map_snapshot_ =
        declare_parameter<bool>("enable_map_snapshot", true);
    lidar_map_max_points_ =
        declare_parameter<int>("lidar_map_max_points", 2500);
    query_min_period_s_ =
        declare_parameter<double>("query_min_period_s", 0.0);
    latency_stress_batch_sizes_ = parse_batch_sizes(
        declare_parameter<std::string>("latency_stress_batch_sizes", ""));
    gnss_epoch_max_age_s_ =
        declare_parameter<double>("gnss_epoch_max_age_s", 0.5);
    gnss_sigma_scale_ =
        declare_parameter<double>("gnss_sigma_scale", 1.0);
    if (!std::isfinite(gnss_sigma_scale_) || gnss_sigma_scale_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "invalid gnss_sigma_scale %.6f; using 1.0",
                  gnss_sigma_scale_);
      gnss_sigma_scale_ = 1.0;
    }

    params_.gnss.geometry_params.min_sats =
        declare_parameter<int>("gnss_min_sats", params_.gnss.geometry_params.min_sats);
    params_.gnss.geometry_params.K_ff =
        declare_parameter<double>("gnss_K_ff", params_.gnss.geometry_params.K_ff);
    params_.gnss.geometry_params.K_fa =
        declare_parameter<double>("gnss_K_fa", params_.gnss.geometry_params.K_fa);
    params_.gnss.geometry_params.K_md =
        declare_parameter<double>("gnss_K_md", params_.gnss.geometry_params.K_md);
    params_.gnss.visibility_params.min_elevation =
        declare_parameter<double>("gnss_min_elevation_rad",
                                  params_.gnss.visibility_params.min_elevation);
    params_.lidar.enable_legacy_observability =
        declare_parameter<bool>("lidar_enable_legacy_observability", false);
    params_.lidar.fim_params.fim_radius_m =
        declare_parameter<double>("lidar_fim_radius_m",
                                  params_.lidar.fim_params.fim_radius_m);
    params_.lidar.fim_params.fim_min_voxels =
        declare_parameter<int>("lidar_fim_min_voxels",
                               params_.lidar.fim_params.fim_min_voxels);
    params_.lidar.fim_params.fim_range_sigma_base =
        declare_parameter<double>("lidar_fim_range_sigma_base",
                                  params_.lidar.fim_params.fim_range_sigma_base);
    params_.lidar.fim_params.fim_condition_max =
        declare_parameter<double>("lidar_fim_condition_max",
                                  params_.lidar.fim_params.fim_condition_max);
    params_.fusion.K_H_adv =
        declare_parameter<double>("fusion_K_H_adv", params_.fusion.K_H_adv);
    params_.fusion.K_V_adv =
        declare_parameter<double>("fusion_K_V_adv", params_.fusion.K_V_adv);
    params_.fusion.fim_epsilon =
        declare_parameter<double>("fusion_fim_epsilon", params_.fusion.fim_epsilon);
    params_.fusion.conservative_max_with_gnss =
        declare_parameter<bool>("fusion_conservative_max_with_gnss",
                                params_.fusion.conservative_max_with_gnss);
    params_.freshness.enabled =
        declare_parameter<bool>("enable_freshness_guard",
                                params_.freshness.enabled);
    params_.freshness.max_odom_age_s =
        declare_parameter<double>("max_odom_age_s",
                                  params_.freshness.max_odom_age_s);
    params_.freshness.max_integrity_age_s =
        declare_parameter<double>("max_integrity_age_s",
                                  params_.freshness.max_integrity_age_s);
    params_.freshness.max_gnss_age_s =
        declare_parameter<double>("max_gnss_age_s",
                                  params_.freshness.max_gnss_age_s);
    params_.freshness.max_snapshot_age_s =
        declare_parameter<double>("max_snapshot_age_s",
                                  params_.freshness.max_snapshot_age_s);
    if (gnss_sigma_scale_ != 1.0) {
      auto& canopy = params_.gnss.visibility_params.canopy;
      canopy.sigma_0 *= gnss_sigma_scale_;
      canopy.sigma_mp *= gnss_sigma_scale_;
      canopy.sigma_c *= gnss_sigma_scale_;
    }

    lidar_primitive_params_.pca_radius_m =
        declare_parameter<double>("lidar_pca_radius_m", lidar_primitive_params_.pca_radius_m);
    lidar_primitive_params_.pca_max_points =
        declare_parameter<int>("lidar_pca_max_points", lidar_primitive_params_.pca_max_points);
    lidar_primitive_params_.pca_min_support =
        declare_parameter<int>("lidar_pca_min_support", lidar_primitive_params_.pca_min_support);
    lidar_primitive_params_.pca_voxel_sample_m =
        declare_parameter<double>("lidar_pca_voxel_sample_m",
                                  lidar_primitive_params_.pca_voxel_sample_m);
    lidar_primitive_params_.pca_max_primitives =
        declare_parameter<int>("lidar_pca_max_primitives",
                               lidar_primitive_params_.pca_max_primitives);
    lidar_primitive_params_.use_cloud_normals_first =
        declare_parameter<bool>("lidar_use_cloud_normals_first",
                                lidar_primitive_params_.use_cloud_normals_first);

    module_.set_params(params_);
    gnss_.set_params(params_.gnss);
    lidar_.set_params(params_.lidar);
    fusion_.set_params(params_.fusion);
    module_.set_local_occupancy(&occupancy_);
    gnss_.set_local_occupancy(&occupancy_);

    open_outputs();
    write_probe_metadata();
    write_summary();

    const rclcpp::QoS qos(50);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          on_odom(*msg);
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        map_topic_, rclcpp::QoS(2).transient_local().reliable(),
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
          const Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
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

    RCLCPP_INFO(get_logger(),
                "predictor probe mode=%s csv=%s summary=%s odom=%s map=%s integrity=%s",
                output_mode_.c_str(), csv_path_.c_str(), summary_path_.c_str(),
                odom_topic_.c_str(), map_topic_.c_str(), integrity_topic_.c_str());
  }

 private:
  void open_outputs() {
    create_parent_directories(csv_path_);
    create_parent_directories(summary_path_);
    create_parent_directories(probe_metadata_path_);
    csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
    csv_file_
        << "stamp,query_index,query_label,query_offset_x,query_offset_y,"
           "query_offset_z,output_mode,odom_stamp_s,integrity_stamp_s,"
           "gnss_epoch_stamp_s,snapshot_stamp_s,odom_age_s,integrity_age_s,"
           "gnss_age_s,snapshot_age_s,query_x,query_y,query_z,"
           "current_variant_label,current_override_active,current_valid,"
           "stale_variant_label,stale_source,freshness_guard_reason,"
           "selected_source,selected_available,"
           "selected_valid,selected_fallback,selected_hpl,selected_vpl,selected_pl,"
           "selected_fallback_reason,module_available,module_valid,module_fallback,"
           "module_fallback_reason,source_flags,has_pose,has_epoch,has_lambda_base,"
           "current_hpl,current_vpl,current_pl_e,current_pl_n,current_pl_u,"
           "current_hal,current_val,current_im,current_state,current_n_sv_used,"
           "current_pdop,current_n_hypotheses,current_n_detected,current_excluded_prns,"
           "gnss_available,gnss_valid,gnss_fallback,gnss_reason,gnss_hpl,gnss_vpl,"
           "gnss_pl,gnss_pl_e,gnss_pl_n,gnss_pl_u,gnss_pdop,gnss_hdop,gnss_vdop,"
           "gnss_sigma_h,gnss_sigma_v,gnss_n_visible,gnss_n_used,gnss_n_excluded,"
           "sat_visible_mask,sat_used_mask,excluded_prn_mask,effective_sigma_mean,"
           "effective_sigma_max,gnss_fim_valid,gnss_lambda_trace,"
           "gnss_lambda_min_eig,gnss_lambda_condition,lidar_available,lidar_valid,"
           "lidar_fallback,lidar_reason,lidar_fim_valid,lidar_n_primitives,"
           "lidar_n_valid_normals,lidar_lambda_trace,lidar_lambda_min_eig,"
           "lidar_lambda_condition,lidar_alpha,lidar_tdop,"
           "fused_available,fused_valid,fused_fallback,"
           "fused_reason,fused_hpl,fused_vpl,fused_pl,fused_prior_valid,"
           "fused_gnss_used,fused_lidar_used,fused_lambda_prior_trace,"
           "fused_lambda_gnss_trace,fused_lambda_lidar_trace,fused_lambda_pred_trace,"
           "fused_lambda_pred_min_eig,fused_lambda_pred_condition,lambda_sum_error,"
           "gnss_us,lidar_us,fusion_us,total_step_us,module_total_us,"
           "query_batch_size,query_batch_index,"
           "cloud_points,lidar_pca_primitives,lidar_pca_valid_normals\n";

    if (!enable_debug_log_) {
      return;
    }
    create_parent_directories(query_debug_csv_path_);
    create_parent_directories(gnss_debug_csv_path_);
    create_parent_directories(gnss_visibility_csv_path_);
    create_parent_directories(lidar_debug_csv_path_);
    create_parent_directories(lidar_primitives_debug_csv_path_);
    create_parent_directories(fusion_debug_csv_path_);
    create_parent_directories(timing_csv_path_);
    create_parent_directories(latency_stress_tick_csv_path_);
    create_parent_directories(fallback_reason_csv_path_);
    create_parent_directories(stale_debug_csv_path_);
    if (enable_map_snapshot_) {
      create_parent_directories(map_snapshot_csv_path_);
    }

    query_debug_csv_.open(query_debug_csv_path_, std::ios::out | std::ios::trunc);
    query_debug_csv_
        << "stamp,query_index,query_label,query_offset_x,query_offset_y,"
           "query_offset_z,query_x,query_y,query_z,query_time_s,horizon_s,"
           "has_pose,has_epoch,has_lambda_base,source_flags,source_flag_names,"
           "selected_source,selected_valid,selected_hpl,selected_vpl,selected_pl,"
           "selected_reason,module_valid,module_available,module_reason,"
           "gnss_valid,gnss_reason,lidar_valid,lidar_reason,fused_valid,fused_reason\n";

    gnss_debug_csv_.open(gnss_debug_csv_path_, std::ios::out | std::ios::trunc);
    gnss_debug_csv_
        << "stamp,query_index,query_label,epoch_sat_count,sat_index,sat_id,constellation,"
           "elevation,azimuth,pr_sigma,excluded,gnss_n_visible,gnss_n_used,"
           "gnss_valid,gnss_fim_valid,gnss_reason\n";

    gnss_visibility_csv_.open(gnss_visibility_csv_path_, std::ios::out | std::ios::trunc);
    gnss_visibility_csv_
        << "stamp,query_index,query_label,query_x,query_y,query_z,"
           "epoch_sat_count,gnss_n_visible,gnss_n_used,gnss_n_excluded,"
           "gnss_pdop,gnss_hdop,gnss_vdop,gnss_sigma_h,gnss_sigma_v,"
           "effective_sigma_mean,effective_sigma_max,sat_visible_mask,"
           "sat_used_mask,excluded_prn_mask,gnss_valid,gnss_reason\n";

    lidar_debug_csv_.open(lidar_debug_csv_path_, std::ios::out | std::ios::trunc);
    lidar_debug_csv_
        << "stamp,query_index,cloud_points,lidar_pca_primitives_total,"
           "lidar_pca_valid_normals,lidar_pca_invalid_normals,"
           "lidar_pca_support_mean,lidar_pca_support_min,lidar_pca_radius_m,"
           "lidar_pca_reason,lidar_valid,lidar_fim_valid,lidar_regularized,"
           "lidar_n_primitives,lidar_n_valid_normals,lidar_lambda_trace,"
           "lidar_lambda_min_eig,lidar_lambda_condition,lidar_alpha,lidar_tdop,"
           "lidar_reason\n";

    lidar_primitives_debug_csv_.open(lidar_primitives_debug_csv_path_,
                                     std::ios::out | std::ios::trunc);
    lidar_primitives_debug_csv_
        << "stamp,map_stamp_s,map_age_s,cloud_update_index,primitive_index,center_x,center_y,center_z,"
           "normal_x,normal_y,normal_z,weight,normal_confidence,support_count\n";

    fusion_debug_csv_.open(fusion_debug_csv_path_, std::ios::out | std::ios::trunc);
    fusion_debug_csv_
        << "stamp,query_index,fused_valid,fused_available,fused_fallback,"
           "prior_valid,gnss_used,lidar_used,epsilon_applied,"
           "degeneracy_regularized,conservative_max_applied,"
        << matrix_header("lambda_prior_") << ','
        << matrix_header("lambda_gnss_") << ','
        << matrix_header("lambda_lidar_") << ','
        << matrix_header("lambda_pred_") << ','
        << matrix_header("sigma_pos_")
        << ",lambda_sum_error,fused_hpl,fused_vpl,fused_pl,fused_reason\n";

    timing_csv_.open(timing_csv_path_, std::ios::out | std::ios::trunc);
    timing_csv_
        << "stamp,query_index,query_label,query_batch_size,query_batch_index,"
           "gnss_us,lidar_us,fusion_us,total_step_us,"
           "module_total_us,selected_source,module_valid,module_reason\n";

    latency_stress_tick_csv_.open(latency_stress_tick_csv_path_,
                                  std::ios::out | std::ios::trunc);
    latency_stress_tick_csv_
        << "stamp,tick_index,query_batch_size,queries_attempted,queries_recorded,"
           "batch_total_us,selected_valid_count,selected_fallback_count,"
           "gnss_valid_count,lidar_valid_count,fused_valid_count,"
           "fusion_source_count\n";

    fallback_reason_csv_.open(fallback_reason_csv_path_, std::ios::out | std::ios::trunc);
    fallback_reason_csv_
        << "stamp,query_index,query_label,selected_source,selected_valid,"
           "selected_fallback,selected_reason,module_reason,gnss_reason,"
           "lidar_reason,fused_reason\n";

    stale_debug_csv_.open(stale_debug_csv_path_, std::ios::out | std::ios::trunc);
    stale_debug_csv_
        << "stamp,query_index,query_label,current_variant_label,stale_variant_label,"
           "stale_source,odom_age_s,integrity_age_s,gnss_age_s,snapshot_age_s,"
           "selected_valid,selected_fallback,fallback_reason,module_valid,"
           "module_fallback,module_reason,selected_hpl,selected_vpl,selected_pl\n";

    if (enable_map_snapshot_) {
      map_snapshot_csv_.open(map_snapshot_csv_path_, std::ios::out | std::ios::trunc);
      map_snapshot_csv_
          << "point_index,x,y,z,has_normal,normal_x,normal_y,normal_z,map_stamp_s\n";
    }
  }

  void write_probe_metadata() const {
    nlohmann::json metadata;
    metadata["output_mode"] = output_mode_;
    metadata["query_set"] = query_set_;
    metadata["enable_debug_log"] = enable_debug_log_;
    metadata["topics"] = {
        {"odom", odom_topic_},
        {"map", map_topic_},
        {"integrity", integrity_topic_},
        {"range_meas", range_meas_topic_},
        {"ephem", ephem_topic_},
        {"glo_ephem", glo_ephem_topic_},
        {"receiver_lla", receiver_lla_topic_},
        {"iono", iono_topic_},
    };
    metadata["outputs"] = {
        {"csv_path", csv_path_},
        {"summary_path", summary_path_},
        {"query_debug_csv_path", query_debug_csv_path_},
        {"gnss_debug_csv_path", gnss_debug_csv_path_},
        {"gnss_visibility_csv_path", gnss_visibility_csv_path_},
        {"lidar_debug_csv_path", lidar_debug_csv_path_},
        {"lidar_primitives_debug_csv_path", lidar_primitives_debug_csv_path_},
        {"fusion_debug_csv_path", fusion_debug_csv_path_},
        {"timing_csv_path", timing_csv_path_},
        {"latency_stress_tick_csv_path", latency_stress_tick_csv_path_},
        {"probe_metadata_path", probe_metadata_path_},
        {"fallback_reason_csv_path", fallback_reason_csv_path_},
        {"stale_debug_csv_path", stale_debug_csv_path_},
        {"map_snapshot_csv_path", map_snapshot_csv_path_},
    };
    metadata["probe"] = {
        {"use_lidar_primitives", use_lidar_primitives_},
        {"enable_current_prior", enable_current_prior_},
        {"require_gnss_for_selected_output", require_gnss_for_selected_output_},
        {"current_variant_set", current_variant_set_},
        {"current_high_hpl_m", current_high_hpl_m_},
        {"current_high_vpl_m", current_high_vpl_m_},
        {"current_unsafe_hpl_m", current_unsafe_hpl_m_},
        {"current_unsafe_vpl_m", current_unsafe_vpl_m_},
        {"current_unsafe_state", current_unsafe_state_},
        {"stale_variant_set", stale_variant_set_},
        {"stale_age_s", stale_age_s_},
        {"enable_map_snapshot", enable_map_snapshot_},
        {"lidar_map_max_points", lidar_map_max_points_},
        {"query_min_period_s", query_min_period_s_},
        {"latency_stress_batch_sizes", latency_stress_batch_sizes_},
        {"gnss_epoch_max_age_s", gnss_epoch_max_age_s_},
        {"gnss_sigma_scale", gnss_sigma_scale_},
        {"query_set", query_set_},
        {"debug_max_lidar_primitives", debug_max_lidar_primitives_},
    };
    metadata["predictor_params"] = {
        {"gnss_min_sats", params_.gnss.geometry_params.min_sats},
        {"gnss_K_ff", params_.gnss.geometry_params.K_ff},
        {"gnss_K_fa", params_.gnss.geometry_params.K_fa},
        {"gnss_K_md", params_.gnss.geometry_params.K_md},
        {"gnss_min_elevation_rad", params_.gnss.visibility_params.min_elevation},
        {"gnss_canopy_sigma_0", params_.gnss.visibility_params.canopy.sigma_0},
        {"gnss_canopy_sigma_mp", params_.gnss.visibility_params.canopy.sigma_mp},
        {"gnss_canopy_sigma_c", params_.gnss.visibility_params.canopy.sigma_c},
        {"gnss_canopy_alpha", params_.gnss.visibility_params.canopy.alpha},
        {"lidar_enable_legacy_observability",
         params_.lidar.enable_legacy_observability},
        {"lidar_fim_radius_m", params_.lidar.fim_params.fim_radius_m},
        {"lidar_fim_min_voxels", params_.lidar.fim_params.fim_min_voxels},
        {"lidar_fim_range_sigma_base",
         params_.lidar.fim_params.fim_range_sigma_base},
        {"lidar_fim_condition_max", params_.lidar.fim_params.fim_condition_max},
        {"fusion_K_H_adv", params_.fusion.K_H_adv},
        {"fusion_K_V_adv", params_.fusion.K_V_adv},
        {"fusion_fim_epsilon", params_.fusion.fim_epsilon},
        {"fusion_conservative_max_with_gnss",
         params_.fusion.conservative_max_with_gnss},
        {"freshness_guard_enabled", params_.freshness.enabled},
        {"max_odom_age_s", params_.freshness.max_odom_age_s},
        {"max_integrity_age_s", params_.freshness.max_integrity_age_s},
        {"max_gnss_age_s", params_.freshness.max_gnss_age_s},
        {"max_snapshot_age_s", params_.freshness.max_snapshot_age_s},
    };
    create_parent_directories(probe_metadata_path_);
    std::ofstream out(probe_metadata_path_, std::ios::out | std::ios::trunc);
    out << metadata.dump(2) << '\n';
  }

  std::string source_flag_names(const uint32_t flags) const {
    std::ostringstream oss;
    bool first = true;
    for (const auto flag : kFlags_) {
      if (!flag_set(flags, flag)) {
        continue;
      }
      if (!first) {
        oss << "|";
      }
      first = false;
      oss << flag_name(flag);
    }
    return oss.str();
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
        latest_odom_p_.allFinite() && std::isfinite(latest_odom_q_.w()) &&
        std::isfinite(latest_odom_q_.x()) && std::isfinite(latest_odom_q_.y()) &&
        std::isfinite(latest_odom_q_.z());
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
    current.valid = is_finite(current.hpl) && is_finite(current.vpl);
    return current;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2& msg) {
    latest_map_stamp_ = stamp_to_sec(msg.header.stamp);
    auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
    auto normals = std::make_shared<std::vector<Eigen::Vector3d>>();
    const bool cloud_has_normals = has_point_field(msg, "normal_x") &&
                                   has_point_field(msg, "normal_y") &&
                                   has_point_field(msg, "normal_z");
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
      if (cloud_has_normals) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_nx(msg, "normal_x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_ny(msg, "normal_y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_nz(msg, "normal_z");
        for (; iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z, ++iter_nx, ++iter_ny, ++iter_nz) {
          const Eigen::Vector3d p(*iter_x, *iter_y, *iter_z);
          const Eigen::Vector3d n(*iter_nx, *iter_ny, *iter_nz);
          if (p.allFinite()) {
            points->push_back(p);
            normals->push_back(n);
          }
        }
      } else {
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          const Eigen::Vector3d p(*iter_x, *iter_y, *iter_z);
          if (p.allFinite()) {
            points->push_back(p);
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "failed to parse predictor map cloud: %s", e.what());
      return;
    }

    auto predictor_points = points;
    std::shared_ptr<std::vector<Eigen::Vector3d>> predictor_normals =
        cloud_has_normals && normals->size() == points->size() ? normals : nullptr;
    if (lidar_map_max_points_ > 0 &&
        static_cast<int>(points->size()) > lidar_map_max_points_) {
      predictor_points = std::make_shared<std::vector<Eigen::Vector3d>>();
      predictor_points->reserve(lidar_map_max_points_);
      if (predictor_normals) {
        predictor_normals = std::make_shared<std::vector<Eigen::Vector3d>>();
        predictor_normals->reserve(lidar_map_max_points_);
      }
      const double stride = static_cast<double>(points->size()) /
                            static_cast<double>(lidar_map_max_points_);
      for (int i = 0; i < lidar_map_max_points_; ++i) {
        const std::size_t idx = std::min<std::size_t>(
            points->size() - 1, static_cast<std::size_t>(std::floor(i * stride)));
        predictor_points->push_back((*points)[idx]);
        if (predictor_normals) {
          predictor_normals->push_back((*normals)[idx]);
        }
      }
    }

    latest_cloud_points_ = *predictor_points;
    latest_cloud_normals_.clear();
    latest_cloud_has_normals_ =
        predictor_normals && predictor_normals->size() == predictor_points->size();
    if (latest_cloud_has_normals_) {
      latest_cloud_normals_ = *predictor_normals;
    }
    occupancy_.reset();
    occupancy_.insert_points(latest_cloud_points_);
    module_.set_lidar_map_points(predictor_points);
    lidar_.set_lidar_map_points(predictor_points);

    if (use_lidar_primitives_) {
      lidar_primitives_ = iap::make_lidar_fim_primitives(
          *predictor_points, predictor_normals.get(), lidar_primitive_params_,
          &latest_lidar_pca_diagnostics_);
    } else {
      lidar_primitives_.reset();
      latest_lidar_pca_diagnostics_ = iap::LidarFimPrimitiveGenerationDiagnostics{};
      latest_lidar_pca_diagnostics_.fallback_reason = "disabled_by_test";
    }
    module_.set_lidar_fim_primitives(lidar_primitives_);
    lidar_.set_lidar_fim_primitives(lidar_primitives_);
    write_map_snapshot_debug();
    write_lidar_primitives_debug();
  }

  static bool has_point_field(const sensor_msgs::msg::PointCloud2& msg,
                              const std::string& name) {
    return std::any_of(msg.fields.begin(), msg.fields.end(),
                       [&](const auto& field) { return field.name == name; });
  }

  void write_lidar_primitives_debug() {
    if (!enable_debug_log_ || !lidar_primitives_debug_csv_.is_open()) {
      return;
    }
    ++cloud_update_index_;
    if (!lidar_primitives_) {
      return;
    }
    const double stamp =
        std::isfinite(latest_odom_stamp_) ? latest_odom_stamp_ : latest_map_stamp_;
    const double map_age =
        std::isfinite(stamp) && std::isfinite(latest_map_stamp_)
            ? stamp - latest_map_stamp_
            : std::numeric_limits<double>::quiet_NaN();
    const int limit =
        debug_max_lidar_primitives_ > 0
            ? std::min<int>(debug_max_lidar_primitives_,
                            static_cast<int>(lidar_primitives_->size()))
            : static_cast<int>(lidar_primitives_->size());
    for (int i = 0; i < limit; ++i) {
      const auto& primitive = (*lidar_primitives_)[static_cast<std::size_t>(i)];
      lidar_primitives_debug_csv_
          << fmt_num(stamp) << ','
          << fmt_num(latest_map_stamp_) << ','
          << fmt_num(map_age) << ','
          << cloud_update_index_ << ','
          << i << ','
          << fmt_num(primitive.center_w.x()) << ','
          << fmt_num(primitive.center_w.y()) << ','
          << fmt_num(primitive.center_w.z()) << ','
          << fmt_num(primitive.normal_w.x()) << ','
          << fmt_num(primitive.normal_w.y()) << ','
          << fmt_num(primitive.normal_w.z()) << ','
          << fmt_num(primitive.weight) << ','
          << fmt_num(primitive.normal_confidence) << ','
          << primitive.support_count << '\n';
    }
    lidar_primitives_debug_csv_.flush();
  }

  void write_map_snapshot_debug() {
    if (!enable_debug_log_ || !enable_map_snapshot_ || map_snapshot_written_ ||
        !map_snapshot_csv_.is_open()) {
      return;
    }
    for (std::size_t i = 0; i < latest_cloud_points_.size(); ++i) {
      const Eigen::Vector3d normal =
          latest_cloud_has_normals_ && i < latest_cloud_normals_.size()
              ? latest_cloud_normals_[i]
              : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
      map_snapshot_csv_
          << i << ','
          << fmt_num(latest_cloud_points_[i].x()) << ','
          << fmt_num(latest_cloud_points_[i].y()) << ','
          << fmt_num(latest_cloud_points_[i].z()) << ','
          << bool_str(latest_cloud_has_normals_) << ','
          << fmt_num(normal.x()) << ','
          << fmt_num(normal.y()) << ','
          << fmt_num(normal.z()) << ','
          << fmt_num(latest_map_stamp_) << '\n';
    }
    map_snapshot_csv_.flush();
    map_snapshot_written_ = true;
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
      if (elevation < params_.gnss.visibility_params.min_elevation) {
        continue;
      }

      double dop_meas = 0.0;
      if (static_cast<int>(obs->dopp.size()) > l1_idx && freq > 0.0 &&
          std::isfinite(obs->dopp[l1_idx])) {
        dop_meas = -obs->dopp[l1_idx] * (kLightSpeed / freq);
      }
      const double raw_pr_sigma =
          static_cast<int>(obs->psr_std.size()) > l1_idx &&
                  obs->psr_std[l1_idx] > 0.05
              ? obs->psr_std[l1_idx]
              : 5.0;
      const double pr_sigma = raw_pr_sigma * gnss_sigma_scale_;
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
    }
  }

  Eigen::Matrix3d current_prior_information(
      const iap::CurrentIntegrityState& current) const {
    Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
    if (!enable_current_prior_ || !current.valid) {
      return lambda;
    }
    const double k_h = std::isfinite(params_.fusion.K_H_adv) &&
                               params_.fusion.K_H_adv > 0.0
                           ? params_.fusion.K_H_adv
                           : 5.0;
    const double k_v = std::isfinite(params_.fusion.K_V_adv) &&
                               params_.fusion.K_V_adv > 0.0
                           ? params_.fusion.K_V_adv
                           : 5.0;
    const double sigma_h =
        std::isfinite(current.hpl) && current.hpl > 0.0 ? current.hpl / k_h : 0.0;
    const double sigma_v =
        std::isfinite(current.vpl) && current.vpl > 0.0 ? current.vpl / k_v : 0.0;
    if (sigma_h > 0.0 && sigma_v > 0.0) {
      lambda(0, 0) = 1.0 / (sigma_h * sigma_h);
      lambda(1, 1) = 1.0 / (sigma_h * sigma_h);
      lambda(2, 2) = 1.0 / (sigma_v * sigma_v);
    }
    return lambda;
  }

  std::vector<CurrentVariantSpec> current_variants(
      const iap::CurrentIntegrityState& observed) const {
    if (current_variant_set_ != "e8_current_advisory_separation") {
      return {{"observed", false, observed}};
    }

    CurrentVariantSpec normal{"normal", false, observed};

    CurrentVariantSpec current_high{"current_high", true, observed};
    current_high.current.hpl = current_high_hpl_m_;
    current_high.current.vpl = current_high_vpl_m_;
    current_high.current.pl_e = current_high_hpl_m_;
    current_high.current.pl_n = current_high_hpl_m_;
    current_high.current.pl_u = current_high_vpl_m_;
    current_high.current.pl =
        iap::current_pl_scalar(current_high.current.hpl, current_high.current.vpl);
    current_high.current.integrity_state = 0;
    current_high.current.valid =
        is_finite(current_high.current.hpl) && is_finite(current_high.current.vpl);

    CurrentVariantSpec current_unsafe{"current_unsafe", true, observed};
    current_unsafe.current.hpl = current_unsafe_hpl_m_;
    current_unsafe.current.vpl = current_unsafe_vpl_m_;
    current_unsafe.current.pl_e = current_unsafe_hpl_m_;
    current_unsafe.current.pl_n = current_unsafe_hpl_m_;
    current_unsafe.current.pl_u = current_unsafe_vpl_m_;
    current_unsafe.current.pl = iap::current_pl_scalar(
        current_unsafe.current.hpl, current_unsafe.current.vpl);
    current_unsafe.current.integrity_state = current_unsafe_state_;
    current_unsafe.current.valid = false;

    return {normal, current_high, current_unsafe};
  }

  std::vector<StaleVariantSpec> stale_variants() const {
    if (stale_variant_set_ != "e11_stale_snapshot_guard") {
      return {{"normal", ""}};
    }
    return {
        {"normal", ""},
        {"stale_odom", "odom"},
        {"stale_integrity", "integrity"},
        {"stale_gnss", "gnss"},
        {"stale_snapshot", "snapshot"},
    };
  }

  std::string freshness_guard_reason(
      const iap::PredictorQueryResult& module_result) const {
    const std::string& reason = module_result.fallback_reason;
    return reason.rfind("stale_", 0) == 0 ? reason : "";
  }

  SelectedOutput selected_from_module_fallback(
      const iap::PredictorQueryResult& module_result) const {
    SelectedOutput out;
    out.source = "NONE";
    out.available = module_result.available;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = module_result.fallback_reason;
    return out;
  }

  std::vector<QuerySpec> query_specs() const {
    if (query_set_ == "e12_latency_stress") {
      std::vector<QuerySpec> specs;
      specs.push_back({"p000", Eigen::Vector3d(0.0, 0.0, 0.0)});
      std::vector<Eigen::Vector3d> offsets;
      for (int x = -5; x <= 5; ++x) {
        for (int y = -5; y <= 5; ++y) {
          if (x == 0 && y == 0) {
            continue;
          }
          offsets.emplace_back(static_cast<double>(x), static_cast<double>(y), 0.0);
        }
      }
      std::sort(offsets.begin(), offsets.end(),
                [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                  const double na = a.squaredNorm();
                  const double nb = b.squaredNorm();
                  if (na != nb) {
                    return na < nb;
                  }
                  if (a.x() != b.x()) {
                    return a.x() < b.x();
                  }
                  return a.y() < b.y();
                });
      for (const auto& offset : offsets) {
        if (specs.size() >= 100) {
          break;
        }
        std::ostringstream label;
        label << 'p' << std::setw(3) << std::setfill('0') << specs.size();
        specs.push_back({label.str(), offset});
      }
      return specs;
    }
    if (query_set_ == "e1_fixed_neighborhood") {
      return {
          {"p0", Eigen::Vector3d(0.0, 0.0, 0.0)},
          {"p_forward", Eigen::Vector3d(5.0, 0.0, 0.0)},
          {"p_back", Eigen::Vector3d(-5.0, 0.0, 0.0)},
          {"p_left", Eigen::Vector3d(0.0, 5.0, 0.0)},
          {"p_right", Eigen::Vector3d(0.0, -5.0, 0.0)},
          {"p_up", Eigen::Vector3d(0.0, 0.0, 2.0)},
      };
    }
    return {{"p0", Eigen::Vector3d(0.0, 0.0, 0.0)}};
  }

  std::vector<int> batch_sizes_for_specs(const std::size_t query_spec_count) const {
    if (query_set_ == "e12_latency_stress") {
      return latency_stress_batch_sizes_.empty()
                 ? std::vector<int>{1, 10, 50, 100}
                 : latency_stress_batch_sizes_;
    }
    return {static_cast<int>(query_spec_count)};
  }

  void on_integrity(const iap::msg::IntegrityReport& msg) {
    const double stamp = stamp_to_sec(msg.header.stamp);
    if (query_min_period_s_ > 0.0 && std::isfinite(last_query_stamp_) &&
        stamp - last_query_stamp_ < query_min_period_s_) {
      return;
    }
    last_query_stamp_ = stamp;

    if (!latest_odom_pose_valid_) {
      ++skipped_no_pose_;
      write_summary();
      return;
    }
    if (stale_variant_set_ == "e11_stale_snapshot_guard" &&
        active_gnss_epoch(stamp) == nullptr) {
      write_summary();
      return;
    }

    const iap::CurrentIntegrityState observed_current = current_from_msg(msg);
    const std::vector<QuerySpec> specs = query_specs();
    const std::vector<int> batch_sizes = batch_sizes_for_specs(specs.size());
    for (const auto& variant : current_variants(observed_current)) {
      for (const auto& stale_variant : stale_variants()) {
        iap::CurrentIntegrityState current = variant.current;
        if (stale_variant.stale_source == "integrity") {
          current.stamp = stamp - stale_age_s_;
        }
        Eigen::Matrix3d lambda_prior = current_prior_information(current);

        std::optional<iap::GnssEpoch> gnss_epoch_copy;
        const iap::GnssEpoch* epoch_ptr = active_gnss_epoch(stamp);
        if (epoch_ptr != nullptr) {
          gnss_epoch_copy = *epoch_ptr;
          if (stale_variant.stale_source == "gnss") {
            gnss_epoch_copy->stamp = stamp - stale_age_s_;
          }
        }

        iap::IntegritySnapshotBuilderInput builder_input;
        builder_input.stamp =
            stale_variant.stale_source == "snapshot" ? stamp - stale_age_s_ : stamp;
        const double pose_stamp =
            stale_variant_set_ == "e11_stale_snapshot_guard" ? stamp
                                                              : latest_odom_stamp_;
        builder_input.pose_stamp =
            stale_variant.stale_source == "odom" ? stamp - stale_age_s_
                                                  : pose_stamp;
        builder_input.has_pose = latest_odom_pose_valid_;
        builder_input.p_wb = latest_odom_p_;
        builder_input.q_wb = latest_odom_q_;
        builder_input.current = current;
        builder_input.gnss_epoch = gnss_epoch_copy ? &*gnss_epoch_copy : nullptr;
        if (lambda_prior.trace() > 0.0 && lambda_prior.allFinite()) {
          builder_input.lambda_base_pos = &lambda_prior;
        }
        const iap::IntegritySnapshot snapshot =
            snapshot_builder_.build_from_latest(builder_input);

        for (const int requested_batch_size : batch_sizes) {
          const int batch_size = std::max(
              0, std::min<int>(requested_batch_size, static_cast<int>(specs.size())));
          BatchStats batch_stats;
          batch_stats.queries_attempted = batch_size;
          const auto batch_t0 = std::chrono::steady_clock::now();
          for (int batch_index = 0; batch_index < batch_size; ++batch_index) {
            const auto& spec = specs[static_cast<std::size_t>(batch_index)];
            iap::PredictorQueryInput input(latest_odom_p_ + spec.offset, snapshot,
                                           stamp, 0.0, "map");

            const auto gnss_t0 = std::chrono::steady_clock::now();
            const iap::GnssAdvisoryResult gnss = gnss_.query(input.query_position_map,
                                                             input.snapshot);
            const auto gnss_t1 = std::chrono::steady_clock::now();
            const iap::LidarAdvisoryResult lidar = lidar_.query(input.query_position_map,
                                                                input.snapshot);
            const auto lidar_t1 = std::chrono::steady_clock::now();
            const iap::FusionAdvisoryResult fused =
                fusion_.query(input.snapshot, gnss, lidar);
            const auto fusion_t1 = std::chrono::steady_clock::now();
            const iap::PredictorQueryResult module_result = timed_module_query(input);
            const auto done_t = std::chrono::steady_clock::now();

            const double gnss_us = us_between(gnss_t0, gnss_t1);
            const double lidar_us = us_between(gnss_t1, lidar_t1);
            const double fusion_us = us_between(lidar_t1, fusion_t1);
            const double total_step_us = us_between(gnss_t0, fusion_t1);
            const double module_total_us = us_between(module_query_start_, done_t);

            SelectedOutput selected = select_output(snapshot, gnss, lidar, fused);
            if (!freshness_guard_reason(module_result).empty()) {
              selected = selected_from_module_fallback(module_result);
            }
            record_query(stamp, spec, selected, module_result, gnss, lidar, fused,
                         snapshot, variant.label, variant.override_active,
                         stale_variant.label, stale_variant.stale_source,
                         batch_size, batch_index,
                         gnss_us, lidar_us, fusion_us, total_step_us,
                         module_total_us);
            ++batch_stats.queries_recorded;
            batch_stats.selected_valid_count += selected.valid ? 1 : 0;
            batch_stats.selected_fallback_count += selected.fallback ? 1 : 0;
            batch_stats.gnss_valid_count += gnss.valid ? 1 : 0;
            batch_stats.lidar_valid_count += lidar.valid ? 1 : 0;
            batch_stats.fused_valid_count += fused.valid ? 1 : 0;
            batch_stats.fusion_source_count += selected.source == "FUSION" ? 1 : 0;
          }
          const double batch_total_us =
              us_between(batch_t0, std::chrono::steady_clock::now());
          write_latency_stress_tick_debug(stamp, requested_batch_size,
                                          batch_stats, batch_total_us);
        }
      }
    }
    write_summary();
  }

  iap::PredictorQueryResult timed_module_query(const iap::PredictorQueryInput& input) {
    module_query_start_ = std::chrono::steady_clock::now();
    return module_.query(input);
  }

  const iap::GnssEpoch* active_gnss_epoch(const double query_stamp) const {
    if (!latest_epoch_) {
      return nullptr;
    }
    const double age_s = query_stamp - latest_epoch_->stamp;
    if (!std::isfinite(age_s)) {
      return nullptr;
    }
    if (gnss_epoch_max_age_s_ >= 0.0 && age_s > gnss_epoch_max_age_s_) {
      return nullptr;
    }
    return &*latest_epoch_;
  }

  SelectedOutput select_output(const iap::IntegritySnapshot& snapshot,
                               const iap::GnssAdvisoryResult& gnss,
                               const iap::LidarAdvisoryResult& lidar,
                               const iap::FusionAdvisoryResult& fused) {
    SelectedOutput out;
    if (output_mode_ == "gnss_only") {
      out.source = gnss.valid ? "GNSS" : "NONE";
      out.available = gnss.available;
      out.valid = gnss.valid;
      out.fallback = gnss.fallback;
      out.fallback_reason = gnss.fallback_reason;
      out.hpl = gnss.hpl;
      out.vpl = gnss.vpl;
      out.pl = gnss.pl_scalar;
      return out;
    }
    if (output_mode_ == "lidar_only") {
      const iap::GnssAdvisoryResult empty_gnss;
      const iap::FusionAdvisoryResult lidar_fused =
          fusion_.query(snapshot, empty_gnss, lidar);
      out.source = lidar_fused.valid && lidar_fused.lidar_used ? "LIDAR" : "NONE";
      out.available = lidar_fused.available;
      out.valid = lidar_fused.valid;
      out.fallback = lidar_fused.fallback;
      out.fallback_reason = lidar_fused.fallback_reason;
      out.hpl = lidar_fused.hpl;
      out.vpl = lidar_fused.vpl;
      out.pl = lidar_fused.pl_scalar;
      return out;
    }
    if (require_gnss_for_selected_output_ && !gnss.valid) {
      out.source = "NONE";
      out.available = fused.available;
      out.valid = false;
      out.fallback = true;
      out.fallback_reason =
          gnss.fallback_reason.empty() ? "gnss:invalid" : "gnss:" + gnss.fallback_reason;
      out.hpl = std::numeric_limits<double>::quiet_NaN();
      out.vpl = std::numeric_limits<double>::quiet_NaN();
      out.pl = std::numeric_limits<double>::quiet_NaN();
      return out;
    }
    out.source = fused.valid ? "FUSION" : "NONE";
    out.available = fused.available;
    out.valid = fused.valid;
    out.fallback = fused.fallback;
    out.fallback_reason = fused.fallback_reason;
    out.hpl = fused.hpl;
    out.vpl = fused.vpl;
    out.pl = fused.pl_scalar;
    return out;
  }

  double lambda_sum_error(const iap::FusionAdvisoryResult& fused) const {
    const Eigen::Matrix3d expected =
        fused.lambda_prior + fused.lambda_gnss + fused.lambda_lidar;
    return (fused.lambda_pred - expected).cwiseAbs().maxCoeff();
  }

  void record_query(const double stamp,
                    const QuerySpec& query_spec,
                    const SelectedOutput& selected,
                    const iap::PredictorQueryResult& module_result,
                    const iap::GnssAdvisoryResult& gnss,
                    const iap::LidarAdvisoryResult& lidar,
                    const iap::FusionAdvisoryResult& fused,
                    const iap::IntegritySnapshot& snapshot,
                    const std::string& current_variant_label,
                    const bool current_override_active,
                    const std::string& stale_variant_label,
                    const std::string& stale_source,
                    const int query_batch_size,
                    const int query_batch_index,
                    const double gnss_us,
                    const double lidar_us,
                    const double fusion_us,
                    const double total_step_us,
                    const double module_total_us) {
    ++query_count_;
    if (selected.valid) {
      ++selected_valid_count_;
    }
    if (selected.fallback) {
      ++selected_fallback_count_;
    }
    if (module_result.valid) {
      ++module_valid_count_;
    }
    fallback_reason_counts_[module_result.fallback_reason]++;
    selected_reason_counts_[selected.fallback_reason]++;
    source_counts_[selected.source]++;
    for (const auto flag : kFlags_) {
      if (flag_set(module_result.source_flags, flag)) {
        source_flag_counts_[flag_name(flag)]++;
      }
    }
    gnss_us_.push_back(gnss_us);
    lidar_us_.push_back(lidar_us);
    fusion_us_.push_back(fusion_us);
    total_us_.push_back(total_step_us);
    module_total_us_.push_back(module_total_us);

    const double gnss_stamp =
        snapshot.has_epoch ? snapshot.gnss_epoch.stamp
                           : std::numeric_limits<double>::quiet_NaN();
    const double odom_stamp =
        std::isfinite(snapshot.pose_stamp) ? snapshot.pose_stamp : latest_odom_stamp_;
    const double odom_age = stamp - odom_stamp;
    const double integrity_age = stamp - snapshot.current.stamp;
    const double gnss_age = stamp - gnss_stamp;
    const double snapshot_age = stamp - snapshot.stamp;
    const std::string freshness_reason = freshness_guard_reason(module_result);

    csv_file_ << fmt_num(stamp) << ','
              << query_count_ << ','
              << query_spec.label << ','
              << fmt_num(query_spec.offset.x()) << ','
              << fmt_num(query_spec.offset.y()) << ','
              << fmt_num(query_spec.offset.z()) << ','
              << output_mode_ << ','
              << fmt_num(odom_stamp) << ','
              << fmt_num(snapshot.current.stamp) << ','
              << fmt_num(gnss_stamp) << ','
              << fmt_num(snapshot.stamp) << ','
              << fmt_num(odom_age) << ','
              << fmt_num(integrity_age) << ','
              << fmt_num(gnss_age) << ','
              << fmt_num(snapshot_age) << ','
              << fmt_num(module_result.query_position_map.x()) << ','
              << fmt_num(module_result.query_position_map.y()) << ','
              << fmt_num(module_result.query_position_map.z()) << ','
              << current_variant_label << ','
              << bool_str(current_override_active) << ','
              << bool_str(snapshot.current.valid) << ','
              << stale_variant_label << ','
              << stale_source << ','
              << csv_escape(freshness_reason) << ','
              << selected.source << ','
              << bool_str(selected.available) << ','
              << bool_str(selected.valid) << ','
              << bool_str(selected.fallback) << ','
              << fmt_num(selected.hpl) << ','
              << fmt_num(selected.vpl) << ','
              << fmt_num(selected.pl) << ','
              << csv_escape(selected.fallback_reason) << ','
              << bool_str(module_result.available) << ','
              << bool_str(module_result.valid) << ','
              << bool_str(module_result.fallback) << ','
              << csv_escape(module_result.fallback_reason) << ','
              << module_result.source_flags << ','
              << bool_str(snapshot.has_pose) << ','
              << bool_str(snapshot.has_epoch) << ','
              << bool_str(snapshot.has_lambda_base) << ','
              << fmt_num(snapshot.current.hpl) << ','
              << fmt_num(snapshot.current.vpl) << ','
              << fmt_num(snapshot.current.pl_e) << ','
              << fmt_num(snapshot.current.pl_n) << ','
              << fmt_num(snapshot.current.pl_u) << ','
              << fmt_num(snapshot.current.hal) << ','
              << fmt_num(snapshot.current.val) << ','
              << fmt_num(snapshot.current.im) << ','
              << snapshot.current.integrity_state << ','
              << snapshot.current.n_sv_used << ','
              << fmt_num(snapshot.current.pdop) << ','
              << snapshot.current.n_hypotheses << ','
              << snapshot.current.n_detected << ','
              << csv_escape(join_ints(snapshot.current.excluded_prns)) << ','
              << bool_str(gnss.available) << ','
              << bool_str(gnss.valid) << ','
              << bool_str(gnss.fallback) << ','
              << csv_escape(gnss.fallback_reason) << ','
              << fmt_num(gnss.hpl) << ','
              << fmt_num(gnss.vpl) << ','
              << fmt_num(gnss.pl_scalar) << ','
              << fmt_num(gnss.pl_e) << ','
              << fmt_num(gnss.pl_n) << ','
              << fmt_num(gnss.pl_u) << ','
              << fmt_num(gnss.pdop) << ','
              << fmt_num(gnss.hdop) << ','
              << fmt_num(gnss.vdop) << ','
              << fmt_num(gnss.sigma_h) << ','
              << fmt_num(gnss.sigma_v) << ','
              << gnss.n_visible << ','
              << gnss.n_used << ','
              << gnss.n_excluded << ','
              << csv_escape(join_ints(gnss.visible_sat_ids)) << ','
              << csv_escape(join_ints(gnss.used_sat_ids)) << ','
              << csv_escape(join_ints(gnss.excluded_sat_ids)) << ','
              << fmt_num(gnss.effective_sigma_mean) << ','
              << fmt_num(gnss.effective_sigma_max) << ','
              << bool_str(gnss.fim_valid) << ','
              << fmt_num(gnss.lambda_trace) << ','
              << fmt_num(gnss.lambda_min_eig) << ','
              << fmt_num(gnss.lambda_condition) << ','
              << bool_str(lidar.available) << ','
              << bool_str(lidar.valid) << ','
              << bool_str(lidar.fallback) << ','
              << csv_escape(lidar.fallback_reason) << ','
              << bool_str(lidar.fim_valid) << ','
              << lidar.n_primitives << ','
              << lidar.n_valid_normals << ','
              << fmt_num(lidar.lambda_trace) << ','
              << fmt_num(lidar.lambda_min_eig) << ','
              << fmt_num(lidar.lambda_condition) << ','
              << fmt_num(lidar.lidar_alpha) << ','
              << fmt_num(lidar.tdop_proxy) << ','
              << bool_str(fused.available) << ','
              << bool_str(fused.valid) << ','
              << bool_str(fused.fallback) << ','
              << csv_escape(fused.fallback_reason) << ','
              << fmt_num(fused.hpl) << ','
              << fmt_num(fused.vpl) << ','
              << fmt_num(fused.pl_scalar) << ','
              << bool_str(fused.prior_valid) << ','
              << bool_str(fused.gnss_used) << ','
              << bool_str(fused.lidar_used) << ','
              << fmt_num(fused.lambda_prior_trace) << ','
              << fmt_num(fused.lambda_gnss_trace) << ','
              << fmt_num(fused.lambda_lidar_trace) << ','
              << fmt_num(fused.lambda_pred_trace) << ','
              << fmt_num(fused.lambda_pred_min_eig) << ','
              << fmt_num(fused.lambda_pred_condition) << ','
              << fmt_num(lambda_sum_error(fused)) << ','
              << fmt_num(gnss_us) << ','
              << fmt_num(lidar_us) << ','
              << fmt_num(fusion_us) << ','
              << fmt_num(total_step_us) << ','
              << fmt_num(module_total_us) << ','
              << query_batch_size << ','
              << query_batch_index << ','
              << latest_cloud_points_.size() << ','
              << latest_lidar_pca_diagnostics_.lidar_pca_primitives_total << ','
              << latest_lidar_pca_diagnostics_.lidar_pca_valid_normals
              << '\n';
    csv_file_.flush();
    write_query_debug(stamp, query_spec, selected, module_result, gnss, lidar, fused,
                      snapshot);
    write_gnss_debug(stamp, query_spec, gnss, snapshot);
    write_gnss_visibility_debug(stamp, query_spec, gnss, module_result, snapshot);
    write_lidar_debug(stamp, lidar);
    write_fusion_debug(stamp, fused);
    write_fallback_reason_debug(stamp, query_spec, selected, module_result, gnss,
                                lidar, fused);
    write_stale_debug(stamp, query_spec, current_variant_label, stale_variant_label,
                      stale_source, selected, module_result, odom_age,
                      integrity_age, gnss_age, snapshot_age);
    write_timing_debug(stamp, query_spec, selected, module_result, gnss_us, lidar_us,
                       fusion_us, total_step_us, module_total_us,
                       query_batch_size, query_batch_index);
  }

  void write_query_debug(const double stamp,
                         const QuerySpec& query_spec,
                         const SelectedOutput& selected,
                         const iap::PredictorQueryResult& module_result,
                         const iap::GnssAdvisoryResult& gnss,
                         const iap::LidarAdvisoryResult& lidar,
                         const iap::FusionAdvisoryResult& fused,
                         const iap::IntegritySnapshot& snapshot) {
    if (!enable_debug_log_ || !query_debug_csv_.is_open()) {
      return;
    }
    query_debug_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << query_spec.label << ','
        << fmt_num(query_spec.offset.x()) << ','
        << fmt_num(query_spec.offset.y()) << ','
        << fmt_num(query_spec.offset.z()) << ','
        << fmt_num(module_result.query_position_map.x()) << ','
        << fmt_num(module_result.query_position_map.y()) << ','
        << fmt_num(module_result.query_position_map.z()) << ','
        << fmt_num(module_result.query_time_s) << ','
        << fmt_num(module_result.horizon_s) << ','
        << bool_str(snapshot.has_pose) << ','
        << bool_str(snapshot.has_epoch) << ','
        << bool_str(snapshot.has_lambda_base) << ','
        << module_result.source_flags << ','
        << csv_escape(source_flag_names(module_result.source_flags)) << ','
        << selected.source << ','
        << bool_str(selected.valid) << ','
        << fmt_num(selected.hpl) << ','
        << fmt_num(selected.vpl) << ','
        << fmt_num(selected.pl) << ','
        << csv_escape(selected.fallback_reason) << ','
        << bool_str(module_result.valid) << ','
        << bool_str(module_result.available) << ','
        << csv_escape(module_result.fallback_reason) << ','
        << bool_str(gnss.valid) << ','
        << csv_escape(gnss.fallback_reason) << ','
        << bool_str(lidar.valid) << ','
        << csv_escape(lidar.fallback_reason) << ','
        << bool_str(fused.valid) << ','
        << csv_escape(fused.fallback_reason) << '\n';
    query_debug_csv_.flush();
  }

  void write_gnss_debug(const double stamp,
                        const QuerySpec& query_spec,
                        const iap::GnssAdvisoryResult& gnss,
                        const iap::IntegritySnapshot& snapshot) {
    if (!enable_debug_log_ || !gnss_debug_csv_.is_open()) {
      return;
    }
    const int epoch_sat_count =
        snapshot.has_epoch ? static_cast<int>(snapshot.gnss_epoch.sats.size()) : 0;
    if (!snapshot.has_epoch) {
      gnss_debug_csv_
          << fmt_num(stamp) << ','
          << query_count_
          << ',' << query_spec.label
          << ",0,-1,-1,,nan,nan,nan,0,"
          << gnss.n_visible << ','
          << gnss.n_used << ','
          << bool_str(gnss.valid) << ','
          << bool_str(gnss.fim_valid) << ','
          << csv_escape(gnss.fallback_reason) << '\n';
      gnss_debug_csv_.flush();
      return;
    }
    for (std::size_t i = 0; i < snapshot.gnss_epoch.sats.size(); ++i) {
      const auto& sat = snapshot.gnss_epoch.sats[i];
      gnss_debug_csv_
          << fmt_num(stamp) << ','
          << query_count_ << ','
          << query_spec.label << ','
          << epoch_sat_count << ','
          << i << ','
          << sat.sat_id << ','
          << sat.constellation << ','
          << fmt_num(sat.elevation) << ','
          << fmt_num(sat.azimuth) << ','
          << fmt_num(sat.pr_sigma) << ','
          << bool_str(sat.excluded) << ','
          << gnss.n_visible << ','
          << gnss.n_used << ','
          << bool_str(gnss.valid) << ','
          << bool_str(gnss.fim_valid) << ','
          << csv_escape(gnss.fallback_reason) << '\n';
    }
    gnss_debug_csv_.flush();
  }

  void write_gnss_visibility_debug(const double stamp,
                                   const QuerySpec& query_spec,
                                   const iap::GnssAdvisoryResult& gnss,
                                   const iap::PredictorQueryResult& module_result,
                                   const iap::IntegritySnapshot& snapshot) {
    if (!enable_debug_log_ || !gnss_visibility_csv_.is_open()) {
      return;
    }
    const int epoch_sat_count =
        snapshot.has_epoch ? static_cast<int>(snapshot.gnss_epoch.sats.size()) : 0;
    gnss_visibility_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << query_spec.label << ','
        << fmt_num(module_result.query_position_map.x()) << ','
        << fmt_num(module_result.query_position_map.y()) << ','
        << fmt_num(module_result.query_position_map.z()) << ','
        << epoch_sat_count << ','
        << gnss.n_visible << ','
        << gnss.n_used << ','
        << gnss.n_excluded << ','
        << fmt_num(gnss.pdop) << ','
        << fmt_num(gnss.hdop) << ','
        << fmt_num(gnss.vdop) << ','
        << fmt_num(gnss.sigma_h) << ','
        << fmt_num(gnss.sigma_v) << ','
        << fmt_num(gnss.effective_sigma_mean) << ','
        << fmt_num(gnss.effective_sigma_max) << ','
        << csv_escape(join_ints(gnss.visible_sat_ids)) << ','
        << csv_escape(join_ints(gnss.used_sat_ids)) << ','
        << csv_escape(join_ints(gnss.excluded_sat_ids)) << ','
        << bool_str(gnss.valid) << ','
        << csv_escape(gnss.fallback_reason) << '\n';
    gnss_visibility_csv_.flush();
  }

  void write_lidar_debug(const double stamp,
                         const iap::LidarAdvisoryResult& lidar) {
    if (!enable_debug_log_ || !lidar_debug_csv_.is_open()) {
      return;
    }
    lidar_debug_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << latest_cloud_points_.size() << ','
        << latest_lidar_pca_diagnostics_.lidar_pca_primitives_total << ','
        << latest_lidar_pca_diagnostics_.lidar_pca_valid_normals << ','
        << latest_lidar_pca_diagnostics_.lidar_pca_invalid_normals << ','
        << fmt_num(latest_lidar_pca_diagnostics_.lidar_pca_support_mean) << ','
        << latest_lidar_pca_diagnostics_.lidar_pca_support_min << ','
        << fmt_num(latest_lidar_pca_diagnostics_.lidar_pca_radius_m) << ','
        << csv_escape(latest_lidar_pca_diagnostics_.fallback_reason) << ','
        << bool_str(lidar.valid) << ','
        << bool_str(lidar.fim_valid) << ','
        << bool_str(lidar.fim_regularized) << ','
        << lidar.n_primitives << ','
        << lidar.n_valid_normals << ','
        << fmt_num(lidar.lambda_trace) << ','
        << fmt_num(lidar.lambda_min_eig) << ','
        << fmt_num(lidar.lambda_condition) << ','
        << fmt_num(lidar.lidar_alpha) << ','
        << fmt_num(lidar.tdop_proxy) << ','
        << csv_escape(lidar.fallback_reason) << '\n';
    lidar_debug_csv_.flush();
  }

  void write_fusion_debug(const double stamp,
                          const iap::FusionAdvisoryResult& fused) {
    if (!enable_debug_log_ || !fusion_debug_csv_.is_open()) {
      return;
    }
    fusion_debug_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << bool_str(fused.valid) << ','
        << bool_str(fused.available) << ','
        << bool_str(fused.fallback) << ','
        << bool_str(fused.prior_valid) << ','
        << bool_str(fused.gnss_used) << ','
        << bool_str(fused.lidar_used) << ','
        << bool_str(fused.epsilon_applied) << ','
        << bool_str(fused.degeneracy_regularized) << ','
        << bool_str(fused.conservative_max_applied) << ','
        << matrix_values(fused.lambda_prior) << ','
        << matrix_values(fused.lambda_gnss) << ','
        << matrix_values(fused.lambda_lidar) << ','
        << matrix_values(fused.lambda_pred) << ','
        << matrix_values(fused.sigma_pos) << ','
        << fmt_num(lambda_sum_error(fused)) << ','
        << fmt_num(fused.hpl) << ','
        << fmt_num(fused.vpl) << ','
        << fmt_num(fused.pl_scalar) << ','
        << csv_escape(fused.fallback_reason) << '\n';
    fusion_debug_csv_.flush();
  }

  void write_fallback_reason_debug(const double stamp,
                                   const QuerySpec& query_spec,
                                   const SelectedOutput& selected,
                                   const iap::PredictorQueryResult& module_result,
                                   const iap::GnssAdvisoryResult& gnss,
                                   const iap::LidarAdvisoryResult& lidar,
                                   const iap::FusionAdvisoryResult& fused) {
    if (!enable_debug_log_ || !fallback_reason_csv_.is_open()) {
      return;
    }
    fallback_reason_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << query_spec.label << ','
        << selected.source << ','
        << bool_str(selected.valid) << ','
        << bool_str(selected.fallback) << ','
        << csv_escape(selected.fallback_reason) << ','
        << csv_escape(module_result.fallback_reason) << ','
        << csv_escape(gnss.fallback_reason) << ','
        << csv_escape(lidar.fallback_reason) << ','
        << csv_escape(fused.fallback_reason) << '\n';
    fallback_reason_csv_.flush();
  }

  void write_stale_debug(const double stamp,
                         const QuerySpec& query_spec,
                         const std::string& current_variant_label,
                         const std::string& stale_variant_label,
                         const std::string& stale_source,
                         const SelectedOutput& selected,
                         const iap::PredictorQueryResult& module_result,
                         const double odom_age,
                         const double integrity_age,
                         const double gnss_age,
                         const double snapshot_age) {
    if (!enable_debug_log_ || !stale_debug_csv_.is_open()) {
      return;
    }
    stale_debug_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << query_spec.label << ','
        << current_variant_label << ','
        << stale_variant_label << ','
        << stale_source << ','
        << fmt_num(odom_age) << ','
        << fmt_num(integrity_age) << ','
        << fmt_num(gnss_age) << ','
        << fmt_num(snapshot_age) << ','
        << bool_str(selected.valid) << ','
        << bool_str(selected.fallback) << ','
        << csv_escape(selected.fallback_reason) << ','
        << bool_str(module_result.valid) << ','
        << bool_str(module_result.fallback) << ','
        << csv_escape(module_result.fallback_reason) << ','
        << fmt_num(selected.hpl) << ','
        << fmt_num(selected.vpl) << ','
        << fmt_num(selected.pl) << '\n';
    stale_debug_csv_.flush();
  }

  void write_timing_debug(const double stamp,
                          const QuerySpec& query_spec,
                          const SelectedOutput& selected,
                          const iap::PredictorQueryResult& module_result,
                          const double gnss_us,
                          const double lidar_us,
                          const double fusion_us,
                          const double total_step_us,
                          const double module_total_us,
                          const int query_batch_size,
                          const int query_batch_index) {
    if (!enable_debug_log_ || !timing_csv_.is_open()) {
      return;
    }
    timing_csv_
        << fmt_num(stamp) << ','
        << query_count_ << ','
        << query_spec.label << ','
        << query_batch_size << ','
        << query_batch_index << ','
        << fmt_num(gnss_us) << ','
        << fmt_num(lidar_us) << ','
        << fmt_num(fusion_us) << ','
        << fmt_num(total_step_us) << ','
        << fmt_num(module_total_us) << ','
        << selected.source << ','
        << bool_str(module_result.valid) << ','
        << csv_escape(module_result.fallback_reason) << '\n';
    timing_csv_.flush();
  }

  void write_latency_stress_tick_debug(const double stamp,
                                       const int requested_batch_size,
                                       const BatchStats& stats,
                                       const double batch_total_us) {
    if (!enable_debug_log_ || !latency_stress_tick_csv_.is_open() ||
        query_set_ != "e12_latency_stress") {
      return;
    }
    ++latency_stress_tick_count_;
    latency_stress_tick_csv_
        << fmt_num(stamp) << ','
        << latency_stress_tick_count_ << ','
        << requested_batch_size << ','
        << stats.queries_attempted << ','
        << stats.queries_recorded << ','
        << fmt_num(batch_total_us) << ','
        << stats.selected_valid_count << ','
        << stats.selected_fallback_count << ','
        << stats.gnss_valid_count << ','
        << stats.lidar_valid_count << ','
        << stats.fused_valid_count << ','
        << stats.fusion_source_count << '\n';
    latency_stress_tick_csv_.flush();
  }

  nlohmann::json latency_json(const std::vector<double>& values) const {
    nlohmann::json out;
    out["p50_us"] = percentile(values, 0.50);
    out["p95_us"] = percentile(values, 0.95);
    out["max_us"] = values.empty()
                         ? std::numeric_limits<double>::quiet_NaN()
                         : *std::max_element(values.begin(), values.end());
    return out;
  }

  void write_summary() {
    nlohmann::json summary;
    summary["output_mode"] = output_mode_;
    summary["query_set"] = query_set_;
    summary["csv_path"] = csv_path_;
    summary["query_count"] = query_count_;
    summary["skipped_no_pose"] = skipped_no_pose_;
    summary["selected_valid_count"] = selected_valid_count_;
    summary["selected_fallback_count"] = selected_fallback_count_;
    summary["module_valid_count"] = module_valid_count_;
    summary["valid_ratio"] =
        query_count_ > 0 ? static_cast<double>(selected_valid_count_) /
                               static_cast<double>(query_count_)
                         : 0.0;
    summary["fallback_ratio"] =
        query_count_ > 0 ? static_cast<double>(selected_fallback_count_) /
                               static_cast<double>(query_count_)
                         : 0.0;
    summary["source_counts"] = source_counts_;
    summary["source_flag_counts"] = source_flag_counts_;
    summary["fallback_reason_histogram"] = fallback_reason_counts_;
    summary["selected_reason_histogram"] = selected_reason_counts_;
    summary["latency"]["gnss"] = latency_json(gnss_us_);
    summary["latency"]["lidar"] = latency_json(lidar_us_);
    summary["latency"]["fusion"] = latency_json(fusion_us_);
    summary["latency"]["total_step"] = latency_json(total_us_);
    summary["latency"]["module_total"] = latency_json(module_total_us_);
    summary["latest_cloud_points"] = latest_cloud_points_.size();
    summary["latest_lidar_pca_primitives"] =
        latest_lidar_pca_diagnostics_.lidar_pca_primitives_total;
    summary["latest_lidar_pca_valid_normals"] =
        latest_lidar_pca_diagnostics_.lidar_pca_valid_normals;
    summary["latest_epoch_sat_count"] =
        latest_epoch_ ? static_cast<int>(latest_epoch_->sats.size()) : 0;
    summary["debug_enabled"] = enable_debug_log_;
    summary["probe_metadata_path"] = probe_metadata_path_;
    if (enable_debug_log_) {
      summary["debug_files"] = {
          {"query_debug_csv_path", query_debug_csv_path_},
          {"gnss_debug_csv_path", gnss_debug_csv_path_},
          {"gnss_visibility_csv_path", gnss_visibility_csv_path_},
          {"lidar_debug_csv_path", lidar_debug_csv_path_},
          {"lidar_primitives_debug_csv_path", lidar_primitives_debug_csv_path_},
          {"fusion_debug_csv_path", fusion_debug_csv_path_},
          {"fallback_reason_csv_path", fallback_reason_csv_path_},
          {"stale_debug_csv_path", stale_debug_csv_path_},
          {"map_snapshot_csv_path", map_snapshot_csv_path_},
          {"latency_stress_tick_csv_path", latency_stress_tick_csv_path_},
          {"timing_csv_path", timing_csv_path_},
      };
    }
    std::ofstream out(summary_path_, std::ios::out | std::ios::trunc);
    out << summary.dump(2) << '\n';
  }

  std::string output_mode_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string integrity_topic_;
  std::string range_meas_topic_;
  std::string ephem_topic_;
  std::string glo_ephem_topic_;
  std::string receiver_lla_topic_;
  std::string iono_topic_;
  std::string csv_path_;
  std::string summary_path_;
  std::string query_set_;
  bool enable_debug_log_ = false;
  std::string query_debug_csv_path_;
  std::string gnss_debug_csv_path_;
  std::string gnss_visibility_csv_path_;
  std::string lidar_debug_csv_path_;
  std::string lidar_primitives_debug_csv_path_;
  std::string fusion_debug_csv_path_;
  std::string timing_csv_path_;
  std::string latency_stress_tick_csv_path_;
  std::string probe_metadata_path_;
  std::string fallback_reason_csv_path_;
  std::string stale_debug_csv_path_;
  std::string map_snapshot_csv_path_;
  int debug_max_lidar_primitives_ = 500;
  bool use_lidar_primitives_ = true;
  bool enable_current_prior_ = true;
  bool require_gnss_for_selected_output_ = false;
  std::string current_variant_set_ = "observed";
  double current_high_hpl_m_ = 1000.0;
  double current_high_vpl_m_ = 1000.0;
  double current_unsafe_hpl_m_ = 500.0;
  double current_unsafe_vpl_m_ = 600.0;
  int current_unsafe_state_ = 2;
  std::string stale_variant_set_ = "observed";
  double stale_age_s_ = 2.0;
  bool enable_map_snapshot_ = true;
  int lidar_map_max_points_ = 2500;
  double query_min_period_s_ = 0.0;
  std::vector<int> latency_stress_batch_sizes_;
  double gnss_epoch_max_age_s_ = 0.5;
  double gnss_sigma_scale_ = 1.0;

  iap::PredictorParams params_;
  iap::PredictorModule module_;
  iap::GnssAdvisoryPredictor gnss_;
  iap::LidarAdvisoryPredictor lidar_;
  iap::FusionAdvisoryPredictor fusion_;
  iap::IntegritySnapshotBuilder snapshot_builder_;
  iap::LocalOccupancyGrid occupancy_;
  iap::LidarFimPrimitiveGenerationParams lidar_primitive_params_;
  iap::LidarFimPrimitiveGenerationDiagnostics latest_lidar_pca_diagnostics_;
  std::shared_ptr<std::vector<iap::LidarFimPrimitive>> lidar_primitives_;

  std::ofstream csv_file_;
  std::ofstream query_debug_csv_;
  std::ofstream gnss_debug_csv_;
  std::ofstream gnss_visibility_csv_;
  std::ofstream lidar_debug_csv_;
  std::ofstream lidar_primitives_debug_csv_;
  std::ofstream fusion_debug_csv_;
  std::ofstream timing_csv_;
  std::ofstream latency_stress_tick_csv_;
  std::ofstream fallback_reason_csv_;
  std::ofstream stale_debug_csv_;
  std::ofstream map_snapshot_csv_;
  int query_count_ = 0;
  int latency_stress_tick_count_ = 0;
  int cloud_update_index_ = 0;
  bool map_snapshot_written_ = false;
  int skipped_no_pose_ = 0;
  int selected_valid_count_ = 0;
  int selected_fallback_count_ = 0;
  int module_valid_count_ = 0;
  std::map<std::string, int> source_counts_;
  std::map<std::string, int> source_flag_counts_;
  std::map<std::string, int> fallback_reason_counts_;
  std::map<std::string, int> selected_reason_counts_;
  std::vector<double> gnss_us_;
  std::vector<double> lidar_us_;
  std::vector<double> fusion_us_;
  std::vector<double> total_us_;
  std::vector<double> module_total_us_;
  double last_query_stamp_ = std::numeric_limits<double>::quiet_NaN();
  std::chrono::steady_clock::time_point module_query_start_{};

  double latest_odom_stamp_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d latest_odom_p_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond latest_odom_q_ = Eigen::Quaterniond::Identity();
  bool latest_odom_pose_valid_ = false;
  double latest_map_stamp_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Vector3d> latest_cloud_points_;
  std::vector<Eigen::Vector3d> latest_cloud_normals_;
  bool latest_cloud_has_normals_ = false;

  bool origin_set_ = false;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
  std::vector<double> iono_params_;
  std::optional<iap::GnssEpoch> latest_epoch_;

  const std::vector<iap::PredictorResultFlags> kFlags_ = {
      iap::PREDICTOR_RESULT_VALID,
      iap::PREDICTOR_RESULT_FALLBACK,
      iap::PREDICTOR_RESULT_GNSS_VALID,
      iap::PREDICTOR_RESULT_LIDAR_VALID,
      iap::PREDICTOR_RESULT_FUSION_VALID,
      iap::PREDICTOR_RESULT_PRIOR_VALID,
      iap::PREDICTOR_RESULT_GNSS_USED,
      iap::PREDICTOR_RESULT_LIDAR_USED,
      iap::PREDICTOR_RESULT_REGULARIZED,
      iap::PREDICTOR_RESULT_CONSERVATIVE_MAX,
      iap::PREDICTOR_RESULT_AVAILABLE,
  };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssMeasMsg>::SharedPtr range_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr glo_ephem_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr receiver_lla_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr iono_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPredictorQueryProbe>());
  rclcpp::shutdown();
  return 0;
}
