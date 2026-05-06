#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_ionosphere_parameter.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kGpsSemiMajorAxisM = 26560000.0;
constexpr double kGpsInclinationRad = 55.0 * kDegToRad;
constexpr double kDefaultIonoAlpha0 = 1.0e-8;
constexpr double kDefaultIonoBeta0 = 9.0e4;

std::chrono::nanoseconds period_from_rate(const double rate_hz)
{
  const double safe_rate = std::max(rate_hz, 1.0e-6);
  const auto nanoseconds =
    static_cast<int64_t>(std::llround(1.0e9 / safe_rate));
  return std::chrono::nanoseconds(std::max<int64_t>(nanoseconds, 1));
}

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

gnss_comm::gtime_t stamp_to_utc_time(const builtin_interfaces::msg::Time& stamp)
{
  gnss_comm::gtime_t t;
  t.time = static_cast<time_t>(stamp.sec);
  t.sec = static_cast<double>(stamp.nanosec) * 1.0e-9;
  return t;
}

bool is_zero_stamp(const builtin_interfaces::msg::Time& stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

double normalize_deg(double deg)
{
  while (deg < 0.0) {
    deg += 360.0;
  }
  while (deg >= 360.0) {
    deg -= 360.0;
  }
  return deg;
}

std_msgs::msg::ColorRGBA color_rgba(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

std::string bool_string(const bool v)
{
  return v ? "true" : "false";
}

std::string trim_copy(const std::string& s)
{
  const auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char c) {
    return std::isspace(c);
  });
  const auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) {
    return std::isspace(c);
  }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

std::string upper_copy(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return s;
}

uint32_t constellation_name_to_sys(const std::string& name)
{
  const std::string n = upper_copy(trim_copy(name));
  if (n == "GPS" || n == "G") {
    return SYS_GPS;
  }
  if (n == "BDS" || n == "BEIDOU" || n == "C") {
    return SYS_BDS;
  }
  if (n == "GAL" || n == "GALILEO" || n == "E") {
    return SYS_GAL;
  }
  if (n == "GLO" || n == "GLONASS" || n == "R") {
    return SYS_GLO;
  }
  return SYS_NONE;
}

std::string sys_to_constellation_name(const uint32_t sys)
{
  switch (sys) {
    case SYS_GPS:
      return "GPS";
    case SYS_BDS:
      return "BDS";
    case SYS_GAL:
      return "GAL";
    case SYS_GLO:
      return "GLO";
    default:
      return "UNKNOWN";
  }
}

bool sys_enabled(const std::unordered_set<uint32_t>& enabled_systems, const uint32_t sys)
{
  return enabled_systems.find(sys) != enabled_systems.end();
}

bool has_non_gps_enabled(const std::unordered_set<uint32_t>& enabled_systems)
{
  for (const auto sys : enabled_systems) {
    if (sys != SYS_GPS) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> split_constellation_csv(const std::string& csv)
{
  std::vector<std::string> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim_copy(item);
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

std::string join_enabled_constellations(const std::unordered_set<uint32_t>& enabled_systems)
{
  std::vector<std::string> names;
  for (const auto sys : {SYS_GPS, SYS_BDS, SYS_GAL, SYS_GLO}) {
    if (sys_enabled(enabled_systems, sys)) {
      names.push_back(sys_to_constellation_name(sys));
    }
  }
  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << names[i];
  }
  return oss.str();
}

diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

geometry_msgs::msg::Point point_msg(const Eigen::Vector3d& p)
{
  geometry_msgs::msg::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
  return msg;
}

}  // namespace

struct ReceiverState
{
  builtin_interfaces::msg::Time stamp;
  Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();
};

class TimeSyncBuffer
{
public:
  void set_duration(const double duration_s) { duration_s_ = std::max(duration_s, 0.1); }
  void set_max_gap(const double max_gap_s) { max_gap_s_ = std::max(max_gap_s, 1.0e-4); }

  void push(const ReceiverState& state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    states_.push_back(state);
    latest_ = state;
    has_latest_ = true;

    const double latest_t = stamp_to_sec(state.stamp);
    while (!states_.empty() && latest_t - stamp_to_sec(states_.front().stamp) > duration_s_) {
      states_.pop_front();
    }
  }

  bool latest(ReceiverState& state) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_) {
      return false;
    }
    state = latest_;
    return true;
  }

  bool interpolate(
    const builtin_interfaces::msg::Time& stamp,
    ReceiverState& state,
    std::string& reason) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (states_.empty()) {
      reason = "truth buffer empty";
      return false;
    }

    const double target = stamp_to_sec(stamp);
    const double first = stamp_to_sec(states_.front().stamp);
    const double last = stamp_to_sec(states_.back().stamp);
    if (target < first) {
      reason = "target stamp older than truth buffer";
      return false;
    }
    if (target > last) {
      reason = "target stamp newer than truth buffer";
      return false;
    }

    for (size_t i = 1; i < states_.size(); ++i) {
      const double t0 = stamp_to_sec(states_[i - 1].stamp);
      const double t1 = stamp_to_sec(states_[i].stamp);
      if (target < t0 || target > t1) {
        continue;
      }
      const double gap = t1 - t0;
      if (gap > max_gap_s_) {
        reason = "truth interpolation gap too large";
        return false;
      }
      const double alpha = gap > 1.0e-9 ? (target - t0) / gap : 0.0;
      state.stamp = stamp;
      state.pos_enu = (1.0 - alpha) * states_[i - 1].pos_enu + alpha * states_[i].pos_enu;
      state.vel_enu = (1.0 - alpha) * states_[i - 1].vel_enu + alpha * states_[i].vel_enu;
      return true;
    }

    reason = "target stamp not bracketed by truth states";
    return false;
  }

private:
  mutable std::mutex mutex_;
  std::deque<ReceiverState> states_;
  ReceiverState latest_;
  bool has_latest_ = false;
  double duration_s_ = 5.0;
  double max_gap_s_ = 0.05;
};

struct SkyMaskEntry
{
  double az_deg = 0.0;
  double min_el_deg = 10.0;
};

struct FaultConfig
{
  uint32_t sys = SYS_NONE;  // Optional constellation filter; SYS_NONE keeps legacy PRN-only matching.
  uint32_t prn = 0;
  double start_time_s = 0.0;
  double duration_s = 0.0;
  double pseudorange_bias_m = 0.0;
  double bias_rate_mps = 0.0;
  double doppler_bias_mps = 0.0;
  bool drop = false;
  double cn0_degrade_dbhz = 0.0;
};

struct ScenarioConfig
{
  bool has_anchor = false;
  Eigen::Vector3d anchor_lla = Eigen::Vector3d::Zero();
  bool skymask_enabled = false;
  double skymask_default_min_el_deg = 10.0;
  std::vector<SkyMaskEntry> skymask;
  std::vector<FaultConfig> faults;
};

class ScenarioLoader
{
public:
  static ScenarioConfig load(const std::string& path, rclcpp::Logger logger)
  {
    ScenarioConfig config;
    if (path.empty()) {
      return config;
    }

    try {
      const YAML::Node root = YAML::LoadFile(path);
      if (root["anchor"]) {
        const auto anchor = root["anchor"];
        config.has_anchor = true;
        config.anchor_lla.x() = anchor["lat_deg"].as<double>();
        config.anchor_lla.y() = anchor["lon_deg"].as<double>();
        config.anchor_lla.z() = anchor["alt_m"].as<double>();
      }
      if (root["skymask"]) {
        const auto skymask = root["skymask"];
        config.skymask_enabled = skymask["enabled"] ? skymask["enabled"].as<bool>() : true;
        if (skymask["default_min_elevation_deg"]) {
          config.skymask_default_min_el_deg =
            skymask["default_min_elevation_deg"].as<double>();
        }
        if (skymask["az_el_deg"]) {
          for (const auto& item : skymask["az_el_deg"]) {
            SkyMaskEntry entry;
            entry.az_deg = normalize_deg(item["az_deg"].as<double>());
            entry.min_el_deg = item["min_el_deg"].as<double>();
            config.skymask.push_back(entry);
          }
          std::sort(
            config.skymask.begin(), config.skymask.end(),
            [](const SkyMaskEntry& a, const SkyMaskEntry& b) {
              return a.az_deg < b.az_deg;
            });
        }
      }
      if (root["faults"]) {
        for (const auto& item : root["faults"]) {
          FaultConfig fault;
          if (item["constellation"]) {
            fault.sys = constellation_name_to_sys(item["constellation"].as<std::string>());
            if (fault.sys == SYS_NONE) {
              RCLCPP_WARN(
                logger,
                "Ignoring GNSS fault with unknown constellation '%s'",
                item["constellation"].as<std::string>().c_str());
              continue;
            }
          }
          fault.prn = item["sat"].as<uint32_t>();
          fault.start_time_s = item["start_time_s"].as<double>();
          fault.duration_s = item["duration_s"].as<double>();
          fault.pseudorange_bias_m =
            item["pseudorange_bias_m"] ? item["pseudorange_bias_m"].as<double>() : 0.0;
          fault.bias_rate_mps =
            item["bias_rate_mps"] ? item["bias_rate_mps"].as<double>() : 0.0;
          fault.doppler_bias_mps =
            item["doppler_bias_mps"] ? item["doppler_bias_mps"].as<double>() : 0.0;
          fault.drop = item["drop"] ? item["drop"].as<bool>() : false;
          fault.cn0_degrade_dbhz =
            item["cn0_degrade_dbhz"] ? item["cn0_degrade_dbhz"].as<double>() : 0.0;
          config.faults.push_back(fault);
        }
      }
      RCLCPP_INFO(logger, "Loaded GNSS scenario file: %s", path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger, "Failed to load GNSS scenario file '%s': %s", path.c_str(), e.what());
    }

    return config;
  }
};

class SatelliteConstellation
{
public:
  void configure(const uint32_t prn_min, const uint32_t prn_max, const uint32_t num_sats)
  {
    gps_prn_min_ = std::max<uint32_t>(1, prn_min);
    gps_prn_max_ = std::max(gps_prn_min_, prn_max);
    num_gps_sats_ = std::max<uint32_t>(1, num_sats);
    ephems_.clear();
  }

  void configure_ephemeris_source(
    const std::string& ephemeris_source,
    const std::string& rinex_nav_file,
    const double rinex_ephem_max_age_s,
    const bool rinex_gps_only,
    const bool fallback_to_synthetic_on_rinex_error,
    const std::unordered_set<uint32_t>& enabled_systems)
  {
    ephemeris_source_ = ephemeris_source == "rinex" ? "rinex" : "synthetic";
    rinex_nav_file_ = rinex_nav_file;
    rinex_ephem_max_age_s_ = std::max(rinex_ephem_max_age_s, 1.0);
    rinex_gps_only_ = rinex_gps_only;
    fallback_to_synthetic_on_rinex_error_ = fallback_to_synthetic_on_rinex_error;
    enabled_systems_ = enabled_systems.empty() ? std::unordered_set<uint32_t>{SYS_GPS} : enabled_systems;
    rinex_load_attempted_ = false;
    rinex_loaded_ = false;
    rinex_fallback_active_ = false;
    rinex_ephem_count_ = 0;
    rinex_glo_ephem_count_ = 0;
    rinex_selected_ephem_count_ = 0;
    rinex_selected_glo_ephem_count_ = 0;
    rinex_selected_min_age_s_ = std::numeric_limits<double>::quiet_NaN();
    rinex_selected_max_age_s_ = std::numeric_limits<double>::quiet_NaN();
    rinex_error_message_.clear();
    rinex_ephems_.clear();
    rinex_glo_ephems_.clear();
    ephems_.clear();
    glo_ephems_.clear();
  }

  bool ensure_ephemerides(const gnss_comm::gtime_t& gpst_time, rclcpp::Logger logger)
  {
    if (ephemeris_source_ == "rinex") {
      if (ensure_rinex_loaded(logger)) {
        select_rinex_ephemerides(gpst_time);
        if (!ephems_.empty() || !glo_ephems_.empty()) {
          rinex_fallback_active_ = false;
          return true;
        }
        rinex_error_message_ = "no RINEX ephemerides valid near current GNSS time";
        RCLCPP_WARN(
          logger,
          "RINEX NAV loaded but no healthy enabled ephemeris is within %.1f s of current epoch; "
          "check that demo7 sim_start_utc matches the RINEX NAV date",
          rinex_ephem_max_age_s_);
      }

      if (!fallback_to_synthetic_on_rinex_error_) {
        rinex_fallback_active_ = false;
        ephems_.clear();
        glo_ephems_.clear();
        return false;
      }

      rinex_fallback_active_ = true;
      ensure_synthetic_ephemerides(gpst_time, logger);
      return !ephems_.empty();
    }

    rinex_fallback_active_ = false;
    ensure_synthetic_ephemerides(gpst_time, logger);
    return !ephems_.empty();
  }

  const std::vector<gnss_comm::EphemPtr>& ephems() const { return ephems_; }
  const std::vector<gnss_comm::GloEphemPtr>& glo_ephems() const { return glo_ephems_; }

  const std::string& ephemeris_source() const { return ephemeris_source_; }
  const std::string& rinex_nav_file() const { return rinex_nav_file_; }
  const std::string& rinex_error_message() const { return rinex_error_message_; }
  const std::unordered_set<uint32_t>& enabled_systems() const { return enabled_systems_; }
  std::string enabled_constellations_string() const { return join_enabled_constellations(enabled_systems_); }
  bool rinex_loaded() const { return rinex_loaded_; }
  bool rinex_fallback_active() const { return rinex_fallback_active_; }
  size_t rinex_ephem_count() const { return rinex_ephem_count_; }
  size_t rinex_glo_ephem_count() const { return rinex_glo_ephem_count_; }
  size_t rinex_selected_ephem_count() const { return rinex_selected_ephem_count_; }
  size_t rinex_selected_glo_ephem_count() const { return rinex_selected_glo_ephem_count_; }
  double rinex_selected_min_age_s() const { return rinex_selected_min_age_s_; }
  double rinex_selected_max_age_s() const { return rinex_selected_max_age_s_; }
  bool rinex_time_consistent() const
  {
    if (ephemeris_source_ != "rinex") {
      return true;
    }
    return rinex_loaded_ && !rinex_fallback_active_ &&
      (rinex_selected_ephem_count_ + rinex_selected_glo_ephem_count_ > 0) &&
      std::isfinite(rinex_selected_max_age_s_) &&
      rinex_selected_max_age_s_ <= rinex_ephem_max_age_s_;
  }
  double rinex_ephem_max_age_s() const { return rinex_ephem_max_age_s_; }
  size_t selected_count_for_sys(const uint32_t sys) const
  {
    if (ephemeris_source_ == "synthetic") {
      return sys == SYS_GPS ? ephems_.size() : 0;
    }
    switch (sys) {
      case SYS_GPS:
        return selected_gps_ephems_;
      case SYS_BDS:
        return selected_bds_ephems_;
      case SYS_GAL:
        return selected_gal_ephems_;
      case SYS_GLO:
        return selected_glo_ephems_;
      default:
        return 0;
    }
  }

private:
  void ensure_synthetic_ephemerides(const gnss_comm::gtime_t& gpst_time, rclcpp::Logger logger)
  {
    if (!ephems_.empty()) {
      const double age = std::abs(gnss_comm::time_diff(gpst_time, ephems_.front()->toe));
      if (age < 3600.0) {
        return;
      }
      RCLCPP_INFO(logger, "Refreshing simulated GPS ephemerides after %.1f s", age);
      ephems_.clear();
      glo_ephems_.clear();
    }

    if (!sys_enabled(enabled_systems_, SYS_GPS)) {
      return;
    }

    uint32_t week = 0;
    const double tow = gnss_comm::time2gpst(gpst_time, &week);
    const uint32_t prn_count =
      std::min<uint32_t>(num_gps_sats_, gps_prn_max_ - gps_prn_min_ + 1);

    ephems_.reserve(prn_count);
    for (uint32_t idx = 0; idx < prn_count; ++idx) {
      const uint32_t prn = gps_prn_min_ + idx;
      const uint32_t plane = idx / 4;
      const uint32_t slot = idx % 4;
      auto eph = std::make_shared<gnss_comm::Ephem>();

      eph->sat = gnss_comm::sat_no(SYS_GPS, prn);
      eph->ttr = gpst_time;
      eph->toe = gpst_time;
      eph->toc = gpst_time;
      eph->toe_tow = tow;
      eph->week = week;
      eph->iode = prn;
      eph->iodc = prn;
      eph->health = 0;
      eph->code = CODE_L1C;
      eph->ura = 2.0;
      eph->A = kGpsSemiMajorAxisM;
      eph->e = 0.01;
      eph->i0 = kGpsInclinationRad;
      eph->omg = 0.0;
      eph->OMG0 =
        static_cast<double>(plane) * (60.0 * kDegToRad) + EARTH_OMG_GPS * tow;
      eph->M0 =
        static_cast<double>(slot) * (90.0 * kDegToRad) +
        static_cast<double>(plane) * (15.0 * kDegToRad);
      eph->delta_n = 0.0;
      eph->OMG_dot = -8.0e-9;
      eph->i_dot = 0.0;
      eph->cuc = 0.0;
      eph->cus = 0.0;
      eph->crc = 0.0;
      eph->crs = 0.0;
      eph->cic = 0.0;
      eph->cis = 0.0;
      eph->af0 = 0.0;
      eph->af1 = 0.0;
      eph->af2 = 0.0;
      eph->tgd[0] = 0.0;
      eph->tgd[1] = 0.0;
      eph->A_dot = 0.0;
      eph->n_dot = 0.0;

      ephems_.push_back(eph);
    }
  }

  bool validate_rinex_nav_header(std::string& reason) const
  {
    if (rinex_nav_file_.empty()) {
      reason = "rinex_nav_file is empty";
      return false;
    }

    std::ifstream file(rinex_nav_file_);
    if (!file.is_open()) {
      reason = "failed to open RINEX NAV file";
      return false;
    }

    bool saw_version = false;
    bool saw_304 = false;
    bool saw_nav = false;
    bool saw_leap_seconds = false;
    bool saw_end_header = false;
    std::string line;
    while (std::getline(file, line)) {
      if (line.find("RINEX VERSION / TYPE") != std::string::npos) {
        saw_version = true;
        saw_304 = line.find("3.04") != std::string::npos;
        saw_nav = line.find("NAV") != std::string::npos || line.find("N:") != std::string::npos;
      }
      if (line.find("LEAP SECONDS") != std::string::npos && line.find("BDS") == std::string::npos) {
        saw_leap_seconds = true;
      }
      if (line.find("END OF HEADER") != std::string::npos) {
        saw_end_header = true;
        break;
      }
    }

    if (!saw_version) {
      reason = "missing RINEX VERSION / TYPE header";
    } else if (!saw_304) {
      reason = "only RINEX 3.04 NAV files are supported by gnss_comm::rinex2ephems";
    } else if (!saw_nav) {
      reason = "RINEX file is not marked as NAV data";
    } else if (!saw_leap_seconds) {
      reason = "missing LEAP SECONDS header required by gnss_comm::rinex2ephems";
    } else if (!saw_end_header) {
      reason = "missing END OF HEADER";
    } else {
      return true;
    }
    return false;
  }

  bool ensure_rinex_loaded(rclcpp::Logger logger)
  {
    if (rinex_load_attempted_) {
      return rinex_loaded_;
    }
    rinex_load_attempted_ = true;

    std::string validation_error;
    if (!validate_rinex_nav_header(validation_error)) {
      rinex_error_message_ = validation_error;
      RCLCPP_WARN(
        logger, "RINEX NAV validation failed for '%s': %s",
        rinex_nav_file_.c_str(), rinex_error_message_.c_str());
      return false;
    }

    std::map<uint32_t, std::vector<gnss_comm::EphemBasePtr>> sat2ephem_base;
    gnss_comm::rinex2ephems(rinex_nav_file_, sat2ephem_base);

    const bool effective_gps_only = rinex_gps_only_ && !has_non_gps_enabled(enabled_systems_);
    for (const auto& [sat_id, base_ephems] : sat2ephem_base) {
      uint32_t prn = 0;
      const uint32_t sys = gnss_comm::satsys(sat_id, &prn);
      if (effective_gps_only && sys != SYS_GPS) {
        continue;
      }
      if (!sys_enabled(enabled_systems_, sys)) {
        continue;
      }

      if (sys == SYS_GLO) {
        auto& out = rinex_glo_ephems_[sat_id];
        for (const auto& base : base_ephems) {
          auto geph = std::dynamic_pointer_cast<gnss_comm::GloEphem>(base);
          if (!geph || geph->sat == 0) {
            continue;
          }
          if (geph->ttr.time == 0) {
            geph->ttr = geph->toe;
          }
          if (geph->iode == 0) {
            geph->iode = prn;
          }
          out.push_back(geph);
        }
        std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
          return gnss_comm::time_diff(lhs->toe, rhs->toe) < 0.0;
        });
        rinex_glo_ephem_count_ += out.size();
        continue;
      }

      auto& out = rinex_ephems_[sat_id];
      for (const auto& base : base_ephems) {
        auto eph = std::dynamic_pointer_cast<gnss_comm::Ephem>(base);
        if (!eph || eph->sat == 0) {
          continue;
        }
        out.push_back(eph);
      }
      std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
        return gnss_comm::time_diff(lhs->toe, rhs->toe) < 0.0;
      });
      rinex_ephem_count_ += out.size();
    }

    if (rinex_ephem_count_ == 0 && rinex_glo_ephem_count_ == 0) {
      rinex_error_message_ = "RINEX NAV file contains no usable enabled ephemerides";
      RCLCPP_WARN(logger, "%s: %s", rinex_nav_file_.c_str(), rinex_error_message_.c_str());
      return false;
    }

    rinex_loaded_ = true;
    rinex_error_message_.clear();
    RCLCPP_INFO(
      logger, "Loaded %zu GPS/BDS/GAL and %zu GLONASS ephemeris records from RINEX NAV file: %s",
      rinex_ephem_count_, rinex_glo_ephem_count_, rinex_nav_file_.c_str());
    return true;
  }

  void select_rinex_ephemerides(const gnss_comm::gtime_t& gpst_time)
  {
    ephems_.clear();
    glo_ephems_.clear();
    rinex_selected_ephem_count_ = 0;
    rinex_selected_glo_ephem_count_ = 0;
    selected_gps_ephems_ = 0;
    selected_bds_ephems_ = 0;
    selected_gal_ephems_ = 0;
    selected_glo_ephems_ = 0;
    rinex_selected_min_age_s_ = std::numeric_limits<double>::quiet_NaN();
    rinex_selected_max_age_s_ = std::numeric_limits<double>::quiet_NaN();

    const auto record_selected_age = [this](const double age) {
      if (!std::isfinite(rinex_selected_min_age_s_) || age < rinex_selected_min_age_s_) {
        rinex_selected_min_age_s_ = age;
      }
      if (!std::isfinite(rinex_selected_max_age_s_) || age > rinex_selected_max_age_s_) {
        rinex_selected_max_age_s_ = age;
      }
    };

    for (const auto& [sat_id, candidates] : rinex_ephems_) {
      uint32_t prn = 0;
      const uint32_t sys = gnss_comm::satsys(sat_id, &prn);
      if (!sys_enabled(enabled_systems_, sys)) {
        continue;
      }
      gnss_comm::EphemPtr best;
      double best_age = std::numeric_limits<double>::infinity();
      for (const auto& eph : candidates) {
        if (!eph || eph->health != 0) {
          continue;
        }
        const double age = std::abs(gnss_comm::time_diff(gpst_time, eph->toe));
        if (age <= rinex_ephem_max_age_s_ && age < best_age) {
          best = eph;
          best_age = age;
        }
      }
      if (best) {
        ephems_.push_back(best);
        record_selected_age(best_age);
        if (sys == SYS_GPS) {
          ++selected_gps_ephems_;
        } else if (sys == SYS_BDS) {
          ++selected_bds_ephems_;
        } else if (sys == SYS_GAL) {
          ++selected_gal_ephems_;
        }
      }
    }

    for (const auto& [sat_id, candidates] : rinex_glo_ephems_) {
      uint32_t prn = 0;
      const uint32_t sys = gnss_comm::satsys(sat_id, &prn);
      if (!sys_enabled(enabled_systems_, sys)) {
        continue;
      }
      gnss_comm::GloEphemPtr best;
      double best_age = std::numeric_limits<double>::infinity();
      for (const auto& geph : candidates) {
        if (!geph || geph->health != 0) {
          continue;
        }
        const double age = std::abs(gnss_comm::time_diff(gpst_time, geph->toe));
        if (age <= rinex_ephem_max_age_s_ && age < best_age) {
          best = geph;
          best_age = age;
        }
      }
      if (best) {
        glo_ephems_.push_back(best);
        record_selected_age(best_age);
        ++selected_glo_ephems_;
      }
    }

    rinex_selected_ephem_count_ = ephems_.size();
    rinex_selected_glo_ephem_count_ = glo_ephems_.size();
  }

  uint32_t gps_prn_min_ = 1;
  uint32_t gps_prn_max_ = 24;
  uint32_t num_gps_sats_ = 24;
  std::string ephemeris_source_ = "synthetic";
  std::string rinex_nav_file_;
  std::string rinex_error_message_;
  double rinex_ephem_max_age_s_ = 7200.0;
  bool rinex_gps_only_ = true;
  bool fallback_to_synthetic_on_rinex_error_ = true;
  bool rinex_load_attempted_ = false;
  bool rinex_loaded_ = false;
  bool rinex_fallback_active_ = false;
  size_t rinex_ephem_count_ = 0;
  size_t rinex_glo_ephem_count_ = 0;
  size_t rinex_selected_ephem_count_ = 0;
  size_t rinex_selected_glo_ephem_count_ = 0;
  double rinex_selected_min_age_s_ = std::numeric_limits<double>::quiet_NaN();
  double rinex_selected_max_age_s_ = std::numeric_limits<double>::quiet_NaN();
  size_t selected_gps_ephems_ = 0;
  size_t selected_bds_ephems_ = 0;
  size_t selected_gal_ephems_ = 0;
  size_t selected_glo_ephems_ = 0;
  std::unordered_set<uint32_t> enabled_systems_ = {SYS_GPS};
  std::map<uint32_t, std::vector<gnss_comm::EphemPtr>> rinex_ephems_;
  std::map<uint32_t, std::vector<gnss_comm::GloEphemPtr>> rinex_glo_ephems_;
  std::vector<gnss_comm::EphemPtr> ephems_;
  std::vector<gnss_comm::GloEphemPtr> glo_ephems_;
};

enum class VisibilityState
{
  kLos,
  kNlos,
  kOccluded,
  kDropped,
  kFaulted,
  kBelowHorizon,
  kLowElevation,
  kLowCn0,
};

std::string visibility_to_string(const VisibilityState state)
{
  switch (state) {
    case VisibilityState::kLos:
      return "LOS";
    case VisibilityState::kNlos:
      return "NLOS";
    case VisibilityState::kOccluded:
      return "OCCLUDED";
    case VisibilityState::kDropped:
      return "DROPPED";
    case VisibilityState::kFaulted:
      return "FAULTED";
    case VisibilityState::kBelowHorizon:
      return "BELOW_HORIZON";
    case VisibilityState::kLowElevation:
      return "LOW_ELEVATION";
    case VisibilityState::kLowCn0:
      return "LOW_CN0";
  }
  return "UNKNOWN";
}

struct SatelliteEval
{
  gnss_comm::EphemPtr eph;
  gnss_comm::GloEphemPtr glo_eph;
  uint32_t sat_id = 0;
  uint32_t sys = SYS_NONE;
  uint32_t prn = 0;
  double freq_hz = FREQ1;
  uint8_t code = CODE_L1C;
  double tgd_s = 0.0;
  double svdt_s = 0.0;
  double svddt_sps = 0.0;
  double earth_omega_rad_s = EARTH_OMG_GPS;
  Eigen::Vector3d sat_pos_ecef = Eigen::Vector3d::Zero();
  Eigen::Vector3d sat_vel_ecef = Eigen::Vector3d::Zero();
  Eigen::Vector3d dir_enu = Eigen::Vector3d::Zero();
  // RViz-only scaled sky-dome display position. This is not the physical satellite altitude.
  Eigen::Vector3d display_pos_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d raycast_hit_enu = Eigen::Vector3d::Zero();
  bool has_raycast_hit = false;
  double az_rad = 0.0;
  double el_rad = 0.0;
  double cn0_dbhz = 45.0;
  double psr_extra_bias_m = 0.0;
  double doppler_bias_mps = 0.0;
  double psr_std_scale = 1.0;
  double doppler_std_scale = 1.0;
  VisibilityState visibility = VisibilityState::kLos;
};

struct VisibilityCounts
{
  size_t los = 0;
  size_t nlos = 0;
  size_t occluded = 0;
  size_t dropped = 0;
  size_t faulted = 0;
  size_t below_horizon = 0;
  size_t low_elevation = 0;
  size_t low_cn0 = 0;
};

struct ObservationCounts
{
  size_t gps = 0;
  size_t bds = 0;
  size_t gal = 0;
  size_t glo = 0;
};

struct VoxelKey
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey& key) const
  {
    const auto hx = std::hash<int>{}(key.x * 73856093);
    const auto hy = std::hash<int>{}(key.y * 19349663);
    const auto hz = std::hash<int>{}(key.z * 83492791);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

class VisibilityModel
{
public:
  void configure(
    const ScenarioConfig& scenario,
    const double min_elevation_rad,
    const bool enable_skymask,
    const bool enable_map_occlusion,
    const bool enable_nlos,
    const double voxel_size_m,
    const double raycast_max_range_m,
    const double raycast_step_m)
  {
    scenario_ = scenario;
    min_elevation_rad_ = min_elevation_rad;
    enable_skymask_ = enable_skymask || scenario.skymask_enabled;
    enable_map_occlusion_ = enable_map_occlusion;
    enable_nlos_ = enable_nlos;
    voxel_size_m_ = std::max(voxel_size_m, 0.05);
    raycast_max_range_m_ = std::max(raycast_max_range_m, 1.0);
    raycast_step_m_ = std::max(raycast_step_m, 0.05);
  }

  void update_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::unordered_set<VoxelKey, VoxelKeyHash> voxels;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const Eigen::Vector3d p(*iter_x, *iter_y, *iter_z);
        if (!p.allFinite()) {
          continue;
        }
        voxels.insert(to_voxel(p));
      }
    } catch (const std::exception&) {
      return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    map_voxels_ = std::move(voxels);
    has_map_ = !map_voxels_.empty();
  }

  SatelliteEval classify(
    SatelliteEval sat,
    const ReceiverState& receiver,
    const double scenario_time_s,
    const std::vector<FaultConfig>& faults,
    const double los_cn0_dbhz,
    const double nlos_cn0_dbhz,
    const double low_cn0_threshold_dbhz,
    const bool enable_fault_injection,
    const double nlos_bias_mean_m,
    const double nlos_bias_std_m,
    const bool enable_multipath,
    const double multipath_amp_m,
    const double multipath_period_s,
    std::mt19937& rng) const
  {
    // Horizon/elevation gating is deliberately before SkyMask, map occlusion,
    // NLOS, multipath, and faults. A satellite below the local ENU horizon or
    // below the configured elevation mask must not be promoted back into a
    // usable GNSS measurement by later degradation/fault logic.
    if (sat.el_rad < 0.0) {
      sat.visibility = VisibilityState::kBelowHorizon;
      sat.cn0_dbhz = 0.0;
      return sat;
    }

    if (sat.el_rad < min_elevation_rad_) {
      sat.visibility = VisibilityState::kLowElevation;
      sat.cn0_dbhz = std::max(
        0.0, los_cn0_dbhz - (min_elevation_rad_ - sat.el_rad) * kRadToDeg * 0.5);
      return sat;
    }

    bool blocked = false;
    if (enable_skymask_) {
      const double mask_el_deg = mask_elevation_deg(sat.az_rad * kRadToDeg);
      blocked = blocked || (sat.el_rad * kRadToDeg < mask_el_deg);
    }

    if (enable_map_occlusion_) {
      Eigen::Vector3d hit;
      if (raycast(receiver.pos_enu, sat.dir_enu, hit)) {
        blocked = true;
        sat.has_raycast_hit = true;
        sat.raycast_hit_enu = hit;
      }
    }

    if (blocked) {
      sat.visibility = enable_nlos_ ? VisibilityState::kNlos : VisibilityState::kOccluded;
    } else {
      sat.visibility = VisibilityState::kLos;
    }

    sat.cn0_dbhz = sat.visibility == VisibilityState::kNlos ? nlos_cn0_dbhz : los_cn0_dbhz;
    if (sat.el_rad < 20.0 * kDegToRad) {
      sat.cn0_dbhz -= (20.0 - sat.el_rad * kRadToDeg) * 0.2;
    }

    if (sat.visibility == VisibilityState::kNlos) {
      std::normal_distribution<double> nlos_bias(nlos_bias_mean_m, std::max(nlos_bias_std_m, 0.0));
      sat.psr_extra_bias_m += std::max(0.0, nlos_bias(rng));
      sat.psr_std_scale *= 4.0;
      sat.doppler_std_scale *= 2.0;
    }

    if (enable_multipath && sat.visibility != VisibilityState::kOccluded) {
      const double phase = static_cast<double>(sat.prn) * 0.73;
      const double period = std::max(multipath_period_s, 0.1);
      sat.psr_extra_bias_m += multipath_amp_m * std::sin(2.0 * kPi * scenario_time_s / period + phase);
    }

    if (enable_fault_injection) {
      for (const auto& fault : faults) {
        if (fault.sys != SYS_NONE && fault.sys != sat.sys) {
          continue;
        }
        if (fault.prn != sat.prn) {
          continue;
        }
        const double fault_t = scenario_time_s - fault.start_time_s;
        if (fault_t < 0.0 || fault_t > fault.duration_s) {
          continue;
        }
        if (fault.drop) {
          sat.visibility = VisibilityState::kDropped;
        } else if (sat.visibility != VisibilityState::kOccluded) {
          sat.visibility = VisibilityState::kFaulted;
          sat.psr_extra_bias_m += fault.pseudorange_bias_m + fault.bias_rate_mps * fault_t;
          sat.doppler_bias_mps += fault.doppler_bias_mps;
          sat.cn0_dbhz -= fault.cn0_degrade_dbhz;
        }
      }
    }

    if (sat.cn0_dbhz < low_cn0_threshold_dbhz &&
      sat.visibility == VisibilityState::kLos)
    {
      sat.visibility = VisibilityState::kLowCn0;
    }

    return sat;
  }

  bool map_ready() const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return has_map_;
  }

private:
  VoxelKey to_voxel(const Eigen::Vector3d& p) const
  {
    return VoxelKey{
      static_cast<int>(std::floor(p.x() / voxel_size_m_)),
      static_cast<int>(std::floor(p.y() / voxel_size_m_)),
      static_cast<int>(std::floor(p.z() / voxel_size_m_))};
  }

  double mask_elevation_deg(const double az_deg) const
  {
    if (scenario_.skymask.empty()) {
      return scenario_.skymask_default_min_el_deg;
    }
    const double az = normalize_deg(az_deg);
    const auto& entries = scenario_.skymask;
    if (entries.size() == 1) {
      return entries.front().min_el_deg;
    }
    for (size_t i = 0; i < entries.size(); ++i) {
      const size_t j = (i + 1) % entries.size();
      const double a0 = entries[i].az_deg;
      double a1 = entries[j].az_deg;
      double q = az;
      if (j == 0) {
        a1 += 360.0;
        if (q < a0) {
          q += 360.0;
        }
      }
      if (q >= a0 && q <= a1) {
        const double alpha = (a1 - a0) > 1.0e-9 ? (q - a0) / (a1 - a0) : 0.0;
        return (1.0 - alpha) * entries[i].min_el_deg + alpha * entries[j].min_el_deg;
      }
    }
    return scenario_.skymask_default_min_el_deg;
  }

  bool raycast(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    Eigen::Vector3d& hit) const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!has_map_ || direction.norm() < 1.0e-6) {
      return false;
    }
    const Eigen::Vector3d dir = direction.normalized();
    for (double r = raycast_step_m_; r <= raycast_max_range_m_; r += raycast_step_m_) {
      const Eigen::Vector3d p = origin + dir * r;
      if (map_voxels_.find(to_voxel(p)) != map_voxels_.end()) {
        hit = p;
        return true;
      }
    }
    return false;
  }

  ScenarioConfig scenario_;
  double min_elevation_rad_ = 10.0 * kDegToRad;
  bool enable_skymask_ = false;
  bool enable_map_occlusion_ = false;
  bool enable_nlos_ = false;
  double voxel_size_m_ = 0.5;
  double raycast_max_range_m_ = 120.0;
  double raycast_step_m_ = 0.25;

  mutable std::mutex map_mutex_;
  std::unordered_set<VoxelKey, VoxelKeyHash> map_voxels_;
  bool has_map_ = false;
};

class GnssSimNode : public rclcpp::Node
{
public:
  GnssSimNode()
  : Node("gnss_sim_node"),
    rng_(static_cast<uint32_t>(declare_parameter<int64_t>("random_seed", 20260429))),
    unit_normal_(0.0, 1.0)
  {
    declare_and_load_parameters();
    create_publishers_and_subscribers();
    create_timers();

    RCLCPP_INFO(
      get_logger(),
      "gnss_sim_node v2 started: truth_odom_topic=%s time_source=%s origin_lla=[%.7f, %.7f, %.2f]",
      truth_odom_topic_.c_str(), time_source_.c_str(),
      origin_lla_.x(), origin_lla_.y(), origin_lla_.z());
  }

private:
  void declare_and_load_parameters()
  {
    truth_odom_topic_ = declare_parameter<std::string>(
      "truth_odom_topic", "/sim/drone_0/truth_odom");
    time_source_ = declare_parameter<std::string>("time_source", "odom_stamp");
    trigger_topic_ = declare_parameter<std::string>("trigger_topic", "/sim/drone_0/lidar");
    scenario_file_ = declare_parameter<std::string>("scenario_file", "");
    ephemeris_source_ = declare_parameter<std::string>("ephemeris_source", "synthetic");
    rinex_nav_file_ = declare_parameter<std::string>("rinex_nav_file", "");
    rinex_ephem_max_age_s_ = declare_parameter<double>("rinex_ephem_max_age_s", 7200.0);
    rinex_gps_only_ = declare_parameter<bool>("rinex_gps_only", true);
    fallback_to_synthetic_on_rinex_error_ =
      declare_parameter<bool>("fallback_to_synthetic_on_rinex_error", true);

    origin_lla_ << declare_parameter<double>("origin_lat_deg", 31.2304),
      declare_parameter<double>("origin_lon_deg", 121.4737),
      declare_parameter<double>("origin_alt_m", 25.0);

    measurement_rate_hz_ = declare_parameter<double>("measurement_rate_hz", 10.0);
    ephem_rate_hz_ = declare_parameter<double>("ephem_rate_hz", 1.0);
    navsat_rate_hz_ = declare_parameter<double>("navsat_rate_hz", 1.0);
    min_elevation_rad_ =
      declare_parameter<double>("min_elevation_deg", 10.0) * kDegToRad;
    pseudorange_noise_std_m_ =
      declare_parameter<double>("pseudorange_noise_std_m", 1.0);
    doppler_noise_std_mps_ =
      declare_parameter<double>("doppler_noise_std_mps", 0.1);
    receiver_clock_bias_m_ =
      declare_parameter<double>("receiver_clock_bias_m", 0.0);
    receiver_clock_drift_mps_ =
      declare_parameter<double>("receiver_clock_drift_mps", 0.0);
    truth_buffer_duration_s_ =
      declare_parameter<double>("truth_buffer_duration_s", 5.0);
    max_truth_interpolation_gap_s_ =
      declare_parameter<double>("max_truth_interpolation_gap_s", 0.05);

    num_gps_sats_ = static_cast<uint32_t>(
      std::max<int64_t>(1, declare_parameter<int64_t>("num_gps_sats", 24)));
    gps_prn_min_ = static_cast<uint32_t>(
      std::max<int64_t>(1, declare_parameter<int64_t>("gps_prn_min", 1)));
    gps_prn_max_ = static_cast<uint32_t>(
      std::max<int64_t>(gps_prn_min_, declare_parameter<int64_t>("gps_prn_max", 24)));
    enabled_constellations_ =
      declare_parameter<std::vector<std::string>>("enabled_constellations", {"GPS"});
    enabled_constellations_csv_ =
      declare_parameter<std::string>("enabled_constellations_csv", "GPS");
    enabled_systems_.clear();
    const auto csv_constellations = split_constellation_csv(enabled_constellations_csv_);
    const auto& selected_constellations =
      csv_constellations.empty() ? enabled_constellations_ : csv_constellations;
    for (const auto& name : selected_constellations) {
      const uint32_t sys = constellation_name_to_sys(name);
      if (sys != SYS_NONE) {
        enabled_systems_.insert(sys);
      }
    }
    if (enabled_systems_.empty()) {
      enabled_systems_.insert(SYS_GPS);
    }
    enabled_constellations_csv_ = join_enabled_constellations(enabled_systems_);

    enable_skymask_ = declare_parameter<bool>("enable_skymask", false);
    enable_map_occlusion_ = declare_parameter<bool>("enable_map_occlusion", false);
    enable_nlos_ = declare_parameter<bool>("enable_nlos", false);
    enable_multipath_ = declare_parameter<bool>("enable_multipath", false);
    enable_fault_injection_ = declare_parameter<bool>("enable_fault_injection", false);
    map_cloud_topic_ = declare_parameter<std::string>(
      "map_cloud_topic", "/map_generator/global_cloud");
    occlusion_voxel_size_m_ =
      declare_parameter<double>("occlusion_voxel_size_m", 0.5);
    raycast_max_range_m_ =
      declare_parameter<double>("raycast_max_range_m", 120.0);
    raycast_step_m_ =
      declare_parameter<double>("raycast_step_m", 0.25);

    los_cn0_dbhz_ = declare_parameter<double>("los_cn0_dbhz", 45.0);
    nlos_cn0_dbhz_ = declare_parameter<double>("nlos_cn0_dbhz", 28.0);
    low_cn0_threshold_dbhz_ =
      declare_parameter<double>("low_cn0_threshold_dbhz", 30.0);
    nlos_bias_mean_m_ = declare_parameter<double>("nlos_bias_mean_m", 15.0);
    nlos_bias_std_m_ = declare_parameter<double>("nlos_bias_std_m", 5.0);
    multipath_amp_m_ = declare_parameter<double>("multipath_amp_m", 0.0);
    multipath_period_s_ = declare_parameter<double>("multipath_period_s", 8.0);

    enable_visualization_ = declare_parameter<bool>("enable_visualization", false);
    visualization_frame_ = declare_parameter<std::string>("visualization_frame", "map");
    satellite_display_radius_m_ =
      declare_parameter<double>("satellite_display_radius_m", 80.0);
    sky_dome_follow_receiver_ =
      declare_parameter<bool>("sky_dome_follow_receiver", false);
    sky_dome_center_enu_ =
      read_vector3_parameter("sky_dome_center_enu", {0.0, 0.0, 0.0});
    signal_ray_width_m_ =
      std::max(0.001, declare_parameter<double>("signal_ray_width_m", 0.08));
    signal_ray_alpha_ =
      std::clamp(declare_parameter<double>("signal_ray_alpha", 0.95), 0.0, 1.0);
    nlos_path_width_m_ =
      std::max(0.001, declare_parameter<double>("nlos_path_width_m", 0.16));
    nlos_path_alpha_ =
      std::clamp(declare_parameter<double>("nlos_path_alpha", 0.95), 0.0, 1.0);
    skyplot_origin_enu_ = read_vector3_parameter("skyplot_origin_enu", {-20.0, -20.0, 8.0});
    skyplot_radius_m_ = declare_parameter<double>("skyplot_radius_m", 8.0);
    enable_sky_dome_visualization_ =
      declare_parameter<bool>("enable_sky_dome_visualization", true);
    sky_dome_show_cardinal_labels_ =
      declare_parameter<bool>("sky_dome_show_cardinal_labels", true);
    sky_dome_ring_count_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("sky_dome_ring_count", 3)));
    sky_dome_meridian_count_ = static_cast<int>(
      std::max<int64_t>(4, declare_parameter<int64_t>("sky_dome_meridian_count", 12)));
    enable_csv_log_ = declare_parameter<bool>("enable_csv_log", false);
    csv_log_path_ = declare_parameter<std::string>("csv_log_path", "");

    scenario_ = ScenarioLoader::load(scenario_file_, get_logger());
    if (scenario_.has_anchor) {
      origin_lla_ = scenario_.anchor_lla;
    }

    origin_ecef_ = gnss_comm::geo2ecef(origin_lla_);
    R_ecef_enu_ = gnss_comm::geo2rotation(origin_lla_);

    state_buffer_.set_duration(truth_buffer_duration_s_);
    state_buffer_.set_max_gap(max_truth_interpolation_gap_s_);
    constellation_.configure(gps_prn_min_, gps_prn_max_, num_gps_sats_);
    constellation_.configure_ephemeris_source(
      ephemeris_source_, rinex_nav_file_, rinex_ephem_max_age_s_,
      rinex_gps_only_, fallback_to_synthetic_on_rinex_error_, enabled_systems_);
    visibility_model_.configure(
      scenario_, min_elevation_rad_, enable_skymask_, enable_map_occlusion_,
      enable_nlos_, occlusion_voxel_size_m_, raycast_max_range_m_, raycast_step_m_);

    if (enable_csv_log_ && !csv_log_path_.empty()) {
      csv_.open(csv_log_path_, std::ios::out | std::ios::trunc);
      if (csv_.is_open()) {
      csv_ << "stamp,scenario_time_s,prn,visibility_state,az_deg,el_deg,cn0_dbhz,"
        "psr_extra_bias_m_nlos_multipath_fault,doppler_bias_mps,"
        "raycast_hit_x,raycast_hit_y,raycast_hit_z\n";
      } else {
        RCLCPP_WARN(get_logger(), "Failed to open GNSS sim CSV log: %s", csv_log_path_.c_str());
      }
    }
  }

  Eigen::Vector3d read_vector3_parameter(
    const std::string& name,
    const std::array<double, 3>& fallback)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, {fallback[0], fallback[1], fallback[2]});
    if (values.size() != 3) {
      return Eigen::Vector3d(fallback[0], fallback[1], fallback[2]);
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void create_publishers_and_subscribers()
  {
    pub_receiver_lla_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/ublox_driver/receiver_lla", 10);
    pub_ephem_ = create_publisher<gnss_comm::msg::GnssEphemMsg>(
      "/ublox_driver/ephem", 50);
    pub_glo_ephem_ = create_publisher<gnss_comm::msg::GnssGloEphemMsg>(
      "/ublox_driver/glo_ephem", 50);
    pub_iono_ = create_publisher<gnss_comm::msg::GnssIonosphereParameter>(
      "/ublox_driver/iono_params", 10);
    pub_range_meas_ = create_publisher<gnss_comm::msg::GnssMeasMsg>(
      "/ublox_driver/range_meas", 100);
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/gnss_sim/diagnostics", 10);

    if (enable_visualization_) {
      pub_satellite_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/satellite_markers", 10);
      pub_signal_rays_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/signal_rays", 10);
      pub_nlos_paths_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/nlos_paths", 10);
      pub_sky_dome_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/sky_dome", 10);
      pub_skyplot_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/skyplot", 10);
      pub_status_text_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/status_text", 10);
      pub_occlusion_points_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gnss_sim/visualization/occlusion_points", 10);
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GnssSimNode::on_odom, this, std::placeholders::_1));

    if (time_source_ == "trigger_topic") {
      trigger_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        trigger_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GnssSimNode::on_trigger_cloud, this, std::placeholders::_1));
    }

    if (enable_map_occlusion_) {
      map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        map_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VisibilityModel::update_map, &visibility_model_, std::placeholders::_1));
    }
  }

  void create_timers()
  {
    const double static_rate_hz = std::max(ephem_rate_hz_, navsat_rate_hz_);
    static_timer_ = create_wall_timer(
      period_from_rate(static_rate_hz),
      std::bind(&GnssSimNode::publish_static_slow_topics, this));

    if (time_source_ == "odom_stamp" || time_source_ == "clock") {
      range_timer_ = create_wall_timer(
        period_from_rate(measurement_rate_hz_),
        std::bind(&GnssSimNode::publish_range_epoch_from_timer, this));
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ReceiverState state;
    state.stamp = msg->header.stamp;
    if (is_zero_stamp(state.stamp)) {
      state.stamp = now();
    }
    state.pos_enu << msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z;
    state.vel_enu << msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z;

    state_buffer_.push(state);

    if (!startup_topics_published_) {
      publish_static_slow_topics();
      startup_topics_published_ = true;
    }
  }

  void on_trigger_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (is_zero_stamp(msg->header.stamp)) {
      publish_range_epoch_at(now());
      return;
    }
    publish_range_epoch_at(msg->header.stamp);
  }

  bool get_state_for_stamp(
    const builtin_interfaces::msg::Time& stamp,
    ReceiverState& state,
    std::string& reason)
  {
    if (time_source_ == "odom_stamp") {
      if (!state_buffer_.latest(state)) {
        reason = "waiting for truth odometry";
        return false;
      }
      return true;
    }
    return state_buffer_.interpolate(stamp, state, reason);
  }

  void publish_static_slow_topics()
  {
    ReceiverState state;
    if (!state_buffer_.latest(state)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for truth odometry before publishing GNSS startup topics");
      publish_diagnostics(
        {}, {}, "waiting_for_truth_odom", 0.0, false,
        std::numeric_limits<double>::quiet_NaN());
      return;
    }

    const gnss_comm::gtime_t utc_time = stamp_to_utc_time(state.stamp);
    const gnss_comm::gtime_t gpst_time = gnss_comm::utc2gpst(utc_time);
    if (!constellation_.ensure_ephemerides(gpst_time, get_logger())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No GNSS ephemerides available; skip startup ephemeris publication");
      publish_receiver_lla(state);
      publish_iono_params(state.stamp);
      ephem_published_ = false;
      return;
    }

    publish_receiver_lla(state);
    publish_iono_params(state.stamp);
    for (const auto& eph : constellation_.ephems()) {
      pub_ephem_->publish(gnss_comm::ephem2msg(eph));
    }
    for (const auto& geph : constellation_.glo_ephems()) {
      pub_glo_ephem_->publish(gnss_comm::glo_ephem2msg(geph));
    }
    ephem_published_ = true;
  }

  void publish_receiver_lla(const ReceiverState& state)
  {
    const Eigen::Vector3d receiver_ecef =
      origin_ecef_ + R_ecef_enu_ * state.pos_enu;
    const Eigen::Vector3d receiver_lla = gnss_comm::ecef2geo(receiver_ecef);

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = state.stamp;
    msg.header.frame_id = "wgs84";
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg.latitude = receiver_lla.x();
    msg.longitude = receiver_lla.y();
    msg.altitude = receiver_lla.z();
    pub_receiver_lla_->publish(msg);
  }

  void publish_iono_params(const builtin_interfaces::msg::Time& stamp)
  {
    gnss_comm::msg::GnssIonosphereParameter msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "gps";
    msg.type = 0;
    msg.parameters = iono_params_;
    pub_iono_->publish(msg);
  }

  void publish_range_epoch_from_timer()
  {
    if (time_source_ == "clock") {
      publish_range_epoch_at(now());
      return;
    }

    ReceiverState latest;
    if (!state_buffer_.latest(latest)) {
      publish_diagnostics(
        {}, {}, "waiting_for_truth_odom", 0.0, false,
        std::numeric_limits<double>::quiet_NaN());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for truth odometry before publishing GNSS range measurements");
      return;
    }
    publish_range_epoch_at(latest.stamp);
  }

  void publish_range_epoch_at(const builtin_interfaces::msg::Time& target_stamp)
  {
    ReceiverState state;
    std::string state_reason;
    if (!get_state_for_stamp(target_stamp, state, state_reason)) {
      publish_diagnostics(
        {}, {}, state_reason, 0.0, false, stamp_to_sec(target_stamp));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skip GNSS epoch at %.6f: %s", stamp_to_sec(target_stamp), state_reason.c_str());
      return;
    }

    if (!ephem_published_) {
      publish_static_slow_topics();
      if (!ephem_published_) {
        return;
      }
    }

    if (!first_epoch_time_s_) {
      first_epoch_time_s_ = stamp_to_sec(state.stamp);
    }
    const double scenario_time_s = stamp_to_sec(state.stamp) - *first_epoch_time_s_;

    const gnss_comm::gtime_t utc_time = stamp_to_utc_time(state.stamp);
    const gnss_comm::gtime_t gpst_rx_time = gnss_comm::utc2gpst(utc_time);
    if (!constellation_.ensure_ephemerides(gpst_rx_time, get_logger())) {
      publish_diagnostics(
        {}, {}, "no_ephemerides_available", scenario_time_s, false,
        stamp_to_sec(state.stamp));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skip GNSS epoch at %.6f: no ephemerides available",
        stamp_to_sec(state.stamp));
      return;
    }

    const Eigen::Vector3d receiver_ecef =
      origin_ecef_ + R_ecef_enu_ * state.pos_enu;
    const Eigen::Vector3d receiver_vel_ecef = R_ecef_enu_ * state.vel_enu;
    const Eigen::Vector3d receiver_lla = gnss_comm::ecef2geo(receiver_ecef);

    std::vector<gnss_comm::ObsPtr> obs_list;
    std::vector<SatelliteEval> evals;
    obs_list.reserve(constellation_.ephems().size() + constellation_.glo_ephems().size());
    evals.reserve(constellation_.ephems().size() + constellation_.glo_ephems().size());

    for (const auto& eph : constellation_.ephems()) {
      auto sat = evaluate_satellite(
        eph, gpst_rx_time, receiver_ecef, receiver_vel_ecef, state, receiver_lla,
        scenario_time_s);
      if (!sat) {
        continue;
      }
      evals.push_back(*sat);
      append_observation_if_usable(*sat, gpst_rx_time, receiver_ecef, receiver_vel_ecef, receiver_lla, obs_list);
    }
    for (const auto& geph : constellation_.glo_ephems()) {
      auto sat = evaluate_satellite(
        geph, gpst_rx_time, receiver_ecef, receiver_vel_ecef, state, receiver_lla,
        scenario_time_s);
      if (!sat) {
        continue;
      }
      evals.push_back(*sat);
      append_observation_if_usable(*sat, gpst_rx_time, receiver_ecef, receiver_vel_ecef, receiver_lla, obs_list);
    }

    const bool enough_sats = obs_list.size() >= 4;
    if (enough_sats) {
      auto msg = gnss_comm::meas2msg(obs_list);
      pub_range_meas_->publish(msg);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GNSS usable satellite count is %zu < 4; skip weak epoch", obs_list.size());
    }

    publish_diagnostics(
      evals, obs_list, enough_sats ? "ok" : "usable_satellite_count_lt_4",
      scenario_time_s, enough_sats, stamp_to_sec(state.stamp));
    publish_visualization(evals, state, scenario_time_s, obs_list.size());
    write_csv(evals, state, scenario_time_s);
  }

  double earth_omega_for_sys(const uint32_t sys) const
  {
    if (sys == SYS_BDS) {
      return EARTH_OMG_BDS;
    }
    if (sys == SYS_GLO) {
      return EARTH_OMG_GLO;
    }
    return EARTH_OMG_GPS;
  }

  double signal_frequency_hz(const uint32_t sys, const gnss_comm::GloEphemPtr& geph = nullptr) const
  {
    if (sys == SYS_BDS) {
      return FREQ1_BDS;
    }
    if (sys == SYS_GLO) {
      const int freqo = geph ? geph->freqo : 0;
      return FREQ1_GLO + static_cast<double>(freqo) * DFRQ1_GLO;
    }
    return FREQ1;
  }

  uint8_t signal_code(const uint32_t sys) const
  {
    if (sys == SYS_BDS) {
      return CODE_L2I;
    }
    return CODE_L1C;
  }

  std::optional<SatelliteEval> evaluate_satellite(
    const gnss_comm::EphemPtr& eph,
    const gnss_comm::gtime_t& gpst_rx_time,
    const Eigen::Vector3d& receiver_ecef,
    const Eigen::Vector3d& receiver_vel_ecef,
    const ReceiverState& receiver_state,
    const Eigen::Vector3d& receiver_lla,
    const double scenario_time_s)
  {
    (void)receiver_vel_ecef;
    (void)receiver_lla;
    double svdt_rx = 0.0;
    const Eigen::Vector3d sat_rx = gnss_comm::eph2pos(gpst_rx_time, eph, &svdt_rx);
    if (!sat_rx.allFinite()) {
      return std::nullopt;
    }

    const double rho0 = (sat_rx - receiver_ecef).norm();
    const gnss_comm::gtime_t tx_time =
      gnss_comm::time_add(gpst_rx_time, -rho0 / LIGHT_SPEED);

    double svdt = 0.0;
    double svddt = 0.0;
    const Eigen::Vector3d sat_pos = gnss_comm::eph2pos(tx_time, eph, &svdt);
    const Eigen::Vector3d sat_vel = gnss_comm::eph2vel(tx_time, eph, &svddt);
    if (!sat_pos.allFinite() || !sat_vel.allFinite()) {
      return std::nullopt;
    }

    double azel[2] = {0.0, 0.0};
    gnss_comm::sat_azel(receiver_ecef, sat_pos, azel);

    SatelliteEval sat;
    sat.eph = eph;
    uint32_t prn = 0;
    const uint32_t sys = gnss_comm::satsys(eph->sat, &prn);
    sat.sat_id = eph->sat;
    sat.sys = sys;
    sat.prn = prn;
    sat.freq_hz = signal_frequency_hz(sys);
    sat.code = signal_code(sys);
    sat.tgd_s = eph->tgd[0];
    sat.svdt_s = svdt;
    sat.svddt_sps = svddt;
    sat.earth_omega_rad_s = earth_omega_for_sys(sys);
    // Measurement-layer satellite state in ECEF meters. This uses the synthetic GPS-like
    // ephemeris scale and must not be confused with the scaled RViz display position.
    sat.sat_pos_ecef = sat_pos;
    sat.sat_vel_ecef = sat_vel;
    sat.az_rad = azel[0];
    sat.el_rad = azel[1];
    sat.dir_enu = R_ecef_enu_.transpose() * (sat_pos - receiver_ecef).normalized();
    // RViz-only scaled sky-dome position derived explicitly from azimuth and
    // elevation. satellite_display_radius_m_ is a drawing radius, not the GPS
    // orbit altitude.
    const Eigen::Vector3d display_dir_enu(
      std::cos(sat.el_rad) * std::sin(sat.az_rad),
      std::cos(sat.el_rad) * std::cos(sat.az_rad),
      std::sin(sat.el_rad));
    sat.display_pos_enu = sky_dome_center(receiver_state) +
      display_dir_enu.normalized() * satellite_display_radius_m_;

    return visibility_model_.classify(
      sat, receiver_state, scenario_time_s, scenario_.faults,
      los_cn0_dbhz_, nlos_cn0_dbhz_, low_cn0_threshold_dbhz_,
      enable_fault_injection_, nlos_bias_mean_m_, nlos_bias_std_m_,
      enable_multipath_, multipath_amp_m_, multipath_period_s_, rng_);
  }

  std::optional<SatelliteEval> evaluate_satellite(
    const gnss_comm::GloEphemPtr& geph,
    const gnss_comm::gtime_t& gpst_rx_time,
    const Eigen::Vector3d& receiver_ecef,
    const Eigen::Vector3d& receiver_vel_ecef,
    const ReceiverState& receiver_state,
    const Eigen::Vector3d& receiver_lla,
    const double scenario_time_s)
  {
    (void)receiver_vel_ecef;
    (void)receiver_lla;
    double svdt_rx = 0.0;
    const Eigen::Vector3d sat_rx = gnss_comm::geph2pos(gpst_rx_time, geph, &svdt_rx);
    if (!sat_rx.allFinite()) {
      return std::nullopt;
    }

    const double rho0 = (sat_rx - receiver_ecef).norm();
    const gnss_comm::gtime_t tx_time =
      gnss_comm::time_add(gpst_rx_time, -rho0 / LIGHT_SPEED);

    double svdt = 0.0;
    double svddt = 0.0;
    const Eigen::Vector3d sat_pos = gnss_comm::geph2pos(tx_time, geph, &svdt);
    const Eigen::Vector3d sat_vel = gnss_comm::geph2vel(tx_time, geph, &svddt);
    if (!sat_pos.allFinite() || !sat_vel.allFinite()) {
      return std::nullopt;
    }

    double azel[2] = {0.0, 0.0};
    gnss_comm::sat_azel(receiver_ecef, sat_pos, azel);

    SatelliteEval sat;
    sat.glo_eph = geph;
    uint32_t prn = 0;
    const uint32_t sys = gnss_comm::satsys(geph->sat, &prn);
    sat.sat_id = geph->sat;
    sat.sys = sys;
    sat.prn = prn;
    sat.freq_hz = signal_frequency_hz(sys, geph);
    sat.code = signal_code(sys);
    sat.tgd_s = 0.0;
    sat.svdt_s = svdt;
    sat.svddt_sps = svddt;
    sat.earth_omega_rad_s = earth_omega_for_sys(sys);
    sat.sat_pos_ecef = sat_pos;
    sat.sat_vel_ecef = sat_vel;
    sat.az_rad = azel[0];
    sat.el_rad = azel[1];
    sat.dir_enu = R_ecef_enu_.transpose() * (sat_pos - receiver_ecef).normalized();
    const Eigen::Vector3d display_dir_enu(
      std::cos(sat.el_rad) * std::sin(sat.az_rad),
      std::cos(sat.el_rad) * std::cos(sat.az_rad),
      std::sin(sat.el_rad));
    sat.display_pos_enu = sky_dome_center(receiver_state) +
      display_dir_enu.normalized() * satellite_display_radius_m_;

    return visibility_model_.classify(
      sat, receiver_state, scenario_time_s, scenario_.faults,
      los_cn0_dbhz_, nlos_cn0_dbhz_, low_cn0_threshold_dbhz_,
      enable_fault_injection_, nlos_bias_mean_m_, nlos_bias_std_m_,
      enable_multipath_, multipath_amp_m_, multipath_period_s_, rng_);
  }

  void append_observation_if_usable(
    const SatelliteEval& sat,
    const gnss_comm::gtime_t& gpst_rx_time,
    const Eigen::Vector3d& receiver_ecef,
    const Eigen::Vector3d& receiver_vel_ecef,
    const Eigen::Vector3d& receiver_lla,
    std::vector<gnss_comm::ObsPtr>& obs_list)
  {
    if (sat.visibility == VisibilityState::kOccluded ||
      sat.visibility == VisibilityState::kDropped ||
      sat.visibility == VisibilityState::kBelowHorizon ||
      sat.visibility == VisibilityState::kLowElevation)
    {
      return;
    }

    const Eigen::Vector3d dp = receiver_ecef - sat.sat_pos_ecef;
    const double rho = dp.norm();
    if (rho <= 1.0 || !std::isfinite(rho)) {
      return;
    }
    const Eigen::Vector3d los_receiver_minus_sat = dp / rho;

    double azel[2] = {sat.az_rad, sat.el_rad};
    const double sagnac = sat.earth_omega_rad_s / LIGHT_SPEED *
      (sat.sat_pos_ecef.x() * receiver_ecef.y() - sat.sat_pos_ecef.y() * receiver_ecef.x());
    const double ion_delay = gnss_comm::calculate_ion_delay(
      gpst_rx_time, iono_params_, receiver_lla, azel);
    const double trop_delay = gnss_comm::calculate_trop_delay(
      gpst_rx_time, receiver_lla, azel);

    const double pr_std = std::max(pseudorange_noise_std_m_ * sat.psr_std_scale, 0.05);
    const double dop_std_mps = std::max(doppler_noise_std_mps_ * sat.doppler_std_scale, 0.01);
    const double pr_noise = pr_std * unit_normal_(rng_);
    const double psr_raw =
      rho + sagnac + receiver_clock_bias_m_ + ion_delay + trop_delay +
      sat.tgd_s * LIGHT_SPEED - sat.svdt_s * LIGHT_SPEED +
      sat.psr_extra_bias_m + pr_noise;

    const Eigen::Vector3d rel_vel = receiver_vel_ecef - sat.sat_vel_ecef;
    const double sagnac_dot = sat.earth_omega_rad_s / LIGHT_SPEED *
      (sat.sat_vel_ecef.x() * receiver_ecef.y() + sat.sat_pos_ecef.x() * receiver_vel_ecef.y() -
       sat.sat_vel_ecef.y() * receiver_ecef.x() - sat.sat_pos_ecef.y() * receiver_vel_ecef.x());
    const double dop_noise = dop_std_mps * unit_normal_(rng_);
    const double range_rate_raw =
      los_receiver_minus_sat.dot(rel_vel) + sagnac_dot +
      receiver_clock_drift_mps_ - sat.svddt_sps * LIGHT_SPEED +
      sat.doppler_bias_mps + dop_noise;
    const double dopp_hz = -range_rate_raw * sat.freq_hz / LIGHT_SPEED;

    auto obs = std::make_shared<gnss_comm::Obs>();
    obs->time = gpst_rx_time;
    obs->sat = sat.sat_id;
    obs->freqs = {sat.freq_hz};
    obs->CN0 = {sat.cn0_dbhz};
    obs->LLI = {0};
    obs->code = {sat.code};
    obs->psr = {psr_raw};
    obs->psr_std = {pr_std};
    obs->cp = {0.0};
    obs->cp_std = {0.0};
    obs->dopp = {dopp_hz};
    obs->dopp_std = {dop_std_mps * sat.freq_hz / LIGHT_SPEED};
    obs->status = {1};
    obs_list.push_back(obs);
  }

  VisibilityCounts count_visibility(const std::vector<SatelliteEval>& evals) const
  {
    VisibilityCounts counts;
    for (const auto& sat : evals) {
      switch (sat.visibility) {
        case VisibilityState::kLos:
          ++counts.los;
          break;
        case VisibilityState::kNlos:
          ++counts.nlos;
          break;
        case VisibilityState::kOccluded:
          ++counts.occluded;
          break;
        case VisibilityState::kDropped:
          ++counts.dropped;
          break;
        case VisibilityState::kFaulted:
          ++counts.faulted;
          break;
        case VisibilityState::kBelowHorizon:
          ++counts.below_horizon;
          break;
        case VisibilityState::kLowElevation:
          ++counts.low_elevation;
          break;
        case VisibilityState::kLowCn0:
          ++counts.low_cn0;
          break;
      }
    }
    return counts;
  }

  ObservationCounts count_observations(const std::vector<gnss_comm::ObsPtr>& obs_list) const
  {
    ObservationCounts counts;
    for (const auto& obs : obs_list) {
      if (!obs) {
        continue;
      }
      const uint32_t sys = gnss_comm::satsys(obs->sat, nullptr);
      if (sys == SYS_GPS) {
        ++counts.gps;
      } else if (sys == SYS_BDS) {
        ++counts.bds;
      } else if (sys == SYS_GAL) {
        ++counts.gal;
      } else if (sys == SYS_GLO) {
        ++counts.glo;
      }
    }
    return counts;
  }

  void publish_diagnostics(
    const std::vector<SatelliteEval>& evals,
    const std::vector<gnss_comm::ObsPtr>& obs_list,
    const std::string& status_text,
    const double scenario_time_s,
    const bool healthy,
    const double gnss_epoch_utc_s)
  {
    const auto counts = count_visibility(evals);
    const auto obs_counts = count_observations(obs_list);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "gnss_sim";
    status.hardware_id = "gnss_sim_node";
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = status_text;
    status.values = {
      kv("time_source", time_source_),
      kv("gnss_epoch_utc", std::to_string(gnss_epoch_utc_s)),
      kv("scenario_time_s", std::to_string(scenario_time_s)),
      kv("simulated_sats", std::to_string(evals.size())),
      kv("los", std::to_string(counts.los)),
      kv("nlos", std::to_string(counts.nlos)),
      kv("occluded", std::to_string(counts.occluded)),
      kv("dropped", std::to_string(counts.dropped)),
      kv("faulted", std::to_string(counts.faulted)),
      kv("below_horizon", std::to_string(counts.below_horizon)),
      kv("low_elevation", std::to_string(counts.low_elevation)),
      kv("low_cn0", std::to_string(counts.low_cn0)),
      kv("display_radius_m", std::to_string(satellite_display_radius_m_)),
      kv("synthetic_orbit_radius_m", std::to_string(kGpsSemiMajorAxisM)),
      kv("display_mode", "scaled_local_sky_dome"),
      kv("map_occlusion_enabled", bool_string(enable_map_occlusion_)),
      kv("map_ready", bool_string(visibility_model_.map_ready())),
      kv("scenario_file", scenario_file_),
      kv("enabled_constellations", constellation_.enabled_constellations_string()),
      kv("selected_gps_ephems", std::to_string(constellation_.selected_count_for_sys(SYS_GPS))),
      kv("selected_bds_ephems", std::to_string(constellation_.selected_count_for_sys(SYS_BDS))),
      kv("selected_gal_ephems", std::to_string(constellation_.selected_count_for_sys(SYS_GAL))),
      kv("selected_glo_ephems", std::to_string(constellation_.selected_count_for_sys(SYS_GLO))),
      kv("used_gps_obs", std::to_string(obs_counts.gps)),
      kv("used_bds_obs", std::to_string(obs_counts.bds)),
      kv("used_gal_obs", std::to_string(obs_counts.gal)),
      kv("used_glo_obs", std::to_string(obs_counts.glo)),
      kv("ephemeris_source", constellation_.ephemeris_source()),
      kv("rinex_nav_file", constellation_.rinex_nav_file()),
      kv("rinex_loaded", bool_string(constellation_.rinex_loaded())),
      kv("rinex_ephem_count", std::to_string(constellation_.rinex_ephem_count())),
      kv("rinex_glo_ephem_count", std::to_string(constellation_.rinex_glo_ephem_count())),
      kv("rinex_selected_ephem_count", std::to_string(constellation_.rinex_selected_ephem_count())),
      kv("rinex_selected_glo_ephem_count", std::to_string(constellation_.rinex_selected_glo_ephem_count())),
      kv("rinex_selected_min_age_s", std::to_string(constellation_.rinex_selected_min_age_s())),
      kv("rinex_selected_max_age_s", std::to_string(constellation_.rinex_selected_max_age_s())),
      kv("rinex_time_consistent", bool_string(constellation_.rinex_time_consistent())),
      kv("rinex_fallback_active", bool_string(constellation_.rinex_fallback_active())),
      kv("rinex_ephem_max_age_s", std::to_string(constellation_.rinex_ephem_max_age_s())),
      kv("rinex_error", constellation_.rinex_error_message()),
    };
    array.status.push_back(status);
    pub_diagnostics_->publish(array);
  }

  void publish_visualization(
    const std::vector<SatelliteEval>& evals,
    const ReceiverState& receiver,
    const double scenario_time_s,
    const size_t used_measurements)
  {
    if (!enable_visualization_) {
      return;
    }
    publish_sky_dome(receiver);
    publish_satellite_markers(evals, receiver.stamp);
    publish_signal_rays(evals, receiver, receiver.stamp);
    publish_nlos_paths(evals, receiver);
    publish_skyplot(evals, receiver.stamp);
    publish_status_text(evals, receiver, scenario_time_s, used_measurements);
    publish_occlusion_points(evals, receiver.stamp);
  }

  visualization_msgs::msg::Marker base_marker(
    const builtin_interfaces::msg::Time& stamp,
    const std::string& ns,
    const int id,
    const int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = visualization_frame_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    return marker;
  }

  std_msgs::msg::ColorRGBA color_for_visibility(const VisibilityState state) const
  {
    switch (state) {
      case VisibilityState::kLos:
        return color_rgba(0.0F, 0.9F, 0.2F, 0.95F);
      case VisibilityState::kNlos:
        return color_rgba(1.0F, 0.35F, 0.0F, 0.95F);
      case VisibilityState::kOccluded:
        return color_rgba(0.25F, 0.25F, 0.25F, 0.5F);
      case VisibilityState::kDropped:
        return color_rgba(0.5F, 0.5F, 0.5F, 0.25F);
      case VisibilityState::kFaulted:
        return color_rgba(1.0F, 0.0F, 1.0F, 0.98F);
      case VisibilityState::kBelowHorizon:
        return color_rgba(0.35F, 0.35F, 0.35F, 0.25F);
      case VisibilityState::kLowElevation:
        return color_rgba(0.95F, 0.85F, 0.15F, 0.65F);
      case VisibilityState::kLowCn0:
        return color_rgba(1.0F, 0.9F, 0.0F, 0.95F);
    }
    return color_rgba(1.0F, 1.0F, 1.0F, 1.0F);
  }

  void publish_delete_all(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& ns) const
  {
    visualization_msgs::msg::MarkerArray array;
    auto marker = base_marker(stamp, ns, 0, visualization_msgs::msg::Marker::SPHERE);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
    pub->publish(array);
  }

  void publish_satellite_markers(
    const std::vector<SatelliteEval>& evals,
    const builtin_interfaces::msg::Time& stamp)
  {
    publish_delete_all(pub_satellite_markers_, stamp, "satellites");
    visualization_msgs::msg::MarkerArray array;
    int id = 1;
    for (const auto& sat : evals) {
      if (sat.visibility == VisibilityState::kBelowHorizon) {
        continue;
      }
      auto sphere = base_marker(stamp, "satellite_spheres", id++, visualization_msgs::msg::Marker::SPHERE);
      sphere.pose.position = point_msg(sat.display_pos_enu);
      sphere.scale.x = 1.2;
      sphere.scale.y = 1.2;
      sphere.scale.z = 1.2;
      sphere.color = color_for_visibility(sat.visibility);
      array.markers.push_back(sphere);

      auto text = base_marker(stamp, "satellite_labels", id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text.pose.position = point_msg(sat.display_pos_enu + Eigen::Vector3d(0.0, 0.0, 1.4));
      text.scale.z = 1.2;
      text.color = color_rgba(1.0F, 1.0F, 1.0F, 0.95F);
      text.text = gnss_comm::sat2str(sat.sat_id) + " " + visibility_to_string(sat.visibility);
      array.markers.push_back(text);
    }
    pub_satellite_markers_->publish(array);
  }

  void publish_signal_rays(
    const std::vector<SatelliteEval>& evals,
    const ReceiverState& receiver,
    const builtin_interfaces::msg::Time& stamp)
  {
    publish_delete_all(pub_signal_rays_, stamp, "signal_rays");
    visualization_msgs::msg::MarkerArray array;
    int id = 1;
    for (const auto& sat : evals) {
      if (sat.visibility == VisibilityState::kBelowHorizon ||
        sat.visibility == VisibilityState::kLowElevation ||
        sat.visibility == VisibilityState::kOccluded ||
        sat.visibility == VisibilityState::kDropped)
      {
        continue;
      }
      auto ray = base_marker(stamp, "signal_rays", id++, visualization_msgs::msg::Marker::LINE_STRIP);
      ray.scale.x = signal_ray_width_m_;
      ray.color = color_for_visibility(sat.visibility);
      ray.color.a = static_cast<float>(signal_ray_alpha_);
      ray.points.push_back(point_msg(receiver.pos_enu));
      ray.points.push_back(point_msg(sat.display_pos_enu));
      array.markers.push_back(ray);
    }
    pub_signal_rays_->publish(array);
  }

  void publish_nlos_paths(
    const std::vector<SatelliteEval>& evals,
    const ReceiverState& receiver)
  {
    publish_delete_all(pub_nlos_paths_, receiver.stamp, "nlos_paths");
    visualization_msgs::msg::MarkerArray array;
    int id = 1;
    for (const auto& sat : evals) {
      if (!sat.has_raycast_hit ||
        (sat.visibility != VisibilityState::kNlos && sat.visibility != VisibilityState::kFaulted))
      {
        continue;
      }
      auto path = base_marker(receiver.stamp, "nlos_paths", id++, visualization_msgs::msg::Marker::LINE_STRIP);
      path.scale.x = nlos_path_width_m_;
      path.color = color_for_visibility(sat.visibility);
      path.color.a = static_cast<float>(nlos_path_alpha_);
      path.points.push_back(point_msg(receiver.pos_enu));
      path.points.push_back(point_msg(sat.raycast_hit_enu));
      path.points.push_back(point_msg(sat.display_pos_enu));
      array.markers.push_back(path);
    }
    pub_nlos_paths_->publish(array);
  }

  Eigen::Vector3d sky_dome_center(const ReceiverState& receiver) const
  {
    return sky_dome_follow_receiver_ ? receiver.pos_enu : sky_dome_center_enu_;
  }

  Eigen::Vector3d sky_dome_point(
    const ReceiverState& receiver,
    const double az_rad,
    const double el_rad,
    const double radius) const
  {
    const Eigen::Vector3d dir(
      std::cos(el_rad) * std::sin(az_rad),
      std::cos(el_rad) * std::cos(az_rad),
      std::sin(el_rad));
    return sky_dome_center(receiver) + dir.normalized() * radius;
  }

  void publish_sky_dome(const ReceiverState& receiver)
  {
    if (!enable_sky_dome_visualization_) {
      return;
    }
    publish_delete_all(pub_sky_dome_, receiver.stamp, "sky_dome");

    visualization_msgs::msg::MarkerArray array;
    int id = 1;
    const double radius = satellite_display_radius_m_;

    auto horizon = base_marker(
      receiver.stamp, "sky_dome_horizon", id++, visualization_msgs::msg::Marker::LINE_STRIP);
    horizon.scale.x = 0.08;
    horizon.color = color_rgba(0.55F, 0.70F, 1.0F, 0.65F);
    for (int i = 0; i <= 144; ++i) {
      const double az = static_cast<double>(i) / 144.0 * 2.0 * kPi;
      horizon.points.push_back(point_msg(sky_dome_point(receiver, az, 0.0, radius)));
    }
    array.markers.push_back(horizon);

    for (int ring = 1; ring <= sky_dome_ring_count_; ++ring) {
      const double el = (static_cast<double>(ring) /
        static_cast<double>(sky_dome_ring_count_ + 1)) * (kPi / 2.0);
      auto latitude = base_marker(
        receiver.stamp, "sky_dome_elevation_rings", id++,
        visualization_msgs::msg::Marker::LINE_STRIP);
      latitude.scale.x = 0.04;
      latitude.color = color_rgba(0.45F, 0.55F, 0.75F, 0.38F);
      for (int i = 0; i <= 144; ++i) {
        const double az = static_cast<double>(i) / 144.0 * 2.0 * kPi;
        latitude.points.push_back(point_msg(sky_dome_point(receiver, az, el, radius)));
      }
      array.markers.push_back(latitude);
    }

    for (int meridian = 0; meridian < sky_dome_meridian_count_; ++meridian) {
      const double az = static_cast<double>(meridian) /
        static_cast<double>(sky_dome_meridian_count_) * 2.0 * kPi;
      auto line = base_marker(
        receiver.stamp, "sky_dome_meridians", id++,
        visualization_msgs::msg::Marker::LINE_STRIP);
      line.scale.x = 0.035;
      line.color = color_rgba(0.45F, 0.55F, 0.75F, 0.32F);
      for (int step = 0; step <= 24; ++step) {
        const double el = static_cast<double>(step) / 24.0 * (kPi / 2.0);
        line.points.push_back(point_msg(sky_dome_point(receiver, az, el, radius)));
      }
      array.markers.push_back(line);
    }

    if (sky_dome_show_cardinal_labels_) {
      const std::array<std::pair<double, const char*>, 4> labels = {
        std::make_pair(0.0, "N"),
        std::make_pair(kPi / 2.0, "E"),
        std::make_pair(kPi, "S"),
        std::make_pair(3.0 * kPi / 2.0, "W")};
      for (const auto& label : labels) {
        auto text = base_marker(
          receiver.stamp, "sky_dome_cardinal_labels", id++,
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        text.pose.position = point_msg(
          sky_dome_point(receiver, label.first, 0.0, radius + 2.0));
        text.scale.z = 1.6;
        text.color = color_rgba(0.75F, 0.85F, 1.0F, 0.85F);
        text.text = label.second;
        array.markers.push_back(text);
      }

      auto up = base_marker(
        receiver.stamp, "sky_dome_up_label", id++,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      up.pose.position =
        point_msg(sky_dome_center(receiver) + Eigen::Vector3d(0.0, 0.0, radius + 2.5));
      up.scale.z = 1.5;
      up.color = color_rgba(0.75F, 0.85F, 1.0F, 0.85F);
      up.text = "UP";
      array.markers.push_back(up);
    }

    pub_sky_dome_->publish(array);
  }

  void publish_skyplot(
    const std::vector<SatelliteEval>& evals,
    const builtin_interfaces::msg::Time& stamp)
  {
    publish_delete_all(pub_skyplot_, stamp, "skyplot");
    visualization_msgs::msg::MarkerArray array;
    int id = 1;

    auto ring = base_marker(stamp, "skyplot", id++, visualization_msgs::msg::Marker::LINE_STRIP);
    ring.scale.x = 0.05;
    ring.color = color_rgba(0.65F, 0.65F, 0.65F, 0.8F);
    for (int i = 0; i <= 72; ++i) {
      const double a = static_cast<double>(i) / 72.0 * 2.0 * kPi;
      const Eigen::Vector3d p = skyplot_origin_enu_ +
        Eigen::Vector3d(std::sin(a) * skyplot_radius_m_, std::cos(a) * skyplot_radius_m_, 0.0);
      ring.points.push_back(point_msg(p));
    }
    array.markers.push_back(ring);

    for (const auto& sat : evals) {
      double r = skyplot_radius_m_ * std::clamp((kPi / 2.0 - sat.el_rad) / (kPi / 2.0), 0.0, 1.0);
      if (sat.visibility == VisibilityState::kBelowHorizon) {
        r = skyplot_radius_m_ * 1.08;
      }
      const Eigen::Vector3d p = skyplot_origin_enu_ +
        Eigen::Vector3d(std::sin(sat.az_rad) * r, std::cos(sat.az_rad) * r, 0.0);
      auto point = base_marker(stamp, "skyplot_sats", id++, visualization_msgs::msg::Marker::SPHERE);
      point.pose.position = point_msg(p);
      point.scale.x = 0.5;
      point.scale.y = 0.5;
      point.scale.z = 0.5;
      point.color = color_for_visibility(sat.visibility);
      array.markers.push_back(point);
    }
    pub_skyplot_->publish(array);
  }

  void publish_status_text(
    const std::vector<SatelliteEval>& evals,
    const ReceiverState& receiver,
    const double scenario_time_s,
    const size_t used_measurements)
  {
    publish_delete_all(pub_status_text_, receiver.stamp, "status_text");
    const auto counts = count_visibility(evals);
    visualization_msgs::msg::MarkerArray array;
    auto text = base_marker(receiver.stamp, "status_text", 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position = point_msg(receiver.pos_enu + Eigen::Vector3d(0.0, 0.0, 5.0));
    text.scale.z = 0.9;
    text.color = color_rgba(1.0F, 1.0F, 1.0F, 0.95F);
    std::ostringstream ss;
    ss << "GNSS t=" << std::fixed << std::setprecision(2) << scenario_time_s
       << " used=" << used_measurements
       << " LOS=" << counts.los
       << " NLOS=" << counts.nlos
       << " OCC=" << counts.occluded
       << " DROP=" << counts.dropped
       << " FAULT=" << counts.faulted
       << " BELOW=" << counts.below_horizon
       << " LOW_EL=" << counts.low_elevation
       << "\ndisplay=scaled_local_sky_dome R=" << satellite_display_radius_m_
       << "m orbit~" << kGpsSemiMajorAxisM / 1000.0 << "km"
       << "\ngreen LOS | orange NLOS | yellow LOW_ELEVATION"
       << "\ngray BELOW/OCCLUDED | magenta FAULTED";
    text.text = ss.str();
    array.markers.push_back(text);
    pub_status_text_->publish(array);
  }

  void publish_occlusion_points(
    const std::vector<SatelliteEval>& evals,
    const builtin_interfaces::msg::Time& stamp)
  {
    publish_delete_all(pub_occlusion_points_, stamp, "occlusion_points");
    visualization_msgs::msg::MarkerArray array;
    int id = 1;
    for (const auto& sat : evals) {
      if (!sat.has_raycast_hit) {
        continue;
      }
      auto hit = base_marker(stamp, "occlusion_points", id++, visualization_msgs::msg::Marker::SPHERE);
      hit.pose.position = point_msg(sat.raycast_hit_enu);
      hit.scale.x = 0.4;
      hit.scale.y = 0.4;
      hit.scale.z = 0.4;
      hit.color = color_rgba(1.0F, 0.15F, 0.0F, 0.95F);
      array.markers.push_back(hit);
    }
    pub_occlusion_points_->publish(array);
  }

  void write_csv(
    const std::vector<SatelliteEval>& evals,
    const ReceiverState& state,
    const double scenario_time_s)
  {
    if (!csv_.is_open()) {
      return;
    }
    for (const auto& sat : evals) {
      csv_ << std::fixed << std::setprecision(6)
        << stamp_to_sec(state.stamp) << ","
        << scenario_time_s << ","
        << sat.prn << ","
        << visibility_to_string(sat.visibility) << ","
        << sat.az_rad * kRadToDeg << ","
        << sat.el_rad * kRadToDeg << ","
        << sat.cn0_dbhz << ","
        << sat.psr_extra_bias_m << ","
        << sat.doppler_bias_mps << ",";
      if (sat.has_raycast_hit) {
        csv_ << sat.raycast_hit_enu.x() << ","
          << sat.raycast_hit_enu.y() << ","
          << sat.raycast_hit_enu.z();
      } else {
        csv_ << ",,";
      }
      csv_ << "\n";
    }
  }

  std::string truth_odom_topic_;
  std::string time_source_;
  std::string trigger_topic_;
  std::string scenario_file_;
  std::string ephemeris_source_;
  std::string rinex_nav_file_;
  std::string enabled_constellations_csv_;
  std::string map_cloud_topic_;
  std::string visualization_frame_;
  std::string csv_log_path_;
  std::vector<std::string> enabled_constellations_;
  std::unordered_set<uint32_t> enabled_systems_ = {SYS_GPS};

  Eigen::Vector3d origin_lla_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_enu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d sky_dome_center_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d skyplot_origin_enu_ = Eigen::Vector3d::Zero();

  double measurement_rate_hz_ = 10.0;
  double ephem_rate_hz_ = 1.0;
  double navsat_rate_hz_ = 1.0;
  double min_elevation_rad_ = 10.0 * kDegToRad;
  double pseudorange_noise_std_m_ = 1.0;
  double doppler_noise_std_mps_ = 0.1;
  double receiver_clock_bias_m_ = 0.0;
  double receiver_clock_drift_mps_ = 0.0;
  double truth_buffer_duration_s_ = 5.0;
  double max_truth_interpolation_gap_s_ = 0.05;
  double rinex_ephem_max_age_s_ = 7200.0;
  double occlusion_voxel_size_m_ = 0.5;
  double raycast_max_range_m_ = 120.0;
  double raycast_step_m_ = 0.25;
  double los_cn0_dbhz_ = 45.0;
  double nlos_cn0_dbhz_ = 28.0;
  double low_cn0_threshold_dbhz_ = 30.0;
  double nlos_bias_mean_m_ = 15.0;
  double nlos_bias_std_m_ = 5.0;
  double multipath_amp_m_ = 0.0;
  double multipath_period_s_ = 8.0;
  double satellite_display_radius_m_ = 80.0;
  double signal_ray_width_m_ = 0.08;
  double signal_ray_alpha_ = 0.95;
  double nlos_path_width_m_ = 0.16;
  double nlos_path_alpha_ = 0.95;
  double skyplot_radius_m_ = 8.0;
  int sky_dome_ring_count_ = 3;
  int sky_dome_meridian_count_ = 12;

  uint32_t num_gps_sats_ = 24;
  uint32_t gps_prn_min_ = 1;
  uint32_t gps_prn_max_ = 24;

  bool enable_skymask_ = false;
  bool enable_map_occlusion_ = false;
  bool enable_nlos_ = false;
  bool enable_multipath_ = false;
  bool enable_fault_injection_ = false;
  bool rinex_gps_only_ = true;
  bool fallback_to_synthetic_on_rinex_error_ = true;
  bool enable_visualization_ = false;
  bool enable_sky_dome_visualization_ = true;
  bool sky_dome_show_cardinal_labels_ = true;
  bool sky_dome_follow_receiver_ = false;
  bool enable_csv_log_ = false;
  bool startup_topics_published_ = false;
  bool ephem_published_ = false;

  const std::vector<double> iono_params_ = {
    kDefaultIonoAlpha0, 0.0, 0.0, 0.0,
    kDefaultIonoBeta0, 0.0, 0.0, 0.0,
  };

  ScenarioConfig scenario_;
  TimeSyncBuffer state_buffer_;
  SatelliteConstellation constellation_;
  VisibilityModel visibility_model_;
  std::optional<double> first_epoch_time_s_;

  std::mt19937 rng_;
  std::normal_distribution<double> unit_normal_;
  std::ofstream csv_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr trigger_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_receiver_lla_;
  rclcpp::Publisher<gnss_comm::msg::GnssEphemMsg>::SharedPtr pub_ephem_;
  rclcpp::Publisher<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr pub_glo_ephem_;
  rclcpp::Publisher<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr pub_iono_;
  rclcpp::Publisher<gnss_comm::msg::GnssMeasMsg>::SharedPtr pub_range_meas_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_satellite_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_signal_rays_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_nlos_paths_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_sky_dome_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_skyplot_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_status_text_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_occlusion_points_;
  rclcpp::TimerBase::SharedPtr static_timer_;
  rclcpp::TimerBase::SharedPtr range_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssSimNode>());
  rclcpp::shutdown();
  return 0;
}
