#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gnss_comm/gnss_constant.hpp>
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

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
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

}  // namespace

class GnssSimNode : public rclcpp::Node
{
public:
  GnssSimNode()
  : Node("gnss_sim_node"),
    rng_(static_cast<uint32_t>(declare_parameter<int64_t>("random_seed", 20260429))),
    unit_normal_(0.0, 1.0)
  {
    truth_odom_topic_ = declare_parameter<std::string>(
      "truth_odom_topic", "/sim/drone_0/truth_odom");
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

    origin_ecef_ = gnss_comm::geo2ecef(origin_lla_);
    R_ecef_enu_ = gnss_comm::geo2rotation(origin_lla_);

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

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GnssSimNode::on_odom, this, std::placeholders::_1));

    const double static_rate_hz = std::max(ephem_rate_hz_, navsat_rate_hz_);
    static_timer_ = create_wall_timer(
      period_from_rate(static_rate_hz),
      std::bind(&GnssSimNode::publish_static_slow_topics, this));
    range_timer_ = create_wall_timer(
      period_from_rate(measurement_rate_hz_),
      std::bind(&GnssSimNode::publish_range_epoch, this));

    RCLCPP_INFO(
      get_logger(),
      "gnss_sim_node started: truth_odom_topic=%s origin_lla=[%.7f, %.7f, %.2f]",
      truth_odom_topic_.c_str(), origin_lla_.x(), origin_lla_.y(), origin_lla_.z());
  }

private:
  struct ReceiverState
  {
    builtin_interfaces::msg::Time stamp;
    Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();
  };

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

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = state;
      has_state_ = true;
    }

    if (!startup_topics_published_) {
      publish_static_slow_topics();
      startup_topics_published_ = true;
    }
  }

  bool get_latest_state(ReceiverState& state)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_) {
      return false;
    }
    state = latest_state_;
    return true;
  }

  void ensure_ephemerides(const gnss_comm::gtime_t& gpst_time)
  {
    if (!ephems_.empty()) {
      const double age = std::abs(gnss_comm::time_diff(gpst_time, ephems_.front()->toe));
      if (age < 3600.0) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Refreshing simulated GPS ephemerides after %.1f s", age);
      ephems_.clear();
    }

    uint32_t week = 0;
    const double tow = gnss_comm::time2gpst(gpst_time, &week);

    ephems_.reserve(24);
    for (uint32_t prn = 1; prn <= 24; ++prn) {
      const uint32_t plane = (prn - 1) / 4;
      const uint32_t slot = (prn - 1) % 4;
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

  void publish_static_slow_topics()
  {
    ReceiverState state;
    if (!get_latest_state(state)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for truth odometry before publishing GNSS startup topics");
      return;
    }

    const gnss_comm::gtime_t utc_time = stamp_to_utc_time(state.stamp);
    const gnss_comm::gtime_t gpst_time = gnss_comm::utc2gpst(utc_time);
    ensure_ephemerides(gpst_time);

    publish_receiver_lla(state);
    publish_iono_params(state.stamp);
    for (const auto& eph : ephems_) {
      pub_ephem_->publish(gnss_comm::ephem2msg(eph));
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
    msg.parameters = {
      kDefaultIonoAlpha0, 0.0, 0.0, 0.0,
      kDefaultIonoBeta0, 0.0, 0.0, 0.0,
    };
    pub_iono_->publish(msg);
  }

  void publish_range_epoch()
  {
    ReceiverState state;
    if (!get_latest_state(state)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for truth odometry before publishing GNSS range measurements");
      return;
    }
    if (!ephem_published_) {
      publish_static_slow_topics();
      if (!ephem_published_) {
        return;
      }
    }

    const gnss_comm::gtime_t utc_time = stamp_to_utc_time(state.stamp);
    const gnss_comm::gtime_t gpst_rx_time = gnss_comm::utc2gpst(utc_time);
    ensure_ephemerides(gpst_rx_time);

    const Eigen::Vector3d receiver_ecef =
      origin_ecef_ + R_ecef_enu_ * state.pos_enu;
    const Eigen::Vector3d receiver_vel_ecef = R_ecef_enu_ * state.vel_enu;
    const Eigen::Vector3d receiver_lla = gnss_comm::ecef2geo(receiver_ecef);

    std::vector<gnss_comm::ObsPtr> obs_list;
    obs_list.reserve(ephems_.size());

    for (const auto& eph : ephems_) {
      double svdt_rx = 0.0;
      const Eigen::Vector3d sat_rx = gnss_comm::eph2pos(gpst_rx_time, eph, &svdt_rx);
      if (!sat_rx.allFinite()) {
        continue;
      }

      const double rho0 = (sat_rx - receiver_ecef).norm();
      const gnss_comm::gtime_t tx_time =
        gnss_comm::time_add(gpst_rx_time, -rho0 / LIGHT_SPEED);

      double svdt = 0.0;
      double svddt = 0.0;
      const Eigen::Vector3d sat_pos = gnss_comm::eph2pos(tx_time, eph, &svdt);
      const Eigen::Vector3d sat_vel = gnss_comm::eph2vel(tx_time, eph, &svddt);
      if (!sat_pos.allFinite() || !sat_vel.allFinite()) {
        continue;
      }

      double azel[2] = {0.0, 0.0};
      gnss_comm::sat_azel(receiver_ecef, sat_pos, azel);
      if (azel[1] < min_elevation_rad_) {
        continue;
      }

      const Eigen::Vector3d dp = receiver_ecef - sat_pos;
      const double rho = dp.norm();
      if (rho <= 1.0 || !std::isfinite(rho)) {
        continue;
      }
      const Eigen::Vector3d los_receiver_minus_sat = dp / rho;

      const double sagnac = EARTH_OMG_GPS / LIGHT_SPEED *
        (sat_pos.x() * receiver_ecef.y() - sat_pos.y() * receiver_ecef.x());
      const double ion_delay = gnss_comm::calculate_ion_delay(
        gpst_rx_time, iono_params_, receiver_lla, azel);
      const double trop_delay = gnss_comm::calculate_trop_delay(
        gpst_rx_time, receiver_lla, azel);

      const double pr_noise = pseudorange_noise_std_m_ * unit_normal_(rng_);
      const double psr_raw =
        rho + sagnac + receiver_clock_bias_m_ + ion_delay + trop_delay +
        eph->tgd[0] * LIGHT_SPEED - svdt * LIGHT_SPEED + pr_noise;

      const Eigen::Vector3d rel_vel = receiver_vel_ecef - sat_vel;
      const double sagnac_dot = EARTH_OMG_GPS / LIGHT_SPEED *
        (sat_vel.x() * receiver_ecef.y() + sat_pos.x() * receiver_vel_ecef.y() -
         sat_vel.y() * receiver_ecef.x() - sat_pos.y() * receiver_vel_ecef.x());
      const double dop_noise = doppler_noise_std_mps_ * unit_normal_(rng_);
      const double range_rate_raw =
        los_receiver_minus_sat.dot(rel_vel) + sagnac_dot +
        receiver_clock_drift_mps_ - svddt * LIGHT_SPEED + dop_noise;
      const double dopp_hz = -range_rate_raw * FREQ1 / LIGHT_SPEED;

      auto obs = std::make_shared<gnss_comm::Obs>();
      obs->time = gpst_rx_time;
      obs->sat = eph->sat;
      obs->freqs = {FREQ1};
      obs->CN0 = {45.0};
      obs->LLI = {0};
      obs->code = {CODE_L1C};
      obs->psr = {psr_raw};
      obs->psr_std = {std::max(pseudorange_noise_std_m_, 0.05)};
      obs->cp = {0.0};
      obs->cp_std = {0.0};
      obs->dopp = {dopp_hz};
      obs->dopp_std = {std::max(doppler_noise_std_mps_, 0.01) * FREQ1 / LIGHT_SPEED};
      obs->status = {1};
      obs_list.push_back(obs);
    }

    if (obs_list.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GNSS visible satellite count is %zu < 4; skip empty/weak epoch",
        obs_list.size());
      return;
    }

    auto msg = gnss_comm::meas2msg(obs_list);
    pub_range_meas_->publish(msg);
  }

  std::string truth_odom_topic_;
  Eigen::Vector3d origin_lla_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_enu_ = Eigen::Matrix3d::Identity();

  double measurement_rate_hz_ = 10.0;
  double ephem_rate_hz_ = 1.0;
  double navsat_rate_hz_ = 1.0;
  double min_elevation_rad_ = 10.0 * kDegToRad;
  double pseudorange_noise_std_m_ = 1.0;
  double doppler_noise_std_mps_ = 0.1;
  double receiver_clock_bias_m_ = 0.0;
  double receiver_clock_drift_mps_ = 0.0;
  const std::vector<double> iono_params_ = {
    kDefaultIonoAlpha0, 0.0, 0.0, 0.0,
    kDefaultIonoBeta0, 0.0, 0.0, 0.0,
  };

  std::mutex state_mutex_;
  ReceiverState latest_state_;
  bool has_state_ = false;
  bool startup_topics_published_ = false;
  bool ephem_published_ = false;

  std::vector<gnss_comm::EphemPtr> ephems_;
  std::mt19937 rng_;
  std::normal_distribution<double> unit_normal_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_receiver_lla_;
  rclcpp::Publisher<gnss_comm::msg::GnssEphemMsg>::SharedPtr pub_ephem_;
  rclcpp::Publisher<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr pub_glo_ephem_;
  rclcpp::Publisher<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr pub_iono_;
  rclcpp::Publisher<gnss_comm::msg::GnssMeasMsg>::SharedPtr pub_range_meas_;
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
