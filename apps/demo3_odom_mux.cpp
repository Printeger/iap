#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <iap/sim/odom_freshness.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using Odom = nav_msgs::msg::Odometry;

class Demo3OdomMux : public rclcpp::Node {
public:
  Demo3OdomMux() : rclcpp::Node("demo3_odom_mux") {
    truth_odom_topic_ =
        declare_parameter<std::string>("truth_odom_topic", "/sim/drone_0/truth_odom");
    iap_odom_topic_ =
        declare_parameter<std::string>("iap_odom_topic", "/iap_rosnode/odom");
    control_odom_topic_ =
        declare_parameter<std::string>("control_odom_topic", "/demo3/control_odom");
    iap_lock_sample_count_ = declare_parameter<int>("iap_lock_sample_count", 3);
    iap_freshness_sec_ = declare_parameter<double>("iap_freshness_sec", 0.3);
    truth_bootstrap_min_duration_sec_ =
        declare_parameter<double>("truth_bootstrap_min_duration_sec", 0.0);
    start_time_ = now();

    control_odom_pub_ = create_publisher<Odom>(control_odom_topic_, rclcpp::QoS(20));

    truth_odom_sub_ = create_subscription<Odom>(
        truth_odom_topic_,
        rclcpp::QoS(20),
        [this](const Odom::SharedPtr msg) { on_truth_odom(msg); });

    iap_odom_sub_ = create_subscription<Odom>(
        iap_odom_topic_,
        rclcpp::QoS(20),
        [this](const Odom::SharedPtr msg) { on_iap_odom(msg); });

    status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() { log_status(); });

    RCLCPP_INFO(
        get_logger(),
        "mode=truth_bootstrap truth_odom_topic=%s iap_odom_topic=%s control_odom_topic=%s min_truth_duration=%.3fs iap_freshness=%.3fs",
        truth_odom_topic_.c_str(),
        iap_odom_topic_.c_str(),
        control_odom_topic_.c_str(),
        truth_bootstrap_min_duration_sec_,
        iap_freshness_sec_);
  }

private:
  void on_truth_odom(const Odom::SharedPtr& msg) {
    if (!msg) {
      return;
    }

    ++truth_odom_count_;
    last_truth_stamp_ = rclcpp::Time(msg->header.stamp);
    have_truth_stamp_ = true;

    if (have_last_iap_stamp_) {
      const rclcpp::Time node_now = now();
      const auto decision = iap::sim::evaluate_odom_freshness(
          last_iap_stamp_.seconds(),
          false,
          0.0,
          true,
          last_truth_stamp_.seconds(),
          node_now.seconds(),
          iap_freshness_sec_);
      last_iap_age_sec_ = decision.age_sec;
      last_iap_reference_stamp_sec_ = decision.reference_stamp_sec;
      last_iap_reference_ = decision.reference;
      if (!decision.fresh_enough) {
        consecutive_valid_iap_samples_ = 0;
        last_iap_reason_ = iap::sim::OdomFreshnessRejectReason::kStale;
        if (mode_ == Mode::kIapLocked) {
          mode_ = Mode::kTruthBootstrap;
          ++stale_watchdog_count_;
          ++histogram_[iap::sim::histogram_index(last_iap_reason_)];
          RCLCPP_WARN(
              get_logger(),
              "mode=truth_bootstrap reason=iap_stale_watchdog iap_stamp=%.6f node_now=%.6f ref_stamp=%.6f ref_source=%s age=%.3f stale_watchdog_count=%zu",
              last_iap_stamp_.seconds(),
              node_now.seconds(),
              last_iap_reference_stamp_sec_,
              iap::sim::to_string(last_iap_reference_),
              last_iap_age_sec_,
              stale_watchdog_count_);
        }
      }
    }

    if (!logged_first_truth_) {
      logged_first_truth_ = true;
      RCLCPP_INFO(
          get_logger(),
          "received first truth odom stamp=%.6f",
          last_truth_stamp_.seconds());
    }

    if (mode_ == Mode::kTruthBootstrap) {
      control_odom_pub_->publish(*msg);
    }
  }

  void on_iap_odom(const Odom::SharedPtr& msg) {
    if (!msg) {
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    const rclcpp::Time node_now = now();
    const auto decision = iap::sim::evaluate_odom_freshness(
        stamp.seconds(),
        have_last_iap_stamp_,
        last_iap_stamp_.seconds(),
        have_truth_stamp_,
        last_truth_stamp_.seconds(),
        node_now.seconds(),
        iap_freshness_sec_);
    const bool valid_sample = decision.valid_sample;

    ++iap_odom_count_;
    ++histogram_[iap::sim::histogram_index(decision.reason)];

    if (!logged_first_iap_) {
      logged_first_iap_ = true;
      RCLCPP_INFO(
          get_logger(),
          "received first iap odom stamp=%.6f node_now=%.6f ref_stamp=%.6f ref_source=%s age=%.3f reason=%s",
          stamp.seconds(),
          node_now.seconds(),
          decision.reference_stamp_sec,
          iap::sim::to_string(decision.reference),
          decision.age_sec,
          iap::sim::to_string(decision.reason));
    }

    if (valid_sample) {
      ++accepted_iap_count_;
      consecutive_valid_iap_samples_ = consecutive_valid_iap_samples_ + 1;
    } else {
      if (mode_ == Mode::kIapLocked) {
        mode_ = Mode::kTruthBootstrap;
        RCLCPP_WARN(
            get_logger(),
            "mode=truth_bootstrap reason=iap_%s stamp=%.6f node_now=%.6f ref_stamp=%.6f ref_source=%s age=%.3f valid_iap_streak=%d",
            iap::sim::to_string(decision.reason),
            stamp.seconds(),
            node_now.seconds(),
            decision.reference_stamp_sec,
            iap::sim::to_string(decision.reference),
            decision.age_sec,
            consecutive_valid_iap_samples_);
      }
      consecutive_valid_iap_samples_ = 0;
      if ((invalid_iap_sample_count_++ % 20) == 0) {
        RCLCPP_WARN(
            get_logger(),
            "iap odom rejected reason=%s stamp_increasing=%s fresh_enough=%s stamp=%.6f node_now=%.6f ref_stamp=%.6f ref_source=%s age=%.3f hist_accepted=%zu hist_stale=%zu hist_non_increasing=%zu hist_zero_stamp=%zu",
            iap::sim::to_string(decision.reason),
            decision.stamp_increasing ? "true" : "false",
            decision.fresh_enough ? "true" : "false",
            stamp.seconds(),
            node_now.seconds(),
            decision.reference_stamp_sec,
            iap::sim::to_string(decision.reference),
            decision.age_sec,
            histogram_[0],
            histogram_[1],
            histogram_[2],
            histogram_[3]);
      }
    }

    last_iap_stamp_ = stamp;
    have_last_iap_stamp_ = true;
    last_iap_age_sec_ = decision.age_sec;
    last_iap_reference_stamp_sec_ = decision.reference_stamp_sec;
    last_iap_reference_ = decision.reference;
    last_iap_reason_ = decision.reason;

    const double truth_bootstrap_age_sec = (node_now - start_time_).seconds();
    const bool min_truth_duration_elapsed =
        truth_bootstrap_age_sec >= truth_bootstrap_min_duration_sec_;

    if (mode_ == Mode::kTruthBootstrap &&
        min_truth_duration_elapsed &&
        consecutive_valid_iap_samples_ >= iap_lock_sample_count_) {
      mode_ = Mode::kIapLocked;
      RCLCPP_INFO(
          get_logger(),
          "mode=iap_locked valid_samples=%d stamp=%.6f node_now=%.6f ref_stamp=%.6f ref_source=%s age=%.3f truth_bootstrap_age=%.3f",
          consecutive_valid_iap_samples_,
          stamp.seconds(),
          node_now.seconds(),
          decision.reference_stamp_sec,
          iap::sim::to_string(decision.reference),
          decision.age_sec,
          truth_bootstrap_age_sec);
    }

    if (mode_ == Mode::kIapLocked) {
      control_odom_pub_->publish(*msg);
    }
  }

private:
  void log_status() {
    const char* mode_name =
        mode_ == Mode::kTruthBootstrap ? "truth_bootstrap" : "iap_locked";

    if (have_last_iap_stamp_) {
      const rclcpp::Time node_now = now();
      const auto decision = iap::sim::evaluate_odom_freshness(
          last_iap_stamp_.seconds(),
          false,
          0.0,
          have_truth_stamp_,
          last_truth_stamp_.seconds(),
          node_now.seconds(),
          iap_freshness_sec_);
      const double elapsed_sec = std::max((node_now - start_time_).seconds(), 1.0e-6);
      const double iap_rx_rate_hz = static_cast<double>(iap_odom_count_) / elapsed_sec;
      const double iap_accept_rate_hz = static_cast<double>(accepted_iap_count_) / elapsed_sec;
      RCLCPP_INFO(
          get_logger(),
          "status mode=%s truth_count=%zu iap_count=%zu accepted_iap=%zu stale_watchdog_count=%zu iap_rx_rate_hz=%.2f iap_accept_rate_hz=%.2f valid_iap_streak=%d iap_age=%.3f iap_stamp=%.6f node_now=%.6f ref_stamp=%.6f ref_source=%s last_reason=%s hist_accepted=%zu hist_stale=%zu hist_non_increasing=%zu hist_zero_stamp=%zu",
          mode_name,
          truth_odom_count_,
          iap_odom_count_,
          accepted_iap_count_,
          stale_watchdog_count_,
          iap_rx_rate_hz,
          iap_accept_rate_hz,
          consecutive_valid_iap_samples_,
          decision.age_sec,
          last_iap_stamp_.seconds(),
          node_now.seconds(),
          decision.reference_stamp_sec,
          iap::sim::to_string(decision.reference),
          iap::sim::to_string(decision.reason),
          histogram_[0],
          histogram_[1],
          histogram_[2],
          histogram_[3]);
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "status mode=%s truth_count=%zu iap_count=%zu valid_iap_streak=%d iap_age=n/a",
        mode_name,
        truth_odom_count_,
        iap_odom_count_,
        consecutive_valid_iap_samples_);
  }

  enum class Mode {
    kTruthBootstrap,
    kIapLocked,
  };

  Mode mode_ = Mode::kTruthBootstrap;

  std::string truth_odom_topic_;
  std::string iap_odom_topic_;
  std::string control_odom_topic_;
  int iap_lock_sample_count_ = 3;
  double iap_freshness_sec_ = 0.3;
  double truth_bootstrap_min_duration_sec_ = 0.0;
  int consecutive_valid_iap_samples_ = 0;
  std::size_t truth_odom_count_ = 0;
  std::size_t iap_odom_count_ = 0;
  std::size_t accepted_iap_count_ = 0;
  std::size_t invalid_iap_sample_count_ = 0;
  std::size_t stale_watchdog_count_ = 0;
  std::size_t histogram_[4] = {0, 0, 0, 0};
  bool logged_first_truth_ = false;
  bool logged_first_iap_ = false;
  bool have_truth_stamp_ = false;
  bool have_last_iap_stamp_ = false;
  double last_iap_age_sec_ = std::numeric_limits<double>::quiet_NaN();
  double last_iap_reference_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  iap::sim::OdomFreshnessReference last_iap_reference_ =
      iap::sim::OdomFreshnessReference::kNodeNowNoTruth;
  iap::sim::OdomFreshnessRejectReason last_iap_reason_ =
      iap::sim::OdomFreshnessRejectReason::kStale;
  rclcpp::Time last_truth_stamp_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_iap_stamp_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time start_time_{0, 0, RCL_SYSTEM_TIME};

  rclcpp::Publisher<Odom>::SharedPtr control_odom_pub_;
  rclcpp::Subscription<Odom>::SharedPtr truth_odom_sub_;
  rclcpp::Subscription<Odom>::SharedPtr iap_odom_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Demo3OdomMux>());
  rclcpp::shutdown();
  return 0;
}
