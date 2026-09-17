// Thin IAP adapter over the public GLIM callbacks declared in
// include/iap/odometry/callbacks.hpp. It reuses GLIM's optimized poses and
// existing CloudDeskewing implementation, but adds an IAP-owned ordered delta
// protocol; it does not copy or alter GLIM's registration pipeline.
#include <iap/common/cloud_deskewing.hpp>
#include <iap/msg/active_lidar_window_delta.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/msg/registered_lidar_frame.hpp>
#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/preprocess/callbacks.hpp>
#include <iap/srv/get_active_lidar_window.hpp>
#include <iap/util/config.hpp>
#include <iap/util/extension_module_ros2.hpp>
#include <iap/util/logging.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace iap {
namespace {

builtin_interfaces::msg::Time toMessageTime(const double stamp_s) {
  builtin_interfaces::msg::Time stamp;
  const double seconds = std::floor(stamp_s);
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(
      std::llround((stamp_s - seconds) * 1.0e9));
  if (stamp.nanosec >= 1000000000U) {
    ++stamp.sec;
    stamp.nanosec -= 1000000000U;
  }
  return stamp;
}

geometry_msgs::msg::Pose toMessagePose(const Eigen::Isometry3d& pose) {
  geometry_msgs::msg::Pose message;
  const Eigen::Quaterniond quaternion(pose.linear());
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = pose.translation().z();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();
  return message;
}

struct FrameSnapshot {
  std::int64_t id = -1;
  double stamp_s = 0.0;
  double scan_end_stamp_s = 0.0;
  std::uint64_t sensor_receipt_steady_ns = 0;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_lidar_imu = Eigen::Isometry3d::Identity();
  glim::RawPoints::ConstPtr raw_points;
  std::shared_ptr<const Eigen::Matrix<double, 8, Eigen::Dynamic>>
      imu_rate_trajectory;
};

struct WindowStateEvent {
  std::uint64_t producer_serial = 0;
  bool complete = false;
  std::vector<FrameSnapshot> frames;
};

bool samePose(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs) {
  const double translation_delta =
      (lhs.translation() - rhs.translation()).norm();
  const Eigen::Quaterniond lhs_q(lhs.linear());
  const Eigen::Quaterniond rhs_q(rhs.linear());
  const double rotation_delta = lhs_q.angularDistance(rhs_q);
  return translation_delta <= 1.0e-5 && rotation_delta <= 1.0e-6;
}

bool hasCertifiedSourceHealth(
    const iap::msg::RegisteredLidarFrame& frame) {
  return frame.source_health_valid &&
      std::isfinite(frame.source_health_stamp_s) &&
      !frame.source_icp_degenerate &&
      std::isfinite(frame.source_icp_rmse) &&
      std::isfinite(frame.source_icp_condition) &&
      std::isfinite(frame.source_icp_gamma_lidar) &&
      std::isfinite(frame.source_lidar_pl_e) &&
      std::isfinite(frame.source_lidar_pl_n) &&
      std::isfinite(frame.source_lidar_pl_u);
}

}  // namespace

class PlannerLocalMapExtension final : public glim::ExtensionModuleROS2 {
 public:
  PlannerLocalMapExtension()
      : logger_(glim::create_module_logger("planner_local_map")) {
    const glim::Config config(
        glim::GlobalConfig::get_config_path("config_ros"));
    current_topic_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "current_topic",
        "/iap/local_map/current_frame");
    delta_topic_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "delta_topic",
        "/iap/local_map/window_delta");
    recovery_service_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "recovery_service",
        "/iap/local_map/get_active_window");
    integrity_topic_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "integrity_topic",
        "/iap/integrity");
    current_hits_map_topic_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "current_hits_map_topic",
        "/iap/local_map/current_hits_map");
    publish_current_hits_map_ = config.param_nested<bool>(
        {"glim_ros", "planner_local_map"}, "publish_current_hits_map", false);
    planner_frame_id_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "planner_frame_id", "map");
    lidar_reference_frame_id_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "lidar_reference_frame_id",
        "iap_lidar_reference");
    frame_contract_id_ = config.param_nested<std::string>(
        {"glim_ros", "planner_local_map"}, "frame_contract_id", "");
    window_rate_hz_ = std::clamp(config.param_nested<double>(
        {"glim_ros", "planner_local_map"}, "window_rate_hz", 2.0),
        0.1, 20.0);
    max_active_keyframes_ = std::max<std::size_t>(1, config.param_nested<int>(
        {"glim_ros", "planner_local_map"}, "max_active_keyframes", 15));
    const auto static_translation = config.param_nested<std::vector<double>>(
        {"glim_ros", "planner_local_map"}, "static_planner_translation_m",
        std::vector<double>{0.0, 0.0, 0.0});
    if (static_translation.size() != 3 ||
        !std::all_of(static_translation.begin(), static_translation.end(),
                     [](const double value) { return std::isfinite(value); })) {
      throw std::runtime_error(
          "planner_local_map.static_planner_translation_m must contain three finite values");
    }
    T_planner_glim_.translation() = Eigen::Vector3d(
        static_translation[0], static_translation[1], static_translation[2]);
    if (frame_contract_id_.empty()) {
      throw std::runtime_error(
          "planner_local_map.frame_contract_id must not be empty");
    }

    update_new_frame_callback_id_ =
        glim::OdometryEstimationCallbacks::on_update_new_frame.add(
        [this](const glim::EstimationFrame::ConstPtr& frame) {
          enqueueCurrent(frame);
        });
    update_keyframes_callback_id_ =
        glim::OdometryEstimationCallbacks::on_update_keyframes.add(
        [this](const std::vector<glim::EstimationFrame::ConstPtr>& frames) {
          enqueueActiveWindow(frames);
        });
    marginalized_keyframes_callback_id_ =
        glim::OdometryEstimationCallbacks::on_marginalized_keyframes.add(
        [this](const std::vector<glim::EstimationFrame::ConstPtr>& frames) {
          enqueueMarginalized(frames);
        });
    raw_points_callback_id_ =
        glim::PreprocessCallbacks::on_raw_points_received.add(
        [this](const glim::RawPoints::ConstPtr& points) {
          rememberRawPoints(points);
        });
    callbacks_registered_.store(true, std::memory_order_release);
    worker_ = std::thread([this] { workerLoop(); });
  }

  ~PlannerLocalMapExtension() override {
    unregisterCallbacks();
    stopWorker();
  }

  void at_exit(const std::string&) override {
    unregisterCallbacks();
    stopWorker();
  }

  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(
      rclcpp::Node& node) override {
    current_publisher_ = node.create_publisher<iap::msg::RegisteredLidarFrame>(
        current_topic_, rclcpp::SensorDataQoS().keep_last(1));
    integrity_subscription_ =
        node.create_subscription<iap::msg::IntegrityReport>(
            integrity_topic_, rclcpp::QoS(256).reliable().transient_local(),
            [this](const iap::msg::IntegrityReport::ConstSharedPtr report) {
              rememberIntegrity(report);
            });
    delta_publisher_ =
        node.create_publisher<iap::msg::ActiveLidarWindowDelta>(
            delta_topic_, rclcpp::QoS(128).reliable());
    recovery_server_ = node.create_service<iap::srv::GetActiveLidarWindow>(
        recovery_service_,
        [this](
            const std::shared_ptr<iap::srv::GetActiveLidarWindow::Request>
                request,
            std::shared_ptr<iap::srv::GetActiveLidarWindow::Response>
                response) { serveActiveWindow(request, response); });
    if (publish_current_hits_map_) {
      current_hits_map_publisher_ =
          node.create_publisher<sensor_msgs::msg::PointCloud2>(
              current_hits_map_topic_, rclcpp::SensorDataQoS().keep_last(1));
    }
    logger_->info(
        "[planner_local_map] current={} current_hits_map={} delta={} recovery={} integrity={} rate={:.1f}Hz contract={}",
        current_topic_, current_hits_map_topic_, delta_topic_,
        recovery_service_, integrity_topic_, window_rate_hz_,
        frame_contract_id_);
    condition_.notify_all();
    return {};
  }

 private:
  static double messageStampSeconds(
      const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  void rememberIntegrity(
      const iap::msg::IntegrityReport::ConstSharedPtr& report) {
    if (!report) return;
    const double stamp_s = messageStampSeconds(report->header.stamp);
    if (!std::isfinite(stamp_s)) return;
    std::lock_guard<std::mutex> lock(integrity_history_mutex_);
    integrity_history_.push_back(*report);
    constexpr std::size_t kIntegrityHistoryCapacity = 2048U;
    while (integrity_history_.size() > kIntegrityHistoryCapacity)
      integrity_history_.pop_front();
  }

  std::optional<iap::msg::IntegrityReport> sourceHealthFor(
      const std::int64_t frame_id, const double stamp_s) const {
    std::lock_guard<std::mutex> lock(integrity_history_mutex_);
    constexpr double kExactStampToleranceS = 1.0e-6;
    for (const auto& candidate : integrity_history_) {
      const double candidate_stamp_s =
          messageStampSeconds(candidate.header.stamp);
      if (candidate.estimation_frame_id == frame_id &&
          std::abs(candidate_stamp_s - stamp_s) <= kExactStampToleranceS)
        return candidate;
    }
    return std::nullopt;
  }

  void unregisterCallbacks() {
    if (!callbacks_registered_.exchange(false, std::memory_order_acq_rel)) {
      return;
    }
    glim::OdometryEstimationCallbacks::on_update_new_frame.remove(
        update_new_frame_callback_id_);
    glim::OdometryEstimationCallbacks::on_update_keyframes.remove(
        update_keyframes_callback_id_);
    glim::OdometryEstimationCallbacks::on_marginalized_keyframes.remove(
        marginalized_keyframes_callback_id_);
    glim::PreprocessCallbacks::on_raw_points_received.remove(
        raw_points_callback_id_);
  }

  void rememberRawPoints(const glim::RawPoints::ConstPtr& points) {
    if (!points || !std::isfinite(points->stamp)) {
      return;
    }
    std::lock_guard<std::mutex> lock(raw_points_mutex_);
    const auto receipt_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    raw_points_history_.push_back({points->stamp, points, receipt_ns});
    // Keep enough scans to cover asynchronous keyframe callbacks without
    // pinning minutes of raw LiDAR storage. At 10 Hz this is about 6.4 s and
    // comfortably exceeds the 15-keyframe active-window handoff delay.
    constexpr std::size_t kRawPointsHistoryCapacity = 64U;
    while (raw_points_history_.size() > kRawPointsHistoryCapacity) {
      raw_points_history_.pop_front();
    }
  }

  glim::RawPoints::ConstPtr findRawPoints(
      const double stamp_s, std::uint64_t* receipt_steady_ns) {
    std::lock_guard<std::mutex> lock(raw_points_mutex_);
    for (auto it = raw_points_history_.rbegin();
         it != raw_points_history_.rend(); ++it) {
      if (std::abs(it->stamp_s - stamp_s) <= 1.0e-6) {
        if (receipt_steady_ns) {
          *receipt_steady_ns = it->receipt_steady_ns;
        }
        return it->points;
      }
    }
    return {};
  }

  std::optional<FrameSnapshot> capture(
      const glim::EstimationFrame::ConstPtr& frame) {
    // Use the public preprocessing callback as the authoritative owner of the
    // original first-hit returns. Some GLIM builds intentionally leave the
    // optional PreprocessedFrame::raw_points slot empty; looking the immutable
    // input up by the exact scan stamp keeps this adapter ABI-neutral.
    std::uint64_t receipt_steady_ns = 0U;
    const auto raw_points = frame && frame->raw_frame
                                ? findRawPoints(frame->raw_frame->stamp,
                                                &receipt_steady_ns)
                                : glim::RawPoints::ConstPtr{};
    if (!frame || !frame->raw_frame || !raw_points || frame->id < 0 ||
        !std::isfinite(frame->stamp) ||
        !frame->T_world_lidar.matrix().allFinite()) {
      const auto count =
          invalid_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
      if (count == 1 || count % 100 == 0) {
        logger_->warn(
            "[planner_local_map] rejected GLIM frame count={} frame={} raw_frame={} raw_points={} id={} stamp_finite={} pose_finite={}",
            count, static_cast<bool>(frame),
            frame && static_cast<bool>(frame->raw_frame),
            frame && frame->raw_frame &&
                static_cast<bool>(raw_points),
            frame ? frame->id : -1,
            frame && std::isfinite(frame->stamp),
            frame && frame->T_world_lidar.matrix().allFinite());
      }
      return std::nullopt;
    }
    FrameSnapshot snapshot;
    snapshot.id = frame->id;
    snapshot.stamp_s = raw_points->stamp;
    snapshot.scan_end_stamp_s = frame->raw_frame->scan_end_time;
    snapshot.sensor_receipt_steady_ns = receipt_steady_ns;
    snapshot.T_map_lidar = T_planner_glim_ * frame->T_world_lidar;
    snapshot.T_lidar_imu = frame->T_lidar_imu;
    snapshot.raw_points = raw_points;
    snapshot.imu_rate_trajectory = std::make_shared<
        const Eigen::Matrix<double, 8, Eigen::Dynamic>>(
        frame->imu_rate_trajectory);
    if (captured_frame_count_.fetch_add(1, std::memory_order_acq_rel) == 0) {
      // The datum is chosen synchronously at the first valid estimator-frame
      // capture.  The latest-wins current queue may later overwrite this
      // frame, but serialization is never allowed to nominate another datum.
      std::int64_t expected_reference = -1;
      reference_frame_id_.compare_exchange_strong(
          expected_reference, snapshot.id, std::memory_order_acq_rel);
      logger_->info(
          "[planner_local_map] captured first GLIM frame id={} raw_points={} imu_trajectory_cols={}",
          snapshot.id, snapshot.raw_points->points.size(),
          snapshot.imu_rate_trajectory->cols());
    }
    return snapshot;
  }

  void enqueueCurrent(const glim::EstimationFrame::ConstPtr& frame) {
    const auto started = std::chrono::steady_clock::now();
    const auto snapshot = capture(frame);
    if (!snapshot) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_current_ = *snapshot;  // latest-wins by design
    }
    condition_.notify_one();
    const double latency_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    callback_latency_ms_.push_back(latency_ms);
    if (callback_latency_ms_.size() >= 100U) {
      std::sort(callback_latency_ms_.begin(), callback_latency_ms_.end());
      logger_->info(
          "[planner_local_map] GLIM callback snapshot latency count=100 p95_ms={:.3f} max_ms={:.3f} budget_ms=0.200",
          callback_latency_ms_[94], callback_latency_ms_.back());
      callback_latency_ms_.clear();
    }
  }

  void enqueueActiveWindow(
      const std::vector<glim::EstimationFrame::ConstPtr>& frames) {
    // Serialize producer-side window callbacks.  GLIM normally invokes these
    // callbacks from one odometry thread, but the ordering contract must not
    // depend on that implementation detail.
    std::lock_guard<std::mutex> producer_lock(producer_window_mutex_);
    const std::uint64_t producer_serial = ++producer_window_serial_;
    const auto retained = producer_snapshots_;
    std::vector<FrameSnapshot> captured;
    const std::size_t first = frames.size() > max_active_keyframes_
        ? frames.size() - max_active_keyframes_ : 0U;
    captured.reserve(frames.size() - first);
    for (std::size_t index = first; index < frames.size(); ++index) {
      const auto& frame = frames[index];
      const auto retained_frame = frame ? retained.find(frame->id)
                                        : retained.end();
      if (retained_frame != retained.end() &&
          frame->T_world_lidar.matrix().allFinite()) {
        auto snapshot = retained_frame->second;
        snapshot.T_map_lidar = T_planner_glim_ * frame->T_world_lidar;
        captured.push_back(std::move(snapshot));
        continue;
      }
      const auto snapshot = capture(frame);
      if (!snapshot) {
        producer_window_complete_ = false;
        enqueueWindowEvent(WindowStateEvent{producer_serial, false, {}});
        return;  // never publish a partial active window as complete
      }
      captured.push_back(*snapshot);
    }
    producer_snapshots_.clear();
    for (const auto& snapshot : captured) {
      producer_snapshots_.emplace(snapshot.id, snapshot);
    }
    producer_window_complete_ = true;
    enqueueWindowEvent(
        WindowStateEvent{producer_serial, true, std::move(captured)});
  }

  void stopWorker() {
    const bool was_stopped = stop_.exchange(true, std::memory_order_acq_rel);
    if (was_stopped) {
      return;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_current_.reset();
    pending_window_events_.clear();
  }

  void enqueueWindowEvent(WindowStateEvent event) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      // Consecutive complete snapshots are full states, so only the newest is
      // needed for the 2 Hz consumer.  A complete/incomplete transition is an
      // ordered safety event and may never be overwritten or skipped.
      if (!pending_window_events_.empty() &&
          pending_window_events_.back().complete == event.complete) {
        pending_window_events_.back() = std::move(event);
      } else {
        pending_window_events_.push_back(std::move(event));
      }
    }
    condition_.notify_one();
  }

  void enqueueMarginalized(
      const std::vector<glim::EstimationFrame::ConstPtr>& frames) {
    std::lock_guard<std::mutex> producer_lock(producer_window_mutex_);
    // A marginalization notification is not a complete active-window
    // capture.  It may refine a known-complete producer state, but it must not
    // heal a preceding capture failure using stale snapshots.
    if (!producer_window_complete_) {
      return;
    }
    for (const auto& frame : frames) {
      if (frame) {
        producer_snapshots_.erase(frame->id);
      }
    }
    std::vector<FrameSnapshot> active;
    active.reserve(producer_snapshots_.size());
    for (const auto& [unused, snapshot] : producer_snapshots_) {
      (void)unused;
      active.push_back(snapshot);
    }
    const std::uint64_t producer_serial = ++producer_window_serial_;
    enqueueWindowEvent(
        WindowStateEvent{producer_serial, true, std::move(active)});
  }

  std::vector<Eigen::Vector4d> deskew(const FrameSnapshot& frame) const {
    const auto& raw = *frame.raw_points;
    if (raw.points.empty()) {
      return {};
    }
    if (raw.times.size() != raw.points.size() ||
        !frame.imu_rate_trajectory ||
        frame.imu_rate_trajectory->cols() == 0) {
      return raw.points;
    }

    std::vector<double> imu_times;
    std::vector<Eigen::Isometry3d> imu_poses;
    imu_times.reserve(frame.imu_rate_trajectory->cols());
    imu_poses.reserve(frame.imu_rate_trajectory->cols());
    for (Eigen::Index column = 0;
         column < frame.imu_rate_trajectory->cols(); ++column) {
      const auto values = frame.imu_rate_trajectory->col(column);
      Eigen::Quaterniond quaternion(
          values[7], values[4], values[5], values[6]);
      if (!std::isfinite(values[0]) || !quaternion.coeffs().allFinite() ||
          quaternion.norm() < 1.0e-9) {
        return raw.points;
      }
      quaternion.normalize();
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = values.segment<3>(1);
      pose.linear() = quaternion.toRotationMatrix();
      imu_times.push_back(values[0]);
      imu_poses.push_back(pose);
    }
    glim::CloudDeskewing deskewer;
    auto result = deskewer.deskew(
        frame.T_lidar_imu.inverse(), imu_times, imu_poses,
        raw.stamp, raw.times, raw.points);
    return result.size() == raw.points.size() ? std::move(result) : raw.points;
  }

  iap::msg::RegisteredLidarFrame makeMessage(
      const FrameSnapshot& frame,
      std::vector<Eigen::Vector4d>* deskewed_points = nullptr) const {
    iap::msg::RegisteredLidarFrame message;
    message.header.stamp = toMessageTime(frame.stamp_s);
    message.header.frame_id = planner_frame_id_;
    message.frame_id = frame.id;
    message.scan_end_stamp_s = frame.scan_end_stamp_s;
    message.sensor_receipt_steady_ns = frame.sensor_receipt_steady_ns;
    message.t_map_lidar = toMessagePose(frame.T_map_lidar);
    message.frame_contract_id = frame_contract_id_;
    message.source_is_map_reference =
        frame.id == reference_frame_id_.load(std::memory_order_acquire);
    if (message.source_is_map_reference) {
      // The first estimator frame defines the planner-map datum.  There is no
      // older frame against which an ICP quality can be measured, so requiring
      // an Integrity report would make the datum permanently uncertifiable.
      // Its pose contribution is exactly zero by definition; scan/deskew error
      // remains part of LocalMotionAssurance's fixed local envelope.
      message.source_health_valid = true;
      message.source_health_stamp_s = frame.stamp_s;
      message.source_icp_degenerate = false;
      message.source_icp_rmse = 0.0;
      message.source_icp_condition = 1.0;
      message.source_icp_gamma_lidar = 1.0;
      message.source_lidar_pl_e = 0.0;
      message.source_lidar_pl_n = 0.0;
      message.source_lidar_pl_u = 0.0;
    } else if (const auto health = sourceHealthFor(frame.id, frame.stamp_s)) {
      const bool finite = std::isfinite(health->lidar_pl_e) &&
          std::isfinite(health->lidar_pl_n) &&
          std::isfinite(health->lidar_pl_u) &&
          std::isfinite(health->icp_rmse) &&
          std::isfinite(health->icp_condition) &&
          std::isfinite(health->icp_gamma_lidar);
      message.source_health_valid = health->lidar_valid && finite &&
          !health->icp_degenerate && health->lidar_pl_e >= 0.0 &&
          health->lidar_pl_n >= 0.0 && health->lidar_pl_u >= 0.0 &&
          health->icp_rmse >= 0.0 && health->icp_condition >= 0.0 &&
          health->icp_gamma_lidar >= 1.0;
      message.source_health_stamp_s =
          messageStampSeconds(health->header.stamp);
      message.source_icp_degenerate = health->icp_degenerate;
      message.source_icp_rmse = health->icp_rmse;
      message.source_icp_condition = health->icp_condition;
      message.source_icp_gamma_lidar = health->icp_gamma_lidar;
      message.source_lidar_pl_e = health->lidar_pl_e;
      message.source_lidar_pl_n = health->lidar_pl_n;
      message.source_lidar_pl_u = health->lidar_pl_u;
    }

    const auto points = deskew(frame);
    if (deskewed_points) {
      *deskewed_points = points;
    }
    sensor_msgs::PointCloud2Modifier modifier(message.deskewed_hits_lidar);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(
        message.deskewed_hits_lidar, "x");
    sensor_msgs::PointCloud2Iterator<float> y(
        message.deskewed_hits_lidar, "y");
    sensor_msgs::PointCloud2Iterator<float> z(
        message.deskewed_hits_lidar, "z");
    for (const auto& point : points) {
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
      ++x;
      ++y;
      ++z;
    }
    message.deskewed_hits_lidar.header.stamp = message.header.stamp;
    message.deskewed_hits_lidar.header.frame_id = lidar_reference_frame_id_;
    return message;
  }

  void publishCurrent(const FrameSnapshot& frame) {
    const auto started = std::chrono::steady_clock::now();
    if (current_publish_count_.load(std::memory_order_relaxed) == 0) {
      logger_->info("[planner_local_map] serializing first current frame id={}",
                    frame.id);
    }
    std::vector<Eigen::Vector4d> points;
    auto message = makeMessage(frame, &points);
    if (current_publisher_) {
      current_publisher_->publish(message);
    }
    if (current_hits_map_publisher_) {
      sensor_msgs::msg::PointCloud2 map_cloud;
      map_cloud.header = message.header;
      sensor_msgs::PointCloud2Modifier modifier(map_cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> x(map_cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> y(map_cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> z(map_cloud, "z");
      for (const auto& point : points) {
        const Eigen::Vector3d map_point =
            frame.T_map_lidar * point.head<3>();
        *x = static_cast<float>(map_point.x());
        *y = static_cast<float>(map_point.y());
        *z = static_cast<float>(map_point.z());
        ++x;
        ++y;
        ++z;
      }
      current_hits_map_publisher_->publish(map_cloud);
    }
    const double latency_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    adapter_publish_latency_ms_.push_back(latency_ms);
    if (adapter_publish_latency_ms_.size() >= 100U) {
      std::sort(adapter_publish_latency_ms_.begin(),
                adapter_publish_latency_ms_.end());
      logger_->info(
          "[planner_local_map] adapter deskew serialize latency count=100 p95_ms={:.3f} max_ms={:.3f} budget_ms=2.000",
          adapter_publish_latency_ms_[94],
          adapter_publish_latency_ms_.back());
      adapter_publish_latency_ms_.clear();
    }
    current_publish_count_.fetch_add(1, std::memory_order_relaxed);
    if (current_publish_count_.load(std::memory_order_relaxed) == 1) {
      logger_->info(
          "[planner_local_map] published first current frame id={} points={} latency_ms={:.3f}",
          frame.id, points.size(), latency_ms);
    }
    if (latency_ms > 2.0) {
      logger_->warn(
          "[planner_local_map] current frame {} deskew+serialize {:.3f} ms exceeds 2 ms budget",
          frame.id, latency_ms);
    }
  }

  void publishWindow(
      std::vector<FrameSnapshot> frames,
      const std::uint64_t producer_serial) {
    std::sort(frames.begin(), frames.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.id < rhs.id;
              });

    iap::msg::ActiveLidarWindowDelta delta;
    delta.header.frame_id = planner_frame_id_;
    delta.header.stamp = frames.empty()
                             ? toMessageTime(0.0)
                             : toMessageTime(frames.back().stamp_s);
    delta.frame_contract_id = frame_contract_id_;
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      delta.base_generation = active_generation_;
      delta.generation = active_generation_ + 1;
      delta.complete = true;

      std::unordered_set<std::int64_t> next_ids;
      for (const auto& frame : frames) {
        next_ids.insert(frame.id);
        const auto found = active_snapshots_.find(frame.id);
        if (found == active_snapshots_.end()) {
          auto message = makeMessage(frame);
          delta.added.push_back(message);
          active_messages_[frame.id] = std::move(message);
        } else if (!hasCertifiedSourceHealth(active_messages_[frame.id])) {
          // The active-window callback can precede the integrity callback for
          // the same estimator frame.  Upgrade that frame atomically once its
          // exact frame-id/stamp report arrives.  Remove+add is an explicit
          // replacement transaction; no adjacent-frame health is borrowed.
          auto refreshed = makeMessage(frame);
          if (hasCertifiedSourceHealth(refreshed)) {
            delta.removed_frame_ids.push_back(frame.id);
            delta.added.push_back(refreshed);
            active_messages_[frame.id] = std::move(refreshed);
          } else if (!samePose(
                         found->second.T_map_lidar, frame.T_map_lidar)) {
            delta.pose_updated_frame_ids.push_back(frame.id);
            delta.updated_t_map_lidar.push_back(
                toMessagePose(frame.T_map_lidar));
            active_messages_[frame.id].t_map_lidar =
                delta.updated_t_map_lidar.back();
          }
        } else if (!samePose(
                       found->second.T_map_lidar, frame.T_map_lidar)) {
          delta.pose_updated_frame_ids.push_back(frame.id);
          delta.updated_t_map_lidar.push_back(
              toMessagePose(frame.T_map_lidar));
          active_messages_[frame.id].t_map_lidar =
              delta.updated_t_map_lidar.back();
        }
      }
      for (const auto& [id, unused] : active_snapshots_) {
        (void)unused;
        if (next_ids.count(id) == 0U) {
          delta.removed_frame_ids.push_back(id);
          active_messages_.erase(id);
        }
      }
      active_snapshots_.clear();
      for (auto& frame : frames) {
        active_snapshots_.emplace(frame.id, std::move(frame));
      }
      active_generation_ = delta.generation;
      active_window_complete_ = true;
      active_producer_serial_ = producer_serial;
    }
    if (delta_publisher_) {
      delta_publisher_->publish(delta);
    }
  }

  void publishIncompleteWindow(const std::uint64_t producer_serial) {
    iap::msg::ActiveLidarWindowDelta delta;
    delta.header.frame_id = planner_frame_id_;
    delta.frame_contract_id = frame_contract_id_;
    delta.complete = false;
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (!active_window_complete_) {
        return;
      }
      delta.base_generation = active_generation_;
      delta.generation = active_generation_ + 1U;
      active_generation_ = delta.generation;
      active_window_complete_ = false;
      active_producer_serial_ = producer_serial;
    }
    if (delta_publisher_) {
      delta_publisher_->publish(delta);
    }
    logger_->warn(
        "[planner_local_map] active window incomplete generation={}; consumer recovery required",
        delta.generation);
  }

  void serveActiveWindow(
      const std::shared_ptr<iap::srv::GetActiveLidarWindow::Request>& request,
      std::shared_ptr<iap::srv::GetActiveLidarWindow::Response>& response) {
    response->header.frame_id = planner_frame_id_;
    response->frame_contract_id = frame_contract_id_;
    if (!request->expected_frame_contract_id.empty() &&
        request->expected_frame_contract_id != frame_contract_id_) {
      response->complete = false;
      response->reason = "frame_contract_mismatch";
      return;
    }
    std::lock_guard<std::mutex> producer_lock(producer_window_mutex_);
    std::lock_guard<std::mutex> lock(active_mutex_);
    response->generation = active_generation_;
    response->complete =
        producer_window_complete_ && active_window_complete_ &&
        active_producer_serial_ == producer_window_serial_;
    response->reason = response->complete
        ? "ok" : "producer_active_window_incomplete";
    if (!response->complete) {
      return;
    }
    std::vector<std::int64_t> ids;
    ids.reserve(active_messages_.size());
    for (const auto& [id, unused] : active_messages_) {
      (void)unused;
      ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    for (const auto id : ids) {
      response->frames.push_back(active_messages_.at(id));
    }
    if (!response->frames.empty()) {
      response->header.stamp = response->frames.back().header.stamp;
    }
  }

  void workerLoop() {
    const auto window_period = std::chrono::duration<double>(
        1.0 / window_rate_hz_);
    auto next_window_publish = std::chrono::steady_clock::now();
    while (!stop_.load(std::memory_order_acquire)) {
      std::optional<FrameSnapshot> current;
      std::optional<WindowStateEvent> window_event;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        const bool urgent_window_event = std::any_of(
            pending_window_events_.begin(), pending_window_events_.end(),
            [](const auto& event) { return !event.complete; });
        const auto wake_deadline = urgent_window_event
            ? std::chrono::steady_clock::now()
            : (!pending_window_events_.empty()
                   ? std::min(next_window_publish,
                              std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(20))
                   : std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(20));
        condition_.wait_until(lock, wake_deadline, [this] {
          return stop_.load(std::memory_order_acquire) ||
                 pending_current_.has_value() ||
                 std::any_of(
                     pending_window_events_.begin(),
                     pending_window_events_.end(),
                     [](const auto& event) { return !event.complete; });
        });
        if (stop_.load(std::memory_order_acquire)) {
          break;
        }
        current.swap(pending_current_);
        const bool has_urgent_window_event = std::any_of(
            pending_window_events_.begin(), pending_window_events_.end(),
            [](const auto& event) { return !event.complete; });
        if (!pending_window_events_.empty() && has_urgent_window_event) {
          window_event = std::move(pending_window_events_.front());
          pending_window_events_.pop_front();
          next_window_publish = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<
                                    std::chrono::steady_clock::duration>(
                                    window_period);
        } else if (std::chrono::steady_clock::now() >= next_window_publish &&
                   !pending_window_events_.empty()) {
          window_event = std::move(pending_window_events_.front());
          pending_window_events_.pop_front();
          next_window_publish = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<
                                    std::chrono::steady_clock::duration>(
                                    window_period);
        }
      }
      if (current) {
        publishCurrent(*current);
      }
      if (window_event) {
        if (window_event->complete) {
          publishWindow(std::move(window_event->frames),
                        window_event->producer_serial);
        } else {
          publishIncompleteWindow(window_event->producer_serial);
        }
      }
    }
  }

  std::shared_ptr<spdlog::logger> logger_;
  std::string current_topic_;
  std::string delta_topic_;
  std::string recovery_service_;
  std::string integrity_topic_;
  std::string current_hits_map_topic_;
  bool publish_current_hits_map_ = false;
  std::string planner_frame_id_;
  std::string lidar_reference_frame_id_;
  std::string frame_contract_id_;
  int update_new_frame_callback_id_ = -1;
  int update_keyframes_callback_id_ = -1;
  int marginalized_keyframes_callback_id_ = -1;
  int raw_points_callback_id_ = -1;
  std::atomic<bool> callbacks_registered_{false};
  double window_rate_hz_ = 2.0;
  std::size_t max_active_keyframes_ = 15U;
  Eigen::Isometry3d T_planner_glim_ = Eigen::Isometry3d::Identity();

  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::optional<FrameSnapshot> pending_current_;
  std::deque<WindowStateEvent> pending_window_events_;

  std::mutex producer_window_mutex_;
  std::uint64_t producer_window_serial_ = 0;
  bool producer_window_complete_ = true;
  std::unordered_map<std::int64_t, FrameSnapshot> producer_snapshots_;
  std::mutex active_mutex_;
  std::uint64_t active_generation_ = 0;
  std::uint64_t active_producer_serial_ = 0;
  bool active_window_complete_ = true;
  std::unordered_map<std::int64_t, FrameSnapshot> active_snapshots_;
  mutable std::atomic<std::int64_t> reference_frame_id_{-1};
  std::unordered_map<std::int64_t, iap::msg::RegisteredLidarFrame>
      active_messages_;

  std::atomic<std::uint64_t> invalid_frame_count_{0};
  std::atomic<std::uint64_t> current_publish_count_{0};
  std::atomic<std::uint64_t> captured_frame_count_{0};
  std::vector<double> callback_latency_ms_;
  std::vector<double> adapter_publish_latency_ms_;
  std::mutex raw_points_mutex_;
  struct RawPointsSnapshot {
    double stamp_s = 0.0;
    glim::RawPoints::ConstPtr points;
    std::uint64_t receipt_steady_ns = 0;
  };
  std::deque<RawPointsSnapshot> raw_points_history_;
  mutable std::mutex integrity_history_mutex_;
  std::deque<iap::msg::IntegrityReport> integrity_history_;
  rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr
      integrity_subscription_;
  rclcpp::Publisher<iap::msg::RegisteredLidarFrame>::SharedPtr
      current_publisher_;
  rclcpp::Publisher<iap::msg::ActiveLidarWindowDelta>::SharedPtr
      delta_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      current_hits_map_publisher_;
  rclcpp::Service<iap::srv::GetActiveLidarWindow>::SharedPtr recovery_server_;
};

}  // namespace iap

extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::PlannerLocalMapExtension();
}
