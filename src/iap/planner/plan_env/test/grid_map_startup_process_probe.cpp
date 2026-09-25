#include <plan_env/grid_map.h>

#include <iap/local_map/active_window_delta_policy.hpp>
#include <iap/msg/active_lidar_window_delta.hpp>
#include <iap/msg/registered_lidar_frame.hpp>
#include <iap/srv/get_active_lidar_window.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

constexpr char kCurrentTopic[] = "/iap/test/local_map/current_frame";
constexpr char kDeltaTopic[] = "/iap/test/local_map/window_delta";
constexpr char kRecoveryService[] = "/iap/test/local_map/get_active_window";
constexpr char kFrameContract[] = "grid_map_startup_process_v1";

iap::msg::RegisteredLidarFrame makeFrame(const std::int64_t frame_id) {
  iap::msg::RegisteredLidarFrame frame;
  frame.header.frame_id = "map";
  frame.header.stamp.sec = 10 + static_cast<std::int32_t>(frame_id);
  frame.frame_id = frame_id;
  frame.scan_end_stamp_s = static_cast<double>(frame.header.stamp.sec);
  frame.sensor_receipt_steady_ns = 1U;
  frame.t_map_lidar.orientation.w = 1.0;
  frame.frame_contract_id = kFrameContract;
  frame.deskewed_hits_lidar.header = frame.header;
  frame.deskewed_hits_lidar.header.frame_id = "iap_lidar_reference";
  sensor_msgs::PointCloud2Modifier modifier(frame.deskewed_hits_lidar);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1U);
  sensor_msgs::PointCloud2Iterator<float> x(frame.deskewed_hits_lidar, "x");
  sensor_msgs::PointCloud2Iterator<float> y(frame.deskewed_hits_lidar, "y");
  sensor_msgs::PointCloud2Iterator<float> z(frame.deskewed_hits_lidar, "z");
  *x = 1.0F;
  *y = 0.0F;
  *z = 0.0F;
  frame.beam_evidence_complete = false;
  frame.source_is_map_reference = true;
  frame.source_health_valid = true;
  frame.source_health_stamp_s = frame.scan_end_stamp_s;
  frame.source_icp_degenerate = false;
  frame.source_icp_rmse = 0.0;
  frame.source_icp_condition = 1.0;
  frame.source_icp_gamma_lidar = 1.0;
  return frame;
}

enum class ProducerMode {
  NORMAL,
  RESPONSE_BEHIND_OBSERVED,
  FIRST_RESPONSE_TIMEOUT,
};

class Producer final : public rclcpp::Node {
 public:
  explicit Producer(const ProducerMode mode)
      : Node("grid_map_startup_test_producer"),
        mode_(mode),
        active_frame_(makeFrame(1)) {
    current_publisher_ = create_publisher<iap::msg::RegisteredLidarFrame>(
        kCurrentTopic, rclcpp::SensorDataQoS().keep_last(1));
    delta_publisher_ = create_publisher<iap::msg::ActiveLidarWindowDelta>(
        kDeltaTopic, rclcpp::QoS(128).reliable());
    service_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    recovery_service_ = create_service<iap::srv::GetActiveLidarWindow>(
        kRecoveryService,
        [this](
            const std::shared_ptr<iap::srv::GetActiveLidarWindow::Request>
                request,
            std::shared_ptr<iap::srv::GetActiveLidarWindow::Response>
                response) {
          const unsigned int request_number =
              request_count_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
          std::uint64_t response_generation = 0U;
          iap::msg::RegisteredLidarFrame response_frame;
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            response_generation = generation_;
            response_frame = active_frame_;
          }
          recovery_started_.store(true, std::memory_order_release);
          std::cerr << "TRANSITION recovery_request serial="
                    << request_number << " base=unknown response_generation="
                    << response_generation << '\n';
          if (mode_ == ProducerMode::FIRST_RESPONSE_TIMEOUT &&
              request_number == 1U) {
            std::cerr << "TRANSITION recovery_response_delayed serial=1\n";
            std::this_thread::sleep_for(1250ms);
          } else if (mode_ == ProducerMode::RESPONSE_BEHIND_OBSERVED &&
                     request_number == 1U) {
            std::this_thread::sleep_for(650ms);
          }
          response->header.frame_id = "map";
          response->frame_contract_id = kFrameContract;
          response->generation = response_generation;
          response->complete =
              request->expected_frame_contract_id == kFrameContract;
          response->reason = response->complete ? "ok" : "contract_mismatch";
          if (response->complete) {
            response->frames.push_back(response_frame);
          }
          std::cerr << "TRANSITION recovery_response serial="
                    << request_number << " generation="
                    << response_generation << '\n';
        }, rclcpp::ServicesQoS(), service_callback_group_);
    current_timer_ = create_wall_timer(50ms, [this]() {
      if (!matched_.load(std::memory_order_acquire) &&
          current_publisher_->get_subscription_count() > 0U) {
        matched_.store(true, std::memory_order_release);
        std::cerr << "TRANSITION current_endpoint_matched\n";
      }
      current_publisher_->publish(makeFrame(next_frame_id_++));
      if (!current_published_.exchange(true, std::memory_order_acq_rel)) {
        std::cerr << "TRANSITION first_current_published\n";
      }
    });
    delta_timer_ = create_wall_timer(500ms, [this]() {
      iap::msg::ActiveLidarWindowDelta delta;
      delta.header.frame_id = "map";
      delta.frame_contract_id = kFrameContract;
      delta.complete = true;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++window_update_attempts_;
        if (generation_ == 0U) {
          delta.added.push_back(active_frame_);
        } else if (mode_ != ProducerMode::NORMAL &&
                   generation_ == 1U &&
                   recovery_started_.load(std::memory_order_acquire)) {
          active_frame_.t_map_lidar.position.y = 0.25;
          delta.pose_updated_frame_ids.push_back(active_frame_.frame_id);
          delta.updated_t_map_lidar.push_back(active_frame_.t_map_lidar);
        }
        if (!iap::local_map::activeWindowDeltaChangesState(
                delta, active_window_complete_)) {
          ++suppressed_empty_updates_;
          return;
        }
        delta.base_generation = generation_;
        delta.generation = generation_ + 1U;
        generation_ = delta.generation;
        active_window_complete_ = true;
      }
      delta_publisher_->publish(delta);
      ++published_delta_count_;
      if (!delta_published_.exchange(true, std::memory_order_acq_rel)) {
        std::cerr << "TRANSITION first_delta base="
                  << delta.base_generation << " generation="
                  << delta.generation << '\n';
      }
    });
    std::cerr << "TRANSITION recovery_service_ready\n";
  }

  ~Producer() override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::cerr << "RESULT producer generation=" << generation_
              << " published_deltas=" << published_delta_count_
              << " update_attempts=" << window_update_attempts_
              << " unchanged_attempts=" << suppressed_empty_updates_
              << '\n';
  }

 private:
  ProducerMode mode_ = ProducerMode::NORMAL;
  iap::msg::RegisteredLidarFrame active_frame_;
  std::atomic<unsigned int> request_count_{0U};
  std::atomic<bool> matched_{false};
  std::atomic<bool> delta_published_{false};
  std::atomic<bool> current_published_{false};
  std::atomic<bool> recovery_started_{false};
  std::mutex state_mutex_;
  std::uint64_t generation_ = 0U;
  std::uint64_t window_update_attempts_ = 0U;
  std::uint64_t suppressed_empty_updates_ = 0U;
  std::uint64_t published_delta_count_ = 0U;
  bool active_window_complete_ = false;
  std::int64_t next_frame_id_ = 2;
  rclcpp::Publisher<iap::msg::RegisteredLidarFrame>::SharedPtr
      current_publisher_;
  rclcpp::Publisher<iap::msg::ActiveLidarWindowDelta>::SharedPtr
      delta_publisher_;
  rclcpp::Service<iap::srv::GetActiveLidarWindow>::SharedPtr recovery_service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::TimerBase::SharedPtr current_timer_;
  rclcpp::TimerBase::SharedPtr delta_timer_;
};

rclcpp::NodeOptions consumerOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("grid_map/resolution", 1.0),
      rclcpp::Parameter("grid_map/map_size_x", 8.0),
      rclcpp::Parameter("grid_map/map_size_y", 8.0),
      rclcpp::Parameter("grid_map/map_size_z", 8.0),
      rclcpp::Parameter("grid_map/local_update_range_x", 4.0),
      rclcpp::Parameter("grid_map/local_update_range_y", 4.0),
      rclcpp::Parameter("grid_map/local_update_range_z", 4.0),
      rclcpp::Parameter("grid_map/obstacles_inflation", 0.0),
      rclcpp::Parameter("grid_map/skip_pixel", 1),
      rclcpp::Parameter("grid_map/ground_height", -4.0),
      rclcpp::Parameter("grid_map/virtual_ceil_height", -1.0),
      rclcpp::Parameter("grid_map/frame_id", std::string("map")),
      rclcpp::Parameter("grid_map/registered_lidar_window_enabled", true),
      rclcpp::Parameter("grid_map/registered_frame_contract_id",
                        std::string(kFrameContract)),
      rclcpp::Parameter("grid_map/registered_current_topic",
                        std::string(kCurrentTopic)),
      rclcpp::Parameter("grid_map/registered_delta_topic",
                        std::string(kDeltaTopic)),
      rclcpp::Parameter("grid_map/registered_recovery_service",
                        std::string(kRecoveryService)),
  });
  return options;
}

int runConsumer(const double timeout_s) {
  auto node = std::make_shared<rclcpp::Node>(
      "grid_map_startup_test_consumer", consumerOptions());
  auto map = std::make_shared<GridMap>();
  map->initMap(node);
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  std::thread spin_thread([&]() { executor.spin(); });

  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
  std::shared_ptr<const FrozenOccupancyEpoch> epoch;
  while (std::chrono::steady_clock::now() < deadline) {
    epoch = map->captureFrozenOccupancyEpoch();
    if (epoch && epoch->generation > 0U &&
        epoch->active_window_generation > 0U) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  if (!epoch) {
    std::cerr << "RESULT snapshot=null generation=0 active_generation=0\n";
    return 1;
  }
  std::cerr << "RESULT snapshot=present generation=" << epoch->generation
            << " active_generation=" << epoch->active_window_generation
            << " current_frame_id=" << epoch->current_frame_id;
  if (epoch->active_window_obstacle_sources &&
      !epoch->active_window_obstacle_sources->empty()) {
    std::cerr << " active_source_y="
              << epoch->active_window_obstacle_sources->front()
                     .metadata.T_map_lidar.translation().y();
  }
  std::cerr << '\n';
  return epoch->generation > 0U && epoch->active_window_generation > 0U
      ? 0 : 1;
}

int runProducer(const ProducerMode mode) {
  auto node = std::make_shared<Producer>(mode);
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 2;
  if (argc >= 2 && std::string(argv[1]) == "consumer") {
    const double timeout_s = argc >= 3 ? std::stod(argv[2]) : 7.0;
    result = runConsumer(timeout_s);
  } else if (argc >= 2 && std::string(argv[1]) == "producer") {
    ProducerMode mode = ProducerMode::NORMAL;
    if (argc >= 3 && std::string(argv[2]) == "response-behind-observed") {
      mode = ProducerMode::RESPONSE_BEHIND_OBSERVED;
    } else if (argc >= 3 &&
               std::string(argv[2]) == "first-response-timeout") {
      mode = ProducerMode::FIRST_RESPONSE_TIMEOUT;
    }
    result = runProducer(mode);
  } else {
    std::cerr << "usage: grid_map_startup_process_probe "
                 "consumer [timeout_s] | producer "
                 "[response-behind-observed|first-response-timeout]\n";
  }
  rclcpp::shutdown();
  return result;
}
