#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rmw/qos_profiles.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <iap/mapping/async_global_mapping.hpp>
#include <iap/mapping/async_sub_mapping.hpp>
#include <iap/mapping/global_mapping_base.hpp>
#include <iap/mapping/sub_mapping_base.hpp>
#include <iap/odometry/async_odometry_estimation.hpp>
#include <iap/odometry/odometry_estimation_base.hpp>
#include <iap/preprocess/cloud_preprocessor.hpp>
#include <iap/util/config.hpp>
#include <iap/util/extension_module.hpp>
#include <iap/util/extension_module_ros2.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/ros_cloud_converter.hpp>
#include <iap/util/time_keeper.hpp>

namespace glim {

class IapRosNode : public rclcpp::Node {
public:
  IapRosNode() : rclcpp::Node("glim_rosnode") {
    declare_parameter<std::string>("config_path", "");
    const std::string config_path = get_parameter("config_path").as_string();

    const std::string resolved_config_path = config_path.empty() ? "config" : config_path;
    GlobalConfig::instance(resolved_config_path, true);

    logger_ = create_module_logger("glim");
    logger_->info("config_path: {}", resolved_config_path);

    preprocessor_ = std::make_unique<CloudPreprocessor>();
    time_keeper_ = std::make_unique<TimeKeeper>();

    const Config config_ros(GlobalConfig::get_config_path("config_ros"));

    enable_local_mapping_ = config_ros.param<bool>("glim_ros", "enable_local_mapping", true);
    enable_global_mapping_ = config_ros.param<bool>("glim_ros", "enable_global_mapping", true);
    keep_raw_points_ = config_ros.param<bool>("glim_ros", "keep_raw_points", false);

    imu_topic_ = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
    points_topic_ = config_ros.param<std::string>("glim_ros", "points_topic", "/points");

    imu_time_offset_ = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
    points_time_offset_ = config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
    configured_acc_scale_ = config_ros.param<double>("glim_ros", "acc_scale", 0.0);
    configured_imu_frame_id_ = config_ros.param<std::string>("glim_ros", "imu_frame_id", "");
    configured_lidar_frame_id_ = config_ros.param<std::string>("glim_ros", "lidar_frame_id", "");
    configured_base_frame_id_ = config_ros.param<std::string>("glim_ros", "base_frame_id", "");

    const Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
    intensity_field_ = sensor_config.param<std::string>("sensors", "intensity_field", "intensity");
    ring_field_ = sensor_config.param<std::string>("sensors", "ring_field", "ring");

    dump_path_ = config_ros.param<std::string>("glim_ros", "dump_path", "/tmp/dump");

    load_core_modules();
    load_extension_modules(config_ros);
    create_core_subscriptions();

    queue_thread_ = std::thread([this] { queue_bridge_loop(); });

    extension_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        for (const auto& ext : extensions_) {
          if (!ext->ok()) {
            logger_->error("extension reported not-ok, shutting down");
            rclcpp::shutdown();
            return;
          }
        }
      });
  }

  ~IapRosNode() override {
    shutdown_pipeline();
  }

  void shutdown_pipeline() {
    if (shutdown_requested_.exchange(true)) {
      return;
    }

    queue_thread_stop_.store(true);
    if (queue_thread_.joinable()) {
      queue_thread_.join();
    }

    if (async_odom_) {
      logger_->info("waiting for odometry estimation");
      async_odom_->join();
    }

    if (async_sub_) {
      logger_->info("waiting for local mapping");
      async_sub_->join();
    }

    if (async_global_) {
      logger_->info("waiting for global mapping");
      async_global_->join();
      async_global_->save(dump_path_);
    }

    auto* global = GlobalConfig::instance();
    if (global) {
      global->dump(dump_path_ + "/config");
    }

    for (const auto& ext : extensions_) {
      ext->at_exit(dump_path_);
    }
  }

private:
  rclcpp::QoS make_qos(const Config& config_ros, const std::string& qos_key, int default_depth) {
    const std::string profile = config_ros.param_nested<std::string>({"glim_ros", qos_key}, "profile", "sensor_data");
    const int depth = config_ros.param_nested<int>({"glim_ros", qos_key}, "depth", default_depth);
    const std::string history = config_ros.param_nested<std::string>({"glim_ros", qos_key}, "history", "system_default");
    const std::string reliability = config_ros.param_nested<std::string>({"glim_ros", qos_key}, "reliability", "system_default");
    const std::string durability = config_ros.param_nested<std::string>({"glim_ros", qos_key}, "durability", "system_default");

    auto apply_overrides = [&](rclcpp::QoS qos) {
      if (history == "keep_all") {
        qos.keep_all();
      } else if (history == "keep_last") {
        qos.keep_last(depth);
      }

      if (reliability == "reliable") {
        qos.reliable();
      } else if (reliability == "best_effort") {
        qos.best_effort();
      }

      if (durability == "transient_local") {
        qos.transient_local();
      } else if (durability == "volatile") {
        qos.durability_volatile();
      }

      return qos;
    };

    if (profile == "system_default") {
      return apply_overrides(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default), rmw_qos_profile_system_default));
    }

    if (profile == "default") {
      return apply_overrides(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default));
    }

    if (profile == "services_default") {
      return apply_overrides(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default), rmw_qos_profile_services_default));
    }

    if (profile == "parameters") {
      return apply_overrides(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameters), rmw_qos_profile_parameters));
    }

    if (profile == "parameter_events") {
      return apply_overrides(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events), rmw_qos_profile_parameter_events));
    }

    if (profile == "sensor_data") {
      return apply_overrides(rclcpp::SensorDataQoS().keep_last(depth));
    }

    return apply_overrides(rclcpp::QoS(rclcpp::KeepLast(depth)));
  }

  void load_core_modules() {
    const Config odom_config(GlobalConfig::get_config_path("config_odometry"));
    const std::string odom_so = odom_config.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
    logger_->info("load {}", odom_so);
    auto odom = OdometryEstimationBase::load_module(odom_so);

    if (!odom) {
      throw std::runtime_error("failed to load odometry module: " + odom_so);
    }

    async_odom_ = std::make_unique<AsyncOdometryEstimation>(odom, odom->requires_imu());

    if (enable_local_mapping_) {
      const Config sub_config(GlobalConfig::get_config_path("config_sub_mapping"));
      const std::string sub_so = sub_config.param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
      logger_->info("load {}", sub_so);
      auto sub = SubMappingBase::load_module(sub_so);
      if (!sub) {
        throw std::runtime_error("failed to load sub mapping module: " + sub_so);
      }
      async_sub_ = std::make_unique<AsyncSubMapping>(sub);
    }

    if (enable_global_mapping_) {
      const Config global_config(GlobalConfig::get_config_path("config_global_mapping"));
      const std::string global_so = global_config.param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
      logger_->info("load {}", global_so);
      auto global = GlobalMappingBase::load_module(global_so);
      if (!global) {
        throw std::runtime_error("failed to load global mapping module: " + global_so);
      }
      async_global_ = std::make_unique<AsyncGlobalMapping>(global);
    }
  }

  void load_extension_modules(const Config& config_ros) {
    const std::vector<std::string> ext_modules =
      config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules", {});

    for (const auto& so_name : ext_modules) {
      logger_->info("load {}", so_name);
      auto module = ExtensionModule::load_module(so_name);
      if (!module) {
        logger_->warn("failed to load extension module {}", so_name);
        continue;
      }

      if (auto ros2_ext = std::dynamic_pointer_cast<ExtensionModuleROS2>(module)) {
        auto subs = ros2_ext->create_subscriptions(*this);
        for (auto& sub : subs) {
          sub->create_subscriber(*this);
          extension_subscriptions_.push_back(sub);
        }
      }

      extensions_.push_back(module);
    }
  }

  void create_core_subscriptions() {
    const Config config_ros(GlobalConfig::get_config_path("config_ros"));
    const auto imu_qos = make_qos(config_ros, "imu_qos", 1000);
    const auto points_qos = make_qos(config_ros, "points_qos", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      imu_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        sync_frame_metadata_from_imu(msg->header.frame_id);

        const double stamp = to_sec(msg->header.stamp) + imu_time_offset_;
        if (!time_keeper_->validate_imu_stamp(stamp)) {
          return;
        }

        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        const double scale = resolved_acc_scale(acc.norm());
        acc *= scale;

        if (async_odom_) {
          async_odom_->insert_imu(stamp, acc, gyro);
        }
        if (async_sub_) {
          async_sub_->insert_imu(stamp, acc, gyro);
        }
        if (async_global_) {
          async_global_->insert_imu(stamp, acc, gyro);
        }
      });

    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_,
      points_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        sync_frame_metadata_from_lidar(msg->header.frame_id);

        auto raw_points = extract_raw_points(*msg, intensity_field_, ring_field_);
        if (!raw_points) {
          return;
        }

        raw_points->stamp = to_sec(msg->header.stamp) + points_time_offset_;
        if (!time_keeper_->process(raw_points)) {
          return;
        }

        auto preprocessed = preprocessor_->preprocess(raw_points);
        if (!preprocessed || preprocessed->size() == 0) {
          return;
        }

        if (!keep_raw_points_) {
          preprocessed->raw_points.reset();
        }

        if (async_odom_) {
          async_odom_->insert_frame(preprocessed);
        }
      });
  }

  void set_meta_param_once(const std::string& key, const std::string& value, bool& written_flag) {
    if (written_flag || value.empty()) {
      return;
    }

    auto* global = GlobalConfig::instance();
    if (!global) {
      return;
    }

    global->override_param("meta", key, value);
    written_flag = true;
    logger_->info("meta/{}={}", key, value);
  }

  void set_base_meta_if_possible() {
    if (base_frame_meta_written_) {
      return;
    }

    if (!configured_base_frame_id_.empty()) {
      set_meta_param_once("base_frame_id", configured_base_frame_id_, base_frame_meta_written_);
      return;
    }

    if (!resolved_imu_frame_id_.empty()) {
      set_meta_param_once("base_frame_id", resolved_imu_frame_id_, base_frame_meta_written_);
    }
  }

  void sync_frame_metadata_from_imu(const std::string& msg_frame_id) {
    std::lock_guard<std::mutex> lock(meta_sync_mutex_);

    const std::string imu_frame_id = configured_imu_frame_id_.empty() ? msg_frame_id : configured_imu_frame_id_;
    if (!imu_frame_id.empty()) {
      resolved_imu_frame_id_ = imu_frame_id;
    }

    set_meta_param_once("imu_frame_id", resolved_imu_frame_id_, imu_frame_meta_written_);
    set_base_meta_if_possible();
  }

  void sync_frame_metadata_from_lidar(const std::string& msg_frame_id) {
    std::lock_guard<std::mutex> lock(meta_sync_mutex_);

    const std::string lidar_frame_id = configured_lidar_frame_id_.empty() ? msg_frame_id : configured_lidar_frame_id_;
    if (!lidar_frame_id.empty()) {
      resolved_lidar_frame_id_ = lidar_frame_id;
    }

    set_meta_param_once("lidar_frame_id", resolved_lidar_frame_id_, lidar_frame_meta_written_);

    if (!configured_base_frame_id_.empty()) {
      set_meta_param_once("base_frame_id", configured_base_frame_id_, base_frame_meta_written_);
    }
  }

  double resolved_acc_scale(double acc_norm) {
    if (configured_acc_scale_ > 0.0) {
      return configured_acc_scale_;
    }

    if (!auto_scale_initialized_) {
      auto_scale_initialized_ = true;
      auto_scale_ = (acc_norm < 3.0) ? 9.80665 : 1.0;
      logger_->info("auto-detected acc_scale={:.5f}", auto_scale_);
    }

    return auto_scale_;
  }

  void queue_bridge_loop() {
    while (!queue_thread_stop_.load()) {
      if (async_odom_) {
        std::vector<EstimationFrame::ConstPtr> estimated;
        std::vector<EstimationFrame::ConstPtr> marginalized;
        async_odom_->get_results(estimated, marginalized);

        if (async_sub_) {
          for (const auto& frame : marginalized) {
            async_sub_->insert_frame(frame);
          }
        }
      }

      if (async_sub_ && async_global_) {
        auto submaps = async_sub_->get_results();
        for (const auto& submap : submaps) {
          async_global_->insert_submap(submap);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

private:
  std::shared_ptr<spdlog::logger> logger_;

  bool enable_local_mapping_ = true;
  bool enable_global_mapping_ = true;
  bool keep_raw_points_ = false;

  std::string imu_topic_;
  std::string points_topic_;
  std::string dump_path_ = "/tmp/dump";
  std::string intensity_field_ = "intensity";
  std::string ring_field_ = "ring";
  std::string configured_imu_frame_id_;
  std::string configured_lidar_frame_id_;
  std::string configured_base_frame_id_;
  std::string resolved_imu_frame_id_;
  std::string resolved_lidar_frame_id_;

  double imu_time_offset_ = 0.0;
  double points_time_offset_ = 0.0;
  double configured_acc_scale_ = 0.0;
  bool auto_scale_initialized_ = false;
  double auto_scale_ = 1.0;

  std::unique_ptr<CloudPreprocessor> preprocessor_;
  std::unique_ptr<TimeKeeper> time_keeper_;

  std::unique_ptr<AsyncOdometryEstimation> async_odom_;
  std::unique_ptr<AsyncSubMapping> async_sub_;
  std::unique_ptr<AsyncGlobalMapping> async_global_;

  std::vector<std::shared_ptr<ExtensionModule>> extensions_;
  std::vector<GenericTopicSubscription::Ptr> extension_subscriptions_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::TimerBase::SharedPtr extension_watchdog_timer_;

  std::mutex meta_sync_mutex_;
  bool imu_frame_meta_written_ = false;
  bool lidar_frame_meta_written_ = false;
  bool base_frame_meta_written_ = false;

  std::thread queue_thread_;
  std::atomic_bool queue_thread_stop_{false};
  std::atomic_bool shutdown_requested_{false};
};

}  // namespace glim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<glim::IapRosNode>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);

  exec.spin();

  node->shutdown_pipeline();

  rclcpp::shutdown();
  return 0;
}
