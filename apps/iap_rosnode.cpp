#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
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

#include <gtsam_points/config.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

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
#include <iap/util/run_log_manager.hpp>
#include <iap/util/ros_cloud_converter.hpp>
#include <iap/util/time_keeper.hpp>

namespace glim {

class IapRosNode : public rclcpp::Node {
public:
  IapRosNode() : rclcpp::Node("glim_rosnode") {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("imu_topic", "");
    declare_parameter<std::string>("points_topic", "");
    const std::string config_path = get_parameter("config_path").as_string();
    const char* env_config_path = std::getenv("IAP_CONFIG_PATH");
    const std::string resolved_config_path =
      !config_path.empty() ? config_path : ((env_config_path && *env_config_path) ? std::string(env_config_path) : "config");
    RCLCPP_INFO(get_logger(), "iap init stage=config path=%s", resolved_config_path.c_str());
    GlobalConfig::instance(resolved_config_path, true);
    auto& run_logs = RunLogManager::initialize("iap_rosnode", resolved_config_path);
    run_logs.write_run_info();
    GlobalConfig::instance()->dump(run_logs.metadata_path("config").string());
    RCLCPP_INFO(get_logger(), "iap init stage=run_logs ready dir=%s", run_logs.run_dir().string().c_str());

    logger_ = create_module_logger("glim");
    glim::set_default_logger(logger_);
    logger_->info("config_path: {}", resolved_config_path);
    logger_->info("run_dir: {}", run_logs.run_dir().string());

  #ifdef GTSAM_POINTS_USE_CUDA
    gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
    logger_->info("registered GPU linearization hook");
  #endif

    preprocessor_ = std::make_unique<CloudPreprocessor>();
    time_keeper_ = std::make_unique<TimeKeeper>();

    const Config config_ros(GlobalConfig::get_config_path("config_ros"));

    enable_local_mapping_ = config_ros.param<bool>("glim_ros", "enable_local_mapping", true);
    enable_global_mapping_ = config_ros.param<bool>("glim_ros", "enable_global_mapping", true);
    keep_raw_points_ = config_ros.param<bool>("glim_ros", "keep_raw_points", false);

    imu_topic_ = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
    points_topic_ = config_ros.param<std::string>("glim_ros", "points_topic", "/points");
    const std::string imu_topic_override = get_parameter("imu_topic").as_string();
    const std::string points_topic_override = get_parameter("points_topic").as_string();
    if (!imu_topic_override.empty()) {
      imu_topic_ = imu_topic_override;
    }
    if (!points_topic_override.empty()) {
      points_topic_ = points_topic_override;
    }

    imu_time_offset_ = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
    points_time_offset_ = config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
    configured_acc_scale_ = config_ros.param<double>("glim_ros", "acc_scale", 0.0);
    configured_imu_frame_id_ = config_ros.param<std::string>("glim_ros", "imu_frame_id", "");
    configured_lidar_frame_id_ = config_ros.param<std::string>("glim_ros", "lidar_frame_id", "");
    configured_base_frame_id_ = config_ros.param<std::string>("glim_ros", "base_frame_id", "");

    const Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
    intensity_field_ = sensor_config.param<std::string>("sensors", "intensity_field", "intensity");
    ring_field_ = sensor_config.param<std::string>("sensors", "ring_field", "ring");

    const std::string legacy_dump_path = config_ros.param<std::string>("glim_ros", "dump_path", "/tmp/dump");
    dump_path_ = run_logs.export_path("dump").string();
    logger_->info("dump_path: {} (legacy config value: {})", dump_path_, legacy_dump_path);

    RCLCPP_INFO(get_logger(), "iap init stage=load_core_modules");
    load_core_modules();
    RCLCPP_INFO(get_logger(), "iap init stage=load_extension_modules");
    load_extension_modules(config_ros);
    RCLCPP_INFO(get_logger(), "iap init stage=create_core_subscriptions");
    create_core_subscriptions();
    RCLCPP_INFO(get_logger(), "iap init stage=create_input_status_timer");
    input_status_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() { log_input_status(); });

    RCLCPP_INFO(get_logger(), "iap init stage=queue_thread_start");
    queue_thread_ = std::thread([this] { queue_bridge_loop(); });
    RCLCPP_INFO(get_logger(), "iap init stage=ready");

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

    for (const auto& ext : extensions_) {
      ext->at_exit(dump_path_);
    }

    if (auto* global = GlobalConfig::get_if_initialized()) {
      auto& run_logs = RunLogManager::instance();
      global->dump(run_logs.metadata_path("config").string());
      run_logs.write_run_info();
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
        ++imu_msg_count_;
        if (!logged_first_imu_) {
          logged_first_imu_ = true;
          RCLCPP_INFO(
            get_logger(),
            "iap input first imu stamp=%.6f frame_id=%s",
            to_sec(msg->header.stamp),
            msg->header.frame_id.c_str());
        }

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

    RCLCPP_INFO(
      get_logger(),
      "iap subscribed imu_topic=%s points_topic=%s",
      imu_topic_.c_str(),
      points_topic_.c_str());

    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_,
      points_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++points_msg_count_;
        if (!logged_first_points_) {
          logged_first_points_ = true;
          RCLCPP_INFO(
            get_logger(),
            "iap input first points stamp=%.6f frame_id=%s width=%u height=%u fields=%zu",
            to_sec(msg->header.stamp),
            msg->header.frame_id.c_str(),
            msg->width,
            msg->height,
            msg->fields.size());
        }

        sync_frame_metadata_from_lidar(msg->header.frame_id);

        auto raw_points = extract_raw_points(*msg, intensity_field_, ring_field_);
        if (!raw_points) {
          ++dropped_points_count_;
          return;
        }

        raw_points->stamp = to_sec(msg->header.stamp) + points_time_offset_;
        if (!time_keeper_->process(raw_points)) {
          ++dropped_points_count_;
          return;
        }

        auto preprocessed = preprocessor_->preprocess(raw_points);
        if (!preprocessed || preprocessed->size() == 0) {
          ++dropped_points_count_;
          return;
        }

        ++accepted_points_frames_;
        if (!logged_first_preprocessed_) {
          logged_first_preprocessed_ = true;
          RCLCPP_INFO(
            get_logger(),
            "iap input first preprocessed frame stamp=%.6f points=%d scan_end=%.6f",
            preprocessed->stamp,
            preprocessed->size(),
            preprocessed->scan_end_time);
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

  void log_input_status() {
    const size_t imu_publishers = imu_sub_ ? imu_sub_->get_publisher_count() : 0;
    const size_t points_publishers = points_sub_ ? points_sub_->get_publisher_count() : 0;
    RCLCPP_INFO(
      get_logger(),
      "iap input status imu_count=%zu points_count=%zu accepted_frames=%zu dropped_points=%zu imu_publishers=%zu points_publishers=%zu",
      imu_msg_count_,
      points_msg_count_,
      accepted_points_frames_,
      dropped_points_count_,
      imu_publishers,
      points_publishers);
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
  rclcpp::TimerBase::SharedPtr input_status_timer_;

  std::mutex meta_sync_mutex_;
  bool imu_frame_meta_written_ = false;
  bool lidar_frame_meta_written_ = false;
  bool base_frame_meta_written_ = false;
  std::size_t imu_msg_count_ = 0;
  std::size_t points_msg_count_ = 0;
  std::size_t accepted_points_frames_ = 0;
  std::size_t dropped_points_count_ = 0;
  bool logged_first_imu_ = false;
  bool logged_first_points_ = false;
  bool logged_first_preprocessed_ = false;

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
