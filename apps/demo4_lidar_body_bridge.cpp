#include <cmath>
#include <cstdint>
#include <algorithm>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace {

struct FieldInfo {
  int offset = -1;
  std::uint8_t datatype = 0;
};

struct StampedPose {
  double stamp = 0.0;
  Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
};

std::optional<FieldInfo> find_field(
    const sensor_msgs::msg::PointCloud2& cloud,
    const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return FieldInfo{static_cast<int>(field.offset), field.datatype};
    }
  }
  return std::nullopt;
}

bool read_scalar(
    const sensor_msgs::msg::PointCloud2& cloud,
    const FieldInfo& field,
    std::size_t point_index,
    double& value) {
  const auto value_size =
      field.datatype == sensor_msgs::msg::PointField::FLOAT64 ? sizeof(double) : sizeof(float);
  const auto offset = point_index * cloud.point_step + static_cast<std::size_t>(field.offset);
  if (offset + value_size > cloud.data.size()) {
    return false;
  }

  const auto* ptr = cloud.data.data() + offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      value = *reinterpret_cast<const float*>(ptr);
      return true;
    case sensor_msgs::msg::PointField::FLOAT64:
      value = *reinterpret_cast<const double*>(ptr);
      return true;
    default:
      return false;
  }
}

void add_xyz_fields(sensor_msgs::msg::PointCloud2& cloud) {
  cloud.fields.resize(3);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;
}

Eigen::Isometry3d odom_to_pose(const nav_msgs::msg::Odometry& odom) {
  Eigen::Quaterniond q(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);
  q.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);
  return pose;
}

Eigen::Isometry3d interpolate_pose(
    const StampedPose& before,
    const StampedPose& after,
    double stamp) {
  const double dt = after.stamp - before.stamp;
  if (dt <= 1e-9) {
    return before.T_map_body;
  }

  const double ratio = std::clamp((stamp - before.stamp) / dt, 0.0, 1.0);
  const Eigen::Vector3d trans =
      before.T_map_body.translation() * (1.0 - ratio) +
      after.T_map_body.translation() * ratio;
  const Eigen::Quaterniond q0(before.T_map_body.linear());
  const Eigen::Quaterniond q1(after.T_map_body.linear());

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = trans;
  pose.linear() = q0.slerp(ratio, q1).normalized().toRotationMatrix();
  return pose;
}

}  // namespace

class Demo4LidarBodyBridge : public rclcpp::Node {
public:
  Demo4LidarBodyBridge() : rclcpp::Node("demo4_lidar_body_bridge") {
    input_cloud_topic_ =
        declare_parameter<std::string>("input_cloud_topic", "/sim/drone_0/lidar");
    input_odom_topic_ =
        declare_parameter<std::string>("input_odom_topic", "/sim/drone_0/truth_odom");
    output_cloud_topic_ =
        declare_parameter<std::string>("output_cloud_topic", "/sim/drone_0/lidar_body");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "lidar");

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_,
        rclcpp::SensorDataQoS().keep_last(5));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_,
        rclcpp::QoS(100),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          add_odom(*msg);
        });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_,
        rclcpp::SensorDataQoS().keep_last(5),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          handle_cloud(msg);
        });

    RCLCPP_INFO(
        get_logger(),
        "demo4 lidar body bridge ready cloud=%s odom=%s output=%s frame=%s",
        input_cloud_topic_.c_str(),
        input_odom_topic_.c_str(),
        output_cloud_topic_.c_str(),
        output_frame_id_.c_str());
  }

private:
  void add_odom(const nav_msgs::msg::Odometry& odom) {
    const StampedPose stamped_pose{
        rclcpp::Time(odom.header.stamp).seconds(),
        odom_to_pose(odom)};

    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_buffer_.push_back(stamped_pose);
      const double keep_after = stamped_pose.stamp - odom_buffer_duration_;
      while (odom_buffer_.size() > 2 && odom_buffer_.front().stamp < keep_after) {
        odom_buffer_.pop_front();
      }
    }

    process_pending_clouds();
  }

  void handle_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    std::optional<Eigen::Isometry3d> pose;
    bool too_old = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = lookup_pose_locked(stamp, too_old);
      if (!pose && !too_old) {
        pending_clouds_.push_back(msg);
        while (pending_clouds_.size() > max_pending_clouds_) {
          RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "dropping queued lidar frame because odom lookup is lagging");
          pending_clouds_.pop_front();
        }
      }
    }

    if (pose) {
      convert_cloud(*msg, pose->inverse());
    } else if (too_old) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "dropping lidar frame stamp=%.6f because it is older than the odom buffer",
          stamp);
    }
  }

  void process_pending_clouds() {
    std::vector<std::pair<sensor_msgs::msg::PointCloud2::ConstSharedPtr, Eigen::Isometry3d>>
        ready_clouds;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!pending_clouds_.empty()) {
        const auto& cloud = pending_clouds_.front();
        const double stamp = rclcpp::Time(cloud->header.stamp).seconds();
        bool too_old = false;
        auto pose = lookup_pose_locked(stamp, too_old);

        if (pose) {
          ready_clouds.emplace_back(cloud, pose->inverse());
          pending_clouds_.pop_front();
          continue;
        }

        if (too_old) {
          RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "dropping queued lidar frame stamp=%.6f because it is older than the odom buffer",
              stamp);
          pending_clouds_.pop_front();
          continue;
        }

        break;
      }
    }

    for (const auto& [cloud, T_body_map] : ready_clouds) {
      convert_cloud(*cloud, T_body_map);
    }
  }

  std::optional<Eigen::Isometry3d> lookup_pose_locked(double stamp, bool& too_old) const {
    too_old = false;
    if (odom_buffer_.empty()) {
      return std::nullopt;
    }

    constexpr double kEpsilon = 1e-6;
    if (stamp < odom_buffer_.front().stamp - kEpsilon) {
      too_old = true;
      return std::nullopt;
    }
    if (stamp > odom_buffer_.back().stamp + kEpsilon) {
      return std::nullopt;
    }

    if (std::abs(stamp - odom_buffer_.front().stamp) <= kEpsilon) {
      return odom_buffer_.front().T_map_body;
    }

    for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
      const auto& before = odom_buffer_[i - 1];
      const auto& after = odom_buffer_[i];
      if (stamp <= after.stamp + kEpsilon) {
        if (std::abs(stamp - after.stamp) <= kEpsilon) {
          return after.T_map_body;
        }
        return interpolate_pose(before, after, stamp);
      }
    }

    return odom_buffer_.back().T_map_body;
  }

  void convert_cloud(const sensor_msgs::msg::PointCloud2& input, const Eigen::Isometry3d& T_body_map) {
    const auto x_field = find_field(input, "x");
    const auto y_field = find_field(input, "y");
    const auto z_field = find_field(input, "z");
    if (!x_field || !y_field || !z_field) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "input cloud is missing x/y/z fields");
      return;
    }

    const std::size_t input_points =
        static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
    std::vector<Eigen::Vector3f> output_points;
    output_points.reserve(input_points);

    for (std::size_t i = 0; i < input_points; ++i) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!read_scalar(input, *x_field, i, x) || !read_scalar(input, *y_field, i, y) ||
          !read_scalar(input, *z_field, i, z)) {
        continue;
      }

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const Eigen::Vector3d p_body = T_body_map * Eigen::Vector3d(x, y, z);
      output_points.push_back(p_body.cast<float>());
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp = input.header.stamp;
    output.header.frame_id = output_frame_id_;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(output_points.size());
    output.is_bigendian = false;
    output.is_dense = false;
    output.point_step = 12;
    output.row_step = output.point_step * output.width;
    add_xyz_fields(output);
    output.data.resize(output.row_step);

    for (std::size_t i = 0; i < output_points.size(); ++i) {
      auto* point = output.data.data() + i * output.point_step;
      *reinterpret_cast<float*>(point + 0) = output_points[i].x();
      *reinterpret_cast<float*>(point + 4) = output_points[i].y();
      *reinterpret_cast<float*>(point + 8) = output_points[i].z();
    }

    cloud_pub_->publish(output);

    if (!logged_first_output_) {
      logged_first_output_ = true;
      RCLCPP_INFO(
          get_logger(),
          "first body cloud stamp=%.6f points=%u frame_id=%s input_frame=%s",
          rclcpp::Time(output.header.stamp).seconds(),
          output.width,
          output.header.frame_id.c_str(),
          input.header.frame_id.c_str());
    }
  }

  std::string input_cloud_topic_;
  std::string input_odom_topic_;
  std::string output_cloud_topic_;
  std::string output_frame_id_;

  std::mutex mutex_;
  std::deque<StampedPose> odom_buffer_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;
  double odom_buffer_duration_ = 2.0;
  std::size_t max_pending_clouds_ = 30;
  bool logged_first_output_ = false;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Demo4LidarBodyBridge>());
  rclcpp::shutdown();
  return 0;
}
