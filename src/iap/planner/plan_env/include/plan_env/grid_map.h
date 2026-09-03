#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <atomic>
#include <functional>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>

#include <iap/msg/active_lidar_window_delta.hpp>
#include <iap/msg/registered_lidar_frame.hpp>
#include <iap/srv/get_active_lidar_window.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>
#include <plan_env/registered_lidar_window.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters
{

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;
  int pose_type_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;
  double independent_cloud_min_interval_s_ = 0.0;
  double independent_cloud_clock_guard_s_ = 0.0;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
  bool show_occ_time_;
  bool unknown_as_occupied_ = false;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<char> occupancy_buffer_raw_cloud_;
  // Online observed-space mask. A cell is set only by a sensor return or an
  // explicit successful-return ray traversal. In registered-window mode it
  // combines the active-keyframe base with the replaceable current overlay;
  // absence of a point never proves free space.
  std::vector<char> observed_buffer_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  // Node-clock receipt time used only by the depth/odometry watchdog.
  // It is never the scientific timestamp of a published occupancy epoch.
  rclcpp::Time last_occ_update_time_;
  double pending_depth_source_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class GridMapObservationState : uint8_t
{
  UNKNOWN = 0,
  OBSERVED_FREE = 1,
  OCCUPIED = 2,
};

struct GridMapOccupancyDiagnostic
{
  bool available = false;
  bool observed = false;
  bool raw_occupied = false;
  bool inflated_occupied = false;
  GridMapObservationState state = GridMapObservationState::UNKNOWN;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double resolution_m = std::numeric_limits<double>::quiet_NaN();
  double inflation_m = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t generation = 0;
  std::string source = "unavailable";
};

using GridMapOccupancyDiagnosticQuery =
    std::function<GridMapOccupancyDiagnostic(const Eigen::Vector3d &)>;

struct FrozenOccupancyEpoch
{
  GridMapOccupancyDiagnosticQuery diagnostic_query;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      raw_occupied_voxel_centers;
  Eigen::Vector3d lattice_origin = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d extent_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3i voxel_dimensions = Eigen::Vector3i::Zero();
  double resolution_m = std::numeric_limits<double>::quiet_NaN();
  double virtual_ceiling_height_m = -1.0;
  std::string frame_id;
  std::string geometry_id;
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t generation = 0;
  uint64_t active_window_generation = 0;
  int64_t current_frame_id = -1;
  std::string frame_contract_id;
};

struct OccupancyCollisionVoxelChange
{
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  bool occupied = false;
};

struct OccupancyCollisionDelta
{
  uint64_t from_generation = 0;
  uint64_t to_generation = 0;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::string geometry_id;
  bool complete = false;
  std::vector<OccupancyCollisionVoxelChange> changes;
};

struct OccupancyCollisionDeltaHistory
{
  uint64_t base_generation = 0;
  uint64_t latest_generation = 0;
  bool complete = false;
  // A current sensor transaction is being committed. This is transient busy
  // state, not a missing journal generation.
  bool update_in_progress = false;
  std::string geometry_id;
  std::vector<std::shared_ptr<const OccupancyCollisionDelta>> deltas;
};

class GridMap : public std::enable_shared_from_this<GridMap>
{
public:
  GridMap() {}
  ~GridMap() {}

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  using OccupancyDiagnostic = GridMapOccupancyDiagnostic;
  using OccupancyDiagnosticQuery = GridMapOccupancyDiagnosticQuery;
  using FrozenOccupancyEpoch = ::FrozenOccupancyEpoch;

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int &x, int &y, int &z);
  inline bool isInMap(const Eigen::Vector3d &pos);
  inline bool isInMap(const Eigen::Vector3i &idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  OccupancyDiagnostic queryOccupancyDiagnostic(
      const Eigen::Vector3d &pos) const;
  OccupancyDiagnosticQuery captureOccupancyDiagnosticQuery() const;
  std::shared_ptr<const FrozenOccupancyEpoch>
  captureFrozenOccupancyEpoch() const;
  OccupancyCollisionDeltaHistory collisionDeltasSince(
      uint64_t base_generation) const;
  uint64_t occupancyGeneration() const;
  // Bind current-body observation evidence to the planner's canonical vehicle
  // radius. This is intentionally not a separate ROS parameter.
  void setCurrentVehicleClearanceRadius(double radius_m);

  inline void boundIndex(Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3d &pos);
  inline bool isKnownFree(const Eigen::Vector3i &id);
  inline bool isKnownOccupied(const Eigen::Vector3i &id);

  void initMap(rclcpp::Node::SharedPtr node);

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishDepth();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  inline double getResolution();
  inline double getObstacleInflation() const;
  inline double getVirtualCeilingHeight() const;
  Eigen::Vector3d getOrigin();
  int getVoxelNum();
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend struct GridMapTestAccess;

  MappingParameters mp_;
  MappingData md_;

  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                         const geometry_msgs::msg::PoseStamped::ConstPtr &pose);
  void extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img, const nav_msgs::msg::Odometry::ConstPtr &odom);
  void independentCloudInputCallback(
      const sensor_msgs::msg::PointCloud2::ConstPtr &img);
  sensor_msgs::msg::PointCloud2::ConstPtr
  takeLatestIndependentCloudAtOrBefore(double clock_stamp_s);
  void processLatestIndependentCloud();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img);
  void registeredCurrentFrameCallback(
      const iap::msg::RegisteredLidarFrame::ConstSharedPtr &message);
  void registeredWindowDeltaCallback(
      const iap::msg::ActiveLidarWindowDelta::ConstSharedPtr &message);
  void requestRegisteredWindowRecovery(const std::string &reason);
  void maintainRegisteredWindowRecovery();
  void applyRegisteredLidarUpdate(
      const RegisteredLidarWindowUpdate &update);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  // update occupancy by raycasting
  void updateOccupancyCallback();
  bool updateOccupancyFromPendingDepth(
      const rclcpp::Time &receipt_time,
      bool *watchdog_timed_out = nullptr,
      double *last_receipt_time_s = nullptr);
  void visCallback();

  // main update process
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();
  void markCurrentVehicleFootprintObserved();
  void beginOccupancyWriteTransaction();
  void recordCollisionStateBeforeMutation(int address);
  void commitOccupancyWriteTransaction(double stamp_s);
  bool collisionOccupiedAtAddress(int address) const;

  inline void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr extrinsic_sub_;
  rclcpp::Subscription<iap::msg::RegisteredLidarFrame>::SharedPtr
      registered_current_sub_;
  rclcpp::Subscription<iap::msg::ActiveLidarWindowDelta>::SharedPtr
      registered_delta_sub_;
  rclcpp::Client<iap::srv::GetActiveLidarWindow>::SharedPtr
      registered_recovery_client_;
  rclcpp::CallbackGroup::SharedPtr independent_cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr independent_cloud_input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr independent_odom_callback_group_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;

  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  rclcpp::TimerBase::SharedPtr independent_cloud_timer_;
  rclcpp::TimerBase::SharedPtr registered_recovery_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
  std::atomic<uint64_t> occupancy_update_sequence_{0};
  std::atomic<double> occupancy_cloud_stamp_s_{
      std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> independent_odom_stamp_s_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_independent_cloud_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  std::deque<sensor_msgs::msg::PointCloud2::ConstPtr>
      pending_independent_clouds_;
  mutable std::mutex independent_cloud_input_mutex_;
  mutable std::mutex occupancy_epoch_mutex_;
  mutable std::mutex collision_delta_mutex_;
  std::unordered_map<int, bool> collision_state_before_transaction_;
  bool collision_transaction_active_ = false;
  std::deque<std::shared_ptr<const OccupancyCollisionDelta>>
      collision_delta_history_;
  static constexpr std::size_t kCollisionDeltaHistoryCapacity = 128;
  double current_vehicle_clearance_radius_m_ = 0.0;
  bool registered_lidar_window_enabled_ = false;
  std::string registered_frame_contract_id_;
  std::string registered_current_topic_;
  std::string registered_delta_topic_;
  std::string registered_recovery_service_;
  std::unique_ptr<RegisteredLidarWindow> registered_lidar_window_;
  std::atomic<bool> registered_recovery_in_flight_{false};
  std::atomic<bool> registered_recovery_pending_{false};
  std::atomic<uint64_t> registered_recovery_serial_{0};
  std::atomic<uint64_t> registered_highest_seen_generation_{0};
  std::atomic<int64_t> registered_recovery_deadline_ns_{0};
  bool registered_active_window_healthy_ = false;
  bool registered_current_frame_healthy_ = false;
  std::string registered_lidar_reference_frame_id_ =
      "iap_lidar_reference";
  // Reused bitset for deduplicating inflation cells touched by registered
  // hit transitions without allocating a large hash table at 10 Hz.
  std::vector<uint64_t> registered_inflation_dirty_bits_;
  std::vector<uint16_t> registered_raw_inflation_count_;
  std::vector<double> registered_current_apply_latency_ms_;
  std::vector<double> registered_sensor_to_occupancy_latency_ms_;
  std::vector<double> registered_delta_apply_latency_ms_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int &x, int &y, int &z)
{
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  const int address = toAddress(id1);
  return address < 0 ||
      address >= static_cast<int>(md_.observed_buffer_.size()) ||
      md_.observed_buffer_[static_cast<std::size_t>(address)] == 0;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return adr >= 0 && adr < static_cast<int>(md_.observed_buffer_.size()) &&
      md_.observed_buffer_[static_cast<std::size_t>(adr)] != 0 &&
      md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  beginOccupancyWriteTransaction();
  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
  const int address = toAddress(id);
  if (address >= 0 && address < static_cast<int>(md_.observed_buffer_.size()))
    md_.observed_buffer_[static_cast<std::size_t>(address)] = 1;
  commitOccupancyWriteTransaction(
      occupancy_cloud_stamp_s_.load(std::memory_order_acquire));
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  beginOccupancyWriteTransaction();
  const int address = toAddress(id);
  recordCollisionStateBeforeMutation(address);
  md_.occupancy_buffer_[address] = occ;
  if (address >= 0 && address < static_cast<int>(md_.observed_buffer_.size()))
    md_.observed_buffer_[static_cast<std::size_t>(address)] = 1;
  commitOccupancyWriteTransaction(
      occupancy_cloud_stamp_s_.load(std::memory_order_acquire));
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  const int address = toAddress(id);
  if (mp_.unknown_as_occupied_ &&
      (address < 0 || address >= static_cast<int>(md_.observed_buffer_.size()) ||
       md_.observed_buffer_[static_cast<std::size_t>(address)] == 0))
    return 1;
  return int(md_.occupancy_buffer_inflate_[address]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1)
  {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
  int num = 0;
  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
      {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

inline double GridMap::getObstacleInflation() const
{
  return mp_.obstacles_inflation_;
}

inline double GridMap::getVirtualCeilingHeight() const
{
  return mp_.virtual_ceil_height_;
}

#endif
