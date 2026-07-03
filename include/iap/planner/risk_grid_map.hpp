#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace iap {

constexpr uint32_t RISK_GRID_SOURCE_OCCUPIED_SKIP = 1u << 31;

struct RiskGridMapParams {
  std::string frame_id = "map";
  double resolution_m = 0.75;
  double size_x_m = 30.0;
  double size_y_m = 30.0;
  double size_z_m = 6.0;
  std::vector<double> horizons_s = {0.0, 0.5, 1.0, 1.5, 2.0};
  double refresh_period_s = 0.5;
  double stale_timeout_s = 1.0;
  double unknown_cost = 10.0;
  double cost_max = 100.0;
  bool skip_occupied_voxels = true;
  bool use_predictor_batch_query = true;
};

struct RiskGridHealth {
  bool ready = false;
  bool stale = true;
  double age_s = std::numeric_limits<double>::infinity();
  double valid_ratio = 0.0;
  double unknown_ratio = 1.0;
  uint64_t generation_id = 0;
  std::string reason = "not_ready";
};

struct RiskVoxel {
  double c_pi = std::numeric_limits<double>::quiet_NaN();
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool stale = true;
  bool unknown = true;
  uint32_t source_flags = 0u;
};

struct RiskCostSample {
  bool valid = false;
  bool stale = true;
  double cost = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  uint64_t generation_id = 0;
  std::string reason = "not_evaluated";
};

struct PredictedPLSample {
  bool available = false;
  bool valid = false;
  bool stale = true;
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t generation_id = 0;
  std::string reason = "not_evaluated";
};

struct RiskPredictionQuery {
  Eigen::Vector3d position_w = Eigen::Vector3d::Zero();
  double query_time_s = std::numeric_limits<double>::quiet_NaN();
  double horizon_s = std::numeric_limits<double>::quiet_NaN();
};

struct RiskPredictionResult {
  bool available = false;
  bool valid = false;
  bool stale = true;
  double hpl_pred = std::numeric_limits<double>::quiet_NaN();
  double vpl_pred = std::numeric_limits<double>::quiet_NaN();
  uint32_t source_flags = 0u;
  std::string reason = "not_evaluated";
};

class RiskPredictionProvider {
 public:
  virtual ~RiskPredictionProvider() = default;
  virtual bool batchQuery(const std::vector<RiskPredictionQuery>& queries,
                          std::vector<RiskPredictionResult>* results) = 0;
};

class RiskGridSnapshot {
 public:
  struct Generation;

  RiskGridSnapshot() = default;

  RiskGridHealth health() const;
  double stamp_s() const;
  uint64_t generation_id() const;
  int horizonCount() const;
  int layerVoxelCount() const;

  const RiskGridMapParams& params() const;
  const Eigen::Vector3d& origin() const;
  const Eigen::Vector3i& voxelNum() const;

  bool posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i* id) const;
  Eigen::Vector3d indexToPos(const Eigen::Vector3i& id) const;
  int toAddress(const Eigen::Vector3i& id) const;
  bool isInMap(const Eigen::Vector3d& pos) const;
  bool isInMap(const Eigen::Vector3i& idx) const;

  bool queryCost(const Eigen::Vector3d& p_w,
                 double query_time_s,
                 RiskCostSample* out) const;

  bool queryPredictedPL(const Eigen::Vector3d& p_w,
                        double query_time_s,
                        PredictedPLSample* out) const;

  bool voxelAt(int horizon_id,
               const Eigen::Vector3i& id,
               RiskVoxel* out) const;

 private:
  explicit RiskGridSnapshot(std::shared_ptr<const Generation> generation);

  std::shared_ptr<const Generation> generation_;

  friend class RiskGridMap;
};

class RiskGridMap {
 public:
  using OccupancyPredicate = std::function<bool(const Eigen::Vector3d&)>;

  RiskGridMap();
  explicit RiskGridMap(RiskGridMapParams params);

  bool configure(RiskGridMapParams params, std::string* reason = nullptr);

  RiskGridHealth health() const;
  RiskGridHealth health(double now_s) const;
  std::shared_ptr<const RiskGridSnapshot> acquireSnapshot() const;

  const RiskGridMapParams& params() const { return params_; }
  const Eigen::Vector3i& voxelNum() const { return voxel_num_; }
  const Eigen::Vector3d& origin() const { return origin_; }

  bool posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i* id) const;
  Eigen::Vector3d indexToPos(const Eigen::Vector3i& id) const;
  int toAddress(const Eigen::Vector3i& id) const;
  bool isInMap(const Eigen::Vector3d& pos) const;
  bool isInMap(const Eigen::Vector3i& idx) const;

  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           std::string* reason = nullptr);
  bool refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                           double now_s,
                           RiskPredictionProvider& provider,
                           const OccupancyPredicate& is_occupied,
                           std::string* reason = nullptr);

  void markRefreshFailure(double now_s, const std::string& reason);

 private:
  bool validateParams(const RiskGridMapParams& params,
                      std::string* reason) const;
  void updateGeometry(const Eigen::Vector3d& center_w);

  mutable std::mutex mutex_;
  RiskGridMapParams params_;
  Eigen::Vector3i voxel_num_ = Eigen::Vector3i::Zero();
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  uint64_t next_generation_id_ = 1;
  RiskGridHealth health_;
  std::shared_ptr<const RiskGridSnapshot::Generation> active_;
};

}  // namespace iap
