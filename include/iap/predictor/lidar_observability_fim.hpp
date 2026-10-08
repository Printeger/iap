#pragma once
// Predictor-side LiDAR advisory observability/FIM component.
//
// This is a future/advisory LOI-style proxy. It is not the certified current
// LiDAR ARAIM monitor and must not be reported as certified LiDAR PL.

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <iap/predictor/advisory_fim_types.hpp>
#include <iap/planner/integrity_snapshot.hpp>

namespace iap {

struct LidarObservabilityResult {
  bool valid = false;
  Eigen::Matrix3d delta_lambda = Eigen::Matrix3d::Zero();  ///< advisory LOI
  double tdop_proxy = 20.0;
  double lidar_alpha = 0.0;
  double condition = 1.0e6;
  int n_primitives = 0;
  double bias_h = 0.0;
  double bias_v = 0.0;
  std::string fallback_reason = "not_evaluated";
};

struct LidarFimPrimitive {
  Eigen::Vector3d center_w =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d normal_w =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double weight = 1.0;
  double normal_confidence = 1.0;
  int support_count = 0;
};

struct LidarFimPrimitiveGenerationParams {
  double pca_radius_m = 1.5;
  int pca_max_points = 2000;
  int pca_min_support = 6;
  double pca_voxel_sample_m = 0.5;
  int pca_max_primitives = 2000;
  bool use_cloud_normals_first = true;
};

struct LidarFimPrimitiveGenerationDiagnostics {
  bool valid = false;
  std::string fallback_reason = "not_evaluated";
  int lidar_pca_primitives_total = 0;
  int lidar_pca_valid_normals = 0;
  int lidar_pca_invalid_normals = 0;
  double lidar_pca_support_mean =
      std::numeric_limits<double>::quiet_NaN();
  int lidar_pca_support_min = 0;
  double lidar_pca_radius_m = 1.5;
};

std::shared_ptr<std::vector<LidarFimPrimitive>> make_lidar_fim_primitives(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>* normals = nullptr,
    const LidarFimPrimitiveGenerationParams& params =
        LidarFimPrimitiveGenerationParams{},
    LidarFimPrimitiveGenerationDiagnostics* diagnostics = nullptr);

class LidarFimPrimitiveIndex {
 public:
  // Geometry and correlation family are invariant over spatial queries.
  struct AdvisoryPrimitive {
    bool valid=false;
    Eigen::Matrix3d normal_outer=Eigen::Matrix3d::Zero();
    Eigen::Matrix<double,6,6> pose_outer_at_origin=Eigen::Matrix<double,6,6>::Zero();
    double confidence=0., weight=1.;
    std::size_t group=0;
  };
  struct Stats {
    std::size_t primitive_count = 0;
    std::size_t finite_primitive_count = 0;
    std::size_t bucket_count = 0;
    double cell_size_m = 1.0;
  };

  LidarFimPrimitiveIndex();
  LidarFimPrimitiveIndex(
      std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives,
      double cell_size_m, double support_voxel_m = .5);

  static std::shared_ptr<const LidarFimPrimitiveIndex> build(
      std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives,
      double cell_size_m, double support_voxel_m = .5);

  bool empty() const;
  const std::vector<LidarFimPrimitive>* primitives() const {
    return primitives_.get();
  }
  const Stats& stats() const { return stats_; }
  const std::vector<AdvisoryPrimitive>* advisoryPrimitives(double support_voxel_m) const {
    return support_voxel_m==support_voxel_m_ ? &advisory_primitives_ : nullptr;
  }
  std::size_t advisoryGroupCount() const { return advisory_group_count_; }
  const std::vector<std::size_t>& advisoryOrder() const { return advisory_order_; }

  void queryRadius(const Eigen::Vector3d& center,
                   double radius_m,
                   std::vector<std::size_t>* indices) const;

 private:
  struct Key {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const;
  };

  Key keyFor(const Eigen::Vector3d& point) const;

  std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives_;
  double cell_size_m_ = 1.0;
  std::unordered_map<Key, std::vector<std::size_t>, KeyHash> buckets_;
  Stats stats_;
  double support_voxel_m_=std::numeric_limits<double>::quiet_NaN();
  std::vector<AdvisoryPrimitive> advisory_primitives_;
  std::size_t advisory_group_count_=0;
  std::vector<std::size_t> advisory_order_;
};

class LidarObservabilityFim {
 public:
  struct Params {
    double search_radius_m = 8.0;
    int min_points = 12;
    int good_points = 80;
    double sigma_lidar_m = 0.5;
    double alpha_min = 0.02;
    double alpha_max = 1.0;
    double condition_ref = 30.0;
    double condition_max = 1.0e6;
    double tdop_ref = 2.0;
    double tdop_max = 20.0;
    double bias_h_m = 0.0;
    double bias_v_m = 0.0;
    double fim_radius_m = 8.0;
    int fim_min_voxels = 6;
    double fim_range_sigma_base = 0.5;
    double fim_condition_max = 1.0e6;
    double fim_weight_scale = 1.0;
    // Same default support scale as PCA generation; independent of GridMap.
    double fim_support_voxel_m = 0.5;
  };

  LidarObservabilityFim();
  explicit LidarObservabilityFim(const Params& params);

  LidarObservabilityResult evaluate(
      const Eigen::Vector3d& p_w,
      const std::vector<Eigen::Vector3d>* map_points,
      const CurrentIntegrityState& current) const;

  LidarAdvisoryFimResult evaluate_advisory_fim(
      const Eigen::Vector3d& p_w,
      const std::vector<LidarFimPrimitive>* primitives,
      const CurrentIntegrityState& current) const;

  LidarAdvisoryFimResult evaluate_advisory_fim(
      const Eigen::Vector3d& p_w,
      const LidarFimPrimitiveIndex* index,
      const CurrentIntegrityState& current) const;

  LidarAdvisoryFimResult evaluate_advisory_fim(
      const Eigen::Vector3d& p_w,
      std::nullptr_t,
      const CurrentIntegrityState& current) const {
    return evaluate_advisory_fim(
        p_w, static_cast<const std::vector<LidarFimPrimitive>*>(nullptr),
        current);
  }

  const Params& params() const { return params_; }

 private:
  Params params_;
};

}  // namespace iap
