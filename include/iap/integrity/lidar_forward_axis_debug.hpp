#pragma once
// Diagnostic-only LiDAR ARAIM forward/lateral axis projection CSV.

#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/lidar_araim.hpp>

#include <Eigen/Core>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

#include <spdlog/spdlog.h>

namespace iap {

class LidarForwardAxisDebugCSV {
 public:
  struct AxisDefinition {
    Eigen::Vector2d forward = Eigen::Vector2d(1.0, 0.0);
    Eigen::Vector2d lateral = Eigen::Vector2d(0.0, 1.0);
    std::string source = "fixed_fallback";
  };

  struct OdomSample {
    double stamp = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  };

  struct OdomProjection {
    bool valid = false;
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double match_dt_s = std::numeric_limits<double>::quiet_NaN();
  };

  struct AxisProjection {
    bool valid = false;
    double d_m = std::numeric_limits<double>::quiet_NaN();
    double sigma_ss_m = std::numeric_limits<double>::quiet_NaN();
    double sigma_k_m = std::numeric_limits<double>::quiet_NaN();
    double bias_m = std::numeric_limits<double>::quiet_NaN();
    double pl_m = std::numeric_limits<double>::quiet_NaN();
    std::string hypothesis_type = "NONE";
    int hypothesis_id = -1;
  };

  LidarForwardAxisDebugCSV() = default;
  LidarForwardAxisDebugCSV(bool enabled,
                           const std::string& path,
                           const AxisDefinition& axis) {
    axis_ = axis;
    if (enabled) {
      enabled_ = true;
      open_file(path);
    }
  }

  void write(const IntegrityReport& report,
             const LidarAraimResult& result,
             long frame_id,
             const OdomSample& odom,
             const std::optional<OdomSample>& truth,
             const std::optional<OdomSample>& desired) {
    if (!enabled_ || !result.valid) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!header_written_) write_header();

    const AxisProjection forward =
        project_axis(result, axis_.forward, "forward");
    const AxisProjection lateral =
        project_axis(result, axis_.lateral, "lateral");
    const OdomProjection odom_proj = project_odom(odom, axis_.forward, odom.stamp);
    const OdomProjection truth_proj =
        truth ? project_odom(*truth, axis_.forward, report.stamp)
              : OdomProjection{};
    const OdomProjection desired_proj =
        desired ? project_odom(*desired, axis_.forward, report.stamp)
                : OdomProjection{};

    const double forward_position_error =
        truth_proj.valid ? odom_proj.position - truth_proj.position : nan();
    const double forward_velocity_error =
        truth_proj.valid ? odom_proj.velocity - truth_proj.velocity : nan();

    file_ << std::scientific << std::setprecision(12)
          << report.stamp << ','
          << frame_id << ','
          << axis_.forward.x() << ','
          << axis_.forward.y() << ','
          << axis_.source << ','
          << forward.d_m << ','
          << lateral.d_m << ','
          << forward.sigma_ss_m << ','
          << lateral.sigma_ss_m << ','
          << forward.sigma_k_m << ','
          << lateral.sigma_k_m << ','
          << forward.bias_m << ','
          << lateral.bias_m << ','
          << forward.pl_m << ','
          << lateral.pl_m << ','
          << forward.hypothesis_type << ','
          << forward.hypothesis_id << ','
          << lateral.hypothesis_type << ','
          << lateral.hypothesis_id << ','
          << odom_proj.position << ','
          << truth_proj.position << ','
          << desired_proj.position << ','
          << odom_proj.velocity << ','
          << truth_proj.velocity << ','
          << desired_proj.velocity << ','
          << forward_position_error << ','
          << forward_velocity_error << ','
          << truth_proj.match_dt_s << ','
          << desired_proj.match_dt_s << '\n';
    file_.flush();
  }

  bool enabled() const { return enabled_; }

  static AxisDefinition make_axis_definition(double init_x,
                                             double init_y,
                                             double goal_x,
                                             double goal_y) {
    AxisDefinition axis;
    const Eigen::Vector2d delta(goal_x - init_x, goal_y - init_y);
    const double norm = delta.norm();
    if (std::isfinite(norm) && norm > 1e-6) {
      axis.forward = delta / norm;
      axis.lateral = Eigen::Vector2d(-axis.forward.y(), axis.forward.x());
      axis.source = "fixed_from_preset";
    }
    return axis;
  }

  static AxisProjection project_axis(const LidarAraimResult& result,
                                     const Eigen::Vector2d& axis,
                                     const std::string&) {
    AxisProjection best;
    if (!result.valid || !axis.allFinite() || axis.norm() <= 1e-12) {
      return best;
    }
    const Eigen::Vector2d e = axis.normalized();
    const Eigen::Matrix2d Sigma0_en =
        result.Sigma0.block<2, 2>(3, 3);

    for (std::size_t i = 0; i < result.subsets.size(); ++i) {
      const auto& ss = result.subsets[i];
      if (!ss.valid) continue;

      const Eigen::Matrix2d Sigmaf_en =
          ss.Sigma_f_pos.block<2, 2>(0, 0);
      const double sigma_k_m2 = quadratic_form(Sigmaf_en, e);
      if (!std::isfinite(sigma_k_m2) || sigma_k_m2 < 0.0) continue;

      const double sigma_ss_raw_m2 =
          quadratic_form(Sigmaf_en - Sigma0_en, e);
      const bool use_raw =
          std::isfinite(sigma_ss_raw_m2) && sigma_ss_raw_m2 > 0.0;
      const double sigma_ss_source_m2 =
          use_raw ? sigma_ss_raw_m2 : sigma_k_m2;
      if (!std::isfinite(sigma_ss_source_m2) || sigma_ss_source_m2 < 0.0) {
        continue;
      }

      const Eigen::Vector2d d_en(ss.d_E, ss.d_N);
      const double d_m = e.dot(d_en);
      const double sigma_ss_m = std::sqrt(sigma_ss_source_m2);
      const double sigma_k_m = std::sqrt(sigma_k_m2);
      const double pl_m =
          std::abs(d_m) + ss.K_fa * sigma_ss_m +
          ss.K_md * sigma_k_m + ss.bias_H;

      if (!std::isfinite(pl_m) ||
          (best.valid && pl_m <= best.pl_m)) {
        continue;
      }

      best.valid = true;
      best.d_m = d_m;
      best.sigma_ss_m = sigma_ss_m;
      best.sigma_k_m = sigma_k_m;
      best.bias_m = ss.bias_H;
      best.pl_m = pl_m;
      if (i < result.hypotheses.size()) {
        const auto& hyp = result.hypotheses[i];
        best.hypothesis_type = LidarAraim::to_string(hyp.type);
        best.hypothesis_id = hypothesis_id(hyp);
      }
    }
    return best;
  }

  static OdomProjection project_odom(const OdomSample& sample,
                                     const Eigen::Vector2d& axis,
                                     double reference_stamp) {
    OdomProjection out;
    if (!axis.allFinite() || axis.norm() <= 1e-12 ||
        !sample.position.allFinite() || !sample.velocity.allFinite()) {
      return out;
    }
    const Eigen::Vector2d e = axis.normalized();
    out.valid = true;
    out.position = e.dot(sample.position.head<2>());
    out.velocity = e.dot(sample.velocity.head<2>());
    out.match_dt_s = sample.stamp - reference_stamp;
    return out;
  }

  static double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

 private:
  void open_file(const std::string& path) {
    path_ = path;
    file_.open(path_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
      spdlog::warn("[lidar_forward_axis_debug] Failed to open CSV: {}", path_);
      enabled_ = false;
    } else {
      spdlog::info("[lidar_forward_axis_debug] CSV logging to: {}", path_);
    }
  }

  void write_header() {
    file_ << "stamp_sec,frame_id,corridor_axis_x,corridor_axis_y,"
          << "corridor_axis_source,"
          << "d_forward_m,d_lateral_m,"
          << "sigma_ss_forward_m,sigma_ss_lateral_m,"
          << "sigma_k_forward_m,sigma_k_lateral_m,"
          << "bias_forward_m,bias_lateral_m,"
          << "pl_forward_m,pl_lateral_m,"
          << "forward_hypothesis_type,forward_hypothesis_id,"
          << "lateral_hypothesis_type,lateral_hypothesis_id,"
          << "odom_forward_position,truth_forward_position,"
          << "desired_forward_position,"
          << "odom_forward_velocity,truth_forward_velocity,"
          << "desired_forward_velocity,"
          << "forward_position_error,forward_velocity_error,"
          << "truth_match_dt_s,desired_match_dt_s\n";
    header_written_ = true;
  }

  static double quadratic_form(const Eigen::Matrix2d& matrix,
                               const Eigen::Vector2d& axis) {
    return axis.dot(matrix * axis);
  }

  static int hypothesis_id(const LidarHypothesis& hyp) {
    if (hyp.type == LidarHypothesis::Type::H_TARGET) {
      return static_cast<int>(hyp.target_frame_id);
    }
    if (hyp.type == LidarHypothesis::Type::H_LEVEL) {
      return hyp.level_id;
    }
    return -1;
  }

  bool enabled_ = false;
  bool header_written_ = false;
  AxisDefinition axis_;
  std::string path_;
  std::ofstream file_;
  std::mutex mutex_;
};

}  // namespace iap
