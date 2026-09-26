// IAP-RQ-300: FGO Information Matrix extraction
// Hooks into on_smoother_update_finish to extract Σ^(0)_{p,p} from iSAM2.

#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/gnss/clock_between_factor.hpp>
#include <iap/gnss/doppler_factor.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/trunk/trunk_factor.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/config.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/factors/reintegrated_imu_factor.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

namespace iap {
namespace {

std::string stable_identity(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

bool touches_local_navigation_state(
    const gtsam::NonlinearFactor::shared_ptr& factor) {
  return std::any_of(factor->keys().begin(), factor->keys().end(),
                     [](const gtsam::Key key) {
                       const char prefix = gtsam::Symbol(key).chr();
                       return prefix == 'x' || prefix == 'v' || prefix == 'b';
                     });
}

bool is_explicitly_excluded_nonlocal_factor(
    const gtsam::NonlinearFactor::shared_ptr& factor) {
  return dynamic_cast<const PseudorangeFactor*>(factor.get()) != nullptr ||
      dynamic_cast<const DopplerFactor*>(factor.get()) != nullptr ||
      dynamic_cast<const ClockBetweenFactor*>(factor.get()) != nullptr ||
      dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam::PriorFactor<gtsam::Vector3>*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam::PriorFactor<
          gtsam::imuBias::ConstantBias>*>(factor.get()) != nullptr ||
      dynamic_cast<const gtsam_points::LinearDampingFactor*>(factor.get()) !=
          nullptr;
}

bool is_admitted_local_factor(
    const gtsam::NonlinearFactor::shared_ptr& factor) {
  return dynamic_cast<const gtsam::ImuFactor*>(factor.get()) != nullptr ||
      dynamic_cast<const gtsam_points::ReintegratedImuFactor*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam::BetweenFactor<
          gtsam::imuBias::ConstantBias>*>(factor.get()) != nullptr ||
      dynamic_cast<const gtsam_points::IntegratedGICPFactor*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam_points::IntegratedVGICPFactor*>(factor.get()) !=
          nullptr ||
      dynamic_cast<const gtsam_points::IntegratedVGICPFactorGPU*>(
          factor.get()) != nullptr ||
      dynamic_cast<const TrunkFactor*>(factor.get()) != nullptr;
}

}  // namespace

// ---------------------------------------------------------------------------
FGOInformationManager::FGOInformationManager() : params_{} {
  logger_ = glim::create_module_logger("fgo_info");
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  params_.local_navigation_maximum_horizon_s = config.param<double>(
      "odometry_estimation", "smoother_lag", 0.0);
}

FGOInformationManager::FGOInformationManager(const Params& params)
: params_(params) {
  logger_ = glim::create_module_logger("fgo_info");
}

// ---------------------------------------------------------------------------
void FGOInformationManager::extract(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    long frame_id,
    double stamp) {

  FGOPositionInfo info;
  info.stamp = stamp;
  info.frame_id = frame_id;

  try {
    const gtsam::Pose3 pose = smoother.calculateEstimate<gtsam::Pose3>(X(frame_id));
    info.p_world = pose.translation();

    // Extract 6×6 marginal covariance for Pose3: [rotation(3) | translation(3)]
    const gtsam::Matrix pose_cov = smoother.marginalCovariance(X(frame_id));

    info.pose_cov_6x6 = pose_cov;
    // Position block is the lower-right 3×3
    info.sigma_p = pose_cov.block<3, 3>(3, 3);
    info.pose_cov_valid = true;

    // Validate: check eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(info.sigma_p);
    if (eig.eigenvalues().minCoeff() < params_.min_eigenvalue) {
      logger_->warn("[fgo_info] Near-singular Σ_p at frame {} (min_eig={:.2e})",
                    frame_id, eig.eigenvalues().minCoeff());
      // Still use it but flag
    }

    info.eig_vals = eig.eigenvalues();
    info.sigma_E = std::sqrt(std::max(0.0, info.sigma_p(0, 0)));
    info.sigma_N = std::sqrt(std::max(0.0, info.sigma_p(1, 1)));
    info.sigma_U = std::sqrt(std::max(0.0, info.sigma_p(2, 2)));

    // Information matrix Λ = Σ^{-1}
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    info.lambda_p = info.sigma_p.llt().solve(I3);

    info.valid = true;

    logger_->trace("[fgo_info] frame={} σ_E={:.4f} σ_N={:.4f} σ_U={:.4f} "
                   "eig=[{:.4f},{:.4f},{:.4f}]",
                   frame_id, info.sigma_E, info.sigma_N, info.sigma_U,
                   info.eig_vals(0), info.eig_vals(1), info.eig_vals(2));

  } catch (const std::exception& e) {
    logger_->warn("[fgo_info] marginalCovariance(X({})) failed: {}",
                  frame_id, e.what());
    info.valid = false;
  }

  // Build the independent local-navigation marginal.  The existing smoother
  // marginal above remains the truthful all-source diagnostic; it is never
  // reused as local execution authority.
  try {
    const auto values = smoother.calculateEstimate();
    const auto& factors = smoother.getFactors();
    gtsam::NonlinearFactorGraph local_graph;
    long oldest_pose_index = frame_id;
    bool have_pose_key = false;
    const gtsam::ImuFactor* latest_imu = nullptr;
    const gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>* latest_bias =
        nullptr;
    for (const auto& factor : factors) {
      if (!factor) {
        continue;
      }
      if (is_explicitly_excluded_nonlocal_factor(factor)) {
        continue;
      }
      if (!touches_local_navigation_state(factor)) {
        continue;
      }
      if (!is_admitted_local_factor(factor)) {
        throw std::runtime_error(
            "unrecognized factor touches local-navigation X/V/B state");
      }
      local_graph.push_back(factor);
      for (const gtsam::Key key : factor->keys()) {
        const gtsam::Symbol symbol(key);
        if (symbol.chr() == 'x') {
          oldest_pose_index = have_pose_key
              ? std::min(oldest_pose_index,
                         static_cast<long>(symbol.index()))
              : static_cast<long>(symbol.index());
          have_pose_key = true;
        }
      }
      if (const auto* imu = dynamic_cast<const gtsam::ImuFactor*>(
              factor.get())) {
        const auto& keys = imu->keys();
        if (keys.size() >= 3 && keys[2] == X(frame_id)) {
          latest_imu = imu;
        }
      }
      if (const auto* bias = dynamic_cast<const gtsam::BetweenFactor<
              gtsam::imuBias::ConstantBias>*>(factor.get())) {
        const auto& keys = bias->keys();
        if (keys.size() == 2 && keys[1] == B(frame_id)) {
          latest_bias = bias;
        }
      }
    }

    if (!have_pose_key || !values.exists(X(frame_id)) ||
        !values.exists(V(frame_id)) || !values.exists(B(frame_id)) ||
        !values.exists(X(oldest_pose_index)) || latest_imu == nullptr ||
        latest_bias == nullptr) {
      throw std::runtime_error("local graph lacks X/V/B or IMU model evidence");
    }

    local_graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3>>(
        X(oldest_pose_index), values.at<gtsam::Pose3>(X(oldest_pose_index)));
    gtsam::Marginals marginals(local_graph, values, gtsam::Marginals::QR);
    const gtsam::KeyVector joint_keys = {
        X(frame_id), V(frame_id), B(frame_id)};
    const gtsam::Matrix joint =
        marginals.jointMarginalCovariance(joint_keys).fullMatrix();
    if (joint.rows() != 15 || joint.cols() != 15 || !joint.allFinite()) {
      throw std::runtime_error("local joint marginal is not finite 15x15");
    }

    auto& source = info.local_navigation_source;
    source.state_covariance = 0.5 * (joint + joint.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> local_eig(
        source.state_covariance);
    if (local_eig.info() != Eigen::Success ||
        local_eig.eigenvalues().minCoeff() <= params_.min_eigenvalue) {
      throw std::runtime_error(
          "local joint marginal is singular or non-positive");
    }
    source.world_R_body =
        values.at<gtsam::Pose3>(X(frame_id)).rotation().matrix();
    const auto& pim = latest_imu->preintegratedMeasurements();
    const double imu_dt = pim.deltaTij();
    if (!(imu_dt > 0.0) || !std::isfinite(imu_dt)) {
      throw std::runtime_error("latest IMU preintegration duration invalid");
    }
    source.corrected_specific_force_body = pim.deltaVij() / imu_dt;
    source.corrected_angular_rate_body =
        gtsam::Rot3::Logmap(pim.deltaRij()) / imu_dt;
    source.stamp_s = stamp;
    source.estimation_frame_id = frame_id;
    source.source_contains_gnss = false;
    source.icp_degenerate = true;  // monitor binds current LiDAR health.

    auto& model = info.local_navigation_model;
    const auto& imu_params = *pim.params();
    model.accelerometer_noise_covariance =
        imu_params.accelerometerCovariance;
    model.gyroscope_noise_covariance = imu_params.gyroscopeCovariance;
    model.integration_noise_covariance = imu_params.integrationCovariance;
    const auto bias_noise = std::dynamic_pointer_cast<
        gtsam::noiseModel::Gaussian>(latest_bias->noiseModel());
    if (!bias_noise) {
      throw std::runtime_error("bias random-walk model is not Gaussian");
    }
    const gtsam::Matrix bias_covariance = bias_noise->covariance();
    if (bias_covariance.rows() != 6 || bias_covariance.cols() != 6 ||
        !bias_covariance.allFinite()) {
      throw std::runtime_error("bias random-walk covariance invalid");
    }
    model.accelerometer_bias_random_walk_covariance =
        bias_covariance.block<3, 3>(0, 0) / imu_dt;
    model.gyroscope_bias_random_walk_covariance =
        bias_covariance.block<3, 3>(3, 3) / imu_dt;
    model.maximum_horizon_s = params_.local_navigation_maximum_horizon_s;

    std::ostringstream provenance;
    provenance << frame_id << ';' << std::hexfloat << stamp << ';'
               << local_graph.size() << ';' << source.state_covariance << ';'
               << model.accelerometer_noise_covariance << ';'
               << model.gyroscope_noise_covariance << ';'
               << model.integration_noise_covariance << ';'
               << model.accelerometer_bias_random_walk_covariance << ';'
               << model.gyroscope_bias_random_walk_covariance << ';'
               << model.maximum_horizon_s;
    source.source_identity = "local_fgo:" + stable_identity(provenance.str());
    model.identity = "imu_error_model:" + stable_identity(provenance.str());
    source.model_identity = model.identity;
    source.valid = true;
    source.invalid_reason = "valid";
    model.valid = true;
  } catch (const std::exception& e) {
    info.local_navigation_source.valid = false;
    info.local_navigation_source.source_contains_gnss = true;
    info.local_navigation_source.invalid_reason = e.what();
    info.local_navigation_model.valid = false;
    logger_->warn("[fgo_info] independent local marginal failed at frame {}: {}",
                  frame_id, e.what());
  }

  // Factor counting (optional, for diagnostics)
  if (params_.count_factors) {
    try {
      std::set<int> gnss_sat_ids;
      std::set<char> gnss_constellations;
      std::set<int> trunk_landmark_ids;
      std::set<std::string> factor_tags;
      std::set<gtsam::Key> window_keys;

      const auto& factors = smoother.getFactors();
      for (const auto& factor : factors) {
        if (!factor) continue;

        ++info.n_total_factors;
        for (gtsam::Key key : factor->keys()) {
          window_keys.insert(key);
        }

        if (const auto* pr = dynamic_cast<const PseudorangeFactor*>(factor.get())) {
          ++info.n_gnss_factors;
          gnss_sat_ids.insert(pr->sat_id());
          gnss_constellations.insert(pr->constellation());
          factor_tags.insert("PseudorangeFactor");
          continue;
        }

        if (const auto* dop = dynamic_cast<const DopplerFactor*>(factor.get())) {
          ++info.n_gnss_factors;
          gnss_sat_ids.insert(dop->sat_id());
          gnss_constellations.insert(dop->constellation());
          factor_tags.insert("DopplerFactor");
          continue;
        }

        if (dynamic_cast<const ClockBetweenFactor*>(factor.get())) {
          ++info.n_clock_factors;
          factor_tags.insert("ClockBetweenFactor");
          continue;
        }

        if (dynamic_cast<const gtsam::ImuFactor*>(factor.get()) ||
            dynamic_cast<const gtsam_points::ReintegratedImuFactor*>(factor.get())) {
          ++info.n_imu_factors;
          factor_tags.insert("ImuFactor");
          continue;
        }

        if (const auto* trunk = dynamic_cast<const TrunkFactor*>(factor.get())) {
          ++info.n_trunk_factors;
          factor_tags.insert("TrunkFactor");
          const auto& keys = trunk->keys();
          if (keys.size() >= 2) {
            const gtsam::Symbol lm_symbol(keys[1]);
            if (lm_symbol.chr() == 'l') {
              trunk_landmark_ids.insert(static_cast<int>(lm_symbol.index()));
            }
          }
          continue;
        }

        ++info.n_other_factors;
      }

      info.window_key_count = static_cast<int>(window_keys.size());
      info.gnss_sat_ids.assign(gnss_sat_ids.begin(), gnss_sat_ids.end());
      info.gnss_constellations.assign(gnss_constellations.begin(), gnss_constellations.end());
      info.trunk_landmark_ids.assign(trunk_landmark_ids.begin(), trunk_landmark_ids.end());
      info.factor_type_tags.assign(factor_tags.begin(), factor_tags.end());

      logger_->trace(
          "[fgo_info] frame={} factors total={} gnss={} trunk={} imu={} clock={} other={} keys={}",
          frame_id, info.n_total_factors, info.n_gnss_factors,
          info.n_trunk_factors, info.n_imu_factors, info.n_clock_factors,
          info.n_other_factors, info.window_key_count);
    } catch (...) {}
  }

  // Thread-safe store
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_info_ = info;
  }
}

// ---------------------------------------------------------------------------
FGOPositionInfo FGOInformationManager::latest() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return latest_info_;
}

// ---------------------------------------------------------------------------
bool FGOInformationManager::has_data() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return latest_info_.valid;
}

}  // namespace iap
