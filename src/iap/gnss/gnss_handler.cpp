// IAP-RQ-020: GnssHandler implementation

#include <iap/gnss/gnss_handler.hpp>
#include <iap/gnss/constellation_clock.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <set>
#include <numeric>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::E;  // ECEF origin of world frame  E(0)
using gtsam::symbol_shorthand::R;  // world→ECEF rotation         R(0)

namespace iap {

Eigen::MatrixXd gnss_postopt_joint_covariance(
    const gtsam::GaussianFactorGraph& linear_graph,
    const gtsam::KeyVector& keys, const std::vector<std::uint32_t>& dimensions) {
  if (keys.empty() || keys.size()!=dimensions.size() || keys.size()>9 ||
      std::set<gtsam::Key>(keys.begin(),keys.end()).size()!=keys.size())
    throw std::invalid_argument("invalid postopt joint keys");
  std::uint32_t size=0;
  for (auto dimension:dimensions) {
    if (dimension==0 || dimension>6) throw std::invalid_argument("invalid postopt block dimension");
    size+=dimension;
  }
  if (size>29) throw std::invalid_argument("postopt joint dimension exceeds model");
  const auto marginal=linear_graph.marginal(keys);
  const auto actual_dimensions=marginal->getKeyDimMap();
  for(std::size_t i=0;i<keys.size();++i) {
    const auto found=actual_dimensions.find(keys[i]);
    if(found==actual_dimensions.end() || found->second!=dimensions[i])
      throw std::runtime_error("postopt joint block dimension mismatch");
  }
  const auto information=marginal->hessian(gtsam::Ordering(keys.begin(),keys.end())).first;
  if (information.rows()!=size || information.cols()!=size || !information.allFinite() ||
      (information-information.transpose()).norm()>1e-9*std::max(1.,information.norm()))
    throw std::runtime_error("postopt joint information unavailable");
  const Eigen::LDLT<Eigen::MatrixXd> factorization(information);
  if (factorization.info()!=Eigen::Success || factorization.vectorD().minCoeff()<=0.)
    throw std::runtime_error("postopt joint information not positive definite");
  const Eigen::MatrixXd covariance=factorization.solve(Eigen::MatrixXd::Identity(size,size));
  if (!covariance.allFinite()) throw std::runtime_error("postopt joint covariance nonfinite");
  return covariance;
}

Eigen::Matrix2d gnss_clock_difference_covariance(
    const gtsam::GaussianFactorGraph& linear_graph,
    gtsam::Key reference, gtsam::Key system) {
  if (reference == system) throw std::invalid_argument("clock difference requires distinct states");
  const auto marginal = linear_graph.marginal(gtsam::KeyVector{reference, system});
  const auto information = marginal->hessian(gtsam::Ordering{reference, system}).first;
  if (information.rows() != 4 || !information.allFinite()) {
    throw std::runtime_error("clock joint information unavailable");
  }
  const Eigen::LDLT<Eigen::Matrix4d> factorization(information);
  if (factorization.info() != Eigen::Success || factorization.vectorD().minCoeff() <= 0.0) {
    throw std::runtime_error("clock joint information not positive definite");
  }
  const Eigen::Matrix4d covariance = factorization.solve(Eigen::Matrix4d::Identity());
  Eigen::Matrix<double, 2, 4> difference;
  difference << -Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d result = difference * covariance * difference.transpose();
  if (!result.allFinite()) throw std::runtime_error("clock difference covariance nonfinite");
  return result;
}


void propagate_gnss_postopt(GnssPostoptEvidence& e,
    const gtsam::PreintegratedImuMeasurements& motion) {
  const double dt=e.gnss_stamp-e.state_stamp;
  if(!e.optimized_valid || !e.covariance_valid || dt<=0. || dt>.1 ||
      std::abs(motion.deltaTij()-dt)>1e-6)
    throw std::invalid_argument("postopt epoch propagation unavailable");
  const int n=std::accumulate(e.tangent_dimensions.begin(),e.tangent_dimensions.end(),0);
  if(e.keys.size()!=e.tangent_dimensions.size() || n<21 || n>29 ||
      e.joint_covariance_row_major.size()!=std::size_t(n*n))
    throw std::invalid_argument("postopt propagation layout invalid");
  const auto decode=[&](const std::vector<double>& m) {
    if(m.size()!=37+2*e.used_constellations.size())throw std::invalid_argument("postopt mean layout invalid");
    gtsam::Values v;
    Eigen::Matrix4d t;for(int i=0;i<16;++i)t(i/4,i%4)=m[i];
    v.insert(e.keys[0],gtsam::Pose3(t));
    v.insert(e.keys[1],gtsam::Vector3(m[16],m[17],m[18]));
    v.insert(e.keys[2],gtsam::imuBias::ConstantBias(gtsam::Vector3(m[19],m[20],m[21]),gtsam::Vector3(m[22],m[23],m[24])));
    Eigen::Matrix3d r;for(int i=0;i<9;++i)r(i/3,i%3)=m[25+i];
    v.insert(e.keys[3],gtsam::Rot3(r));v.insert(e.keys[4],gtsam::Vector3(m[34],m[35],m[36]));
    for(std::size_t i=5;i<e.keys.size();++i)v.insert(e.keys[i],gtsam::Vector2(m[37+2*(i-5)],m[38+2*(i-5)]));
    return v;
  };
  const auto propagate=[&](const gtsam::Values& v) {
    auto out=v;
    const auto state=motion.predict(gtsam::NavState(v.at<gtsam::Pose3>(e.keys[0]),v.at<gtsam::Vector3>(e.keys[1])),
                                    v.at<gtsam::imuBias::ConstantBias>(e.keys[2]));
    out.update(e.keys[0],state.pose());out.update(e.keys[1],state.velocity());
    for(std::size_t i=5;i<e.keys.size();++i) {
      auto clock=v.at<gtsam::Vector2>(e.keys[i]);clock(0)+=dt*clock(1);out.update(e.keys[i],clock);
    }
    return out;
  };
  const auto encode=[&](const gtsam::Values& v) {
    auto m=e.linearization_means;
    const auto t=v.at<gtsam::Pose3>(e.keys[0]).matrix();
    for(int i=0;i<16;++i)m[i]=t(i/4,i%4);
    for(int i=0;i<3;++i)m[16+i]=v.at<gtsam::Vector3>(e.keys[1])(i);
    for(std::size_t i=5;i<e.keys.size();++i)for(int j=0;j<2;++j)m[37+2*(i-5)+j]=v.at<gtsam::Vector2>(e.keys[i])(j);
    return m;
  };
  const auto linear=decode(e.linearization_means),pred=propagate(linear);
  e.propagated_linearization_means=encode(pred);
  // Preserve separately captured optimized R/E/B as well.
  const auto optimized=decode(e.optimized_means);
  e.propagated_optimized_means=e.optimized_means;
  const auto op=encode(propagate(optimized));
  std::copy(op.begin(),op.begin()+19,e.propagated_optimized_means.begin());
  std::copy(op.begin()+37,op.end(),e.propagated_optimized_means.begin()+37);
  Eigen::MatrixXd f(n,n);int offset=0;
  for(std::size_t k=0;k<e.keys.size();++k) {
    for(unsigned j=0;j<e.tangent_dimensions[k];++j) {
      gtsam::VectorValues d;
      for(std::size_t i=0;i<e.keys.size();++i)d.insert(e.keys[i],gtsam::Vector::Zero(e.tangent_dimensions[i]));
      d.at(e.keys[k])(j)=1e-5;
      const auto plus=propagate(linear.retract(d));d.at(e.keys[k])(j)=-1e-5;
      const auto minus=propagate(linear.retract(d));int row=0;
      for(std::size_t i=0;i<e.keys.size();++i) {
        const auto vp=pred.at(e.keys[i]).localCoordinates_(plus.at(e.keys[i]));
        const auto vm=pred.at(e.keys[i]).localCoordinates_(minus.at(e.keys[i]));
        f.block(row,offset+j,e.tangent_dimensions[i],1)=(vp-vm)/(2e-5);row+=e.tangent_dimensions[i];
      }
    }
    offset+=e.tangent_dimensions[k];
  }
  // PIM noise is [delta rotation, delta position, delta velocity] in initial
  // body coordinates. Convert to endpoint Pose3-local p, world v. All original
  // X/V/B/R/E/clock cross terms propagate through the full transition matrix.
  Eigen::Matrix<double,Eigen::Dynamic,9> j=Eigen::MatrixXd::Zero(n,9);
  j.topLeftCorner<3,3>().setIdentity();
#ifdef GTSAM_TANGENT_PREINTEGRATION
  j.topLeftCorner<3,3>()=gtsam::Rot3::ExpmapDerivative(gtsam::Rot3::Logmap(motion.deltaRij()));
#endif
  j.block<3,3>(3,3)=motion.deltaRij().matrix().transpose();
  j.block<3,3>(6,6)=linear.at<gtsam::Pose3>(e.keys[0]).rotation().matrix();
  const Eigen::MatrixXd q=j*motion.preintMeasCov()*j.transpose();
  Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> p(e.joint_covariance_row_major.data(),n,n);
  // The same IMU segment also participates in the GNSS factor. Its error can
  // therefore correlate with the optimized state error; an independent Q sum
  // would claim unsupported covariance. Bound that unknown cross term by
  // Cauchy, while retaining every known state-state cross term in F P F^T.
  const Eigen::MatrixXd propagated=2.*(f*p*f.transpose()+q);
  auto flatten=[](const Eigen::MatrixXd& a) {std::vector<double> out;out.reserve(a.size());for(int i=0;i<a.rows();++i)for(int j=0;j<a.cols();++j)out.push_back(a(i,j));return out;};
  e.propagation_transition=flatten(f);e.propagation_noise=flatten(q);
  e.propagated_joint_covariance=flatten((propagated+propagated.transpose())*.5);
  e.propagation="IMU_TO_GNSS_EPOCH";
}

GnssHandler::GnssHandler() : params_(Params{}) {}
GnssHandler::GnssHandler(const Params& params) : params_(params) {}

void GnssHandler::insert_epoch(const GnssEpoch& epoch) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (static_cast<int>(epoch_queue_.size()) >= params_.max_epoch_queue) {
    epoch_queue_.pop_front();  // back-pressure: drop oldest
  }
  epoch_queue_.push_back(epoch);
}

std::size_t GnssHandler::queue_size() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return epoch_queue_.size();
}

double GnssHandler::pr_sigma(double elevation, double kappa) const {
  const double s = std::sin(std::max(elevation, params_.min_elevation));
  const double base_sigma = params_.pr_noise_base / std::pow(s, params_.elev_noise_exp);

  // Open-sky epochs should be controlled by the experiment's receiver noise
  // setting. The full canopy model intentionally inflates obstructed LOS, but
  // its canopy floor is too conservative for nominal open-sky validation.
  if (kappa <= 1.0e-6) {
    return std::max(base_sigma, 0.05);
  }

  return std::max(base_sigma, sigma_eff_canopy(params_.canopy, kappa, elevation));
}

double GnssHandler::dop_sigma(double elevation) const {
  const double s = std::sin(std::max(elevation, params_.min_elevation));
  return params_.dop_noise_base / std::pow(s, params_.elev_noise_exp);
}

gtsam::NonlinearFactorGraph GnssHandler::get_factors(
    int                     frame_idx,
    double                  frame_stamp,
    const Eigen::Vector3d&  anc_ecef,
    std::vector<GnssEpoch>* out_epochs, const MotionProvider& motion_provider) {

  // Bind at most one receiver epoch to a state. Multiple observations from
  // distinct times constrain distinct clock/velocity states and must never be
  // collapsed onto one LiDAR frame during startup backlog recovery.
  std::vector<GnssEpoch> matched;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto& q = epoch_queue_;
    const double oldest_allowed = frame_stamp - params_.time_tolerance;
    auto it = q.begin();
    while (it != q.end()) {
      if (it->stamp < oldest_allowed) {
        it = q.erase(it);  // too old — discard
      } else {
        ++it;
      }
    }

    auto nearest = q.end();
    double nearest_dt = params_.time_tolerance;
    for (it = q.begin(); it != q.end(); ++it) {
      const double dt = std::abs(it->stamp - frame_stamp);
      // Actual IMU propagation is forward-only. Never silently attach an
      // earlier acquisition epoch to a later state in the production path.
      if(it->stamp<frame_stamp) continue;
      if (dt <= nearest_dt) {
        nearest = it;
        nearest_dt = dt;
      }
    }
    if (nearest != q.end()) {
      const double selected_stamp = nearest->stamp;
      matched.push_back(std::move(*nearest));
      // The selected epoch supersedes every older buffered epoch. Preserve
      // later epochs, including those also inside this frame's tolerance, for
      // their own future state.
      q.erase(std::remove_if(q.begin(), q.end(), [selected_stamp](const GnssEpoch& epoch) {
        return epoch.stamp <= selected_stamp;
      }), q.end());
    }
  }

  if (out_epochs) {
    *out_epochs = matched;
  }

  gtsam::NonlinearFactorGraph graph;

  for (const auto& epoch : matched) {
    const double dt=epoch.stamp-frame_stamp;
    const auto motion=motion_provider && dt>1e-6 ? motion_provider(frame_stamp,epoch.stamp) : nullptr;
    if(dt>1e-6 && !motion) {
      if(out_epochs) out_epochs->clear();
      continue; // Missing or incomplete IMU coverage is not zero motion.
    }
    // One common preintegration error affects all PR/Doppler rows. For m
    // scalar projections of one covariance Q, Cauchy gives R <= m diag(Rii).
    // A PR projection has squared norm <= (1+|lever|)^2; a far-field Doppler
    // projection has squared norm <= 2 (relative speed / range <= 1 s^-1).
    // Thus 2 m (1+|lever|)^2 trace(Q) bounds every row, including common
    // correlations. This is a declared normal far-field model, not calibrated
    // multipath, IMU bias evolution or fault integrity.
    const double common_variance=motion ? 4.*epoch.sats.size()*
        motion->preintMeasCov().trace()*std::pow(1.+params_.lever_arm.norm(),2) : 0.;
    for (const auto& sat : epoch.sats) {
      if (sat.excluded || sat.elevation < params_.min_elevation) continue;
      const auto clock_key = gnss_clock_key(sat.constellation, frame_idx);

      // ── PseudorangeFactor ────────────────────────────────────────────────
      // Keys: X(i), constellation receiver clock(i), E(0), R(0)
      // The receiver-declared sigma and modeled floor have one authority.
      // An elevation/canopy floor must never replace a larger measurement
      // uncertainty (e.g. deliberate receiver noise degradation).
      const double sigma_pr = std::max(pr_sigma(sat.elevation, sat.kappa),sat.pr_sigma);
      graph.emplace_shared<PseudorangeFactor>(
        X(frame_idx), clock_key, E(0), R(0),
        sat.pr_meas,
        sat.sat_pos,
        sat.tgd,
        epoch.gps_sec,
        epoch.iono_params,
        gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(sigma_pr*sigma_pr+common_variance)),
        params_.lever_arm,
        sat.sat_id,
        sat.constellation,
        sat.elevation);
      if(motion) std::static_pointer_cast<PseudorangeFactor>(graph.back())->bind_epoch_motion(
          V(frame_idx),gtsam::symbol_shorthand::B(frame_idx),motion);

      // ── DopplerFactor ──────────────────────────────────────────────────
      // Keys: X(i), V(i), constellation receiver clock(i), R(0)
      const double sigma_dop = std::max(dop_sigma(sat.elevation),sat.dop_sigma);
      graph.emplace_shared<DopplerFactor>(
        X(frame_idx), V(frame_idx), clock_key, R(0),
        sat.dop_meas,
        sat.sat_pos,
        sat.sat_vel,
        anc_ecef,
        gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(sigma_dop*sigma_dop+common_variance)),
        sat.sat_id,
        sat.constellation,
        sat.elevation);
      if(motion) std::static_pointer_cast<DopplerFactor>(graph.back())->bind_epoch_motion(
          gtsam::symbol_shorthand::B(frame_idx),motion);
    }
  }

  return graph;
}

}  // namespace iap
