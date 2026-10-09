#include <gtest/gtest.h>
#include <iap/gnss/gnss_extension.hpp>
#include <iap/gnss/constellation_clock.hpp>
#include <iap/util/config.hpp>
#include <iap/util/shared_state.hpp>
#include <iap/msg/gnss_postopt_evidence.hpp>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace iap {
struct GnssClockInjectionTestAccess {
  static void prepare(GnssExtensionModule& module, bool gps_owned, const GnssEpoch& epoch) {
    module.reset_clock_chain_state_("test_reset", epoch.stamp);
    module.ext_vars_inserted_ = false;
    module.origin_set_ = true;
    module.origin_ecef_.setZero();
    module.last_frame_id_ = 1;
    module.last_frame_stamp_ = epoch.stamp;
    module.gnss_owns_clock_ = gps_owned;
    module.gnss_handler_->insert_epoch(epoch);
  }
  static void inject(GnssExtensionModule& module,
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph& factors, gtsam::Values& values,
      std::map<std::uint64_t, double>& stamps) {
    module.on_smoother_update_(smoother, factors, values, stamps);
  }
  static void finish(GnssExtensionModule& module,
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
    module.on_smoother_update_finish_(smoother);
  }
  static void steer_frame_only(GnssExtensionModule& module,long id,double stamp) {
    std::lock_guard<std::mutex> lock(module.frame_mutex_);
    module.last_frame_id_=id;module.last_frame_stamp_=stamp;
  }
  static void advance(GnssExtensionModule& module, long frame, const GnssEpoch& epoch, bool reset) {
    if (reset) module.reset_clock_chain_state_("test_reset", epoch.stamp);
    module.last_frame_id_ = frame;
    module.last_frame_stamp_ = epoch.stamp;
    module.gnss_handler_->insert_epoch(epoch);
  }
};
}  // namespace iap

namespace {
gtsam::ISAM2Params production_clock_params() {
  glim::RelinearizationPolicyRegistry registry;
  registry.register_policy('x', 6, gtsam::Vector6::Constant(0.1));
  registry.register_policy('b', 6, gtsam::Vector6::Constant(0.1));
  for (const char symbol : {'v', 'e', 'r'}) registry.register_policy(symbol, 3, gtsam::Vector3::Constant(0.1));
  iap::register_gnss_clock_relinearization(registry, gtsam::Vector2(500, 5));
  registry.validate_or_throw();
  gtsam::ISAM2Params params;
  params.setRelinearizeThreshold(registry.build_map());
  return params;
}
}  // namespace

TEST(GnssClockInjection, RemovingEitherConstellationDoesNotCreateUnusedClock) {
  const auto root = std::filesystem::temp_directory_path() /
      ("iap_clock_injection_" + std::to_string(::getpid()));
  struct Cleanup { std::filesystem::path path; ~Cleanup() { std::filesystem::remove_all(path); } } cleanup{root};
  std::filesystem::create_directories(root);
  std::ofstream(root / "config.json") << R"({"global":{"config_path":"","config_gnss":"config_gnss.json","config_odometry":"config_odometry.json"}})";
  std::ofstream(root / "config_gnss.json") << R"({"gnss":{"enable_debug_csv":false}})";
  std::ofstream(root / "config_odometry.json") << R"({"odometry_estimation":{"clock_owner_mode":"gnss"}})";
  glim::GlobalConfig::instance(root.string(), true);
  iap::GnssExtensionModule extension;
  for (const int owner_case : {0, 1, 2}) {
    const bool gps_owned = owner_case == 0;
    for (const std::string systems : {"G", "C", "GC", ""}) {
      SCOPED_TRACE(systems + ":owner_case=" + std::to_string(owner_case));
      iap::GnssEpoch epoch;
      epoch.stamp = 100;
      epoch.source_identity = 1701;
      for (const char system : systems) {
        for (int i = 0; i < 8; ++i) {
          iap::SatObs satellite;
          satellite.constellation = system;
          satellite.sat_id = (system == 'G' ? 1 : 98) + i;
          satellite.elevation = 1;
          satellite.sat_pos = Eigen::Vector3d(21000000 + i * 300000,
              14000000 - i * 170000, 17000000 + (i % 3) * 400000);
          satellite.pr_meas = satellite.sat_pos.norm();
          epoch.sats.push_back(satellite);
        }
      }
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback smoother(5, production_clock_params());
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values values;
      std::map<std::uint64_t, double> stamps;
      const auto pose = gtsam::Symbol('x', 1), velocity = gtsam::Symbol('v', 1);
      values.insert(pose, gtsam::Pose3());
      const auto bias=gtsam::Symbol('b',1);
      values.insert(bias,gtsam::imuBias::ConstantBias());
      factors.addPrior<gtsam::imuBias::ConstantBias>(bias,gtsam::imuBias::ConstantBias(),gtsam::noiseModel::Isotropic::Sigma(6,.01));
      stamps[bias]=epoch.stamp;
      values.insert(velocity, gtsam::Vector3::Zero().eval());
      factors.addPrior<gtsam::Pose3>(pose, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 0.001));
      factors.addPrior<gtsam::Vector3>(velocity, gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 0.001));
      stamps[pose] = stamps[velocity] = epoch.stamp;
      // Existing odometry GPS state is preserved, even without GPS data.
      if (!gps_owned && (systems.find('G') != std::string::npos || owner_case == 1)) {
        const auto clock = iap::gnss_clock_key('G', 1);
        values.insert(clock, gtsam::Vector2::Zero().eval());
        factors.addPrior<gtsam::Vector2>(clock, gtsam::Vector2::Zero(), gtsam::noiseModel::Isotropic::Sigma(2, 1));
        stamps[clock] = epoch.stamp;
      }
      iap::GnssClockInjectionTestAccess::prepare(extension, gps_owned, epoch);
      iap::GnssClockInjectionTestAccess::inject(extension, smoother, factors, values, stamps);
      EXPECT_EQ(values.exists(iap::gnss_clock_key('G', 1)), systems.find('G') != std::string::npos || owner_case == 1);
      EXPECT_EQ(values.exists(iap::gnss_clock_key('C', 1)), systems.find('C') != std::string::npos);
      EXPECT_EQ(iap::IapSharedState::instance().is_clock_ready(1), systems.find('G') != std::string::npos);
      if (gps_owned && systems.find('G') == std::string::npos && values.exists(iap::gnss_clock_key('G', 1))) continue;
      ASSERT_NO_THROW(smoother.update(factors, values, stamps));
      EXPECT_FALSE(smoother.fallbackHappened());
      if (!systems.empty()) {
        // A newer sensor frame must not relabel the injected state/epoch.
        iap::GnssClockInjectionTestAccess::steer_frame_only(extension,999,101.);
        ASSERT_NO_THROW(iap::GnssClockInjectionTestAccess::finish(extension, smoother));
        EXPECT_FALSE(smoother.fallbackHappened());
        const auto bundle=iap::IapSharedState::instance().get_gnss_postopt_bundle();
        ASSERT_TRUE(bundle.epoch);EXPECT_EQ(bundle.epoch->source_identity,epoch.source_identity);
        EXPECT_EQ(bundle.coordinates.frame_id,1);EXPECT_DOUBLE_EQ(bundle.coordinates.stamp,100.);
        EXPECT_EQ(bundle.state.frame_id,1);EXPECT_DOUBLE_EQ(bundle.state.state_stamp,100.);
        EXPECT_EQ(bundle.state.epoch_source_identity,epoch.source_identity);
        EXPECT_EQ(bundle.state.used_constellations,systems=="GC" ? "CG" : systems);
        EXPECT_TRUE(bundle.state.optimized_valid)<<bundle.state.failure_reason;
        EXPECT_TRUE(bundle.state.covariance_valid)<<bundle.state.failure_reason;
        EXPECT_EQ(bundle.state.propagation,"NOT_PROPAGATED");
        const auto dimension=21+2*systems.size();
        EXPECT_EQ(bundle.state.joint_covariance_row_major.size(),dimension*dimension);
        EXPECT_EQ(bundle.state.optimized_means.size(),37+2*systems.size());
        EXPECT_EQ(bundle.state.linearization_means.size(),37+2*systems.size());
        const auto wire=iap::copy_gnss_postopt_evidence<iap::msg::GnssPostoptEvidence>(bundle.state);
        const auto copy=iap::copy_gnss_postopt_evidence<iap::GnssPostoptEvidence>(wire);
        EXPECT_EQ(copy.joint_covariance_row_major,bundle.state.joint_covariance_row_major);
        EXPECT_EQ(copy.linearization_means,bundle.state.linearization_means);
        ASSERT_NO_THROW(iap::GnssClockInjectionTestAccess::finish(extension,smoother));
        const auto missing=iap::IapSharedState::instance().get_gnss_postopt_bundle();
        EXPECT_FALSE(missing.epoch);EXPECT_FALSE(missing.coordinates.valid);
        EXPECT_FALSE(missing.state.optimized_valid);EXPECT_FALSE(missing.state.covariance_valid);
        EXPECT_TRUE(missing.state.joint_covariance_row_major.empty());
        EXPECT_GT(missing.state.update_sequence,bundle.state.update_sequence);
      }
    }
  }

  // Drive the production chains across independent drift, GPS disappearance,
  // the original two-second gap guard, and reset; no replay gets a new time.
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback smoother(20, production_clock_params());
  const double times[] = {100.0, 100.1, 100.2, 103.2, 103.3};
  const std::string systems[] = {"GC", "GC", "C", "GC", "GC"};
  for (long frame = 1; frame <= 5; ++frame) {
    SCOPED_TRACE("chain frame " + std::to_string(frame));
    iap::GnssEpoch epoch;
    epoch.stamp = times[frame - 1];
    for (const char system : systems[frame - 1]) {
      for (int i = 0; i < 8; ++i) {
        iap::SatObs sat;
        sat.constellation = system;
        sat.sat_id = (system == 'G' ? 1 : 98) + i;
        sat.elevation = 1;
        sat.sat_pos = Eigen::Vector3d(21000000 + i * 300000, 14000000 - i * 170000,
            17000000 + (i % 3) * 400000);
        sat.pr_meas = sat.sat_pos.norm() + (system == 'G' ? 12 : 45) +
            (system == 'G' ? 0.3 : 0.8) * (epoch.stamp - 100);
        sat.dop_meas = system == 'G' ? 0.3 : 0.8;
        epoch.sats.push_back(sat);
      }
    }
    if (frame == 1) iap::GnssClockInjectionTestAccess::prepare(extension, true, epoch);
    else iap::GnssClockInjectionTestAccess::advance(extension, frame, epoch, frame == 5);
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    std::map<std::uint64_t, double> stamps;
    const auto pose = gtsam::Symbol('x', frame), velocity = gtsam::Symbol('v', frame);
    values.insert(pose, gtsam::Pose3());
    values.insert(velocity, gtsam::Vector3::Zero().eval());
    factors.addPrior<gtsam::Pose3>(pose, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 0.001));
    factors.addPrior<gtsam::Vector3>(velocity, gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 0.001));
    stamps[pose] = stamps[velocity] = epoch.stamp;
    iap::GnssClockInjectionTestAccess::inject(extension, smoother, factors, values, stamps);
    std::map<char, int> chains;
    for (const auto& factor : factors) {
      if (std::dynamic_pointer_cast<iap::ClockBetweenFactor>(factor)) {
        const char from = gtsam::Symbol(factor->keys()[0]).chr();
        EXPECT_EQ(from, gtsam::Symbol(factor->keys()[1]).chr());
        ++chains[from];
      }
    }
    EXPECT_EQ(chains['c'], frame == 2 ? 1 : 0);
    EXPECT_EQ(chains['d'], frame == 2 || frame == 3 ? 1 : 0);
    EXPECT_EQ(values.exists(iap::gnss_clock_key('G', frame)), frame != 3);
    EXPECT_EQ(iap::IapSharedState::instance().is_clock_ready(frame), frame != 3);
    for (const char system : systems[frame - 1]) {
      const auto initial = values.at<gtsam::Vector2>(iap::gnss_clock_key(system, frame));
      if (frame == 2 || frame == 3) {
        const auto previous = smoother.calculateEstimate().at<gtsam::Vector2>(iap::gnss_clock_key(system, frame - 1));
        const double dt = epoch.stamp - times[frame - 2];
        EXPECT_NEAR(initial(0), previous(0) + dt * previous(1), 1e-10);
        EXPECT_NEAR(initial(1), previous(1), 1e-10);
      } else EXPECT_LT(initial.norm(), 1e-12);
    }
    ASSERT_NO_THROW(smoother.update(factors, values, stamps));
    ASSERT_NO_THROW(iap::GnssClockInjectionTestAccess::finish(extension, smoother));
    EXPECT_FALSE(smoother.fallbackHappened());
    // This deliberately pose/velocity-only graph lacks B. It can still own
    // the new residual epoch but cannot reuse an earlier full covariance.
    const auto incomplete=iap::IapSharedState::instance().get_gnss_postopt_bundle();
    ASSERT_TRUE(incomplete.epoch);
    EXPECT_EQ(incomplete.coordinates.frame_id,frame);
    EXPECT_FALSE(incomplete.state.optimized_valid);EXPECT_FALSE(incomplete.state.covariance_valid);
    EXPECT_TRUE(incomplete.state.joint_covariance_row_major.empty());
    EXPECT_FALSE(incomplete.state.failure_reason.empty());
  }
}

TEST(GnssEpochMotion, MotionResidualUsesAcquisitionEpochAndAllStateKeys) {
  auto params=gtsam::PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance=.0025*gtsam::Matrix3::Identity();
  params->gyroscopeCovariance=.0004*gtsam::Matrix3::Identity();
  params->integrationCovariance=.000001*gtsam::Matrix3::Identity();
  auto motion=std::make_shared<gtsam::PreintegratedImuMeasurements>(params);
  for(int i=0;i<40;++i)motion->integrateMeasurement(gtsam::Vector3(2.,0.,9.81),gtsam::Vector3::Zero(),.002);
  const double truth_x=20.*.08+.5*2.*.08*.08;
  using namespace gtsam::symbol_shorthand;
  auto noise=gtsam::noiseModel::Isotropic::Sigma(1,1.);
  iap::PseudorangeFactor pr(X(1),C(1),E(0),R(0),2e7-truth_x,
      gtsam::Vector3(2e7,0,0),0.,100.,{},noise);
  iap::DopplerFactor dop(X(1),V(1),C(1),R(0),-20.16,
      gtsam::Vector3(2e7,0,0),gtsam::Vector3::Zero(),gtsam::Vector3::Zero(),noise);
  gtsam::Values values;values.insert(X(1),gtsam::Pose3());values.insert(V(1),gtsam::Vector3(20.,0.,0.));
  values.insert(B(1),gtsam::imuBias::ConstantBias());values.insert(C(1),gtsam::Vector2::Zero().eval());
  values.insert(E(0),gtsam::Vector3::Zero().eval());values.insert(R(0),gtsam::Rot3());
  // Red witness: an unpropagated factor attaches a 1.6064 m motion error.
  EXPECT_NEAR(pr.unwhitenedError(values)(0),-truth_x,1e-8);
  EXPECT_NEAR(dop.unwhitenedError(values)(0),-.16,1e-8);
  pr.bind_epoch_motion(V(1),B(1),motion);dop.bind_epoch_motion(B(1),motion);
  std::vector<gtsam::Matrix> h;
  EXPECT_NEAR(pr.unwhitenedError(values,&h)(0),0.,1e-8);
  ASSERT_EQ(h.size(),6u);EXPECT_NEAR(h[4](0,0),.08,3e-4);EXPECT_GT(h[5].norm(),0.);
  EXPECT_NEAR(dop.unwhitenedError(values,&h)(0),0.,1e-8);
  ASSERT_EQ(h.size(),5u);EXPECT_GT(h[4].norm(),0.);
  EXPECT_GT(motion->preintMeasCov().trace(),0.);
}

TEST(GnssEpochMotion, MissingImuAndBackwardAcquisitionDoNotCreateFactors) {
  iap::GnssEpoch epoch;epoch.stamp=10.05;epoch.gps_sec=28.05;
  iap::SatObs sat;sat.elevation=1.;epoch.sats.push_back(sat);
  iap::GnssHandler handler;handler.insert_epoch(epoch);
  const iap::GnssHandler::MotionProvider missing=[](double,double) {return std::shared_ptr<const gtsam::PreintegratedImuMeasurements>{};};
  std::vector<iap::GnssEpoch> used;
  EXPECT_TRUE(handler.get_factors(1,10.,Eigen::Vector3d::Zero(),&used,missing).empty());
  EXPECT_TRUE(used.empty());epoch.stamp=9.95;handler.insert_epoch(epoch);
  EXPECT_TRUE(handler.get_factors(1,10.,Eigen::Vector3d::Zero(),&used,missing).empty());
}

TEST(GnssEpochMotion, StationaryVerticalProcessNoiseHasIndependentDiscreteIntegral) {
  auto params=gtsam::PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance=.0025*gtsam::Matrix3::Identity();
  params->gyroscopeCovariance=.0004*gtsam::Matrix3::Identity();
  params->integrationCovariance=.000001*gtsam::Matrix3::Identity();
  gtsam::PreintegratedImuMeasurements motion(params);
  for(int i=0;i<40;++i)motion.integrateMeasurement(gtsam::Vector3(0,0,9.81),gtsam::Vector3::Zero(),.002);
  const auto q=motion.preintMeasCov();
  // Independent discrete white acceleration integration, plus integration Q.
  const double t=.08,h=.002;
  EXPECT_NEAR(q(5,5),.0025*(t*t*t/3.-h*h*t/12.)+.000001*t,1e-12);
  EXPECT_NEAR(q(8,8),.0025*t,1e-12);
  EXPECT_NEAR(q(5,8),.0025*t*t/2.,1e-12);
}
