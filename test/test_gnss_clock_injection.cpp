#include <gtest/gtest.h>
#include <iap/gnss/gnss_extension.hpp>
#include <iap/gnss/constellation_clock.hpp>
#include <iap/util/config.hpp>
#include <iap/util/shared_state.hpp>
#include <gtsam/nonlinear/PriorFactor.h>
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
        ASSERT_NO_THROW(iap::GnssClockInjectionTestAccess::finish(extension, smoother));
        EXPECT_FALSE(smoother.fallbackHappened());
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
  }
}
