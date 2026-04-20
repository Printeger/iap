#include <iap/gnss/gnss_epoch_builder.hpp>

#include <gnss_comm/gnss_utility.hpp>

#include <algorithm>
#include <cmath>

namespace iap {

namespace {

static constexpr double kSpeedOfLight = 2.99792458e8;

char constellation_char(uint32_t sys) {
  switch (sys) {
    case SYS_GLO:
      return 'R';
    case SYS_GAL:
      return 'E';
    case SYS_BDS:
      return 'C';
    default:
      return 'G';
  }
}

}  // namespace

GnssEpochBuilder::GnssEpochBuilder() : params_(Params{}) {}
GnssEpochBuilder::GnssEpochBuilder(const Params& params) : params_(params) {}

void GnssEpochBuilder::set_anchor(const GnssAnchorState& anchor) {
  std::lock_guard<std::mutex> lk(mutex_);
  anchor_ = anchor;
  anchor_ready_ = true;
}

bool GnssEpochBuilder::anchor_ready() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return anchor_ready_;
}

void GnssEpochBuilder::set_iono_params(const std::vector<double>& iono_params) {
  std::lock_guard<std::mutex> lk(mutex_);
  iono_params_ = iono_params;
}

void GnssEpochBuilder::update_ephemeris(const GnssEphemerisUpdate& update) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (update.ephem) {
    ephem_cache_[update.sat_id ? update.sat_id : update.ephem->sat] = update.ephem;
  }
  if (update.glo_ephem) {
    glo_ephem_cache_[update.sat_id ? update.sat_id : update.glo_ephem->sat] = update.glo_ephem;
  }
}

void GnssEpochBuilder::update_ephemeris(const gnss_comm::EphemPtr& ephem) {
  if (!ephem) {
    return;
  }
  update_ephemeris(GnssEphemerisUpdate{ephem->sat, ephem, nullptr});
}

void GnssEpochBuilder::update_glo_ephemeris(const gnss_comm::GloEphemPtr& ephem) {
  if (!ephem) {
    return;
  }
  update_ephemeris(GnssEphemerisUpdate{ephem->sat, nullptr, ephem});
}

GnssEpochBuilder::BuildResult GnssEpochBuilder::build_epoch(const GnssRawObservationBatch& batch) const {
  BuildResult result;
  result.observation_count = batch.observations.size();
  if (batch.observations.empty()) {
    result.status = BuildStatus::EmptyBatch;
    return result;
  }

  GnssAnchorState anchor;
  std::vector<double> iono_params;
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!anchor_ready_) {
      result.status = BuildStatus::MissingAnchor;
      return result;
    }
    anchor = anchor_;
    iono_params = iono_params_;
    ephem_cache = ephem_cache_;
    glo_ephem_cache = glo_ephem_cache_;
  }

  GnssEpoch epoch;
  const auto utc_t = gnss_comm::gpst2utc(batch.observations.front()->time);
  epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
  epoch.gps_sec = static_cast<double>(batch.observations.front()->time.time) + batch.observations.front()->time.sec;
  epoch.iono_params = iono_params;

  bool saw_ephemeris = false;
  for (const auto& obs : batch.observations) {
    if (!obs) {
      continue;
    }

    int l1_idx = -1;
    const double freq = gnss_comm::L1_freq(obs, &l1_idx);
    if (l1_idx < 0 || freq <= 0.0) {
      continue;
    }

    if (static_cast<int>(obs->psr.size()) <= l1_idx) {
      continue;
    }

    const double pr = obs->psr[l1_idx];
    if (pr <= 0.0 || !std::isfinite(pr)) {
      continue;
    }

    const uint32_t sat_id = obs->sat;
    const uint32_t sys = gnss_comm::satsys(sat_id, nullptr);
    const double tau0 = pr / kSpeedOfLight;
    const auto t_tx = gnss_comm::time_add(obs->time, -tau0);

    Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
    double svdt = 0.0;
    double svddt = 0.0;
    double tgd = 0.0;

    if (sys == SYS_GLO) {
      const auto it = glo_ephem_cache.find(sat_id);
      if (it == glo_ephem_cache.end()) {
        continue;
      }
      saw_ephemeris = true;
      sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
      sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
    } else {
      const auto it = ephem_cache.find(sat_id);
      if (it == ephem_cache.end()) {
        continue;
      }
      saw_ephemeris = true;
      sat_ecef_pos = gnss_comm::eph2pos(t_tx, it->second, &svdt);
      sat_ecef_vel = gnss_comm::eph2vel(t_tx, it->second, &svddt);
      tgd = it->second->tgd[0];
    }

    if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) {
      continue;
    }

    double azel[2] = {0.0, M_PI / 2.0};
    gnss_comm::sat_azel(anchor.origin_ecef, sat_ecef_pos, azel);
    const double elevation = azel[1];
    const double azimuth = azel[0];
    if (elevation < params_.min_elevation) {
      continue;
    }

    double dop_meas = 0.0;
    if (static_cast<int>(obs->dopp.size()) > l1_idx) {
      const double dopp_hz = obs->dopp[l1_idx];
      if (std::isfinite(dopp_hz)) {
        dop_meas = -dopp_hz * (kSpeedOfLight / freq);
      }
    }

    double pr_sigma_override = -1.0;
    if (static_cast<int>(obs->psr_std.size()) > l1_idx) {
      pr_sigma_override = obs->psr_std[l1_idx];
    }

    double dop_sigma_override = -1.0;
    if (static_cast<int>(obs->dopp_std.size()) > l1_idx) {
      dop_sigma_override = obs->dopp_std[l1_idx] * (kSpeedOfLight / freq);
    }

    SatObs sat;
    sat.sat_id = static_cast<int>(sat_id);
    sat.constellation = constellation_char(sys);
    sat.pr_meas = pr + svdt * kSpeedOfLight;
    sat.dop_meas = dop_meas + svddt * kSpeedOfLight;
    sat.pr_sigma = (pr_sigma_override > 0.05) ? pr_sigma_override : params_.default_pr_sigma;
    sat.dop_sigma = (dop_sigma_override > 0.01) ? dop_sigma_override : params_.default_dop_sigma;
    sat.sat_pos = sat_ecef_pos;
    sat.sat_vel = sat_ecef_vel;
    sat.elevation = elevation;
    sat.azimuth = azimuth;
    sat.tgd = tgd;
    sat.svddt = svddt;

    epoch.sats.push_back(sat);
  }

  result.valid_satellite_count = epoch.sats.size();
  if (epoch.sats.empty()) {
    result.status = saw_ephemeris ? BuildStatus::NoValidSatellite : BuildStatus::MissingEphemeris;
    return result;
  }

  result.status = BuildStatus::Success;
  result.epoch = std::move(epoch);
  return result;
}

}  // namespace iap
