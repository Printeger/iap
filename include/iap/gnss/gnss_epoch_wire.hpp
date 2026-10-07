#pragma once

#include <iap/gnss/gnss_types.hpp>
#include <optional>

namespace iap {

// One representation shared by the monitor producer and prediction consumers.
// Transport rejected observations intact; PredictorModule owns admission.
template <typename Report> void write_advisory_epoch(const GnssEpoch* epoch, Report& msg) {
  msg.advisory_has_gnss_epoch = epoch != nullptr;
  msg.advisory_gnss_observations.clear();
  msg.advisory_gnss_iono_params.clear();
  msg.advisory_gnss_stamp = epoch ? epoch->stamp : 0.0;
  msg.advisory_gnss_gps_sec = epoch ? epoch->gps_sec : 0.0;
  msg.advisory_gnss_source_identity = epoch ? gnss_epoch_identity(*epoch) : 0;
  if (!epoch) return;
  msg.advisory_gnss_iono_params = epoch->iono_params;
  for (const auto& sat : epoch->sats) {
    typename decltype(msg.advisory_gnss_observations)::value_type wire;
    wire.sat_id=sat.sat_id; wire.constellation=static_cast<unsigned char>(sat.constellation);
    wire.pr_meas=sat.pr_meas; wire.dop_meas=sat.dop_meas;
    wire.pr_sigma=sat.pr_sigma; wire.dop_sigma=sat.dop_sigma;
    for (int axis=0;axis<3;++axis) { wire.sat_pos[axis]=sat.sat_pos[axis]; wire.sat_vel[axis]=sat.sat_vel[axis]; }
    wire.tgd=sat.tgd; wire.svddt=sat.svddt;
    wire.elevation=sat.elevation; wire.azimuth=sat.azimuth; wire.kappa=sat.kappa;
    wire.pr_residual=sat.pr_residual; wire.nis_pr=sat.nis_pr; wire.nis_dop=sat.nis_dop;
    wire.excluded=sat.excluded;
    msg.advisory_gnss_observations.push_back(wire);
  }
}

template <typename Report> std::optional<GnssEpoch> read_advisory_epoch(const Report& msg) {
  if (!msg.advisory_has_gnss_epoch) return std::nullopt;
  GnssEpoch epoch;
  epoch.stamp=msg.advisory_gnss_stamp; epoch.gps_sec=msg.advisory_gnss_gps_sec;
  epoch.source_identity=msg.advisory_gnss_source_identity;
  epoch.iono_params=msg.advisory_gnss_iono_params;
  for (const auto& wire : msg.advisory_gnss_observations) {
    SatObs sat;
    sat.sat_id=wire.sat_id; sat.constellation=static_cast<char>(wire.constellation);
    sat.pr_meas=wire.pr_meas; sat.dop_meas=wire.dop_meas;
    sat.pr_sigma=wire.pr_sigma; sat.dop_sigma=wire.dop_sigma;
    for (int axis=0;axis<3;++axis) { sat.sat_pos[axis]=wire.sat_pos[axis]; sat.sat_vel[axis]=wire.sat_vel[axis]; }
    sat.tgd=wire.tgd; sat.svddt=wire.svddt;
    sat.elevation=wire.elevation; sat.azimuth=wire.azimuth; sat.kappa=wire.kappa;
    sat.pr_residual=wire.pr_residual; sat.nis_pr=wire.nis_pr; sat.nis_dop=wire.nis_dop;
    sat.excluded=wire.excluded;
    epoch.sats.push_back(sat);
  }
  return epoch;
}
}  // namespace iap
