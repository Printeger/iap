#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Shared GNSS front-end that converts raw observation batches + ephemeris
// snapshots into processed GnssEpoch packets in ECEF.

#include <iap/gnss/gnss_types.hpp>

#include <gnss_comm/gnss_constant.hpp>

#include <cmath>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

namespace iap {

struct GnssRawObservationBatch {
  double ros_stamp = 0.0;
  std::vector<gnss_comm::ObsPtr> observations;
};

struct GnssEphemerisUpdate {
  uint32_t sat_id = 0;
  gnss_comm::EphemPtr ephem;
  gnss_comm::GloEphemPtr glo_ephem;
};

class GnssEpochBuilder {
 public:
  enum class BuildStatus {
    Success,
    EmptyBatch,
    MissingAnchor,
    MissingEphemeris,
    NoValidSatellite,
  };

  struct BuildResult {
    BuildStatus status = BuildStatus::EmptyBatch;
    std::optional<GnssEpoch> epoch;
    std::size_t observation_count = 0;
    std::size_t valid_satellite_count = 0;
  };

  struct Params {
    double min_elevation = 10.0 * M_PI / 180.0;
    double default_pr_sigma = 5.0;
    double default_dop_sigma = 0.5;
  };

  GnssEpochBuilder();
  explicit GnssEpochBuilder(const Params& params);

  void set_anchor(const GnssAnchorState& anchor);
  bool anchor_ready() const;
  void set_iono_params(const std::vector<double>& iono_params);
  void update_ephemeris(const GnssEphemerisUpdate& update);
  void update_ephemeris(const gnss_comm::EphemPtr& ephem);
  void update_glo_ephemeris(const gnss_comm::GloEphemPtr& ephem);

  BuildResult build_epoch(const GnssRawObservationBatch& batch) const;

 private:
  Params params_;
  mutable std::mutex mutex_;
  bool anchor_ready_ = false;
  GnssAnchorState anchor_;
  std::vector<double> iono_params_;
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
};

}  // namespace iap
