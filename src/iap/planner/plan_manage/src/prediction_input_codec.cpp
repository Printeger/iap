#include <ego_planner/prediction_input.h>
#include <plan_env/local_evidence_snapshot.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <zlib.h>
#include <sstream>
#include <stdexcept>
namespace boost::serialization {
template<class A, class T, int R, int C, int O, int MR, int MC>
void serialize(A& ar, Eigen::Matrix<T,R,C,O,MR,MC>& value, unsigned) {
  static_assert(R != Eigen::Dynamic && C != Eigen::Dynamic);
  for (int i=0;i<value.size();++i) ar & value.data()[i];
}
template<class A> void serialize(A& ar, Eigen::Quaterniond& value, unsigned) {
  ar & value.coeffs();
}
template<class A> void serialize(A& ar, iap::AdvisoryCoordinateContract& c, unsigned) {
  ar & c.valid & c.frame_id & c.stamp & c.epoch_source_identity;
  ar & c.enu_origin_ecef & c.anchor_ecef & c.R_ecef_enu & c.R_ecef_world;
  ar & c.T_map_world & c.T_world_imu & c.T_lidar_imu & c.lever_arm_imu;
  ar & c.map_frame & c.body_frame & c.time_contract & c.failure_reason;
}
template<class A> void serialize(A& ar, iap::GnssPostoptEvidence& value, unsigned) {
  ar & value.model;
  ar & value.update_sequence;
  ar & value.frame_id;
  ar & value.state_stamp;
  ar & value.gnss_stamp;
  ar & value.epoch_source_identity;
  ar & value.used_constellations;
  ar & value.optimized_valid;
  ar & value.covariance_valid;
  ar & value.failure_reason;
  ar & value.propagation;
  ar & value.keys;
  ar & value.tangent_dimensions;
  ar & value.mean_dimensions;
  ar & value.optimized_means;
  ar & value.linearization_means;
  ar & value.joint_covariance_row_major;
  if constexpr (A::is_loading::value) {
    if(value.keys.size()>9 || value.tangent_dimensions.size()>9 || value.mean_dimensions.size()>9 ||
       value.optimized_means.size()>45 || value.linearization_means.size()>45 ||
       value.joint_covariance_row_major.size()>841)
      throw std::runtime_error("postopt evidence exceeds bounded model");
  }
}
template<class A> void serialize(A& ar, iap::SatObs& value, unsigned) {
  ar & value.sat_id;
  ar & value.constellation;
  ar & value.pr_meas;
  ar & value.dop_meas;
  ar & value.pr_sigma;
  ar & value.dop_sigma;
  ar & value.sat_pos;
  ar & value.sat_vel;
  ar & value.tgd;
  ar & value.svddt;
  ar & value.elevation;
  ar & value.azimuth;
  ar & value.kappa;
  ar & value.pr_residual;
  ar & value.nis_pr;
  ar & value.nis_dop;
  ar & value.excluded;
  ar & value.admission_hysteresis_pending;
}
template<class A> void serialize(A& ar, iap::GnssEpoch& value, unsigned) {
  ar & value.stamp;
  ar & value.gps_sec;
  ar & value.sats;
  ar & value.iono_params;
  ar & value.source_identity;
}
template<class A> void serialize(A& ar, iap::CurrentIntegrityState& value, unsigned) {
  ar & value.stamp;
  ar & value.estimation_frame_id;
  ar & value.current_motion_quality;
  ar & value.current_motion_error_proxy_m;
  ar & value.current_external_support_age_s;
  ar & value.current_motion_reason;
  ar & value.valid;
  ar & value.gnss_valid;
  ar & value.gnss_hpl;
  ar & value.gnss_vpl;
  ar & value.gnss_epoch_stamp;
  ar & value.gnss_epoch_identity;
  ar & value.lidar_valid;
  ar & value.lidar_hpl;
  ar & value.lidar_vpl;
  ar & value.lidar_pl_e;
  ar & value.lidar_pl_n;
  ar & value.lidar_pl_u;
  ar & value.icp_degenerate;
  ar & value.icp_rmse;
  ar & value.icp_condition;
  ar & value.icp_gamma_lidar;
  ar & value.integrity_state;
  ar & value.hpl;
  ar & value.vpl;
  ar & value.pl_e;
  ar & value.pl_n;
  ar & value.pl_u;
  ar & value.pl;
  ar & value.hal;
  ar & value.val;
  ar & value.im;
  ar & value.pl_ff;
  ar & value.pl_ff_v;
  ar & value.k_ff_used;
  ar & value.k_fa_used;
  ar & value.n_sv_used;
  ar & value.n_constellations;
  ar & value.pdop;
  ar & value.sigma_h;
  ar & value.n_hypotheses;
  ar & value.n_detected;
  ar & value.excluded_prns;
  ar & value.excluded_trunk_ids;
  ar & value.n_trunks_observed;
  ar & value.tdop;
}
template<class A> void serialize(A& ar, iap::IntegritySnapshot& value, unsigned) {
  ar & value.stamp;
  ar & value.valid;
  ar & value.has_pose;
  ar & value.pose_stamp;
  ar & value.p_wb;
  ar & value.q_wb;
  ar & value.current;
  ar & value.has_epoch;
  ar & value.gnss_epoch;
  ar & value.has_lambda_base;
  ar & value.lambda_base_pos;
  ar & value.prior_source_generation;
  ar & value.has_lidar_snapshot;
  ar & value.lidar_snapshot_valid;
  ar & value.lidar_block_count;
  ar & value.lidar_alpha;
  ar & value.has_lidar_araim_result;
  ar & value.lidar_araim_valid;
  ar & value.lidar_araim_n_hypotheses;
  ar & value.lidar_araim_n_detected;
}
template<class A> void serialize(A& ar, iap::CanopyNoiseParams& value, unsigned) {
  ar & value.sigma_0;
  ar & value.sigma_mp;
  ar & value.sigma_c;
  ar & value.alpha;
}
template<class A> void serialize(A& ar, iap::VisibilityPredictor::Params& value, unsigned) {
  ar & value.min_elevation;
  ar & value.occ_range;
  ar & value.occ_L;
  ar & value.ray_start_offset;
  ar & value.hard_occlusion;
  ar & value.clearance_transition_m;
  ar & value.canopy;
}
template<class A> void serialize(A& ar, iap::GnssGeometryPlPredictorParams& value, unsigned) {
  ar & value.P_HMI_req;
  ar & value.P_FA_req;
  ar & value.dynamic_budget;
  ar & value.K_fa;
  ar & value.K_md;
  ar & value.K_ff;
  ar & value.p_sat_default;
  ar & value.eps_degen;
  ar & value.min_sats;
  ar & value.parallel_hypotheses;
  ar & value.hypothesis_threads;
  ar & value.exact_cache_capacity;
}
template<class A> void serialize(A& ar, iap::GnssAdvisoryPredictorParams& value, unsigned) {
  ar & value.geometry_params;
  ar & value.visibility_params;
  ar & value.measured_epoch_support_radius_m;
  ar & value.measured_epoch_integrity_max_delta_s;
  ar & value.fallback_pl;
  ar & value.fim_clock_epsilon;
  ar & value.fim_psd_epsilon;
}
template<class A> void serialize(A& ar, iap::LidarObservabilityFim::Params& value, unsigned) {
  ar & value.search_radius_m;
  ar & value.min_points;
  ar & value.good_points;
  ar & value.sigma_lidar_m;
  ar & value.alpha_min;
  ar & value.alpha_max;
  ar & value.condition_ref;
  ar & value.condition_max;
  ar & value.tdop_ref;
  ar & value.tdop_max;
  ar & value.bias_h_m;
  ar & value.bias_v_m;
  ar & value.fim_radius_m;
  ar & value.fim_min_voxels;
  ar & value.fim_range_sigma_base;
  ar & value.fim_condition_max;
  ar & value.fim_weight_scale;
}
template<class A> void serialize(A& ar, iap::LidarAdvisoryPredictorParams& value, unsigned) {
  ar & value.fim_params;
  ar & value.enable_legacy_observability;
}
template<class A> void serialize(A& ar, iap::FusionAdvisoryPredictorParams& value, unsigned) {
  ar & value.fim_epsilon;
  ar & value.K_H_adv;
  ar & value.K_V_adv;
  ar & value.b_H_pred;
  ar & value.b_V_pred;
  ar & value.s_H_pred;
  ar & value.s_V_pred;
  ar & value.conservative_max_with_gnss;
}
template<class A> void serialize(A& ar, iap::PredictorFreshnessGuardParams& value, unsigned) {
  ar & value.enabled;
  ar & value.max_odom_age_s;
  ar & value.max_integrity_age_s;
  ar & value.max_gnss_age_s;
  ar & value.max_snapshot_age_s;
}
template<class A> void serialize(A& ar, iap::EmpiricalCovarianceGrowthParams& value, unsigned) {
  ar & value.sigma_grow_m_sqrt_s;
}
template<class A> void serialize(A& ar, iap::PredictorParams& value, unsigned) {
  ar & value.gnss;
  ar & value.lidar;
  ar & value.fusion;
  ar & value.freshness;
  ar & value.covariance_growth;
  ar & value.source_mode;
  ar & value.gnss_epoch_policy;
  ar & value.execution_batch_worker_count;
}
template<class A> void serialize(A& ar, LocalEvidenceSource& value, unsigned) {
  ar & value.frame_id;
  ar & value.observation_stamp_s;
  ar & value.sensor_model_identity;
  ar & value.content_hash;
}
template<class A> void serialize(A& ar, LocalEvidenceIdentity& value, unsigned) {
  ar & value.occupancy_generation;
  ar & value.active_window_generation;
  ar & value.coordinate_contract;
  ar & value.sensor_model_identity;
  ar & value.horizontal_samples;
  ar & value.vertical_samples;
  ar & value.horizontal_fov_rad;
  ar & value.vertical_min_rad;
  ar & value.vertical_max_rad;
  ar & value.min_range_m;
  ar & value.max_range_m;
  ar & value.source_set_hash;
  ar & value.content_hash;
}
template<class A> void serialize(A& ar, LocalEvidenceSnapshot::Geometry& value, unsigned) {
  ar & value.origin;
  ar & value.dimensions;
  ar & value.resolution_m;
}
template<class A> void serialize(A& ar, LocalEvidenceSnapshot::ReadOnlyData& value, unsigned) {
  ar & value.identity;
  ar & value.geometry;
  ar & value.packed_states;
  ar & value.source_indices;
  ar & value.sources;
  ar & value.freshness_s;
}
}
namespace ego_planner {
namespace {
constexpr size_t kMaximumBytes = 256u*1024u*1024u;
template<class Archive> void fields(Archive& ar, FrozenOccupancyEpoch& value) {
  ar & value.lattice_origin;
  ar & value.extent_m;
  ar & value.voxel_dimensions;
  ar & value.resolution_m;
  ar & value.resolution_inv;
  ar & value.virtual_ceiling_height_m;
  ar & value.map_inflation_m;
  ar & value.frame_id;
  ar & value.geometry_id;
  ar & value.cloud_stamp_s;
  ar & value.generation;
  ar & value.active_window_generation;
  ar & value.current_frame_id;
  ar & value.current_frame_content_hash;
  ar & value.frame_contract_id;
  ar & value.current_vehicle_position;
  ar & value.current_vehicle_clearance_radius_m;
}
template<class Archive> void transfer(Archive& ar, PredictionInput& input) {
  std::string schema="iap_prediction_input_v8";
  ar & schema;
  if (schema!="iap_prediction_input_v1" && schema!="iap_prediction_input_v2" && schema!="iap_prediction_input_v3" && schema!="iap_prediction_input_v4" && schema!="iap_prediction_input_v5" && schema!="iap_prediction_input_v6" && schema!="iap_prediction_input_v7" && schema!="iap_prediction_input_v8") throw std::runtime_error("unsupported prediction export schema");
  ar & input.reference_time_s & input.validity_s & input.integrity & input.params;
  // v1 reads preserve historical fields; new parameters are an explicit v2 tail.
  if (schema != "iap_prediction_input_v1") ar & input.params.fusion.max_regularization_fraction;
  if (schema == "iap_prediction_input_v3" || schema == "iap_prediction_input_v4" || schema == "iap_prediction_input_v5" || schema == "iap_prediction_input_v6" || schema == "iap_prediction_input_v7" || schema == "iap_prediction_input_v8") ar & input.params.lidar.fim_params.fim_support_voxel_m & input.recording_codec_version;
  else input.recording_codec_version = schema == "iap_prediction_input_v1" ? 1 : 2;
  if(schema == "iap_prediction_input_v4" || schema == "iap_prediction_input_v5" || schema == "iap_prediction_input_v6" || schema == "iap_prediction_input_v7" || schema == "iap_prediction_input_v8") ar & input.params.gnss.measurement_noise_scale;
  if(schema == "iap_prediction_input_v5" || schema == "iap_prediction_input_v6" || schema == "iap_prediction_input_v7" || schema == "iap_prediction_input_v8") ar & input.integrity.require_coordinates & input.integrity.coordinates & input.integrity.gnss_epoch.R_query_enu & input.integrity.gnss_epoch.antenna_offset_query;
  if (schema == "iap_prediction_input_v6" || schema == "iap_prediction_input_v7" || schema == "iap_prediction_input_v8") ar & input.clock_model;
  else input.clock_model = "legacy_common_clock";
  if(schema == "iap_prediction_input_v7" || schema == "iap_prediction_input_v8") ar & input.gnss_fault_model;
  else {
    input.gnss_fault_model="legacy_single_satellite_v1";
    // An older wire schema cannot claim a newer recording authority.
    input.recording_codec_version=std::min(input.recording_codec_version,6u);
  }
  if(schema == "iap_prediction_input_v8") ar & input.integrity.postopt_evidence;
  else {
    input.integrity.postopt_evidence=iap::GnssPostoptEvidence{};
    input.integrity.postopt_evidence.model="";
    input.integrity.postopt_evidence.failure_reason="historical_state_evidence_unavailable";
    input.recording_codec_version=std::min(input.recording_codec_version,7u);
  }
  auto epoch=std::make_shared<FrozenOccupancyEpoch>();
  if constexpr (Archive::is_saving::value) *epoch=*input.occupancy;
  fields(ar,*epoch);
  std::vector<uint8_t> flags;
  std::vector<Eigen::Vector3d> raw, environment;
  bool has_evidence=false;
  LocalEvidenceSnapshot::ReadOnlyData evidence;
  if constexpr (Archive::is_saving::value) {
    if (!epoch->cells || !epoch->cells->addresses.empty()) throw std::runtime_error("export requires full physical epoch");
    flags=epoch->cells->flags;
    if (epoch->raw_occupied_voxel_centers) raw=*epoch->raw_occupied_voxel_centers;
    if (epoch->environment_occupied_voxel_centers) environment=*epoch->environment_occupied_voxel_centers;
    has_evidence=bool(epoch->local_evidence_snapshot);
    if (has_evidence) evidence=epoch->local_evidence_snapshot->readOnlyData();
  }
  ar & flags & raw & environment & has_evidence;
  if (has_evidence) ar & evidence;
  if constexpr (Archive::is_loading::value) {
    if (!(epoch->resolution_m>0) || !std::isfinite(epoch->resolution_m) ||
        !epoch->lattice_origin.allFinite() || (epoch->voxel_dimensions.array()<=0).any() ||
        epoch->frame_id.empty() || !epoch->extent_m.allFinite() ||
        !(epoch->resolution_inv>0) || !std::isfinite(epoch->resolution_inv)) throw std::runtime_error("invalid exported geometry");
    uint64_t count=1;
    for(int axis=0;axis<3;++axis) {
      const auto size=static_cast<uint64_t>(epoch->voxel_dimensions[axis]);
      if(size>50000000/count) throw std::runtime_error("invalid exported cell count");
      count*=size;
    }
    if (count>50000000 || flags.size()!=count) throw std::runtime_error("invalid exported cell count");
    auto cells=std::make_shared<FrozenOccupancyCells>(); cells->flags=std::move(flags);
    cells->raw_row_offsets.resize(static_cast<size_t>(epoch->voxel_dimensions.x())*epoch->voxel_dimensions.y()+1,0);
    for (size_t i=0;i<cells->flags.size();++i) if (cells->flags[i]&1) {
      cells->raw_addresses.push_back(i); ++cells->raw_row_offsets[i/epoch->voxel_dimensions.z()+1];
    }
    for (size_t i=1;i<cells->raw_row_offsets.size();++i) cells->raw_row_offsets[i]+=cells->raw_row_offsets[i-1];
    epoch->cells=std::move(cells);
    epoch->raw_occupied_voxel_centers=std::make_shared<const std::vector<Eigen::Vector3d>>(std::move(raw));
    epoch->environment_occupied_voxel_centers=std::make_shared<const std::vector<Eigen::Vector3d>>(std::move(environment));
    if (has_evidence) {
      epoch->local_evidence_snapshot=LocalEvidenceSnapshot::fromReadOnlyData(std::move(evidence));
      if (!epoch->local_evidence_snapshot || epoch->local_evidence_snapshot->identity().occupancy_generation!=epoch->generation)
        throw std::runtime_error("invalid exported observation evidence");
    }
    FrozenOccupancyEpoch query_epoch=*epoch;
    epoch->diagnostic_query=[query_epoch](const Eigen::Vector3d& p){return GridMap::queryFrozenOccupancy(query_epoch,p,true);};
    input.occupancy=std::move(epoch);
  }
}
}
std::vector<uint8_t> encodePredictionInput(const PredictionInput& source) {
  if (!source.occupancy) throw std::runtime_error("no physical epoch for display export");
  PredictionInput input=source;
  std::ostringstream stream(std::ios::binary);
  { boost::archive::binary_oarchive ar(stream); transfer(ar,input); }
  const auto bytes=stream.str();
  if (bytes.size()>kMaximumBytes) throw std::runtime_error("prediction export too large");
  uLongf size=compressBound(bytes.size());
  std::vector<uint8_t> compressed(size+8);
  const uint64_t length=bytes.size();
  for (int i=0;i<8;++i) compressed[i]=(length>>(8*i))&255;
  if (compress2(compressed.data()+8,&size,reinterpret_cast<const Bytef*>(bytes.data()),bytes.size(),1)!=Z_OK)
    throw std::runtime_error("prediction export compression failed");
  compressed.resize(size+8); return compressed;
}
PredictionInput decodePredictionInput(const std::vector<uint8_t>& payload) {
  if (payload.size()<8 || payload.size()>kMaximumBytes) throw std::runtime_error("invalid prediction export payload");
  uint64_t length=0;
  for(int i=0;i<8;++i) length|=uint64_t(payload[i])<<(8*i);
  if (!length || length>kMaximumBytes) throw std::runtime_error("invalid prediction export length");
  std::string bytes(length,'\0'); uLongf size=length;
  if (uncompress(reinterpret_cast<Bytef*>(bytes.data()),&size,payload.data()+8,payload.size()-8)!=Z_OK || size!=length)
    throw std::runtime_error("prediction export decompression failed");
  std::istringstream stream(bytes,std::ios::binary);
  boost::archive::binary_iarchive ar(stream); PredictionInput input; transfer(ar,input); return input;
}
}

namespace ego_planner {
uint64_t predictionInputIdentity(const PredictionInput& input) {
  auto snapshot=input.integrity;
  // Capture time alone must not repeat identical PL. Source times and all
  // actual predictor inputs remain in this identity; age is displayed from
  // the original reference and never grants fresh validity to cached values.
  snapshot.stamp=0;
  auto params=input.params;
  std::ostringstream stream(std::ios::binary);
  { boost::archive::binary_oarchive ar(stream);
    const std::string model="pose6_fixed_two_source_envelope_v2";
    ar << model; // Algorithm policy is part of cached PL identity.
    ar & snapshot & params;
    if(input.recording_codec_version>=5) ar & snapshot.require_coordinates & snapshot.coordinates & snapshot.gnss_epoch.R_query_enu & snapshot.gnss_epoch.antenna_offset_query;
    ar & params.fusion.max_regularization_fraction & params.lidar.fim_params.fim_support_voxel_m & params.gnss.measurement_noise_scale;
    if(input.recording_codec_version>=6) ar & input.clock_model;
    if(input.recording_codec_version>=7) ar & input.gnss_fault_model;
    if(input.recording_codec_version>=8) ar & snapshot.postopt_evidence;
    ar << input.recording_codec_version << input.validity_s << input.occupancy->generation << input.occupancy->geometry_id;
  }
  uint64_t hash=1469598103934665603ULL;
  for (const unsigned char c:stream.str()) { hash^=c; hash*=1099511628211ULL; }
  return hash;
}
}
