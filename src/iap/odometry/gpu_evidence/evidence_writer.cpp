#include <iap/odometry/gpu_evidence/evidence_writer.hpp>
#include <iap/util/run_log_manager.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <iomanip>
#include <stdexcept>

namespace iap {
namespace {
using Json=nlohmann::json;
std::string csv_string(const std::string& value) {
  std::string result="\"";
  for(const char c:value) { if(c=='\"') result+='\"';result+=c; }
  return result+'\"';
}
template<class Matrix> Json row_major(const Matrix& matrix) {
  auto values=Json::array();
  for(int r=0;r<matrix.rows();++r) for(int c=0;c<matrix.cols();++c) values.push_back(matrix(r,c));
  return values;
}
Json serialize(const GpuMatchEvidencePacket& p) {
  auto samples=Json::array();
  for(const auto& s:p.capture.samples) samples.push_back({
    {"source_index",s.source_index},{"target_index",s.target_index},{"target_point_count",s.target_point_count},
    {"residual_m",s.residual_m},{"covariance_m2_row_major",s.covariance_m2_row_major},
    {"source_covariance_m2_row_major",s.source_covariance_m2_row_major},
    {"target_covariance_m2_row_major",s.target_covariance_m2_row_major},
    {"source_mean_m",s.source_mean_m},{"target_mean_m",s.target_mean_m},
    {"mahalanobis_squared",s.mahalanobis_squared}});
  return {{"schema","iap_gpu_match_residual_v1"},{"model","actual_cuda_postopt_quality_linearization_v1"},
    {"request_id",p.request_id},{"capture_requested",p.capture_requested},{"source_frame_id",p.owner.source_frame_id},{"target_frame_id",p.owner.target_frame_id},
    {"source_stamp",p.owner.source_stamp},{"target_stamp",p.owner.target_stamp},
    {"target_is_fixed",p.owner.target_is_fixed},{"level_id",p.owner.level_id},{"voxel_resolution_m",p.owner.voxel_resolution_m},
    {"factor_keys",std::vector<std::uint64_t>(p.factor_keys.begin(),p.factor_keys.begin()+p.key_count)},
    {"available",p.capture.available},{"failure_reason",p.capture.failure_reason},{"sequence",p.capture.sequence},
    {"original_source_count",p.capture.original_source_count},{"original_inlier_count",p.capture.original_inlier_count},
    {"original_cost",p.capture.original_cost},{"sampling_stride",p.capture.sampling_stride},
    {"linearization_transform_row_major",row_major(p.capture.linearization_transform)},
    {"T_world_source_row_major",row_major(p.T_world_source)},{"T_world_target_row_major",row_major(p.T_world_target)},
    {"T_lidar_imu_row_major",row_major(p.T_lidar_imu)},
    {"gnss",{{"owner_matches",p.gnss_owner_matches},{"frame_id",p.gnss_frame_id},{"update_sequence",p.gnss_update_sequence},
      {"epoch_source_identity",p.gnss_epoch_identity},{"state_stamp",p.gnss_state_stamp},{"epoch_stamp",p.gnss_epoch_stamp},
      {"used_constellations",p.used_constellations},{"propagation","NOT_PROPAGATED"}}},
    {"samples",samples},{"qualified_noise",false},{"qualified_motion",false}};
}
}
GpuMatchEvidenceWriter::GpuMatchEvidenceWriter(const glim::RunLogManager& logs)
: raw_path_(logs.export_path("glio/gpu_match_residuals.jsonl")),
  ledger_path_(logs.export_path("glio/gpu_match_requests.csv")),
  manifest_path_(logs.metadata_path("manifests/gpu_match_evidence.json")) {
  for(const auto& path:{raw_path_,ledger_path_,manifest_path_}) {
    if(std::filesystem::exists(path)) throw std::runtime_error("GPU evidence refuses existing artifact: "+path.string());
    std::filesystem::create_directories(path.parent_path());
  }
  raw_.open(raw_path_);ledger_.open(ledger_path_);
  if(!raw_ || !ledger_) throw std::runtime_error("GPU evidence cannot open run-scoped artifacts");
  if(std::filesystem::exists(logs.metadata_path("manifests/forest_runtime_identity.json")))
    runtime_identity_="metadata/manifests/forest_runtime_identity.json";
  ledger_<<"request_id,source_frame_id,source_stamp,target_frame_id,level_id,capture_requested,available,samples,queue_accepted,failure_reason,"
    "target_stamp,target_is_fixed,voxel_resolution_m,key_count,key0,key1,gnss_frame_id,gnss_update_sequence,gnss_epoch_identity,"
    "gnss_state_stamp,gnss_epoch_stamp,gnss_owner_matches,used_constellations,sequence,source_count,inlier_count,stride,cost\n"<<std::setprecision(17);
  worker_=std::thread(&GpuMatchEvidenceWriter::drain,this);
}
GpuMatchEvidenceWriter::~GpuMatchEvidenceWriter() {
  {
    std::lock_guard<std::mutex> lock(mutex_);stopping_=true;
  }
  cv_.notify_one();worker_.join();
  raw_.flush();ledger_.flush();
  if(!raw_) ++write_failures_;
  if(!ledger_) ++write_failures_;
  try {
    Json summary={{"schema","iap_gpu_match_evidence_manifest_v1"},
      {"upstream_revision","85d0f4c43098b1f071bbb07710692e3829347c6c"},
      {"model","actual_cuda_postopt_quality_linearization_v1"},
      {"version",IAP_VERSION},{"build_type",IAP_BUILD_TYPE},
      {"raw","export/glio/gpu_match_residuals.jsonl"},{"requests","export/glio/gpu_match_requests.csv"},
      {"runtime_identity",runtime_identity_.empty() ? Json(nullptr) : Json(runtime_identity_)},
      {"runtime_identity_available",!runtime_identity_.empty()},
      {"run_identity","metadata/run_manifest.json"},
      {"config_identity",std::filesystem::exists(manifest_path_.parent_path()/"launch_profile_manifest.json") ?
        Json("metadata/manifests/launch_profile_manifest.json") : Json(nullptr)},
      {"entries",entries_},{"requested",requests_},{"available",available_},{"queue_dropped",queue_dropped_},
      {"written",written_},{"samples_written",samples_written_},{"write_failures",write_failures_},
      {"queue_limit",kQueueLimit},{"per_factor_sample_limit",kGpuMatchEvidenceLimit},
      {"per_factor_pinned_bytes",sizeof(GpuMatchTransfer)},
      {"per_factor_device_evidence_bytes",sizeof(GpuMatchResidual)*kGpuMatchEvidenceLimit},
      {"sampling","deterministic_source_index_stride"},{"qualified_noise",false},{"qualified_motion",false}};
    std::ofstream manifest(manifest_path_);manifest<<summary.dump(2)<<'\n';manifest.flush();
    if(!manifest) throw std::runtime_error("summary write failed");
  } catch(const std::exception& e) {
    spdlog::error("[gpu_evidence] finalize failed: {}",e.what());
  }
}
void GpuMatchEvidenceWriter::submit(GpuMatchEvidencePacket packet) {
  if(packet.key_count>packet.factor_keys.size() || packet.capture.samples.size()>kGpuMatchEvidenceLimit)
    throw std::invalid_argument("GPU evidence packet exceeds bounded contract");
  std::lock_guard<std::mutex> lock(mutex_);
  packet.request_id=++entries_;requests_+=packet.capture_requested;available_+=packet.capture.available;
  const bool accepted=queue_.size()<kQueueLimit;
  // Small buffered identity ledger retains every request even when raw queue is
  // full. Large residual serialization never runs on the odometry producer.
  ledger_<<packet.request_id<<','<<packet.owner.source_frame_id<<','<<packet.owner.source_stamp<<','
    <<packet.owner.target_frame_id<<','<<packet.owner.level_id<<','<<packet.capture_requested<<','<<packet.capture.available<<','
    <<packet.capture.samples.size()<<','<<accepted<<','<<csv_string(packet.capture.failure_reason)<<','
    <<packet.owner.target_stamp<<','<<packet.owner.target_is_fixed<<','<<packet.owner.voxel_resolution_m<<','
    <<packet.key_count<<','<<packet.factor_keys[0]<<','<<packet.factor_keys[1]<<','<<packet.gnss_frame_id<<','
    <<packet.gnss_update_sequence<<','<<packet.gnss_epoch_identity<<','<<packet.gnss_state_stamp<<','
    <<packet.gnss_epoch_stamp<<','<<packet.gnss_owner_matches<<','<<csv_string(packet.used_constellations)<<','
    <<packet.capture.sequence<<','<<packet.capture.original_source_count<<','<<packet.capture.original_inlier_count<<','
    <<packet.capture.sampling_stride<<','<<packet.capture.original_cost<<'\n';
  if(!accepted) { ++queue_dropped_;return; }
  queue_.push_back(std::move(packet));
  cv_.notify_one();
}
void GpuMatchEvidenceWriter::drain() {
  while(true) {
    GpuMatchEvidencePacket packet;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock,[this]{return stopping_ || !queue_.empty();});
      if(queue_.empty()) return;
      packet=std::move(queue_.front());queue_.pop_front();
    }
    try {
      raw_<<serialize(packet).dump()<<'\n';
      if(!raw_) throw std::runtime_error("raw write failed");
      ++written_;samples_written_+=packet.capture.samples.size();
    } catch(const std::exception& e) {
      if(write_failures_++==0) spdlog::error("[gpu_evidence] export failed: {}",e.what());
    }
  }
}
}
