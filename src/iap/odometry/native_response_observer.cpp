#include <iap/odometry/native_response_observer.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/odometry/gpu_evidence/integrated_vgicp_factor_gpu.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <spdlog/spdlog.h>
#include <chrono>
#include <fstream>
#include <cstring>
#include <zlib.h>

namespace iap {
namespace {
using Json=nlohmann::json;
template<class Matrix> Json row_major(const Matrix& m) {
  if(!m.allFinite()) throw std::runtime_error("NONFINITE_NATIVE_STATE");
  Json out=Json::array();
  for(int r=0;r<m.rows();++r) for(int c=0;c<m.cols();++c) out.push_back(m(r,c));
  return out;
}
// Copy large coefficient blocks on the solver thread; expand only on writer.
template<class Matrix> Json coefficient_bytes(const Matrix& m) {
  if(!m.allFinite()) throw std::runtime_error("NONFINITE_NATIVE_COEFFICIENTS");
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> copy=m;
  const auto* first=reinterpret_cast<const std::uint8_t*>(copy.data());
  return Json::binary(std::vector<std::uint8_t>(first,first+copy.size()*sizeof(double)));
}
void expand_coefficients(Json& value) {
  if(value.is_binary()) {
    const auto& bytes=value.get_binary();std::vector<double> numbers(bytes.size()/sizeof(double));
    std::memcpy(numbers.data(),bytes.data(),bytes.size());value=numbers;
  } else if(value.is_array()||value.is_object()) for(auto& child:value) expand_coefficients(child);
}
}
NativeResponseObserver::NativeResponseObserver(const glim::RunLogManager& logs,const std::vector<long>& frames)
: frames_(frames.begin(),frames.end()), raw_path_(logs.export_path("glio/native_response.jsonl.gz")),
  manifest_path_(logs.metadata_path("manifests/native_response.json")) {
  if(frames.empty() || frames.size()>kMaxFrames || frames_.size()!=frames.size() || *frames_.begin()<0)
    throw std::invalid_argument("native response requires 1..31 unique nonnegative frame IDs");
  for(const auto& path:{raw_path_,manifest_path_}) {
    if(std::filesystem::exists(path)) throw std::runtime_error("native response artifact already exists");
    std::filesystem::create_directories(path.parent_path());
  }
  raw_=gzopen(raw_path_.c_str(),"wb1");
  if(!raw_) throw std::runtime_error("native response cannot open owned gzip artifact");
  try { worker_=std::thread(&NativeResponseObserver::drain,this); }
  catch(...) { gzclose(static_cast<gzFile>(raw_));throw; }
}
NativeResponseObserver::~NativeResponseObserver() {
  { std::lock_guard<std::mutex> lock(mutex_);stopping_=true; }
  cv_.notify_one();worker_.join();
  if(gzclose(static_cast<gzFile>(raw_))!=Z_OK) ++write_failures_;
  try {
    Json missing=Json::array();for(auto frame:frames_) if(!seen_.count(frame)) missing.push_back(frame);
    Json manifest={{"schema","iap_native_response_manifest_v1"},{"version",IAP_VERSION},
      {"build_type",IAP_BUILD_TYPE},{"run_identity","metadata/run_manifest.json"},
      {"raw","export/glio/native_response.jsonl.gz"},{"selection_frames",frames_},
      {"missing_frames",missing},{"initial_x0_elimination_seen",initial_seen_},{"requests",ledger_},
      {"written",written_},{"write_failures",write_failures_},{"queue_limit",kQueueLimit},
      {"max_snapshots",kMaxFrames+1},{"max_keys",kMaxKeys},{"max_factors",kMaxFactors},
      {"max_index_entries",kMaxIndexEntries},{"max_retained_gpu_factors",kMaxRetainedGpuFactors},
      {"max_factor_tangent_dim",kMaxFactorDim},{"matrix_byte_limit",kMaxBytes},
      {"copy_budget_ms",kCopyBudgetMs},{"physical_joint_error_law_available",false}};
    std::ofstream file(manifest_path_);file<<manifest.dump(2)<<'\n';file.flush();
    if(!file) throw std::runtime_error("manifest write failed");
  } catch(const std::exception& e) { spdlog::error("[native_response] finalize failed: {}",e.what()); }
}
void NativeResponseObserver::prepare(gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::NonlinearFactorGraph& new_factors,long frame,double stamp) {
#ifdef IAP_NATIVE_GPU_MATCH
  // CPU-only retention of the reduction already computed by the solver. Each
  // factor retains one fixed 12x12 block; the original lag owns its lifetime.
  if((seen_.size()<frames_.size() || !initial_seen_) && new_factors.size()<=kMaxFactors) {
    retained_gpu_factors_.erase(std::remove_if(retained_gpu_factors_.begin(),retained_gpu_factors_.end(),
        [](const auto& f){return f.expired();}),retained_gpu_factors_.end());
    for(const auto& f:new_factors) if(auto gpu=dynamic_cast<gtsam_points::IapObservedVGICPFactorGPU*>(f.get())) {
      if(retained_gpu_factors_.size()==kMaxRetainedGpuFactors) break;
      gpu->retain_native_response();retained_gpu_factors_.push_back(f);
    }
  }
#endif
  const auto weak=weak_from_this();
  smoother.setPreMarginalizationObserver([weak,frame,stamp](const auto& native,const auto& discarded) {
    if(auto self=weak.lock()) self->observe(native,discarded,frame,stamp);
  });
}
Json NativeResponseObserver::snapshot(const gtsam_points::IncrementalFixedLagSmootherExt& native,
    const gtsam::KeyVector& discarded,long frame,double stamp) {
  const auto start=std::chrono::steady_clock::now();
  Json out={{"schema","iap_native_response_v1"},{"frame_id",frame},{"stamp",stamp},
    {"lag_s",native.smootherLag()},{"discarded_keys",discarded},{"complete",false},
    {"coefficient_semantics","cached_native_loss_normal_equations_not_physical_noise"},
    {"physical_joint_error_law_available",false},{"states",Json::array()},{"factors",Json::array()}};
  std::size_t bytes=0;
  const auto& theta=native.getLinearizationPoint();
  const auto& factors=native.getLinearFactors();
  const auto& index=native.getISAM2().getVariableIndex();
  out["native_key_count"]=theta.size();out["native_factor_slot_count"]=factors.size();
  out["native_index_entries"]=index.nEntries();
  auto elapsed=[&]{return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();};
  try {
    if(theta.size()>kMaxKeys || index.nEntries()>kMaxIndexEntries) throw std::runtime_error("NATIVE_GRAPH_CAP_EXCEEDED");
    // The solver's VariableIndex contains only active relationships. Iterating
    // it avoids an unbounded scan of old null factor slots without changing the
    // estimator's slot reuse policy.
    std::set<std::size_t> active;
    for(const auto& entry:index) {
      if(elapsed()>kCopyBudgetMs) throw std::runtime_error("COPY_TIME_CAP_EXCEEDED");
      active.insert(entry.second.begin(),entry.second.end());
      if(active.size()>kMaxFactors) throw std::runtime_error("NATIVE_ACTIVE_FACTOR_CAP_EXCEEDED");
    }
    out["native_active_factor_count"]=active.size();
    const auto& delta=native.getDelta();
    for(const auto& kv:theta) {
      if(elapsed()>kCopyBudgetMs) throw std::runtime_error("COPY_TIME_CAP_EXCEEDED");
      const auto key=kv.key;const char type=gtsam::Symbol(key).chr();
      Json point;std::string representation;
      if(type=='x') {point=row_major(theta.at<gtsam::Pose3>(key).matrix());representation="Pose3_matrix_row_major";}
      else if(type=='r') {point=row_major(theta.at<gtsam::Rot3>(key).matrix());representation="Rot3_matrix_row_major";}
      else if(type=='b') {point=row_major(theta.at<gtsam::imuBias::ConstantBias>(key).vector());representation="ConstantBias_acc_gyro";}
      else if(type=='v'||type=='e') {point=row_major(theta.at<gtsam::Vector3>(key));representation="Vector3";}
      else if(type=='c'||type=='d'||type=='h'||type=='j') {point=row_major(theta.at<gtsam::Vector2>(key));representation="Vector2";}
      else throw std::runtime_error("UNSUPPORTED_NATIVE_VALUE_TYPE");
      bytes+=(point.size()+kv.value.dim())*sizeof(double);
      if(bytes>kMaxBytes) throw std::runtime_error("NATIVE_MATRIX_CAP_EXCEEDED");
      out["states"].push_back({{"key",key},{"type",std::string(1,type)},{"dim",kv.value.dim()},
        {"linearization_point",point},{"representation",representation},{"delta",row_major(delta.at(key))}});
    }
    const auto& nonlinear=native.getFactors();
    const auto& new_indices=native.getISAM2Result().newFactorsIndices;
    for(const auto i:active) {
      if(elapsed()>kCopyBudgetMs) throw std::runtime_error("COPY_TIME_CAP_EXCEEDED");
      if(i>=factors.size() || !factors[i]) throw std::runtime_error("NATIVE_LINEAR_CACHE_MISSING");
      const auto& factor=factors[i];
      std::size_t dim=0;std::vector<std::size_t> dims;
      for(auto it=factor->begin();it!=factor->end();++it) {auto d=factor->getDim(it);dims.push_back(d);dim+=d;}
      if(dim>kMaxFactorDim || (bytes+=(dim+1)*(dim+1)*sizeof(double))>kMaxBytes)
        throw std::runtime_error("NATIVE_MATRIX_CAP_EXCEEDED");
      const auto matrix=factor->augmentedInformation();
      if(!matrix.allFinite()) throw std::runtime_error("NONFINITE_NATIVE_COEFFICIENTS");
      Json row={{"index",i},{"keys",factor->keys()},{"dims",dims},{"rows",matrix.rows()},
        {"augmented_information_row_major",coefficient_bytes(matrix)},
        {"new_this_update",std::find(new_indices.begin(),new_indices.end(),i)!=new_indices.end()}};
      const auto* nf=i<nonlinear.size() ? nonlinear[i].get() : nullptr;
      row["native_type"]=nf ? typeid(*nf).name() : "missing";
      if(const auto* pr=dynamic_cast<const PseudorangeFactor*>(nf)) {
        row["pseudorange"]={{"sat_id",pr->sat_id()},{"constellation",std::string(1,pr->constellation())},
          {"pr_meas_m",pr->pr_meas()},{"elevation_rad",pr->elevation()}};
      }
#ifdef IAP_NATIVE_GPU_MATCH
      if(const auto* gpu=dynamic_cast<const gtsam_points::IapObservedVGICPFactorGPU*>(nf)) {
        const auto& owner=gpu->get_evidence_owner();
        row["vgicp"]={{"source_frame_id",owner.source_frame_id},{"target_frame_id",owner.target_frame_id},
          {"source_stamp",owner.source_stamp},{"target_stamp",owner.target_stamp},{"level_id",owner.level_id},
          {"target_is_fixed",owner.target_is_fixed},{"voxel_resolution_m",owner.voxel_resolution_m}};
        const auto& response=gpu->native_response();
        row["vgicp"]["target_response_available"]=response.has_value();
        if(response) {
          bytes+=(13*13+4*4)*sizeof(double);
          if(bytes>kMaxBytes) throw std::runtime_error("NATIVE_MATRIX_CAP_EXCEEDED");
          row["vgicp"]["full_augmented_information_row_major"]=coefficient_bytes(response->augmented_information);
          row["vgicp"]["linearization_transform_row_major"]=row_major(response->linearization_transform);
        }
        if(owner.target_is_fixed) row["vgicp"]["fixed_target_pose_row_major"]=row_major(gpu->get_fixed_target_pose().matrix());
      }
#endif
      out["factors"].push_back(std::move(row));
    }
    out["complete"]=true;
  } catch(const std::exception& e) {
    // Never export a partial graph as a complete native response.
    out["states"]=Json::array();out["factors"]=Json::array();out["failure_reason"]=e.what();
  }
  out["copied_matrix_bytes"]=bytes;out["copy_ms"]=elapsed();return out;
}
void NativeResponseObserver::observe(const gtsam_points::IncrementalFixedLagSmootherExt& native,
    const gtsam::KeyVector& discarded,long frame,double stamp) {
  const bool initial=!initial_seen_ && std::find(discarded.begin(),discarded.end(),gtsam::Symbol('x',0))!=discarded.end();
  if((!frames_.count(frame)||seen_.count(frame)) && !initial) return;
  if(frames_.count(frame)) seen_.insert(frame);
  if(initial) initial_seen_=true;
  auto packet=snapshot(native,discarded,frame,stamp);packet["initial_x0_elimination"]=initial;
  std::lock_guard<std::mutex> lock(mutex_);
  const bool accepted=queue_.size()<kQueueLimit;
  ledger_.push_back({{"frame_id",frame},{"stamp",stamp},{"initial_x0_elimination",initial},
    {"complete",packet["complete"]},{"failure_reason",packet.value("failure_reason","")},
    {"copy_ms",packet["copy_ms"]},{"queue_accepted",accepted}});
  if(!accepted) return;
  queue_.push_back(std::move(packet));cv_.notify_one();
}
void NativeResponseObserver::drain() {
  for(;;) {
    Json packet;
    { std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stopping_||!queue_.empty();});
      if(queue_.empty()) return;packet=std::move(queue_.front());queue_.pop_front(); }
    expand_coefficients(packet);const auto line=packet.dump()+"\n";
    if(gzwrite(static_cast<gzFile>(raw_),line.data(),line.size())!=static_cast<int>(line.size())) ++write_failures_;
    else ++written_;
  }
}
}
