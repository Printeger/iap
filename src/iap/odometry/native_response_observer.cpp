#include <iap/odometry/native_response_observer.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/odometry/gpu_evidence/integrated_vgicp_factor_gpu.hpp>
#include <iap/odometry/gpu_evidence/match_residual.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <spdlog/spdlog.h>
#include <chrono>
#include <fstream>
#include <zlib.h>

namespace iap {
namespace {
using Json=nlohmann::json;
template<class Matrix> std::vector<double> row_major(const Matrix& m) {
  if(!m.allFinite()) throw std::runtime_error("NONFINITE_NATIVE_STATE");
  std::vector<double> out;out.reserve(m.size());
  for(int r=0;r<m.rows();++r) for(int c=0;c<m.cols();++c) out.push_back(m(r,c));
  return out;
}
std::vector<double> packed_upper(const Eigen::MatrixXd& m) {
  if(!m.allFinite() || m.rows()!=m.cols()) throw std::runtime_error("NONFINITE_NATIVE_COEFFICIENTS");
  std::vector<double> out;out.reserve(m.rows()*(m.rows()+1)/2);
  for(int r=0;r<m.rows();++r) for(int c=r;c<m.cols();++c) out.push_back(m(r,c));
  return out;
}
struct ResponseState {
  std::uint64_t key;std::size_t dim;const char* representation;
  std::vector<double> point,delta;
};
struct ResponseFactor {
  std::size_t index,rows;gtsam::KeyVector keys;std::vector<std::size_t> dims;
  const char* native_type="missing";bool new_this_update=false;
  gtsam::GaussianFactor::shared_ptr cached;
  bool pseudorange=false;int sat_id=0;char constellation='?';double pr_meas=0.,elevation=0.;
  bool vgicp=false,target_response_available=false;
  GpuMatchOwner owner;
  std::vector<double> target_upper,linearization_transform,fixed_target_pose;
};
}
struct NativeResponsePacket {
  long frame;double stamp,lag,copy_ms=0.;bool complete=false,initial=false;
  std::string failure;
  std::size_t bytes=0,cached_bytes=0,request_id=0,keys=0,slots=0,entries=0,active=0,max_factor_dim=0;
  gtsam::KeyVector discarded;
  std::vector<ResponseState> states;
  std::vector<ResponseFactor> factors;
  std::vector<std::string> selection_roles;
  long selected_source_id=-1;
};
NativeResponseObserver::NativeResponseObserver(const glim::RunLogManager& logs,const std::vector<long>& frames,
    bool first_source_freeze)
: frames_(frames.begin(),frames.end()), source_freeze_enabled_(first_source_freeze), raw_path_(logs.export_path("glio/native_response.jsonl.gz")),
  manifest_path_(logs.metadata_path("manifests/native_response.json")) {
  if(frames.empty() || frames.size()>(first_source_freeze ? kMaxFrames-5 : kMaxFrames) || frames_.size()!=frames.size() || *frames_.begin()<0)
    throw std::invalid_argument("native response requires unique nonnegative frame IDs: 1..31, or 1..26 with source freeze selection");
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
    Json manifest={{"schema","iap_native_response_manifest_v3"},{"version",IAP_VERSION},
      {"build_type",IAP_BUILD_TYPE},{"run_identity","metadata/run_manifest.json"},
      {"raw","export/glio/native_response.jsonl.gz"},{"selection_frames",frames_},
      {"missing_frames",missing},{"initial_x0_elimination_seen",initial_seen_},{"requests",ledger_},
      {"written",written_},{"write_failures",write_failures_},{"queue_limit",kQueueLimit},
      {"max_snapshots",kMaxFrames+1},{"max_keys",kMaxKeys},{"max_factors",kMaxFactors},
      {"max_index_entries",kMaxIndexEntries},{"max_retained_gpu_factors",kMaxRetainedGpuFactors},
      {"cached_factor_byte_limit",kMaxCachedBytes},
      {"max_factor_tangent_dim",kMaxFactorDim},{"matrix_encoding","symmetric_upper_triangle_row_major"},{"matrix_byte_limit",kMaxBytes},
      {"copy_budget_ms",kCopyBudgetMs},{"physical_joint_error_law_available",false}};
    manifest["max_total_observer_ms"]=max_observer_ms_;
    manifest["observer_budget_overrun_count"]=observer_budget_overruns_;
    manifest["first_source_freeze"]={{"enabled",source_freeze_enabled_},{"source_id",source_id_},
      {"binary_first_seen_frame",source_birth_frame_},{"first_fixed_use_frame",source_freeze_frame_},
      {"previous_frame",source_previous_frame_},{"retirement_frame",source_retirement_frame_},
      {"selection_done",source_selection_done_},{"failure_reason",source_selection_failure_},
      {"tracking_frame_limit",100},{"extra_snapshot_limit",5},{"rolling_packet_limit",1},
      {"rolling_copy_count",rolling_copies_},{"rolling_copy_failures",rolling_failures_},
      {"one_second_probe_definition","first_fixed_use_frame_plus_10_not_primary_evaluation"}};
    std::ofstream file(manifest_path_);file<<manifest.dump(2)<<'\n';file.flush();
    if(!file) throw std::runtime_error("manifest write failed");
  } catch(const std::exception& e) { spdlog::error("[native_response] finalize failed: {}",e.what()); }
}
void NativeResponseObserver::prepare(gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::NonlinearFactorGraph& new_factors,long frame,double stamp) {
#ifdef IAP_NATIVE_GPU_MATCH
  // CPU-only retention of the reduction already computed by the solver. Each
  // factor retains one fixed 12x12 block; the original lag owns its lifetime.
  if((seen_.size()<frames_.size() || !initial_seen_ || (source_freeze_enabled_ && !source_selection_done_)) && new_factors.size()<=kMaxFactors) {
    retained_gpu_factors_.erase(std::remove_if(retained_gpu_factors_.begin(),retained_gpu_factors_.end(),
        [](const auto& f){return f.expired();}),retained_gpu_factors_.end());
    for(const auto& f:new_factors) if(auto gpu=dynamic_cast<gtsam_points::IapObservedVGICPFactorGPU*>(f.get())) {
      const auto& owner=gpu->get_evidence_owner();
      if(source_freeze_enabled_ && !source_selection_done_) {
        if(source_id_<0 && owner.target_frame_id>0 && owner.target_is_keyframe && !owner.target_is_fixed) {
          source_id_=owner.target_frame_id;source_birth_frame_=frame;
        }
        if(source_id_==owner.target_frame_id && owner.target_is_fixed && source_freeze_frame_<0)
          source_freeze_frame_=frame;
      }
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
std::unique_ptr<NativeResponsePacket> NativeResponseObserver::capture(
    const gtsam_points::IncrementalFixedLagSmootherExt& native,
    const gtsam::KeyVector& discarded,long frame,double stamp) {
  const auto start=std::chrono::steady_clock::now();
  auto p=std::make_unique<NativeResponsePacket>();
  p->frame=frame;p->stamp=stamp;p->lag=native.smootherLag();p->discarded=discarded;
  const auto& theta=native.getLinearizationPoint();
  const auto& factors=native.getLinearFactors();
  const auto& index=native.getISAM2().getVariableIndex();
  p->keys=theta.size();p->slots=factors.size();p->entries=index.nEntries();
  auto elapsed=[&]{return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();};
  auto check_time=[&]{if(elapsed()>kCopyBudgetMs) throw std::runtime_error("COPY_TIME_CAP_EXCEEDED");};
  auto charge=[&](std::size_t bytes) {
    if(bytes>kMaxBytes-p->bytes) throw std::runtime_error("NATIVE_MATRIX_CAP_EXCEEDED");
    p->bytes+=bytes;
  };
  try {
    if(theta.size()>kMaxKeys || index.nEntries()>kMaxIndexEntries) throw std::runtime_error("NATIVE_GRAPH_CAP_EXCEEDED");
    // Native VariableIndex contains active relationships, including marginals;
    // no scan of historical null slots and no estimator slot-policy change.
    std::vector<std::size_t> active;active.reserve(index.nEntries());
    for(const auto& entry:index) {
      check_time();active.insert(active.end(),entry.second.begin(),entry.second.end());
    }
    std::sort(active.begin(),active.end());active.erase(std::unique(active.begin(),active.end()),active.end());
    p->active=active.size();
    if(active.size()>kMaxFactors) throw std::runtime_error("NATIVE_ACTIVE_FACTOR_CAP_EXCEEDED");
    const auto& delta=native.getDelta();p->states.reserve(theta.size());
    for(const auto& kv:theta) {
      check_time();const auto key=kv.key;const char type=gtsam::Symbol(key).chr();
      ResponseState s;s.key=key;s.dim=kv.value.dim();
      if(type=='x') {s.point=row_major(theta.at<gtsam::Pose3>(key).matrix());s.representation="Pose3_matrix_row_major";}
      else if(type=='r') {s.point=row_major(theta.at<gtsam::Rot3>(key).matrix());s.representation="Rot3_matrix_row_major";}
      else if(type=='b') {s.point=row_major(theta.at<gtsam::imuBias::ConstantBias>(key).vector());s.representation="ConstantBias_acc_gyro";}
      else if(type=='v'||type=='e') {s.point=row_major(theta.at<gtsam::Vector3>(key));s.representation="Vector3";}
      else if(type=='c'||type=='d'||type=='h'||type=='j') {s.point=row_major(theta.at<gtsam::Vector2>(key));s.representation="Vector2";}
      else throw std::runtime_error("UNSUPPORTED_NATIVE_VALUE_TYPE");
      s.delta=row_major(delta.at(key));charge((s.point.size()+s.delta.size())*sizeof(double));
      p->states.push_back(std::move(s));
    }
    const auto& nonlinear=native.getFactors();
    auto new_indices=native.getISAM2Result().newFactorsIndices;std::sort(new_indices.begin(),new_indices.end());
    p->factors.reserve(active.size());
    for(const auto i:active) {
      check_time();if(i>=factors.size() || !factors[i]) throw std::runtime_error("NATIVE_LINEAR_CACHE_MISSING");
      const auto& factor=factors[i];ResponseFactor f;f.index=i;f.keys=factor->keys();
      std::size_t dim=0;f.dims.reserve(factor->size());
      for(auto it=factor->begin();it!=factor->end();++it) {auto d=factor->getDim(it);f.dims.push_back(d);dim+=d;}
      p->max_factor_dim=std::max(p->max_factor_dim,dim);
      if(dim>kMaxFactorDim) throw std::runtime_error("NATIVE_FACTOR_DIM_CAP_EXCEEDED");
      f.rows=dim+1;charge(f.rows*(f.rows+1)/2*sizeof(double));
      std::size_t stored_bytes=0;
      if(const auto* jac=dynamic_cast<const gtsam::JacobianFactor*>(factor.get())) {
        if(jac->rows()>kMaxFactorDim) throw std::runtime_error("NATIVE_FACTOR_ROW_CAP_EXCEEDED");
        stored_bytes=jac->rows()*f.rows*sizeof(double);
      } else if(dynamic_cast<const gtsam::HessianFactor*>(factor.get())) stored_bytes=f.rows*f.rows*sizeof(double);
      else throw std::runtime_error("UNSUPPORTED_NATIVE_GAUSSIAN_TYPE");
#ifdef IAP_NATIVE_GPU_MATCH
      if(dynamic_cast<const gtsam_points::IapCachedVGICPResponse*>(factor.get())) stored_bytes+=(13*13+4*4)*sizeof(double);
#endif
      if(stored_bytes>kMaxCachedBytes-p->cached_bytes) throw std::runtime_error("NATIVE_CACHED_FACTOR_CAP_EXCEEDED");
      p->cached_bytes+=stored_bytes;
      // Deep-copy the already-cached CPU factor; worker computes A^T A. No
      // references to subsequently updated solver state or GPU objects escape.
      f.cached=factor->clone();
      f.new_this_update=std::binary_search(new_indices.begin(),new_indices.end(),i);
      const auto* nf=i<nonlinear.size() ? nonlinear[i].get() : nullptr;
      f.native_type=nf ? typeid(*nf).name() : "missing";
      if(const auto* pr=dynamic_cast<const PseudorangeFactor*>(nf)) {
        f.pseudorange=true;f.sat_id=pr->sat_id();f.constellation=pr->constellation();
        f.pr_meas=pr->pr_meas();f.elevation=pr->elevation();
      }
#ifdef IAP_NATIVE_GPU_MATCH
      if(const auto* gpu=dynamic_cast<const gtsam_points::IapObservedVGICPFactorGPU*>(nf)) {
        f.vgicp=true;f.owner=gpu->get_evidence_owner();
        const auto* bound=dynamic_cast<const gtsam_points::IapCachedVGICPResponse*>(factor.get());
        f.target_response_available=bound!=nullptr;
        if(bound) {
          const auto& response=bound->native_response();
          charge((13*14/2+4*4)*sizeof(double));
          f.target_upper=packed_upper(response.augmented_information);
          f.linearization_transform=row_major(response.linearization_transform);
        }
        if(f.owner.target_is_fixed) {charge(16*sizeof(double));f.fixed_target_pose=row_major(gpu->get_fixed_target_pose().matrix());}
      }
#endif
      p->factors.push_back(std::move(f));
    }
    check_time();p->complete=true;
  } catch(const std::exception& e) {p->failure=e.what();}
  // Partial packet disposal and all formatting occur on the writer, not here.
  p->copy_ms=elapsed();return p;
}
Json NativeResponseObserver::serialize(const NativeResponsePacket& p) {
  Json out={{"schema","iap_native_response_v2"},{"frame_id",p.frame},{"stamp",p.stamp},{"lag_s",p.lag},
    {"discarded_keys",p.discarded},{"complete",p.complete},{"failure_reason",p.failure},
    {"initial_x0_elimination",p.initial},{"copy_ms",p.copy_ms},{"copied_matrix_bytes",p.bytes},
    {"cached_factor_bytes",p.cached_bytes},
    {"selection_roles",p.selection_roles},{"selected_source_id",p.selected_source_id},
    {"native_key_count",p.keys},{"native_factor_slot_count",p.slots},{"native_index_entries",p.entries},
    {"native_active_factor_count",p.active},{"max_observed_factor_tangent_dim",p.max_factor_dim},
    {"coefficient_semantics","cached_native_loss_normal_equations_not_physical_noise"},
    {"matrix_encoding","symmetric_upper_triangle_row_major"},{"physical_joint_error_law_available",false},
    {"states",Json::array()},{"factors",Json::array()}};
  if(!p.complete) return out;
  for(const auto& s:p.states) out["states"].push_back({{"key",s.key},{"type",std::string(1,gtsam::Symbol(s.key).chr())},
    {"dim",s.dim},{"linearization_point",s.point},{"representation",s.representation},{"delta",s.delta}});
  for(const auto& f:p.factors) {
    const auto native_matrix=f.cached->augmentedInformation();
    double target_block_difference=0.;
#ifdef IAP_NATIVE_GPU_MATCH
    if(f.vgicp && f.target_response_available) {
      const auto* bound=dynamic_cast<const gtsam_points::IapCachedVGICPResponse*>(f.cached.get());
      if(!bound) throw std::runtime_error("NATIVE_TARGET_RESPONSE_OWNER_MISSING");
      const auto& full=bound->native_response().augmented_information;
      Eigen::MatrixXd expected=f.owner.target_is_fixed ? Eigen::MatrixXd(full.bottomRightCorner<7,7>()) : Eigen::MatrixXd(full);
      if(native_matrix.rows()!=expected.rows()) throw std::runtime_error("NATIVE_TARGET_RESPONSE_DIMENSION_MISMATCH");
      target_block_difference=(native_matrix-expected).cwiseAbs().maxCoeff();
      if(!native_matrix.isApprox(expected,1e-12)) throw std::runtime_error("NATIVE_TARGET_RESPONSE_COEFFICIENT_MISMATCH");
    }
#endif
    Json row={{"index",f.index},{"keys",f.keys},{"dims",f.dims},{"rows",f.rows},
      {"augmented_information_upper",packed_upper(native_matrix)},
      {"new_this_update",f.new_this_update},{"native_type",f.native_type}};
    if(f.pseudorange) row["pseudorange"]={{"sat_id",f.sat_id},{"constellation",std::string(1,f.constellation)},
      {"pr_meas_m",f.pr_meas},{"elevation_rad",f.elevation}};
    if(f.vgicp) {
      const auto& o=f.owner;
      row["vgicp"]={{"source_frame_id",o.source_frame_id},{"target_frame_id",o.target_frame_id},
        {"source_stamp",o.source_stamp},{"target_stamp",o.target_stamp},{"level_id",o.level_id},
        {"target_is_fixed",o.target_is_fixed},{"voxel_resolution_m",o.voxel_resolution_m},
        {"target_is_keyframe",o.target_is_keyframe},
        {"target_response_available",f.target_response_available}};
      row["vgicp"]["target_response_from_cached_gaussian"]=f.target_response_available;
      row["vgicp"]["target_block_absolute_difference"]=target_block_difference;
      if(f.target_response_available) {
        row["vgicp"]["full_augmented_information_upper"]=f.target_upper;
        row["vgicp"]["linearization_transform_row_major"]=f.linearization_transform;
      }
      if(o.target_is_fixed) row["vgicp"]["fixed_target_pose_row_major"]=f.fixed_target_pose;
    }
    out["factors"].push_back(std::move(row));
  }
  return out;
}
Json NativeResponseObserver::snapshot(const gtsam_points::IncrementalFixedLagSmootherExt& native,
    const gtsam::KeyVector& discarded,long frame,double stamp) {
  return serialize(*capture(native,discarded,frame,stamp));
}
void NativeResponseObserver::observe(const gtsam_points::IncrementalFixedLagSmootherExt& native,
    const gtsam::KeyVector& discarded,long frame,double stamp) {
  const auto observer_start=std::chrono::steady_clock::now();
  struct Timing {
    NativeResponseObserver& self;std::chrono::steady_clock::time_point start;
    ~Timing() {
      const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      self.max_observer_ms_=std::max(self.max_observer_ms_,ms);
      if(ms>kCopyBudgetMs) ++self.observer_budget_overruns_;
    }
  } timing{*this,observer_start};
  const bool initial=!initial_seen_ && std::find(discarded.begin(),discarded.end(),gtsam::Symbol('x',0))!=discarded.end();
  std::vector<std::string> roles;
  if(frames_.count(frame) && !seen_.count(frame)) roles.push_back("explicit");
  if(initial) roles.push_back("initial_x0_elimination");
  bool rolling=source_freeze_enabled_ && !source_selection_done_ && source_id_>=0 && source_freeze_frame_<0;
  if(source_freeze_enabled_ && !source_selection_done_ && source_id_>=0) {
    if(frame-source_birth_frame_>100) {
      source_selection_failure_="SOURCE_BOUNDARY_NOT_OBSERVED_WITHIN_TRACKING_CAP";
      source_selection_done_=true;rolling=false;previous_.reset();
    } else {
      if(frame==source_freeze_frame_) roles.push_back("first_nonzero_fixed_use");
      if(source_freeze_frame_>=0 && frame==source_freeze_frame_+1) roles.push_back("first_update_after_fixed_use");
      if(source_freeze_frame_>=0 && frame==source_freeze_frame_+10) roles.push_back("fixed_use_plus_10_updates");
      if(source_retirement_frame_<0 && std::find(discarded.begin(),discarded.end(),gtsam::Symbol('x',source_id_))!=discarded.end()) {
        source_retirement_frame_=frame;roles.push_back("nonzero_source_native_retirement");
      }
    }
  }
  if(roles.empty() && !rolling) return;
  if(frames_.count(frame)) seen_.insert(frame);if(initial) initial_seen_=true;
  auto packet=capture(native,discarded,frame,stamp);packet->initial=initial;
  packet->selection_roles=roles;packet->selected_source_id=source_id_;
  // A single rolling packet before the selected fixed-use boundary. Never a
  // trajectory archive; original CPU clone/deadline caps apply on every copy.
  if(roles.empty()) {
    ++rolling_copies_;if(!packet->complete) ++rolling_failures_;
    previous_=std::move(packet);return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto enqueue=[&](std::unique_ptr<NativeResponsePacket> p) {
    if(ledger_.size()>=kMaxFrames+1) throw std::runtime_error("NATIVE_SELECTION_CAP_EXCEEDED");
    const bool accepted=queue_.size()<kQueueLimit;p->request_id=ledger_.size();
    ledger_.push_back({{"frame_id",p->frame},{"stamp",p->stamp},{"initial_x0_elimination",p->initial},
      {"selection_roles",p->selection_roles},{"complete",p->complete},{"failure_reason",p->failure},{"copy_ms",p->copy_ms},
      {"native_active_factor_count",p->active},{"native_factor_slot_count",p->slots},
      {"max_observed_factor_tangent_dim",p->max_factor_dim},{"queue_accepted",accepted}});
    if(accepted) queue_.push_back(std::move(p));
  };
  if(frame==source_freeze_frame_) {
    if(previous_ && previous_->frame==frame-1) {
      source_previous_frame_=previous_->frame;previous_->selection_roles={"update_before_first_fixed_use"};
      enqueue(std::move(previous_));
    } else source_selection_failure_="PREVIOUS_NATIVE_UPDATE_NOT_RETAINED";
  }
  // Explicit selections during tracking keep no second full packet. If the
  // next update freezes the source, absence is explicit rather than filled.
  if(rolling) previous_.reset();
  enqueue(std::move(packet));cv_.notify_one();
  if(source_freeze_frame_>=0 && frame>=source_freeze_frame_+10 && source_retirement_frame_>=0)
    source_selection_done_=true;
}
void NativeResponseObserver::drain() {
  for(;;) {
    std::unique_ptr<NativeResponsePacket> packet;
    {std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stopping_||!queue_.empty();});
      if(queue_.empty()) return;packet=std::move(queue_.front());queue_.pop_front();}
    try {
      Json encoded;
      try {encoded=serialize(*packet);}
      catch(const std::exception& error) {
        packet->complete=false;packet->failure=error.what();encoded=serialize(*packet);
        std::lock_guard<std::mutex> lock(mutex_);
        ledger_[packet->request_id]["complete"]=false;
        ledger_[packet->request_id]["failure_reason"]=packet->failure;
      }
      const auto line=encoded.dump()+"\n";
      if(gzwrite(static_cast<gzFile>(raw_),line.data(),line.size())!=static_cast<int>(line.size())) ++write_failures_;
      else ++written_;
    } catch(const std::exception& error) {
      ++write_failures_;spdlog::error("[native_response] serialize failed: {}",error.what());
    }
  }
}
}
