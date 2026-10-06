// Offline experiments use the production module and frozen input codec.
// No diagnostics are written back to GridMap or an execution authority.
#include <ego_planner/prediction_input.h>
#include <iap/util/run_log_manager.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {
using Input = ego_planner::PredictionInput;
using Clock = std::chrono::steady_clock;
using Point = Eigen::Vector3d;
std::string campaign_namespace;
double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now()-start).count();
}
void number(std::ostream& out, double value) {
  if (std::isfinite(value)) out << std::setprecision(17) << value;
  else out << "null";
}
template<class Derived> void array(std::ostream& out, const Eigen::MatrixBase<Derived>& v) {
  out << '[';
  for (int r=0; r<v.rows(); ++r) for (int c=0; c<v.cols(); ++c) {
    if (r || c) out << ',';
    number(out,v(r,c));
  }
  out << ']';
}
void matrix(std::ostream& out, const char* name, const Eigen::Matrix3d& m, bool evaluated) {
  out << ",\"" << name << "\":";
  if (!evaluated) { out << "null"; return; }
  out << "{\"row_major\":"; array(out,m);
  out << ",\"trace\":"; number(out,m.trace());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig((m+m.transpose())*.5);
  out << ",\"eigenvalues\":"; array(out,eig.eigenvalues());
  out << ",\"weak_direction\":";
  array(out,eig.eigenvectors().col(std::string(name)=="covariance"?2:0));
  out << '}';
}
void csvString(std::ostream& out, const std::string& s) {
  out << '"'; for (char c:s) { if(c=='"') out << '"'; out << c; } out << '"';
}
void csvNumber(std::ostream& out,double value) {
  if (std::isfinite(value)) out << std::setprecision(17) << value;
}
void bytes(const std::filesystem::path& path,const std::vector<uint8_t>& data) {
  std::ofstream out(path,std::ios::binary); out.write(reinterpret_cast<const char*>(data.data()),data.size());
  if (!out) throw std::runtime_error("cannot write frozen input");
}
Input fixture() {
  Input in; in.reference_time_s=100.; in.validity_s=.5;
  auto& s=in.integrity;
  s.valid=s.has_pose=s.current.valid=s.has_lambda_base=s.has_epoch=true;
  s.stamp=s.pose_stamp=s.current.stamp=s.gnss_epoch.stamp=100.;
  s.p_wb=Point(.05,.05,1.05); s.current.estimation_frame_id=1;
  s.current.current_motion_quality=1; s.current.current_motion_error_proxy_m=.012;
  s.lambda_base_pos=std::pow(3./.012,2)*Eigen::Matrix3d::Identity();
  s.prior_source_generation=1; s.current.gnss_valid=true;
  s.current.gnss_hpl=.4; s.current.gnss_vpl=.6;
  s.gnss_epoch.source_identity=1;
  for(int i=0;i<12;++i) {
    iap::SatObs sat; sat.sat_id=300+i; sat.constellation='G'; sat.pr_sigma=3.;
    sat.azimuth=i*2.*M_PI/12.; sat.elevation=(20.+(i%3)*25.)*M_PI/180.;
    s.gnss_epoch.sats.push_back(sat);
  }
  s.current.gnss_epoch_stamp=100.;
  s.current.gnss_epoch_identity=iap::gnss_epoch_identity(s.gnss_epoch,s.current.excluded_prns);
  in.params.lidar.enable_legacy_observability=false;
  in.params.freshness.enabled=true; in.params.freshness.max_gnss_age_s=2.;
  in.params.covariance_growth.sigma_grow_m_sqrt_s=0.;
  // Covers default soft-canopy 5 m LOS plus the 10 m scan. This is explicit
  // synthetic observed support, not a claim about a forest's unknown cells.
  auto e=std::make_shared<FrozenOccupancyEpoch>();
  e->lattice_origin=Point(-10,-10,-2); e->voxel_dimensions=Eigen::Vector3i(200,200,80);
  e->resolution_m=.1; e->resolution_inv=10.; e->extent_m=e->voxel_dimensions.cast<double>()*.1;
  e->generation=1; e->active_window_generation=1; e->frame_id="map";
  e->geometry_id="synthetic_six_planes_0.1m_v1"; e->cloud_stamp_s=100.;
  e->map_inflation_m=0.; e->current_vehicle_position=s.p_wb;
  auto cells=std::make_shared<FrozenOccupancyCells>();
  cells->flags.assign(size_t(e->voxel_dimensions.prod()),4);
  auto points=std::make_shared<std::vector<Point>>();
  // Surfaces stay below satellite LOS starts; spatial LiDAR information is
  // stronger near these patches and retains an inspectable weak direction.
  for(int axis=0;axis<3;++axis) for(int side:{-1,1})
    for(int a=-6;a<=6;++a) for(int b=-6;b<=6;++b) {
      Point p=s.p_wb; p[axis]+=side*2.; p[(axis+1)%3]+=.2*a; p[(axis+2)%3]+=.2*b;
      const Eigen::Vector3i index=((p-e->lattice_origin)*e->resolution_inv).array().floor().cast<int>();
      p=e->lattice_origin+(index.cast<double>().array()+.5).matrix()*e->resolution_m;
      const size_t address=(size_t(index.x())*e->voxel_dimensions.y()+index.y())*e->voxel_dimensions.z()+index.z();
      cells->flags.at(address)|=1; points->push_back(p);
    }
  e->cells=cells; e->raw_occupied_voxel_centers=points; e->environment_occupied_voxel_centers=points;
  in.occupancy=e;
  return in;
}
std::string coordinateReason(const Input& in) {
  if (!in.occupancy) return "missing_physical_epoch";
  const auto& e=*in.occupancy;
  if (!std::isfinite(in.reference_time_s) || !in.integrity.p_wb.allFinite()) return "invalid_coordinate_or_reference_time";
  if (e.frame_id!="map" && e.frame_id!="enu") return "unsupported_query_frame";
  if (std::abs(e.resolution_m*e.resolution_inv-1.)>1e-12 ||
      (e.extent_m-e.voxel_dimensions.cast<double>()*e.resolution_m).cwiseAbs().maxCoeff()>e.resolution_m+1e-12)
    return "inconsistent_exported_geometry";
  return "";
}
void describe(const Input& in,const std::filesystem::path& path) {
  std::ofstream o(path); const auto& s=in.integrity;
  o << "{\"schema\":\"iap_advisory_frozen_metadata_v1\",\"reference_time_s\":"; number(o,in.reference_time_s);
  o << ",\"pose_stamp\":"; number(o,s.pose_stamp);
  o << ",\"current_stamp\":"; number(o,s.current.stamp);
  o << ",\"snapshot_stamp\":"; number(o,s.stamp);
  o << ",\"gnss_stamp\":"; number(o,s.gnss_epoch.stamp);
  o << ",\"has_epoch\":" << (s.has_epoch?"true":"false") << ",\"epoch_identity\":" << s.current.gnss_epoch_identity;
  o << ",\"prior_generation\":" << s.prior_source_generation << ",\"position\":"; array(o,s.p_wb);
  o << ",\"quaternion_xyzw\":"; array(o,s.q_wb.coeffs());
  o << ",\"prior_row_major\":"; array(o,s.lambda_base_pos);
  o << ",\"source_mode\":" << static_cast<int>(in.params.source_mode);
  o << ",\"gnss_epoch_policy\":" << static_cast<int>(in.params.gnss_epoch_policy);
  o << ",\"gnss_sigma\": [";
  for(size_t i=0;i<s.gnss_epoch.sats.size();++i) {if(i)o<<',';number(o,s.gnss_epoch.sats[i].pr_sigma);} o<<']';
  o << ",\"excluded_prns\": [";
  for(size_t i=0;i<s.current.excluded_prns.size();++i) {if(i)o<<',';o<<s.current.excluded_prns[i];} o<<']';
  o << ",\"lidar_sigma\":";number(o,in.params.lidar.fim_params.fim_range_sigma_base);
  o << ",\"conservative_max_with_gnss\":" << (in.params.fusion.conservative_max_with_gnss?"true":"false");
  o << ",\"parameter_authority\":\"complete serialized PredictorParams in input.bin\",\"primitive_derivation\":\"make_lidar_fim_primitives(default generation params), frozen raw centers\"";
  if(in.occupancy) {
    const auto& e=*in.occupancy;
    o<<",\"frame_id\":"<<std::quoted(e.frame_id)<<",\"geometry_id\":"<<std::quoted(e.geometry_id)<<",\"generation\":"<<e.generation;
    o<<",\"cloud_stamp\":"; number(o,e.cloud_stamp_s);
    o<<",\"origin\":";array(o,e.lattice_origin);o<<",\"dimensions\":";array(o,e.voxel_dimensions);
    o<<",\"resolution_m\":";number(o,e.resolution_m);
  }
  o << "}\n";
}

void evaluate(const Input& source,const std::string& label,const std::string& identity,
              bool scan,double budget_s,glim::RunLogManager& log,bool weak_normals=false) {
  const auto dir=log.export_path("advisory/validation/"+campaign_namespace+label+"/points.csv").parent_path();
  std::filesystem::create_directories(dir);
  if(std::filesystem::exists(dir/"points.csv")) throw std::runtime_error("refusing to overwrite experiment: "+label);
  const auto codec_start=Clock::now();
  Input in=source;
  if(source.occupancy) {
    const auto payload=ego_planner::encodePredictionInput(source);
    bytes(dir/"input.bin",payload);
    in=ego_planner::decodePredictionInput(payload);
  }
  const double codec_s=seconds(codec_start);
  describe(in,dir/"input.json");
  {std::ofstream variant(dir/"variant.json");
   variant<<"{\"identity\":"<<std::quoted(identity)<<",\"label\":"<<std::quoted(label)
     <<",\"weak_normal_support_override\":"<<(weak_normals?"true":"false")
     <<",\"support_rule\":\""<<(weak_normals?"derive default PCA then retain abs(normal_w.x)<0.1":"default production derivation")<<"\"}\n";}
  std::ofstream csv(dir/"points.csv"), matrices(dir/"matrices.jsonl");
  std::string timing_label=campaign_namespace+label;
  std::replace(timing_label.begin(),timing_label.end(),'/','_');
  std::ofstream timing(log.profiling_path("advisory_validation_"+timing_label+".csv"));
  csv<<"id,identity,label,x,y,z,ix,iy,iz,generation,reference_time_s,cloud_stamp_s,pose_stamp_s,current_stamp_s,gnss_stamp_s,observed,raw_occupied,inflated_occupied,status,reason,wrapper_bound,wrapper_called,module_called,valid,gnss_used,lidar_used,prior_used,gnss_valid,lidar_valid,gnss_reason,lidar_reason,gnss_hpl,gnss_vpl,gnss_raw_hpl,gnss_raw_vpl,lidar_hpl,lidar_vpl,prior_hpl,prior_vpl,pre_hpl,pre_vpl,fused_hpl,fused_vpl,floor_h,floor_v,module_valid,module_hpl,module_vpl,repeat_equal,batch_equal,wrapper_equal,query_s,codec_equal,wrapper_original_hpl,wrapper_original_vpl\n";
  timing<<"label,codec_s,preparation_s,query_total_s,total_s,requested,wrapper_calls,module_calls\n";
  auto total=Clock::now(), started=total;
  auto calls=std::make_shared<std::atomic<uint64_t>>(0); std::string binding_reason;
  auto context=ego_planner::makeRiskPrediction(in,calls,&binding_reason);
  auto before_codec=ego_planner::makeRiskPrediction(source);
  std::unique_ptr<iap::PredictorModule> module;
  if(in.occupancy) {
    module=std::make_unique<iap::PredictorModule>(ego_planner::makeFrozenPredictor(in));
    if(weak_normals) {
      // Diagnostic observation support only; the physical flags stay fixed.
      auto primitives=iap::make_lidar_fim_primitives(*in.occupancy->raw_occupied_voxel_centers);
      auto selected=std::make_shared<std::vector<iap::LidarFimPrimitive>>();
      for(const auto& p:*primitives) if(std::abs(p.normal_w.x())<.1) selected->push_back(p);
      module->set_lidar_fim_primitives(selected);
    }
  }
  const double preparation=seconds(started);
  std::string input_reason=coordinateReason(in);
  if(preparation>budget_s) input_reason="preparation_budget_exceeded";
  std::vector<Point> points;
  Point p=in.integrity.p_wb;
  if(scan && in.occupancy && p.allFinite() && (p-in.occupancy->lattice_origin).cwiseAbs().maxCoeff()<1e6) {
    std::set<std::array<int,3>> seen;
    for(int x=0;x<10;++x) for(int y=0;y<10;++y) {
      const auto& e=*in.occupancy;
      Point q=p+Point(x-4.5,y-4.5,0);
      const Eigen::Vector3i index=((q-e.lattice_origin)*e.resolution_inv).array().floor().cast<int>();
      if(seen.insert({index.x(),index.y(),index.z()}).second)
        points.push_back(e.lattice_origin+(index.cast<double>().array()+.5).matrix()*e.resolution_m);
    }
  } else points.push_back(p);  // Exact receiver for same-reference-time error comparisons.
  size_t direct_calls=0; double query_total=0.;
  for(size_t id=0;id<points.size();++id) {
    started=Clock::now(); const Point& center=points[id];
    const bool in_bounds=in.occupancy && center.allFinite() &&
      (center.array()>=in.occupancy->lattice_origin.array()).all() &&
      (center.array()<(in.occupancy->lattice_origin+in.occupancy->voxel_dimensions.cast<double>()*in.occupancy->resolution_m).array()).all();
    auto physical=in_bounds?GridMap::queryFrozenOccupancy(*in.occupancy,center):GridMapOccupancyDiagnostic{};
    std::string status="INPUT_UNAVAILABLE", reason=input_reason.empty()?binding_reason:input_reason;
    bool physical_ok=false;
    if(input_reason.empty() && in.occupancy) {
      if(!physical.available) {status="COORDINATE_ERROR";reason="out_of_map";}
      else if(physical.raw_occupied || physical.inflated_occupied) {status="PHYSICAL_FILTERED";reason="occupied";}
      else if(!physical.observed) {status="PHYSICAL_FILTERED";reason="unobserved";}
      else physical_ok=true;
    }
    if(input_reason=="preparation_budget_exceeded") status="BUDGET_EXCEEDED";
    else if(!input_reason.empty()) status=input_reason=="missing_physical_epoch"?"INPUT_UNAVAILABLE":"COORDINATE_ERROR";
    iap::PredictorQueryResult result; GridRiskVoxel wrapped;
    bool repeat_equal=true,batch_equal=true,wrapper_equal=true,codec_equal=bool(context.predict)==bool(before_codec.predict),called=false;
    if(physical_ok && module) {
      iap::PredictorQueryInput query(center,in.integrity,in.reference_time_s,0.,in.occupancy->frame_id,in.reference_time_s);
      result=module->query(query); ++direct_calls; called=true;
      const auto again=module->query(query); ++direct_calls;
      const auto batch=module->queryBatch({query,query}); direct_calls+=2;
      const auto equal=[&](const iap::PredictorQueryResult& a) {
        return a.valid==result.valid && a.available==result.available && a.fallback_reason==result.fallback_reason &&
          a.freshness_status==result.freshness_status && (!result.valid ||
          (std::abs(a.fused.hpl-result.fused.hpl)<=1e-12 && std::abs(a.fused.vpl-result.fused.vpl)<=1e-12));
      };
      repeat_equal=equal(again);batch_equal=batch.size()==2 && equal(batch[0]) && equal(batch[1]);
      if(context.predict) {
        wrapped=context.predict(center);
        const auto original=before_codec.predict(center);
        codec_equal=original.status==wrapped.status && (wrapped.status!=GridRiskStatus::VALID ||
          (std::abs(wrapped.hpl-original.hpl)<=1e-12 && std::abs(wrapped.vpl-original.vpl)<=1e-12));
        const auto projected=ego_planner::predictionRiskVoxel(result);
        wrapper_equal=weak_normals || (wrapped.status==projected.status &&
          (wrapped.status!=GridRiskStatus::VALID || (std::abs(wrapped.hpl-projected.hpl)<=1e-12 && std::abs(wrapped.vpl-projected.vpl)<=1e-12)));
        status=wrapped.status==GridRiskStatus::VALID?"VALID":wrapped.status==GridRiskStatus::STALE?"STALE":wrapped.status==GridRiskStatus::PREDICTED_DEGRADED?"MODEL_DEGRADED":"MODEL_INVALID";
        reason=result.fallback_reason;
      } else status=binding_reason.find("stale")!=std::string::npos?"STALE":"INPUT_UNAVAILABLE";
      if(weak_normals) {status="DIAGNOSTIC_ONLY";reason="offline_normal_support_override";}
      if(!repeat_equal || !batch_equal || !wrapper_equal || !codec_equal) throw std::runtime_error("replay equivalence failed: "+label);
    }
    const auto& f=result.fused;
    const bool valid=physical_ok && context.predict && wrapped.status==GridRiskStatus::VALID && !weak_normals;
    csv<<id<<','<<identity<<','<<label<<','<<std::setprecision(17)<<center.x()<<','<<center.y()<<','<<center.z()<<','<<physical.voxel_index.x()<<','<<physical.voxel_index.y()<<','<<physical.voxel_index.z()<<','<<(in.occupancy?in.occupancy->generation:0)<<','<<in.reference_time_s<<',';
    for(double v:{in.occupancy?in.occupancy->cloud_stamp_s:NAN,in.integrity.pose_stamp,in.integrity.current.stamp,in.integrity.has_epoch?in.integrity.gnss_epoch.stamp:NAN}) {csvNumber(csv,v);csv<<',';}
    csv<<physical.observed<<','<<physical.raw_occupied<<','<<physical.inflated_occupied<<','<<status<<',';csvString(csv,reason);
    csv<<','<<bool(context.predict)<<','<<(physical_ok&&bool(context.predict))<<','<<called<<','<<valid<<','<<f.gnss_used<<','<<f.lidar_used<<','<<(f.prior_valid&&(f.gnss_used||f.lidar_used))<<','<<result.gnss.valid<<','<<result.lidar.valid<<',';
    csvString(csv,result.gnss.fallback_reason);csv<<',';csvString(csv,result.lidar.fallback_reason);csv<<',';
    for(double v:{result.gnss.valid?result.gnss.hpl:NAN,result.gnss.valid?result.gnss.vpl:NAN,
        result.gnss.valid?result.gnss.raw_hpl:NAN,result.gnss.valid?result.gnss.raw_vpl:NAN,
        f.lidar_only_hpl,f.lidar_only_vpl,f.prior_only_hpl,f.prior_only_vpl,f.pre_conservative_hpl,f.pre_conservative_vpl,
        valid?wrapped.hpl:NAN,valid?wrapped.vpl:NAN,called?f.floor_increment_h:NAN,called?f.floor_increment_v:NAN}) {csvNumber(csv,v);csv<<',';}
    csv<<result.valid<<','; csvNumber(csv,result.valid?f.hpl:NAN);csv<<',';csvNumber(csv,result.valid?f.vpl:NAN);
    const double elapsed=seconds(started);query_total+=elapsed;
    csv<<',';if(called) csv<<repeat_equal;csv<<',';if(called) csv<<batch_equal;csv<<',';
    if(!weak_normals && context.predict && physical_ok) csv<<wrapper_equal;
    csv<<','<<elapsed<<','<<codec_equal<<',';csvNumber(csv,wrapped.hpl);csv<<',';csvNumber(csv,wrapped.vpl);csv<<'\n';
    matrices<<"{\"id\":"<<id<<",\"label\":"<<std::quoted(label)<<",\"identity\":"<<std::quoted(identity)<<",\"module_valid\":"<<(result.valid?"true":"false");
    matrix(matrices,"prior",f.lambda_prior,called);matrix(matrices,"gnss",f.lambda_gnss,called);
    matrix(matrices,"lidar",f.lambda_lidar,called);matrix(matrices,"fused_information",f.lambda_pred,called && result.valid);
    matrix(matrices,"covariance",f.sigma_pos,called && result.valid);
    matrices<<",\"weak_direction_prior_fraction\":";
    if(result.valid) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(f.lambda_pred);
      const Point weak=eig.eigenvectors().col(0);const double total_info=weak.dot(f.lambda_pred*weak);
      number(matrices,total_info>0?weak.dot(f.lambda_prior*weak)/total_info:NAN);
    } else matrices<<"null";
    matrices<<"}\n";
  }
  timing<<label<<','<<std::setprecision(17)<<codec_s<<','<<preparation<<','<<query_total<<','<<seconds(total)+codec_s<<','<<points.size()<<','<<calls->load()<<','<<direct_calls<<'\n';
  if(!csv || !matrices || !timing) throw std::runtime_error("failed experiment write");
}

void campaign(const Input& original,const std::string& identity,double budget,glim::RunLogManager& log) {
  evaluate(original,"S0",identity,true,budget,log);
  for(const std::string group:{"S1","S2","S3","S4"})
    for(int level=0;level<3;++level) {
      auto in=original; const double scale=level==0?1.:level==1?10.:100.;
      if(group=="S1" || group=="S3") {
        for(auto& sat:in.integrity.gnss_epoch.sats) sat.pr_sigma*=scale;
        in.integrity.current.gnss_epoch_identity=iap::gnss_epoch_identity(in.integrity.gnss_epoch,in.integrity.current.excluded_prns);
      }
      if(group=="S2" || group=="S3") in.params.lidar.fim_params.fim_range_sigma_base*=scale;
      if(group=="S4") {
        in.integrity.lambda_base_pos*=level==0?1.:level==1?.1:0.;
        if(level==2) in.integrity.has_lambda_base=false;
      }
      evaluate(in,group+"_"+std::to_string(level),identity,false,budget,log);
    }
  // Paired prior-free dual-source degradation for the signal-retention ratio.
  for(int level=0;level<3;++level) {
    auto in=original;in.integrity.has_lambda_base=false;in.integrity.lambda_base_pos.setZero();
    const double scale=level==0?1.:level==1?10.:100.;
    for(auto& sat:in.integrity.gnss_epoch.sats) sat.pr_sigma*=scale;
    in.integrity.current.gnss_epoch_identity=iap::gnss_epoch_identity(in.integrity.gnss_epoch,in.integrity.current.excluded_prns);
    in.params.lidar.fim_params.fim_range_sigma_base*=scale;
    evaluate(in,"S3_no_prior_"+std::to_string(level),identity,false,budget,log);
  }
  evaluate(original,"S2_weak_normal_support",identity,false,budget,log,true);
  for(const std::string mode:{"gnss","lidar"}) {
    auto in=original;in.params.source_mode=mode=="gnss"?iap::PredictorSourceMode::GnssOnly:iap::PredictorSourceMode::LidarOnly;
    evaluate(in,"source_"+mode,identity,false,budget,log);
  }
  for(const std::string name:{"missing_gnss","stale_gnss","missing_lidar","both_missing","no_observations","missing_physical","stale_pose","stale_current","stale_snapshot","stale_cloud","missing_pose","invalid_current","wrong_frame","out_of_map","physical_occupied","physical_unobserved","preparation_budget"}) {
    auto in=original;
    if(name=="missing_gnss" || name=="both_missing") in.integrity.has_epoch=false;
    if(name=="missing_lidar" || name=="both_missing" || name=="no_observations") {
      auto e=std::make_shared<FrozenOccupancyEpoch>(*in.occupancy);
      e->raw_occupied_voxel_centers=std::make_shared<const std::vector<Point>>();in.occupancy=e;
    }
    if(name=="stale_gnss") in.integrity.gnss_epoch.stamp-=10.;
    if(name=="stale_pose") in.integrity.pose_stamp-=10.;
    if(name=="stale_current") in.integrity.current.stamp-=10.;
    if(name=="stale_snapshot") in.integrity.stamp-=10.;
    if(name=="stale_cloud") {auto e=std::make_shared<FrozenOccupancyEpoch>(*in.occupancy);e->cloud_stamp_s-=10.;in.occupancy=e;}
    if(name=="missing_pose") in.integrity.has_pose=false;
    if(name=="invalid_current") in.integrity.current.valid=false;
    if(name=="wrong_frame") {auto e=std::make_shared<FrozenOccupancyEpoch>(*in.occupancy);e->frame_id="wrong_frame";in.occupancy=e;}
    if(name=="out_of_map") in.integrity.p_wb.x()=1000.;
    if(name=="no_observations") {in.integrity.gnss_epoch.sats.clear();in.integrity.current.gnss_valid=false;}
    if(name=="physical_occupied") in.integrity.p_wb=in.occupancy->raw_occupied_voxel_centers->front();
    if(name=="physical_unobserved") {
      auto e=std::make_shared<FrozenOccupancyEpoch>(*in.occupancy);
      auto cells=std::make_shared<FrozenOccupancyCells>(*e->cells);
      const Eigen::Vector3i index=((in.integrity.p_wb-e->lattice_origin)*e->resolution_inv).array().floor().cast<int>();
      cells->flags[(size_t(index.x())*e->voxel_dimensions.y()+index.y())*e->voxel_dimensions.z()+index.z()]&=~uint8_t(4);
      e->cells=cells;in.occupancy=e;
    }
    if(name=="missing_physical") in.occupancy.reset();
    evaluate(in,"S5_"+name,identity,false,name=="preparation_budget"?0.:budget,log);
  }
}
} // namespace

int main(int argc,char** argv) {
  try {
    if(argc<3) throw std::invalid_argument("advisory_validation fixture|replay LABEL [PAYLOAD] [BUDGET_S] [campaign]");
    if(!std::getenv("IAP_RUN_DIR")) throw std::runtime_error("IAP_RUN_DIR must be allocated by the Python owner");
    const std::string mode=argv[1],label=argv[2];
    if(label.empty() || label.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
      throw std::invalid_argument("unsafe label");
    auto& log=glim::RunLogManager::initialize("advisory_validation");
    Input in; double budget=120.;
    if(mode=="fixture") in=fixture();
    else if(mode=="replay" && argc>=4) {
      std::ifstream file(argv[3],std::ios::binary|std::ios::ate);
      if(!file || file.tellg()<=0 || file.tellg()>256*1024*1024) throw std::runtime_error("invalid payload file");
      const size_t size=file.tellg();file.seekg(0);std::vector<uint8_t> data(size);
      file.read(reinterpret_cast<char*>(data.data()),size);in=ego_planner::decodePredictionInput(data);
      if(argc>=5) budget=std::stod(argv[4]);
    } else throw std::invalid_argument("invalid validation mode");
    if(!std::isfinite(budget) || budget<0.) throw std::invalid_argument("invalid preparation budget");
    // A fixture campaign's labels are reserved. Real replays use caller labels.
    if(mode=="fixture") {campaign_namespace=label+"/";campaign(in,"SYNTHETIC_MECHANISM",budget,log);}
    else {
      evaluate(in,label,"REAL_REPLAY",true,budget,log);
      evaluate(in,label+"_current","REAL_REPLAY",false,budget,log);
    }
    std::cout<<log.run_dir()<<'\n';
  } catch(const std::exception& e) {std::cerr<<"advisory_validation: "<<e.what()<<'\n';return 1;}
  return 0;
}
