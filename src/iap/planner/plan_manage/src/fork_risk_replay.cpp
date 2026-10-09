#include <ego_planner/prediction_input.h>
#include <iap/sim/forked_forest_geometry.hpp>
#include <path_searching/dyn_a_star.h>
#include <iap/util/run_log_manager.hpp>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <iostream>

namespace {
using Point=Eigen::Vector3d;
void number(std::ostream& out,double value) {if(std::isfinite(value))out<<value;else out<<"null";}
void point(std::ostream& out,const Point& p) {out<<'['<<p.x()<<','<<p.y()<<','<<p.z()<<']';}
struct Sample {Point position;GridMapOccupancyDiagnostic occupancy;GridPlanningCell physical;
  iap::PredictorQueryResult prediction;bool queried=false;};
}

int main(int argc,char** argv) {
  try {
    if(argc!=4 || !std::getenv("IAP_RUN_DIR")) throw std::invalid_argument("fork_risk_replay PAYLOAD LABEL FORK_INDEX; IAP_RUN_DIR required");
    const std::string label=argv[2];const int fork=std::stoi(argv[3]);
    if(label.empty() || label.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos || fork<0 || fork>=4)
      throw std::invalid_argument("invalid fork label/index");
    std::ifstream file(argv[1],std::ios::binary);std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),{});
    if(!file || bytes.empty())throw std::invalid_argument("input payload unavailable");
    auto in=ego_planner::decodePredictionInput(bytes);const auto e=in.occupancy;
    if(!e || !e->cells || (in.recording_codec_version<8 || in.recording_codec_version>9))throw std::invalid_argument("complete v8/v9 frozen input required");
    iap::sim::ForkedForestConfig geometry;double width,z;const double reserve=.5*e->resolution_m,taper=.5;
    GridMotionContext motion;GridPlanningRiskPolicy policy;Point task_goal;
    if(!(std::cin>>geometry.fork_x_min_m>>geometry.fork_length_m>>geometry.low_risk_amplitude_m
         >>geometry.high_risk_amplitude_m>>width>>geometry.junction_clearance_radius_m>>geometry.fork_risk_seed>>z
         >>motion.body_radius_m>>motion.tracking_reserve_m>>motion.motion_budget_m>>motion.max_motion_age_s>>motion.max_environment_age_s
         >>policy.hpl_budget_m>>policy.vpl_budget_m>>policy.reserve_h_m>>policy.reserve_v_m>>policy.unknown_multiplier>>policy.stale_soft_seconds
         >>task_goal.x()>>task_goal.y()>>task_goal.z()))throw std::invalid_argument("missing captured geometry/planning policy");
    if(!(width>0 && geometry.fork_length_m>0 && taper>0) || !std::isfinite(z))throw std::invalid_argument("invalid geometry");
    geometry.corridor_half_width_m=width*.5;
    const double x0=geometry.fork_x_min_m+fork*geometry.fork_length_m;
    const bool entrance_qualified=std::abs(in.integrity.p_wb.x()-x0)<=.5 && std::hypot(in.integrity.p_wb.x()-x0,in.integrity.p_wb.y())<=geometry.junction_clearance_radius_m;
    GridMapFailureSnapshot snapshot;snapshot.origin=e->lattice_origin;snapshot.max_boundary=e->lattice_origin+e->extent_m;
    snapshot.dimensions=e->voxel_dimensions;snapshot.resolution_m=e->resolution_m;snapshot.cloud_stamp_s=e->cloud_stamp_s;
    snapshot.generation=e->generation;snapshot.frame_id=e->frame_id;snapshot.cell_flags=e->cells->flags;
    snapshot.virtual_ceiling_height_m=e->virtual_ceiling_height_m;snapshot.inflation_radius_m=e->map_inflation_m;
    auto map=GridMap::fromFailureSnapshot(snapshot);
    motion.quality=in.integrity.current.current_motion_quality;motion.stamp_s=in.integrity.current.stamp;
    motion.error_proxy_m=in.integrity.current.current_motion_error_proxy_m;
    const auto context=map->preparePlanningQuery(in.reference_time_s,motion,e);
    std::string rejection;auto risk=ego_planner::makeRiskPrediction(in,{},&rejection);
    const auto version=map->bindRiskContext(std::move(risk));
    auto frozen=map->capturePlanningRiskQuery(version,in.reference_time_s,policy,nullptr,e->generation,true);
    auto predictor=ego_planner::makeFrozenPredictor(in);
    auto& log=glim::RunLogManager::initialize("fork_risk_replay");
    const auto root=log.export_path("advisory/forks/"+label+"/samples.csv").parent_path();
    std::filesystem::create_directories(root);
    if(std::filesystem::exists(root/"result.json"))throw std::runtime_error("refusing to overwrite fork evidence");
    std::map<std::array<int,3>,Sample> samples;
    const auto center=[&](const Point& p) {
      const Eigen::Vector3i index=((p-e->lattice_origin)*e->resolution_inv).array().floor().cast<int>();
      return Point(e->lattice_origin+(index.cast<double>().array()+.5).matrix()*e->resolution_m);
    };
    const auto include=[&](const Point& p) {
      const auto q=center(p);const Eigen::Vector3i i=((q-e->lattice_origin)*e->resolution_inv).array().floor().cast<int>();
      Sample sample;sample.position=q;
      samples.emplace(std::array<int,3>{i.x(),i.y(),i.z()},std::move(sample));
    };
    std::map<std::string,std::vector<Point>> routes;
    for(const auto arm:{iap::sim::ForkArm::kLowRisk,iap::sim::ForkArm::kHighRisk}) {
      const std::string side=iap::sim::forkArmSign(geometry,fork,arm)>0?"left":"right";
      auto& route=routes[side];route.push_back(in.integrity.p_wb);
      const int count=std::ceil(geometry.fork_length_m/(e->resolution_m*.5));
      for(int i=0;i<=count;++i) {
        const double x=x0+geometry.fork_length_m*double(i)/count;
        const double y=iap::sim::forkArmCenterY(geometry,fork,arm,x);
        const double slope=iap::sim::forkArmSlope(geometry,fork,arm,x);
        const Point normal=Point(-slope,1,0).normalized();
        const Point p(x,y,z);const Point c=center(p);
        if((route.back()-c).norm()>1e-9)route.push_back(c);
        for(double offset:{-.8,0.,.8})include(p+offset*geometry.corridor_half_width_m*normal);
      }
      // Include both the state->entrance connection and the real exit connection.
      route.push_back(Point(x0+geometry.fork_length_m,0,z));
      for(size_t i=1;i<route.size();++i) {
        const int count=std::max(1,int(std::ceil((route[i]-route[i-1]).norm()/(e->resolution_m*.25))));
        for(int j=0;j<=count;++j)include(route[i-1]+double(j)/count*(route[i]-route[i-1]));
      }
    }
    std::vector<Sample*> batch_samples;std::vector<iap::PredictorQueryInput> batch;
    const auto flush=[&]() {
      const auto results=predictor.queryBatch(batch);
      if(results.size()!=batch_samples.size())throw std::runtime_error("incomplete prediction batch");
      for(size_t i=0;i<results.size();++i){batch_samples[i]->prediction=results[i];batch_samples[i]->queried=true;}
      batch.clear();batch_samples.clear();
    };
    for(auto& item:samples) {
      auto& s=item.second;s.occupancy=GridMap::queryFrozenOccupancy(*e,s.position);
      s.physical=map->queryPlanningCell(s.position,0,in.reference_time_s,policy,motion,true,&context);
      if(s.occupancy.available && s.occupancy.observed && !s.occupancy.raw_occupied && !s.occupancy.inflated_occupied && rejection.empty()) {
        batch.push_back(ego_planner::frozenPredictionQuery(in,s.position));batch_samples.push_back(&s);
        if(batch.size()==64)flush();
      }
    }
    flush();
    std::ofstream csv(root/"samples.csv");csv<<std::setprecision(17);
    csv<<"ix,iy,iz,x,y,z,observed,raw_occupied,inflated_occupied,physical_reason,queried,valid,gnss_valid,gnss_used,lidar_used,gnss_reason,gnss_visible,gnss_unknown,gnss_blocked,lidar_groups,gnss_hpl,gnss_vpl,hpl,vpl,gnss_information_trace,lidar_information_trace,fused_information_trace,fusion_reason,numerical_status\n";
    for(const auto& item:samples) {
      const auto& s=item.second;const auto& r=s.prediction;
      csv<<item.first[0]<<','<<item.first[1]<<','<<item.first[2]<<','<<s.position.x()<<','<<s.position.y()<<','<<s.position.z()<<','
         <<s.occupancy.observed<<','<<s.occupancy.raw_occupied<<','<<s.occupancy.inflated_occupied<<','<<gridExecutionReasonName(s.physical.execution_reason)<<','
         <<s.queried<<','<<r.valid<<','<<r.gnss.valid<<','<<r.fused.gnss_used<<','<<r.fused.lidar_used<<','<<std::quoted(r.gnss.fallback_reason)<<','
         <<r.gnss.n_visible<<','<<r.gnss.n_unknown_support<<','<<r.gnss.n_blocked<<','<<r.lidar.n_support_groups;
      for(double v:{r.gnss.valid?r.gnss.hpl:NAN,r.gnss.valid?r.gnss.vpl:NAN,r.valid?r.fused.hpl:NAN,r.valid?r.fused.vpl:NAN,
          s.queried?r.fused.lambda_gnss.trace():NAN,s.queried?r.fused.lambda_lidar.trace():NAN,s.queried?r.fused.lambda_pred.trace():NAN}) {
        csv<<',';if(std::isfinite(v))csv<<v;
      }csv<<','<<std::quoted(r.fallback_reason)<<','<<int(r.fused.numerical_status)<<'\n';
    }
    std::ofstream out(root/"result.json");out<<std::setprecision(17)<<"{\"identity\":\"FROZEN_FORK_DIAGNOSTIC\",\"execution_authorized\":false,\"fork_index\":"<<fork
      <<",\"entrance_qualified\":"<<(entrance_qualified?"true":"false")<<",\"generation\":"<<e->generation<<",\"reference_time_s\":"<<in.reference_time_s
      <<",\"frame_id\":"<<std::quoted(e->frame_id)<<",\"geometry_id\":"<<std::quoted(e->geometry_id)
      <<",\"state_position_m\":";point(out,in.integrity.p_wb);
    out<<",\"binding_reason\":"<<std::quoted(rejection)<<",\"sample_count\":"<<samples.size()<<",\"routes\":{";
    bool first=true;
    for(const auto& item:routes) {
      if(!first)out<<',';
      first=false;const auto& route=item.second;
      double length=0,total=0;bool executable=true;size_t unknown=0,model_unknown=0,integral_model_unknown=0;
      for(const auto& p:route) {
        const auto physical=map->queryPlanningCell(p,0,in.reference_time_s,policy,motion,false,&context);
        const auto r=physical.executable()?frozen(p):GridPlanningRisk{};
        if(r.query_status!=GridRiskStatus::VALID || !std::isfinite(r.hpl) || !std::isfinite(r.vpl))++model_unknown;
      }
      AStar checker;checker.setSearchMap(map);checker.setFrozenEpoch(e);
      checker.setPlanningQuery([&](const Point& p) {
        auto c=context;const double distance=std::min((p-route.front()).norm(),(p-route.back()).norm());
        c.required_clearance_m+=reserve*std::clamp(distance/taper,0.,1.);
        auto physical=map->queryPlanningCell(p,0,in.reference_time_s,policy,motion,false,&c);
        const auto advisory=physical.executable()?frozen(p):GridPlanningRisk{};
        if(advisory.query_status!=GridRiskStatus::VALID || !std::isfinite(advisory.hpl) || !std::isfinite(advisory.vpl))++integral_model_unknown;
        GridSearchCell cell;cell.execution_reason=physical.execution_reason;cell.advisory_class=advisory.classification;cell.cost_multiplier=advisory.cost_multiplier;return cell;
      },true);
      for(size_t i=1;i<route.size();++i) {
        length+=(route[i]-route[i-1]).norm();const auto cost=checker.diagnosticSegmentCost(route[i-1],route[i]);
        if(cost)total+=*cost;else executable=false;
        if(!GridMap::queryFrozenOccupancy(*e,route[i]).observed)++unknown;
      }
      const double terminal=(route.back()-task_goal).norm();
      out<<std::quoted(item.first)<<":{\"reference_polyline_m\":[";
      for(size_t i=0;i<route.size();++i){if(i)out<<',';point(out,route[i]);}
      out<<"],\"length_m\":"<<length<<",\"reference_executable\":"<<(executable?"true":"false")<<",\"unknown_vertices\":"<<unknown<<",\"model_unknown_vertices\":"<<model_unknown
         <<",\"model_unknown_integral_samples\":"<<integral_model_unknown<<",\"risk_addition_m\":";number(out,executable?total-length:NAN);
      out<<",\"terminal_m\":"<<terminal<<",\"total_cost_m\":";number(out,executable?total+terminal:NAN);out<<'}';
    }
    out<<"},\"scope\":\"Three width bands of existing voxel centers, all along-path connections; blocked reference does not prove entire branch unreachable. Costs use production segment quadrature; no actual Curve or multi-terminal search is claimed.\"}\n";
    csv.close();out.close();
    if(!csv || !out)throw std::runtime_error("fork output write/close failed");
    std::cout<<root<<'\n';
  }catch(const std::exception& e){std::cerr<<"fork_risk_replay: "<<e.what()<<'\n';return 1;}
}
