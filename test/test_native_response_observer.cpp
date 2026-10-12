#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <iap/odometry/native_response_observer.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <zlib.h>

namespace {
std::vector<gtsam::Pose3> exercise(int mode, bool* pre_elimination=nullptr,
                                  iap::NativeResponseObserver* writer=nullptr) {
  gtsam::ISAM2Params parameters;parameters.findUnusedFactorSlots=true;
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback smoother(5.,parameters);
  const auto noise=gtsam::noiseModel::Isotropic::Sigma(6,.1);
  std::vector<gtsam::Pose3> result;
  for(int i=0;i<85;++i) {
    const gtsam::Key key=gtsam::Symbol('x',i);
    const gtsam::Pose3 pose(gtsam::Rot3(),gtsam::Point3(i*.03,0.,0.));
    gtsam::NonlinearFactorGraph factors;gtsam::Values values;values.insert(key,pose);
    if(i==0) factors.add(gtsam::PriorFactor<gtsam::Pose3>(key,pose,noise));
    else factors.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x',i-1),key,
        gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(.03,0.,0.)),noise));
    if(writer) writer->prepare(smoother,factors,i,.1*i);
    else if(mode) smoother.setPreMarginalizationObserver([&](const auto& native,const auto& discarded) {
      if(std::find(discarded.begin(),discarded.end(),gtsam::Symbol('x',0))!=discarded.end()) {
        ASSERT_TRUE(native.getLinearizationPoint().exists(gtsam::Symbol('x',0)));
        *pre_elimination=true;
      }
      const auto packet=iap::NativeResponseObserver::snapshot(native,discarded,i,.1*i);
      EXPECT_TRUE(packet["complete"].template get<bool>())<<packet.dump();
      EXPECT_EQ(packet["lag_s"],5.);
      EXPECT_FALSE(packet["physical_joint_error_law_available"].template get<bool>());
      if(mode==2) throw std::runtime_error("EXPECTED_OBSERVER_FAILURE");
    });
    smoother.update(factors,values,{{key,.1*i}});
    EXPECT_FALSE(smoother.fallbackHappened());
    if(mode==2) EXPECT_EQ(smoother.preMarginalizationObserverFailure(),"EXPECTED_OBSERVER_FAILURE");
    if(i>51) EXPECT_FALSE(smoother.getLinearizationPoint().exists(gtsam::Symbol('x',0)));
    result.push_back(smoother.calculateEstimate<gtsam::Pose3>(key));
  }
  return result;
}
}
TEST(NativeResponseObserver, ActualFiveSecondLagOnAndExceptionPreserveEveryEstimate) {
  bool on_seen=false,exception_seen=false;
  const auto off=exercise(0),on=exercise(1,&on_seen),failure=exercise(2,&exception_seen);
  EXPECT_TRUE(on_seen);EXPECT_TRUE(exception_seen);
  ASSERT_EQ(off.size(),on.size());ASSERT_EQ(off.size(),failure.size());
  for(std::size_t i=0;i<off.size();++i) {
    EXPECT_TRUE(off[i].equals(on[i],1e-12));EXPECT_TRUE(off[i].equals(failure[i],1e-12));
  }
}
TEST(NativeResponseObserver, OwnedWriterHasFiniteSelectionAndRetainsMissingRequests) {
  std::string pattern=(std::filesystem::temp_directory_path()/"iap_response_test_XXXXXX").string();
  auto* allocated=::mkdtemp(pattern.data());ASSERT_NE(allocated,nullptr);
  struct Cleanup {std::filesystem::path path;~Cleanup(){std::filesystem::remove_all(path);}} cleanup{allocated};
  auto run=cleanup.path/"run";std::filesystem::create_directory(run);setenv("IAP_RUN_DIR",run.c_str(),1);
  auto& logs=glim::RunLogManager::initialize("native_response_test");
  EXPECT_THROW(iap::NativeResponseObserver(logs,{}),std::invalid_argument);
  EXPECT_THROW(iap::NativeResponseObserver(logs,{1,1}),std::invalid_argument);
  EXPECT_THROW(iap::NativeResponseObserver(logs,{-1}),std::invalid_argument);
  std::vector<long> many(32);std::iota(many.begin(),many.end(),0);
  EXPECT_THROW(iap::NativeResponseObserver(logs,many),std::invalid_argument);
  {
    auto observer=std::make_shared<iap::NativeResponseObserver>(logs,std::vector<long>{10,50,70,1000});
    auto off=exercise(0),on=exercise(0,nullptr,observer.get());
    for(std::size_t i=0;i<off.size();++i) EXPECT_TRUE(off[i].equals(on[i],1e-12));
  }
  nlohmann::json manifest;std::ifstream(logs.metadata_path("manifests/native_response.json"))>>manifest;
  EXPECT_EQ(manifest["missing_frames"],nlohmann::json::array({1000}));
  EXPECT_TRUE(manifest["initial_x0_elimination_seen"].get<bool>());
  EXPECT_EQ(manifest["requests"].size(),4u);EXPECT_EQ(manifest["write_failures"],0);
  EXPECT_LE(manifest["written"].get<int>(),4);
  EXPECT_FALSE(std::filesystem::exists(logs.export_path("glio/native_response.jsonl")));
  gzFile file=gzopen(logs.export_path("glio/native_response.jsonl.gz").c_str(),"rb");ASSERT_NE(file,nullptr);
  std::string data;char buffer[65536];int size;
  while((size=gzread(file,buffer,sizeof(buffer)))>0)data.append(buffer,size);
  EXPECT_EQ(gzclose(file),Z_OK);std::istringstream lines(data);std::string line;int rows=0;
  while(std::getline(lines,line)) {
    const auto packet=nlohmann::json::parse(line);++rows;
    EXPECT_TRUE(packet["complete"].get<bool>());
    for(const auto& factor:packet["factors"]) EXPECT_TRUE(factor["augmented_information_row_major"].is_array());
    EXPECT_LE(packet["copied_matrix_bytes"].get<int>(),iap::NativeResponseObserver::kMaxBytes);
  }
  EXPECT_EQ(rows,manifest["written"]);
}
