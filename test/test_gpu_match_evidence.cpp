#include <gtest/gtest.h>
#include <cuda_runtime_api.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <iap/odometry/gpu_evidence/integrated_vgicp_factor_gpu.hpp>
#include <iap/local_map/source_sampling_evidence.hpp>
#include <iap/odometry/native_response_observer.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <numeric>
#include <Eigen/Cholesky>
#include <set>
#include <dlfcn.h>

namespace {
bool fail_evidence_transform_download=false;
bool fail_evidence_host_allocation=false, fail_evidence_sync=false;
cudaStream_t evidence_stream=nullptr;
}
// Test-only CUDA interposition exercises the real evidence failure path. The
// matcher and reduction downloads retain the ordinary runtime implementation.
extern "C" cudaError_t cudaMemcpyAsync(void* dst, const void* src, size_t bytes,
                                      cudaMemcpyKind kind, cudaStream_t stream) {
  using Copy=cudaError_t (*)(void*,const void*,size_t,cudaMemcpyKind,cudaStream_t);
  static auto copy=reinterpret_cast<Copy>(dlsym(RTLD_NEXT,"cudaMemcpyAsync"));
  if(kind==cudaMemcpyDeviceToHost && bytes==sizeof(Eigen::Matrix4f)) evidence_stream=stream;
  if(fail_evidence_transform_download && kind==cudaMemcpyDeviceToHost && bytes==sizeof(Eigen::Matrix4f)) {
    fail_evidence_transform_download=false;
    return cudaErrorInvalidValue;
  }
  return copy ? copy(dst,src,bytes,kind,stream) : cudaErrorUnknown;
}
extern "C" cudaError_t cudaHostAlloc(void** ptr, size_t bytes, unsigned int flags) {
  using Allocate=cudaError_t (*)(void**,size_t,unsigned int);
  static auto allocate=reinterpret_cast<Allocate>(dlsym(RTLD_NEXT,"cudaHostAlloc"));
  if(fail_evidence_host_allocation && bytes==sizeof(iap::GpuMatchTransfer)) {
    fail_evidence_host_allocation=false;return cudaErrorMemoryAllocation;
  }
  return allocate ? allocate(ptr,bytes,flags) : cudaErrorUnknown;
}
extern "C" cudaError_t cudaStreamSynchronize(cudaStream_t stream) {
  using Sync=cudaError_t (*)(cudaStream_t);
  static auto sync=reinterpret_cast<Sync>(dlsym(RTLD_NEXT,"cudaStreamSynchronize"));
  const auto result=sync ? sync(stream) : cudaErrorUnknown;
  if(fail_evidence_sync && stream==evidence_stream) {
    fail_evidence_sync=false;return cudaErrorUnknown;
  }
  return result;
}

TEST(GpuMatchEvidence, ActualKernelSamplesPreserveOriginalCostAndHessian) {
  int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
  for(const int source_count:{100,1000}) {
  SCOPED_TRACE(source_count);
  std::vector<Eigen::Vector4d> points,normals;
  std::vector<Eigen::Matrix4d> covariances;
  for(int i=0;i<source_count;++i) {
    points.emplace_back(.12+.3*(i%10),.13+.3*(i/10),.04*(i%3),1.);
    normals.emplace_back(0.,0.,1.,0.);
    Eigen::Matrix4d c=Eigen::Matrix4d::Zero();c.diagonal()<<.02,.03,.01,0.;covariances.push_back(c);
  }
  gtsam_points::PointCloudCPU cpu(points);cpu.add_normals(normals);cpu.add_covs(covariances);
  const auto native_means=iap::local_map::copyNativeMatchingMeans(cpu.points,cpu.size());
  const auto verify_sources=[&](const iap::GpuMatchCapture& capture) {
    ASSERT_TRUE(capture.source_geometry_available);
    ASSERT_EQ(capture.source_samples.size(),static_cast<std::size_t>(capture.source_slots));
    for(std::size_t slot=0;slot<capture.source_samples.size();++slot) {
      const auto& record=capture.source_samples[slot];
      const int index=slot*capture.sampling_stride;
      ASSERT_EQ(record.source_index,index);ASSERT_LT(index,source_count);
      for(int row=0;row<3;++row) {
        EXPECT_FLOAT_EQ(record.mean_m[row],native_means[index][row]);
        for(int col=0;col<3;++col)
          EXPECT_FLOAT_EQ(record.covariance_m2_row_major[3*row+col],static_cast<float>(covariances[index](row,col)));
      }
    }
  };
  auto source=gtsam_points::PointCloudGPU::clone(cpu);
  auto target=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  target->insert(*source);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  const gtsam::Pose3 fixed(gtsam::Rot3::Rz(.13),gtsam::Point3(.4,-.2,.1));
  gtsam::Values values;values.insert(1,fixed);
  values.insert(2,fixed*gtsam::Pose3(gtsam::Rot3::Rx(.02),gtsam::Point3(.035,-.015,.01)));
  for(const bool unary:{false,true}) for(const bool validate:{false,true}) {
    SCOPED_TRACE(std::string(unary ? "unary" : "binary")+(validate ? " validated" : " unvalidated"));
    gtsam_points::IntegratedVGICPFactorGPU::shared_ptr original;
    gtsam_points::IapObservedVGICPFactorGPU::shared_ptr observed;
    if(unary) {
      original=gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(fixed,2,target,source);
      observed=gtsam::make_shared<gtsam_points::IapObservedVGICPFactorGPU>(fixed,2,target,source);
    } else {
      original=gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(1,2,target,source);
      observed=gtsam::make_shared<gtsam_points::IapObservedVGICPFactorGPU>(1,2,target,source);
    }
    original->set_enable_surface_validation(validate);observed->set_enable_surface_validation(validate);
    observed->retain_native_response();
    gtsam::NonlinearFactorGraph graph;graph.add(original);graph.add(observed);
    gtsam_points::NonlinearFactorSetGPU factor_set;factor_set.add(graph);
    observed->request_evidence();fail_evidence_host_allocation=true;factor_set.linearize(values);
    EXPECT_FALSE(fail_evidence_host_allocation);
    const auto allocation_failure=observed->take_evidence();EXPECT_FALSE(allocation_failure.available);
    EXPECT_TRUE(allocation_failure.samples.empty());EXPECT_NE(allocation_failure.failure_reason.find("cudaMallocHost"),std::string::npos);
    observed->request_evidence();factor_set.linearize(values);
    auto capture=observed->take_evidence();ASSERT_TRUE(capture.available);
    verify_sources(capture);
    ASSERT_GT(capture.samples.size(),0u);EXPECT_LE(capture.samples.size(),iap::kGpuMatchEvidenceLimit);
    EXPECT_EQ(capture.original_inlier_count,original->num_inliers());
    if(source_count<=128) EXPECT_EQ(capture.samples.size(),static_cast<std::size_t>(capture.original_inlier_count));
    EXPECT_EQ(capture.sampling_stride,(source_count+127)/128);EXPECT_EQ(capture.original_source_count,source_count);
    const auto cached=observed->linearize(values);
    const auto* response=dynamic_cast<const gtsam_points::IapCachedVGICPResponse*>(cached.get());
    const gtsam::HessianFactor old_h(*original->linearize(values)),new_h(*cached);
    EXPECT_TRUE(old_h.augmentedInformation().isApprox(new_h.augmentedInformation(),1e-6));
    ASSERT_NE(response,nullptr);
    const auto& retained=response->native_response().augmented_information;
    if(unary) {
      Eigen::Matrix<double,7,7> unary_block;
      unary_block.topLeftCorner<6,6>()=retained.block<6,6>(6,6);
      unary_block.topRightCorner<6,1>()=retained.block<6,1>(6,12);
      unary_block.bottomLeftCorner<1,6>()=retained.block<1,6>(12,6);
      unary_block(6,6)=retained(12,12);
      EXPECT_TRUE(new_h.augmentedInformation().isApprox(unary_block,1e-12));
      // A real target block exists even though the solver's unary graph drops it.
      EXPECT_GT((retained.block<6,6>(0,0).norm()),0.);
      const Eigen::Matrix<double,6,6> cross=retained.block<6,6>(6,0);
      // Explicit one-component physical displacement, not posterior noise:
      // move the original fixed target by +/-1 mm in its local x chart. The
      // native GPU residual/normal RHS must match the retained target response.
      constexpr double step=.001;
      gtsam::Vector6 direction=gtsam::Vector6::Zero();direction(3)=step;
      gtsam_points::IapObservedVGICPFactorGPU plus(fixed.retract(direction),2,target,source);
      gtsam_points::IapObservedVGICPFactorGPU minus(fixed.retract(-direction),2,target,source);
      plus.set_enable_surface_validation(validate);minus.set_enable_surface_validation(validate);
      const auto hp=plus.linearize(values)->augmentedInformation();
      const auto hm=minus.linearize(values)->augmentedInformation();
      const auto derivative=(hp.block<6,1>(0,6)-hm.block<6,1>(0,6))/(2.*step);
      const auto expected=-cross.col(3);
      EXPECT_LT((derivative-expected).norm()/std::max(1.,expected.norm()),.003);
    } else EXPECT_TRUE(new_h.augmentedInformation().isApprox(retained,1e-12));
    // Native cache A survives a later quality linearization B. The target
    // response and matrix must still belong to A, including after clone.
    auto quality_values=values;
    quality_values.update(2,values.at<gtsam::Pose3>(2)*gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(.02,.01,0.)));
    const auto old_response=retained;
    observed->linearize(quality_values);
    EXPECT_TRUE(response->native_response().augmented_information.isApprox(old_response,1e-12));
    auto clone=cached->clone();
    const auto* copied=dynamic_cast<const gtsam_points::IapCachedVGICPResponse*>(clone.get());
    ASSERT_NE(copied,nullptr);
    EXPECT_TRUE(copied->native_response().augmented_information.isApprox(old_response,1e-12));
    factor_set.linearize(values);
    const double original_error=original->error(values),observed_error=observed->error(values);
    EXPECT_NEAR(original_error,observed_error,1e-6*std::max(1.,std::abs(original_error)));
    EXPECT_NEAR(capture.original_cost,original_error,1e-6*std::max(1.,std::abs(original_error)));
    double sample_cost=0.;std::set<int> identifiers;bool multi_point_voxel=false;
    for(const auto& record:capture.samples) {
      ASSERT_TRUE(record.valid);ASSERT_GE(record.source_index,0);ASSERT_GE(record.target_index,0);
      EXPECT_TRUE(identifiers.insert(record.source_index).second);
      EXPECT_EQ(record.source_index%capture.sampling_stride,0);
      Eigen::Matrix3d covariance;Eigen::Vector3d residual;
      for(int row=0;row<3;++row) {
        residual[row]=record.residual_m[row];
        for(int col=0;col<3;++col) covariance(row,col)=record.covariance_m2_row_major[3*row+col];
      }
      const auto predicted=residual.dot(covariance.ldlt().solve(residual));
      // Point count belongs to evidence, not the original matching weight.
      EXPECT_NEAR(predicted,record.mahalanobis_squared,1e-5*std::max(1.,predicted));
      sample_cost+=record.mahalanobis_squared;multi_point_voxel|=record.target_point_count>1;
    }
    EXPECT_TRUE(multi_point_voxel);
    if(capture.sampling_stride==1) EXPECT_NEAR(sample_cost,original_error,1e-5*std::max(1.,std::abs(original_error)));
    observed->request_evidence(iap::kGpuMatchDenseEvidenceLimit);factor_set.linearize(values);
    const auto dense=observed->take_evidence();ASSERT_TRUE(dense.available);
    verify_sources(dense);
    EXPECT_EQ(dense.sample_limit,iap::kGpuMatchDenseEvidenceLimit);
    EXPECT_EQ(dense.source_slots,source_count);EXPECT_EQ(dense.sampling_stride,1);
    EXPECT_EQ(dense.samples.size(),static_cast<std::size_t>(original->num_inliers()));
    double dense_cost=0.;std::set<int> dense_sources;
    for(const auto& record:dense.samples) {
      EXPECT_TRUE(dense_sources.insert(record.source_index).second);
      dense_cost+=record.mahalanobis_squared;
    }
    EXPECT_NEAR(dense_cost,original_error,1e-5*std::max(1.,std::abs(original_error)));
    const gtsam::HessianFactor dense_h(*observed->linearize(values));
    EXPECT_TRUE(old_h.augmentedInformation().isApprox(dense_h.augmentedInformation(),1e-6));
    EXPECT_FALSE(observed->take_evidence().available);
    // Evidence is opt-in and consumed once; a new optimization cannot reuse it.
    factor_set.linearize(values);EXPECT_FALSE(observed->take_evidence().available);
    observed->request_evidence();factor_set.linearize(values);
    observed->request_evidence();EXPECT_FALSE(observed->take_evidence().available);
    factor_set.linearize(values);EXPECT_TRUE(observed->take_evidence().available);
    // An uncollected A must not survive a subsequent unrequested B.
    observed->request_evidence();factor_set.linearize(values);
    factor_set.linearize(values);EXPECT_FALSE(observed->take_evidence().available);
    auto missing=values;missing.update(2,fixed*gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(100.,100.,100.)));
    observed->request_evidence();factor_set.linearize(missing);
    const auto empty=observed->take_evidence();EXPECT_TRUE(empty.available);EXPECT_TRUE(empty.samples.empty());
    verify_sources(empty); // Source geometry survives zero original matches.
    EXPECT_EQ(empty.original_inlier_count,0);EXPECT_EQ(original->num_inliers(),0);
    EXPECT_DOUBLE_EQ(original->error(missing),0.);EXPECT_DOUBLE_EQ(observed->error(missing),0.);
    original->linearize(missing);observed->linearize(missing); // Consume cached results.
    original->linearize(values);
    observed->request_evidence();observed->linearize(values);
    EXPECT_TRUE(observed->take_evidence().available); // Actual synchronous path.
    observed->request_evidence();fail_evidence_transform_download=true;
    factor_set.linearize(values);
    EXPECT_FALSE(fail_evidence_transform_download);
    const auto failed=observed->take_evidence();EXPECT_FALSE(failed.available);
    EXPECT_TRUE(failed.samples.empty());EXPECT_NE(failed.failure_reason.find("transform cudaMemcpyAsync"),std::string::npos);
    EXPECT_FALSE(failed.source_geometry_available);EXPECT_TRUE(failed.source_samples.empty());
    EXPECT_EQ(observed->num_inliers(),original->num_inliers());
    EXPECT_NEAR(observed->error(values),original->error(values),1e-6*std::max(1.,std::abs(original_error)));
    observed->request_evidence();factor_set.linearize(values);
    EXPECT_TRUE(observed->take_evidence().available); // Recover with new bytes.
    observed->request_evidence();fail_evidence_sync=true;factor_set.linearize(values);
    EXPECT_FALSE(fail_evidence_sync);
    const auto sync_failure=observed->take_evidence();EXPECT_FALSE(sync_failure.available);
    EXPECT_TRUE(sync_failure.samples.empty());EXPECT_NE(sync_failure.failure_reason.find("cudaStreamSynchronize"),std::string::npos);
    observed->request_evidence();observed->cancel_evidence();factor_set.linearize(values);
    EXPECT_FALSE(observed->take_evidence().available);
  }
  }
}

TEST(GpuMatchEvidence, FirstNonzeroKeyframeFreezeSelectionIsBoundedAndReadOnly) {
  int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
  std::string pattern=(std::filesystem::temp_directory_path()/"iap_freeze_test_XXXXXX").string();
  const auto allocated=::mkdtemp(pattern.data());ASSERT_NE(allocated,nullptr);
  struct Cleanup {std::filesystem::path path;~Cleanup(){std::filesystem::remove_all(path);}} cleanup{allocated};
  const auto run=cleanup.path/"run";std::filesystem::create_directory(run);setenv("IAP_RUN_DIR",run.c_str(),1);
  const auto& logs=glim::RunLogManager::initialize("source_freeze_test");
  std::vector<long> many(27);std::iota(many.begin(),many.end(),0);
  EXPECT_THROW(iap::NativeResponseObserver(logs,many,true),std::invalid_argument);
  std::vector<Eigen::Vector4d> points{{0.,0.,0.,1.},{.1,.1,.1,1.}};
  gtsam_points::PointCloudCPU cpu(points);
  cpu.add_covs(std::vector<Eigen::Matrix4d>(points.size(),Eigen::Matrix4d::Identity()));
  auto cloud=gtsam_points::PointCloudGPU::clone(cpu);
  auto map=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  // Selection exercises actual VGICP owner types, without solving these dummy
  // factors. Solver equality uses the same original CPU graph in OFF/ON.
  gtsam::ISAM2Params params;params.findUnusedFactorSlots=true;
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback off(5.,params),on(5.,params);
  const auto noise=gtsam::noiseModel::Isotropic::Sigma(6,.1);
  {
    auto writer=std::make_shared<iap::NativeResponseObserver>(logs,std::vector<long>{10},true);
    for(int i=0;i<85;++i) {
      const auto key=gtsam::Symbol('x',i);gtsam::Values values;values.insert(key,gtsam::Pose3());
      gtsam::NonlinearFactorGraph factors;
      if(i==0) factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key,gtsam::Pose3(),noise);
      else factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x',i-1),key,gtsam::Pose3(),noise);
      gtsam::NonlinearFactorGraph owners;
      // Consecutive non-keyframe targets must never arm this selection.
      auto recent=gtsam::make_shared<gtsam_points::IapObservedVGICPFactorGPU>(gtsam::Symbol('x',std::max(0,i-1)),key,map,cloud);
      recent->set_evidence_owner({i,std::max(0,i-1),.1*i,.1*std::max(0,i-1),.5,0,false,false});owners.add(recent);
      if(i>=25) {
        gtsam_points::IapObservedVGICPFactorGPU::shared_ptr source;
        if(i<70) source=gtsam::make_shared<gtsam_points::IapObservedVGICPFactorGPU>(gtsam::Symbol('x',20),key,map,cloud);
        else source=gtsam::make_shared<gtsam_points::IapObservedVGICPFactorGPU>(gtsam::Pose3(),key,map,cloud);
        source->set_evidence_owner({i,20,.1*i,2.,.5,0,i>=70,true});owners.add(source);
      }
      writer->prepare(on,owners,i,.1*i);
      off.update(factors,values,{{key,.1*i}});on.update(factors,values,{{key,.1*i}});
      // Production performs an additional native update in the same frame.
      // Previous snapshot must be the last update; selected boundaries occur
      // once, retaining native call identity and actual discarded keys.
      off.update();on.update();
      EXPECT_FALSE(on.fallbackHappened());EXPECT_TRUE(off.calculateEstimate<gtsam::Pose3>(key).equals(on.calculateEstimate<gtsam::Pose3>(key),1e-12));
    }
  }
  nlohmann::json m;std::ifstream(logs.metadata_path("manifests/native_response.json"))>>m;
  const auto& selection=m["first_source_freeze"];
  EXPECT_EQ(selection["source_id"],20);EXPECT_EQ(selection["previous_frame"],70);EXPECT_EQ(selection["first_fixed_use_frame"],70);
  EXPECT_TRUE(selection["selection_done"].get<bool>());EXPECT_EQ(selection["failure_reason"],"");
  EXPECT_EQ(selection["rolling_copy_failures"],0);EXPECT_LT(selection["rolling_copy_count"].get<int>(),100);
  EXPECT_EQ(m["observer_budget_overrun_count"],0);EXPECT_EQ(m["write_failures"],0);EXPECT_LE(m["written"].get<int>(),7);
  // This accelerated fixture runs 85 updates in milliseconds. Queue rejection
  // is a retained interface failure, never a model PASS or a dropped request.
  int rejected=0;bool retirement_seen=false,probe_seen=false;
  std::set<std::pair<long,std::size_t>> selected_updates;
  for(const auto& row:m["requests"]) {
    EXPECT_TRUE(selected_updates.emplace(row["frame_id"].get<long>(),row["native_update_in_frame"].get<std::size_t>()).second);
    if(row["frame_id"]==70 && row["selection_roles"]==nlohmann::json::array({"update_before_native_retirement"})) {
      EXPECT_EQ(row["native_update_in_frame"],1);
    }
    EXPECT_TRUE(row["complete"].get<bool>());if(!row["queue_accepted"].get<bool>()) ++rejected;
    for(const auto& role:row["selection_roles"]) {
      retirement_seen|=role=="nonzero_source_native_retirement";probe_seen|=role=="fixed_use_plus_10_updates";
    }
  }
  EXPECT_TRUE(retirement_seen);EXPECT_TRUE(probe_seen);
  EXPECT_EQ(m["written"].get<int>()+rejected,m["requests"].size());
}
