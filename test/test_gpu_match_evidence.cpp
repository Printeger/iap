#include <gtest/gtest.h>
#include <cuda_runtime_api.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <iap/odometry/gpu_evidence/integrated_vgicp_factor_gpu.hpp>
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
  if(fail_evidence_host_allocation && bytes==iap::kGpuMatchEvidenceLimit*sizeof(iap::GpuMatchResidual)) {
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
    gtsam::NonlinearFactorGraph graph;graph.add(original);graph.add(observed);
    gtsam_points::NonlinearFactorSetGPU factor_set;factor_set.add(graph);
    observed->request_evidence();fail_evidence_host_allocation=true;factor_set.linearize(values);
    EXPECT_FALSE(fail_evidence_host_allocation);
    const auto allocation_failure=observed->take_evidence();EXPECT_FALSE(allocation_failure.available);
    EXPECT_TRUE(allocation_failure.samples.empty());EXPECT_NE(allocation_failure.failure_reason.find("cudaMallocHost"),std::string::npos);
    observed->request_evidence();factor_set.linearize(values);
    auto capture=observed->take_evidence();ASSERT_TRUE(capture.available);
    ASSERT_GT(capture.samples.size(),0u);EXPECT_LE(capture.samples.size(),iap::kGpuMatchEvidenceLimit);
    EXPECT_EQ(capture.original_inlier_count,original->num_inliers());
    if(source_count<=128) EXPECT_EQ(capture.samples.size(),static_cast<std::size_t>(capture.original_inlier_count));
    EXPECT_EQ(capture.sampling_stride,(source_count+127)/128);EXPECT_EQ(capture.original_source_count,source_count);
    const gtsam::HessianFactor old_h(*original->linearize(values)),new_h(*observed->linearize(values));
    EXPECT_TRUE(old_h.augmentedInformation().isApprox(new_h.augmentedInformation(),1e-6));
    const double original_error=original->error(values),observed_error=observed->error(values);
    EXPECT_NEAR(original_error,observed_error,1e-6*std::max(1.,std::abs(original_error)));
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
    EXPECT_EQ(observed->num_inliers(),original->num_inliers());
    EXPECT_NEAR(observed->error(values),original->error(values),1e-6*std::max(1.,std::abs(original_error)));
    observed->request_evidence();factor_set.linearize(values);
    EXPECT_TRUE(observed->take_evidence().available); // Recover with new bytes.
    observed->request_evidence();fail_evidence_sync=true;factor_set.linearize(values);
    EXPECT_FALSE(fail_evidence_sync);
    const auto sync_failure=observed->take_evidence();EXPECT_FALSE(sync_failure.available);
    EXPECT_TRUE(sync_failure.samples.empty());EXPECT_NE(sync_failure.failure_reason.find("cudaStreamSynchronize"),std::string::npos);
  }
  }
}
