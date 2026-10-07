// IAP adaptation of gtsam_points src/gtsam_points/factors/integrated_vgicp_derivatives.cu
// Upstream 85d0f4c43098b1f071bbb07710692e3829347c6c. Matching mathematics
// remain unchanged; IAP adds bounded evidence from the actual CUDA reduction.
// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#include <iap/odometry/gpu_evidence/integrated_vgicp_derivatives.cuh>

#include <iostream>
#include <algorithm>

#include <gtsam_points/cuda/check_error.cuh>
#include <gtsam_points/cuda/kernels/linearized_system.cuh>
#include <gtsam_points/cuda/kernels/vgicp_derivatives.cuh>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/cuda/cuda_malloc_async.hpp>

#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>

namespace gtsam_points {

IapObservedVGICPDerivatives::IapObservedVGICPDerivatives(
  const GaussianVoxelMapGPU::ConstPtr& target,
  const PointCloud::ConstPtr& source,
  CUstream_st* ext_stream,
  std::shared_ptr<TempBufferManager> temp_buffer)
: enable_offloading(false),
  enable_surface_validation(false),
  inlier_update_thresh_trans(1e-6),
  inlier_update_thresh_angle(1e-6),
  target(target),
  source(source),
  external_stream(true),
  stream(ext_stream),
  temp_buffer(temp_buffer),
  num_inliers(0),
  source_inliers(nullptr) {
  //
  if (stream == nullptr) {
    external_stream = false;
    cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
  }

  if (this->temp_buffer == nullptr) {
    this->temp_buffer.reset(new TempBufferManager());
  }

  check_error << cudaMallocAsync(&num_inliers_gpu, sizeof(int), stream);
  // check_error << cudaHostRegister(&num_inliers, sizeof(int), cudaHostRegisterDefault);
}

IapObservedVGICPDerivatives::~IapObservedVGICPDerivatives() {
  // Pending downloads own pinned memory until this stream has completed.
  if(evidence_cpu) { cudaStreamSynchronize(stream); cudaFreeHost(evidence_cpu); }
  if(evidence_gpu) check_error << cudaFreeAsync(evidence_gpu,stream);
  check_error << cudaFreeAsync(source_inliers, stream);
  check_error << cudaFreeAsync(num_inliers_gpu, stream);
  // check_error << cudaHostUnregister(&num_inliers);

  if (!external_stream) {
    cudaStreamDestroy(stream);
  }
}

void IapObservedVGICPDerivatives::sync_stream() {
  const auto result = cudaStreamSynchronize(stream);
  check_error << result;
  evidence_sync_ok = result == cudaSuccess;
  if(evidence_pending && !evidence_sync_ok) evidence_operation(result,"cudaStreamSynchronize");
}

void IapObservedVGICPDerivatives::touch_points() {
  if (!enable_offloading) {
    return;
  }

  auto target_ = const_cast<GaussianVoxelMapGPU*>(target.get());
  target_->touch(stream);

  auto source_gpu_const = dynamic_cast<const PointCloudGPU*>(source.get());
  if (!source_gpu_const) {
    return;
  }

  auto source_gpu = const_cast<PointCloudGPU*>(source_gpu_const);
  source_gpu->touch(stream);
}

LinearizedSystem6 IapObservedVGICPDerivatives::linearize(const Eigen::Isometry3f& x) {
  thrust::device_vector<Eigen::Isometry3f> x_ptr(1);
  thrust::device_vector<LinearizedSystem6> output_ptr(1);

  x_ptr[0] = x;

  reset_inliers(x, thrust::raw_pointer_cast(x_ptr.data()));
  issue_linearize(thrust::raw_pointer_cast(x_ptr.data()), thrust::raw_pointer_cast(output_ptr.data()));
  sync_stream();

  LinearizedSystem6 linearized = output_ptr[0];
  harvest_evidence(linearized.num_inliers);

  return linearized;
}

double IapObservedVGICPDerivatives::compute_error(const Eigen::Isometry3f& d_xl, const Eigen::Isometry3f& d_xe) {
  thrust::device_vector<Eigen::Isometry3f> xs_ptr(2);
  xs_ptr[0] = d_xl;
  xs_ptr[1] = d_xe;
  thrust::device_vector<float> output_ptr(1);

  issue_compute_error(
    thrust::raw_pointer_cast(xs_ptr.data()),
    thrust::raw_pointer_cast(xs_ptr.data() + 1),
    thrust::raw_pointer_cast(output_ptr.data()));
  sync_stream();

  float error = output_ptr[0];
  return error;
}

void IapObservedVGICPDerivatives::request_evidence() {
  last_evidence=iap::GpuMatchCapture{}; // New requests cannot read a prior pass.
  evidence_requested=true;
}
void IapObservedVGICPDerivatives::begin_evidence(const Eigen::Isometry3f* linearization_point) {
  last_evidence=iap::GpuMatchCapture{};
  evidence_pending=false;
  evidence_sync_ok=false;
  if(!evidence_requested) return;
  evidence_requested=false;
  if(!evidence_gpu) {
    if(!evidence_operation(cudaMallocAsync(&evidence_gpu,sizeof(iap::GpuMatchResidual)*iap::kGpuMatchEvidenceLimit,stream),"cudaMallocAsync")) return;
  }
  if(!evidence_cpu && !evidence_operation(cudaMallocHost(&evidence_cpu,sizeof(iap::GpuMatchResidual)*iap::kGpuMatchEvidenceLimit),"cudaMallocHost")) return;
  evidence_stride=std::max(1,static_cast<int>((source->size()+iap::kGpuMatchEvidenceLimit-1)/iap::kGpuMatchEvidenceLimit));
  evidence_slots=static_cast<int>((source->size()+evidence_stride-1)/evidence_stride);
  if(!evidence_operation(cudaMemsetAsync(evidence_gpu,0,sizeof(iap::GpuMatchResidual)*evidence_slots,stream),"cudaMemsetAsync")) return;
  if(!evidence_operation(cudaMemcpyAsync(evidence_transform.data(),linearization_point,sizeof(Eigen::Matrix4f),cudaMemcpyDeviceToHost,stream),"transform cudaMemcpyAsync")) return;
  evidence_pending=true;
}
bool IapObservedVGICPDerivatives::evidence_operation(int result, const char* operation) {
  const auto error=static_cast<cudaError_t>(result);
  if(error==cudaSuccess) return true;
  check_error << error;
  evidence_pending=false;
  last_evidence=iap::GpuMatchCapture{};
  last_evidence.failure_reason=std::string(operation)+": "+cudaGetErrorString(error);
  return false;
}
void IapObservedVGICPDerivatives::finish_evidence() {
  if(evidence_pending) evidence_operation(cudaMemcpyAsync(evidence_cpu,evidence_gpu,
      sizeof(iap::GpuMatchResidual)*evidence_slots,cudaMemcpyDeviceToHost,stream),"residual cudaMemcpyAsync");
}
void IapObservedVGICPDerivatives::harvest_evidence(int inliers) {
  if(!evidence_pending || !evidence_sync_ok) return;
  // Called after NonlinearFactorSetGPU has synchronized its result stream.
  last_evidence=iap::GpuMatchCapture{};last_evidence.available=true;
  last_evidence.sequence=++evidence_sequence;
  last_evidence.original_source_count=source->size();last_evidence.original_inlier_count=inliers;
  last_evidence.sampling_stride=evidence_stride;last_evidence.linearization_transform=evidence_transform;
  for(int i=0;i<evidence_slots;++i) if(evidence_cpu[i].valid) last_evidence.samples.push_back(evidence_cpu[i]);
  evidence_pending=false;
}
iap::GpuMatchCapture IapObservedVGICPDerivatives::take_evidence() {
  auto result=std::move(last_evidence);last_evidence=iap::GpuMatchCapture{};return result;
}

}  // namespace gtsam_points
