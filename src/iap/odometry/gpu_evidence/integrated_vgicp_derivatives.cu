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

namespace {
// Read the actual immutable CUDA source arrays before matching admission.
// This shares the existing bounded capture slots and stream; no extra sync,
// lookup, matcher input, or reduction contribution is introduced.
__global__ void observe_source_geometry(const Eigen::Vector3f* means,
                                       const Eigen::Matrix3f* covariances,
                                       iap::GpuMatchResidual* output,
                                       int stride, int slots) {
  const int slot=blockIdx.x*blockDim.x+threadIdx.x;
  if(slot>=slots) return;
  const int index=slot*stride;
  auto& record=output[slot];
  record.source_available=1;record.source_index=index;
  for(int row=0;row<3;++row) {
    record.source_mean_m[row]=means[index][row];
    for(int col=0;col<3;++col)
      record.source_covariance_m2_row_major[3*row+col]=covariances[index](row,col);
  }
}
}

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
  if(evidence_samples_cpu) cudaFreeHost(evidence_samples_cpu);
  for(auto* retired:retired_evidence_samples_cpu) cudaFreeHost(retired);
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
  harvest_evidence();

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

void IapObservedVGICPDerivatives::request_evidence(std::size_t sample_limit) {
  last_evidence=iap::GpuMatchCapture{}; // New requests cannot read a prior pass.
  // Only two bounded capture modes. This changes observation, never matching.
  evidence_limit=sample_limit>iap::kGpuMatchEvidenceLimit ? iap::kGpuMatchDenseEvidenceLimit : iap::kGpuMatchEvidenceLimit;
  evidence_requested=true;
}
void IapObservedVGICPDerivatives::cancel_evidence() {
  evidence_requested=false;evidence_pending=false;evidence_sync_ok=false;
  last_evidence=iap::GpuMatchCapture{};
}
void IapObservedVGICPDerivatives::begin_evidence(const Eigen::Isometry3f* linearization_point) {
  last_evidence=iap::GpuMatchCapture{};
  evidence_pending=false;
  evidence_sync_ok=false;
  if(!evidence_requested) return;
  evidence_requested=false;
  if(!evidence_cpu && !evidence_operation(cudaMallocHost(&evidence_cpu,sizeof(iap::GpuMatchTransfer)),"cudaMallocHost")) return;
  if(evidence_capacity<evidence_limit) {
    iap::GpuMatchResidual* new_cpu=nullptr;
    if(!evidence_operation(cudaMallocHost(&new_cpu,sizeof(iap::GpuMatchResidual)*evidence_limit),"sample cudaMallocHost")) return;
    iap::GpuMatchResidual* new_gpu=nullptr;
    if(!evidence_operation(cudaMallocAsync(&new_gpu,sizeof(iap::GpuMatchResidual)*evidence_limit,stream),"cudaMallocAsync")) {
      cudaFreeHost(new_cpu);return;
    }
    // Retain the old pinned buffer until the existing destructor sync, including
    // a failed prior sync. No additional per-factor synchronization is allowed.
    if(evidence_samples_cpu) retired_evidence_samples_cpu.push_back(evidence_samples_cpu);
    if(evidence_gpu) cudaFreeAsync(evidence_gpu,stream);
    evidence_samples_cpu=new_cpu;evidence_gpu=new_gpu;evidence_capacity=evidence_limit;
  }
  evidence_stride=std::max(1,static_cast<int>((source->size()+evidence_limit-1)/evidence_limit));
  evidence_slots=static_cast<int>((source->size()+evidence_stride-1)/evidence_stride);
  if(!evidence_operation(cudaMemsetAsync(evidence_gpu,0,sizeof(iap::GpuMatchResidual)*evidence_slots,stream),"cudaMemsetAsync")) return;
  if(evidence_slots>0) {
    observe_source_geometry<<<(evidence_slots+127)/128,128,0,stream>>>(
      source->points_gpu,source->covs_gpu,evidence_gpu,evidence_stride,evidence_slots);
    if(!evidence_operation(cudaGetLastError(),"source geometry kernel")) return;
  }
  if(!evidence_operation(cudaMemcpyAsync(evidence_cpu->transform_column_major,linearization_point,sizeof(Eigen::Matrix4f),cudaMemcpyDeviceToHost,stream),"transform cudaMemcpyAsync")) return;
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
void IapObservedVGICPDerivatives::finish_evidence(const LinearizedSystem6* output) {
  if(!evidence_pending) return;
  if(!evidence_operation(cudaMemcpyAsync(evidence_samples_cpu,evidence_gpu,
      sizeof(iap::GpuMatchResidual)*evidence_slots,cudaMemcpyDeviceToHost,stream),"residual cudaMemcpyAsync")) return;
  if(!evidence_operation(cudaMemcpyAsync(&evidence_cpu->inliers,&output->num_inliers,sizeof(int),cudaMemcpyDeviceToHost,stream),"inlier cudaMemcpyAsync")) return;
  evidence_operation(cudaMemcpyAsync(&evidence_cpu->cost,&output->error,sizeof(float),cudaMemcpyDeviceToHost,stream),"cost cudaMemcpyAsync");
}
void IapObservedVGICPDerivatives::harvest_evidence() {
  if(!evidence_pending || !evidence_sync_ok) return;
  // Called after NonlinearFactorSetGPU has synchronized its result stream.
  last_evidence=iap::GpuMatchCapture{};last_evidence.available=true;
  last_evidence.sequence=++evidence_sequence;
  last_evidence.original_source_count=source->size();last_evidence.original_inlier_count=evidence_cpu->inliers;
  last_evidence.original_cost=evidence_cpu->cost;
  last_evidence.sampling_stride=evidence_stride;
  last_evidence.sample_limit=evidence_limit;last_evidence.source_slots=evidence_slots;
  last_evidence.linearization_transform=Eigen::Map<const Eigen::Matrix4f>(evidence_cpu->transform_column_major);
  for(int i=0;i<evidence_slots;++i) {
    const auto& record=evidence_samples_cpu[i];
    if(!record.source_available || record.source_index!=i*evidence_stride) {
      last_evidence=iap::GpuMatchCapture{};
      last_evidence.failure_reason="source geometry capture incomplete";
      evidence_pending=false;return;
    }
    iap::GpuSourceGaussian source_record;source_record.source_index=record.source_index;
    std::copy_n(record.source_mean_m,3,source_record.mean_m);
    std::copy_n(record.source_covariance_m2_row_major,9,source_record.covariance_m2_row_major);
    last_evidence.source_samples.push_back(source_record);
    if(record.valid) last_evidence.samples.push_back(record);
  }
  last_evidence.source_geometry_available=true;
  evidence_pending=false;
}
iap::GpuMatchCapture IapObservedVGICPDerivatives::take_evidence() {
  auto result=std::move(last_evidence);last_evidence=iap::GpuMatchCapture{};return result;
}

}  // namespace gtsam_points
