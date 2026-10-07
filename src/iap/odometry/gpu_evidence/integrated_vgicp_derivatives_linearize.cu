// IAP adaptation of gtsam_points src/gtsam_points/factors/integrated_vgicp_derivatives_linearize.cu
// Upstream 85d0f4c43098b1f071bbb07710692e3829347c6c. Matching mathematics
// remain unchanged; IAP adds bounded evidence from the actual CUDA reduction.
// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#include <iap/odometry/gpu_evidence/integrated_vgicp_derivatives.cuh>

#include <iostream>
#include <thrust/remove.h>
#include <thrust/iterator/transform_iterator.h>

#include <cub/device/device_reduce.cuh>

#include <gtsam_points/cuda/kernels/pose.cuh>
#include <gtsam_points/cuda/kernels/untie.cuh>
#include <gtsam_points/cuda/kernels/lookup_voxels.cuh>
#include <gtsam_points/cuda/kernels/linearized_system.cuh>
#include <gtsam_points/cuda/kernels/vgicp_derivatives.cuh>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>

#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>

namespace gtsam_points {

namespace {
struct observed_derivatives_kernel {
  vgicp_derivatives_kernel original;
  iap::GpuMatchResidual* output;
  int stride, slots;
  __device__ LinearizedSystem6 operator()(const thrust::pair<int,int>& pair) const {
    const auto result=original(pair);
    if(!output || pair.first<0 || pair.second<0 || pair.first%stride || pair.first/stride>=slots) return result;
    auto& record=output[pair.first/stride];
    const auto& x=*original.linearization_point_ptr;
    const auto& a=original.source_means_ptr[pair.first];
    const auto& b=original.voxel_means_ptr[pair.second];
    const auto& ca=original.source_covs_ptr[pair.first];
    const auto& cb=original.voxel_covs_ptr[pair.second];
    const Eigen::Vector3f residual=b-(x.linear()*a+x.translation());
    const Eigen::Matrix3f covariance=cb+x.linear()*ca*x.linear().transpose();
    record.valid=1;record.source_index=pair.first;record.target_index=pair.second;
    record.target_point_count=original.voxel_num_points_ptr[pair.second];
    record.mahalanobis_squared=result.error;
    for(int row=0;row<3;++row) {
      record.residual_m[row]=residual[row];record.source_mean_m[row]=a[row];record.target_mean_m[row]=b[row];
      for(int col=0;col<3;++col) {
        record.covariance_m2_row_major[3*row+col]=covariance(row,col);
        record.source_covariance_m2_row_major[3*row+col]=ca(row,col);
        record.target_covariance_m2_row_major[3*row+col]=cb(row,col);
      }
    }
    return result;
  }
};
}

void IapObservedVGICPDerivatives::issue_linearize(const Eigen::Isometry3f* d_x, LinearizedSystem6* d_output) {
  begin_evidence(d_x);
  lookup_voxels_kernel corr_kernel(enable_surface_validation, *target, source->points_gpu, source->normals_gpu, d_x);
  auto corr_first = thrust::make_transform_iterator(source_inliers, corr_kernel);

  vgicp_derivatives_kernel deriv_kernel(d_x, *target, source->points_gpu, source->covs_gpu);
  observed_derivatives_kernel observed{deriv_kernel,evidence_pending ? evidence_gpu : nullptr,evidence_stride,evidence_slots};
  auto first = thrust::make_transform_iterator(corr_first, observed);

  void* temp_storage = nullptr;
  size_t temp_storage_bytes = 0;

  const auto query_status=cub::DeviceReduce::Reduce(
    temp_storage,
    temp_storage_bytes,
    first,
    d_output,
    num_inliers,
    thrust::plus<LinearizedSystem6>(),
    LinearizedSystem6::zero(),
    stream);

  temp_storage = temp_buffer->get_buffer(temp_storage_bytes);

  const auto reduction_status=cub::DeviceReduce::Reduce(
    temp_storage,
    temp_storage_bytes,
    first,
    d_output,
    num_inliers,
    thrust::plus<LinearizedSystem6>(),
    LinearizedSystem6::zero(),
    stream);
  if(evidence_pending) {
    evidence_operation(query_status,"CUB storage query");
    evidence_operation(reduction_status,"CUB derivative reduction");
  }
  finish_evidence(d_output);
}

}  // namespace gtsam_points
