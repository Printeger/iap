# Config file for the iap package
#
# Usage from an external project:
#
#  find_package(iap REQUIRED)
#  target_link_libraries(MY_TARGET_NAME iap::iap)
#


include_guard()

set(BUILD_WITH_CUDA ON)
set(BUILD_WITH_VIEWER ON)
set(GLIM_USE_OPENCV 1)

include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED serialization)
find_dependency(Eigen3 REQUIRED)
find_dependency(gtsam_points REQUIRED)
find_dependency(GTSAM REQUIRED)
find_dependency(OpenMP REQUIRED)
find_dependency(spdlog REQUIRED)

if(BUILD_WITH_CUDA)
  find_dependency(CUDAToolkit REQUIRED)
endif()

if(BUILD_WITH_VIEWER)
  find_dependency(Iridescence REQUIRED)
endif()

if(GLIM_USE_OPENCV)
  find_dependency(OpenCV REQUIRED COMPONENTS core)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/iap-targets.cmake")
