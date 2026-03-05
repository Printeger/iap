# Config file for the iap package
#
# Usage from an external project:
#
#  find_package(iap REQUIRED)
#  target_link_libraries(MY_TARGET_NAME iap::iap)
#

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was iap-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

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
