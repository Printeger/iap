cmake_minimum_required(VERSION 3.16)
project(iap VERSION 1.2.1 LANGUAGES C CXX)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Version string available as IAP_VERSION in all targets
add_compile_definitions(IAP_VERSION="${PROJECT_VERSION}")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_CUDA_MULTIARCH "Build with GPU cross-platform support" OFF)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_WITH_OPENCV "Build with OpenCV to support camera images" ON)

if(BUILD_WITH_MARCH_NATIVE)
  add_definitions(-march=native)
  add_definitions(-DBUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

find_package(Boost REQUIRED serialization)
find_package(Eigen3 REQUIRED)
find_package(GTSAM 4.2 REQUIRED)
find_package(gtsam_points 1.2.0 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)

if(BUILD_WITH_VIEWER)
  find_package(Iridescence REQUIRED)
endif()

if(BUILD_WITH_OPENCV)
  set(GLIM_USE_OPENCV 1)
  find_package(OpenCV REQUIRED COMPONENTS core)
else()
  set(GLIM_USE_OPENCV 0)
endif()

###########
## Build ##
###########

# GPU-related
if(BUILD_WITH_CUDA)
  if(NOT GTSAM_POINTS_USE_CUDA)
    message(WARNING "gtsam_points is not built with CUDA!!")
    set(BUILD_WITH_CUDA OFF CACHE BOOL "gtsam_points is not built with CUDA" FORCE)
  else()
    set(GPU_SRCS src/iap/odometry/odometry_estimation_gpu.cpp)
  endif()
endif()

add_library(iap SHARED
  # util
  src/iap/util/config.cpp
  src/iap/util/logging.cpp
  src/iap/util/export_factors.cpp
  src/iap/util/time_keeper.cpp
  src/iap/util/trajectory_manager.cpp
  src/iap/util/extension_module.cpp
  src/iap/util/load_module.cpp
  src/iap/util/data_validator.cpp
  src/iap/util/relinearization_policy.cpp
  src/iap/util/key_lifecycle_monitor.cpp
  src/iap/util/serialization.cpp
  src/iap/util/debug.cpp
  # preprocess
  src/iap/preprocess/callbacks.cpp
  src/iap/preprocess/cloud_preprocessor.cpp
  # common process
  src/iap/common/imu_integration.cpp
  src/iap/common/imu_validation.cpp
  src/iap/common/cloud_deskewing.cpp
  src/iap/common/cloud_covariance_estimation.cpp
  # gnss (IAP-RQ-020)
  src/iap/gnss/pseudorange_factor.cpp
  src/iap/gnss/doppler_factor.cpp
  src/iap/gnss/clock_between_factor.cpp
  src/iap/gnss/gnss_handler.cpp
  # gnss visibility predictor (IAP-RQ-312/313)
  src/iap/gnss/visibility_predictor.cpp
  # local occupancy map (IAP-RQ-311)
  src/iap/map/local_occupancy.cpp
  # trunk detection (IAP-RQ-100/110/120)
  src/iap/trunk/trunk_detector.cpp
  # trunk map data association + factor (IAP-RQ-131/132)
  src/iap/trunk/trunk_map.cpp
  src/iap/trunk/trunk_factor.cpp
  # trunk_extension.cpp is compiled into a separate ROS2 plugin (libtrunk_extension.so)
  # ARAIM engine (IAP-RQ-241–246)
  src/iap/integrity/araim.cpp
  # FGO information matrix extraction (IAP-RQ-300)
  src/iap/integrity/fgo_information_manager.cpp
  # predicted ARAIM for planner (IAP-RQ-331)
  src/iap/planner/predicted_araim.cpp
  # integrity monitoring (IAP-RQ-200/210/220)
  src/iap/integrity/integrity_monitor.cpp
  # planner (IAP-RQ-300/310/320)
  src/iap/planner/trajectory_generator.cpp
  src/iap/planner/predicted_integrity.cpp
  # planner integrity-aware (IAP-RQ-400/410)
  src/iap/planner/integrity_planner.cpp
  # odometry
  src/iap/odometry/bspline_control_window.cpp
  src/iap/odometry/callbacks.cpp
  src/iap/odometry/bspline_trajectory.cpp
  src/iap/odometry/estimation_frame.cpp
  src/iap/odometry/initial_state_estimation.cpp
  src/iap/odometry/integrated_bspline_gicp_factor.cpp
  src/iap/odometry/integrated_bspline_gnss_factor.cpp
  src/iap/odometry/integrated_bspline_imu_factor.cpp
  src/iap/odometry/integrated_bspline_velocity_factor.cpp
  src/iap/odometry/loose_initial_state_estimation.cpp
  src/iap/odometry/odometry_estimation_base.cpp
  src/iap/odometry/async_odometry_estimation.cpp
  src/iap/odometry/odometry_estimation_imu.cpp
  src/iap/odometry/odometry_estimation_bspline.cpp
  src/iap/odometry/odometry_estimation_ct.cpp
  src/iap/odometry/odometry_estimation_cpu.cpp
  # mapping
  src/iap/mapping/callbacks.cpp
  src/iap/mapping/sub_map.cpp
  src/iap/mapping/sub_mapping_base.cpp
  src/iap/mapping/global_mapping_base.cpp
  src/iap/mapping/async_sub_mapping.cpp
  src/iap/mapping/async_global_mapping.cpp
  src/iap/mapping/sub_mapping.cpp
  src/iap/mapping/sub_mapping_passthrough.cpp
  src/iap/mapping/global_mapping.cpp
  src/iap/mapping/global_mapping_pose_graph.cpp
  # GPU-related
  ${GPU_SRCS}
)
target_include_directories(iap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/json/include>
  $<INSTALL_INTERFACE:include/iap>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(iap
  Boost::boost
  Boost::serialization
  Eigen3::Eigen
  gtsam
  gtsam_points::gtsam_points
  OpenMP::OpenMP_CXX
  spdlog::spdlog
  $<TARGET_NAME_IF_EXISTS:Iridescence::Iridescence>
  ${OpenCV_LIBRARIES}
)
if(BUILD_WITH_OPENCV)
  target_compile_definitions(iap PUBLIC GLIM_USE_OPENCV)
endif()
list(APPEND iap_LIBRARIES iap)

# Create shared libraries for estimation modules
list(APPEND module_srcs
  src/iap/odometry/odometry_estimation_bspline_create.cpp
  src/iap/odometry/odometry_estimation_ct_create.cpp
  src/iap/odometry/odometry_estimation_cpu_create.cpp
  src/iap/mapping/sub_mapping_create.cpp
  src/iap/mapping/sub_mapping_passthrough_create.cpp
  src/iap/mapping/global_mapping_create.cpp
  src/iap/mapping/global_mapping_pose_graph_create.cpp
)
if(BUILD_WITH_CUDA)
  list(APPEND module_srcs
    src/iap/odometry/odometry_estimation_gpu_create.cpp
  )
endif()

foreach(module_src IN LISTS module_srcs)
  get_filename_component(module_src_filename ${module_src} NAME_WE)
  string(REGEX REPLACE "_create" "" module_name ${module_src_filename})

  add_library(${module_name} SHARED
    ${module_src}
  )
  target_link_libraries(${module_name}
    iap
  )
  list(APPEND iap_LIBRARIES ${module_name})
endforeach()


if(BUILD_WITH_VIEWER)
  # Standard viewer
  add_library(standard_viewer SHARED
    src/iap/viewer/standard_viewer.cpp
    src/iap/viewer/standard_viewer_ui.cpp
    src/iap/viewer/standard_viewer_callbacks.cpp
    src/iap/viewer/standard_viewer_mem.cpp
  )
  target_link_libraries(standard_viewer
    iap
  )

  # Interactive viewer
  add_library(interactive_viewer SHARED
    src/iap/viewer/interactive_viewer.cpp
    src/iap/viewer/interactive/manual_loop_close_modal.cpp
    src/iap/viewer/interactive/bundle_adjustment_modal.cpp
    src/iap/viewer/offline_viewer.cpp
  )
  target_link_libraries(interactive_viewer
    iap
  )

  # Map editor
  add_library(map_editor SHARED
    src/iap/viewer/editor/map_cell.cpp
    src/iap/viewer/editor/points_selector.cpp
    src/iap/viewer/map_editor.cpp
  )
  target_link_libraries(map_editor
    iap
  )

  # Memory monitor
  add_library(memory_monitor SHARED
    src/iap/viewer/memory_monitor.cpp
  )
  target_link_libraries(memory_monitor
    iap
  )

  list(APPEND iap_LIBRARIES standard_viewer interactive_viewer memory_monitor map_editor)
endif()

#############
## Install ##
#############

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    # No idea
    set(CMAKE_INSTALL_INCLUDEDIR "include/iap")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_BINDIR "bin/iap")
    set(CMAKE_INSTALL_DATADIR "share/iap")
    set(CMAKE_CONFIG_INSTALL_DIR "share/iap")
  elseif($ENV{ROS_VERSION} EQUAL 2)
    set(CMAKE_INSTALL_INCLUDEDIR "include/iap")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_BINDIR "bin/iap")
    set(CMAKE_INSTALL_DATADIR "share/iap")
    set(CMAKE_CONFIG_INSTALL_DIR "share/iap")
  else()
    message(FATAL_ERROR "Unsupported ROS version")
  endif()
else()
  include(GNUInstallDirs)
  set(CMAKE_CONFIG_INSTALL_DIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/iap"
    CACHE PATH "Install directory for CMake config files"
  )
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${iap_LIBRARIES}
  EXPORT iap-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
include(CMakePackageConfigHelpers)
install(EXPORT iap-targets
  FILE iap-targets.cmake
  NAMESPACE iap::
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/iap-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/iap-config.cmake"
  INSTALL_DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/iap-config-version.cmake"
  VERSION ${VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/iap-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/iap-config-version.cmake"
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)

#################
## ROS-related ##
#################
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)

    ## Message generation (IAP-RQ-500: ROS2 msgs)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}_msgs
      "msg/IntegrityReport.msg"
      "msg/DynamicALResult.msg"
      "msg/TrunkLandmark.msg"
      "msg/MDPState.msg"
      LIBRARY_NAME ${PROJECT_NAME}
      DEPENDENCIES std_msgs
    )

    ## IAP-RQ-003: standalone dependencies (rviz_viewer + standard_viewer)
    find_package(tf2_ros REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)

    ## GNSS ROS2 extension module (IAP-RQ-020 bridge)
    find_package(glog REQUIRED)
    find_package(gnss_comm REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(rclcpp REQUIRED)

    add_library(gnss_extension SHARED
      src/iap/gnss/gnss_extension.cpp
    )
    target_include_directories(gnss_extension PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    # gnss_comm headers needed by pseudorange_factor.cpp / doppler_factor.cpp
    # which are compiled into the iap library target
    ament_target_dependencies(iap gnss_comm)
    target_link_libraries(gnss_extension iap)
    ament_target_dependencies(gnss_extension rclcpp sensor_msgs gnss_comm)
    list(APPEND iap_LIBRARIES gnss_extension)
    install(TARGETS gnss_extension DESTINATION lib)

    ## Trunk extension module (IAP-RQ-130: factor graph + RViz2 visualization)
    add_library(trunk_extension SHARED
      src/iap/trunk/trunk_extension.cpp
    )
    target_include_directories(trunk_extension PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(trunk_extension iap)
    ament_target_dependencies(trunk_extension rclcpp visualization_msgs)
    list(APPEND iap_LIBRARIES trunk_extension)
    install(TARGETS trunk_extension DESTINATION lib)

    ## Integrity extension module (IAP-RQ-200/210/220/240: ARAIM pipeline integration)
    rosidl_get_typesupport_target(iap_msgs_cpp_ts ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
    add_library(integrity_extension SHARED
      src/iap/integrity/integrity_extension.cpp
    )
    target_include_directories(integrity_extension PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(integrity_extension iap "${iap_msgs_cpp_ts}")
    ament_target_dependencies(integrity_extension rclcpp std_msgs)
    list(APPEND iap_LIBRARIES integrity_extension)
    install(TARGETS integrity_extension DESTINATION lib)

    ## RViz2 visualization extension module (IAP-RQ-003: standalone operation)
    add_library(rviz_viewer SHARED
      src/iap/util/rviz_viewer.cpp
    )
    target_include_directories(rviz_viewer PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_definitions(rviz_viewer PRIVATE GLIM_ROS2)
    target_link_libraries(rviz_viewer iap)
    ament_target_dependencies(rviz_viewer rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)
    list(APPEND iap_LIBRARIES rviz_viewer)
    install(TARGETS rviz_viewer DESTINATION lib)

    ## Demo / smoke-test executable (IAP-RQ-002)
    add_executable(iap_status apps/iap_status.cpp)
    target_link_libraries(iap_status iap)
    list(APPEND iap_LIBRARIES iap_status)

    ## Standalone ROS2 runtime host (phase-1: replace glim_rosnode)
    add_executable(iap_rosnode apps/iap_rosnode.cpp)
    target_link_libraries(iap_rosnode iap)
    target_compile_definitions(iap_rosnode PRIVATE GLIM_ROS2)
    ament_target_dependencies(iap_rosnode rclcpp sensor_msgs)
    list(APPEND iap_LIBRARIES iap_rosnode)

    ## Experiment runner (IAP-RQ-500/510)
    add_executable(iap_experiment apps/iap_experiment.cpp)
    target_link_libraries(iap_experiment iap)
    list(APPEND iap_LIBRARIES iap_experiment)

    install(TARGETS iap_status iap_experiment iap_rosnode DESTINATION lib/iap)
    install(DIRECTORY launch DESTINATION share/iap)
    install(DIRECTORY config DESTINATION share/iap)

    ## Tests (IAP-RQ-241–246, RQ-200, RQ-131)
    if(BUILD_TESTING)
      find_package(ament_cmake_gtest REQUIRED)
      ament_add_gtest(test_araim test/test_araim.cpp)
      target_link_libraries(test_araim iap)
      ament_add_gtest(test_bspline_control_window test/test_bspline_control_window.cpp)
      target_link_libraries(test_bspline_control_window iap)
      ament_add_gtest(test_bspline_imu_factor test/test_bspline_imu_factor.cpp)
      target_link_libraries(test_bspline_imu_factor iap)
      ament_add_gtest(test_bspline_gnss_factor test/test_bspline_gnss_factor.cpp)
      target_link_libraries(test_bspline_gnss_factor iap)
      ament_add_gtest(test_integrity_planner test/test_integrity_planner.cpp)
      target_link_libraries(test_integrity_planner iap)
      ament_add_gtest(test_bspline_velocity_factor test/test_bspline_velocity_factor.cpp)
      target_link_libraries(test_bspline_velocity_factor iap)
      ament_add_gtest(test_bspline_trajectory test/test_bspline_trajectory.cpp)
      target_link_libraries(test_bspline_trajectory iap)
    endif()
    ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    ament_export_libraries(${iap_LIBRARIES} ${GTSAM_LIBRARIES} fmt spdlog::spdlog)
    ament_export_targets(iap-targets HAS_LIBRARY_TARGET)
    ament_package(CONFIG_EXTRAS "cmake/iap-config.cmake.in")
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${iap_LIBRARIES} ${GTSAM_LIBRARIES} fmt spdlog::spdlog
    )
    install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
endif()
