cmake_minimum_required(VERSION 3.16)
project(iap VERSION 1.2.1 LANGUAGES C CXX)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Version string available as IAP_VERSION in all targets
add_compile_definitions(IAP_VERSION="${PROJECT_VERSION}")
add_compile_definitions(IAP_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
add_compile_definitions(IAP_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_CUDA_MULTIARCH "Build with GPU cross-platform support" OFF)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_WITH_OPENCV "Build with OpenCV to support camera images" ON)
option(IAP_ENABLE_DETAILED_TIMING "Enable per-cell cumulative timing in advisory grid rebuild" ON)

if(IAP_ENABLE_DETAILED_TIMING)
  add_compile_definitions(IAP_ENABLE_DETAILED_TIMING)
endif()

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

# ===========================================================================
# Step 10: Narrow integrity interface targets (for test linking & documentation)
# ===========================================================================
# These are header/link-only targets. All sources remain compiled into iap.
# Tests link these to declare their narrow dependency scope.

# --- iap_integrity_common ---
add_library(iap_integrity_common INTERFACE)
target_include_directories(iap_integrity_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(iap_integrity_common INTERFACE Eigen3::Eigen spdlog::spdlog)

# --- iap_current_integrity ---
add_library(iap_current_integrity INTERFACE)
target_include_directories(iap_current_integrity INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/json/include>
)
target_link_libraries(iap_current_integrity INTERFACE
  iap_integrity_common Eigen3::Eigen gtsam gtsam_points::gtsam_points spdlog::spdlog
)

# --- iap_prediction_integrity ---
add_library(iap_prediction_integrity INTERFACE)
target_include_directories(iap_prediction_integrity INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(iap_prediction_integrity INTERFACE
  iap_current_integrity Eigen3::Eigen gtsam gtsam_points::gtsam_points spdlog::spdlog
)

# --- iap_predictor ---
add_library(iap_predictor INTERFACE)
target_include_directories(iap_predictor INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(iap_predictor INTERFACE
  iap_prediction_integrity Eigen3::Eigen spdlog::spdlog
)

# --- iap_planner_risk_grid ---
add_library(iap_planner_risk_grid INTERFACE)
target_include_directories(iap_planner_risk_grid INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(iap_planner_risk_grid INTERFACE
  iap_predictor Eigen3::Eigen spdlog::spdlog
)

add_library(iap SHARED
  # util
  src/iap/util/config.cpp
  src/iap/util/logging.cpp
  src/iap/util/run_log_manager.cpp
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
  src/iap/integrity/lidar_araim.cpp
  # FGO information matrix extraction (IAP-RQ-300)
  src/iap/integrity/fgo_information_manager.cpp
  # predictor-side advisory integrity support (IAP-RQ-331)
  src/iap/planner/integrity_snapshot.cpp
  src/iap/planner/future_pl_query_result.cpp
  src/iap/predictor/lidar_observability_fim.cpp
  src/iap/planner/pl_grid.cpp
  src/iap/planner/future_pl_field_predictor.cpp
  src/iap/planner/pi_cost_adapter.cpp
  src/iap/planner/unified_risk_grid.cpp
  src/iap/planner/predicted_araim.cpp
  src/iap/planner/risk_grid_map.cpp
  src/iap/planner/p1_accepted_context_validation.cpp
  src/iap/predictor/gnss_geometry_pl_predictor.cpp
  # independent advisory predictor query module
  src/iap/predictor/gnss_advisory_predictor.cpp
  src/iap/predictor/lidar_advisory_predictor.cpp
  src/iap/predictor/fusion_advisory_predictor.cpp
  src/iap/predictor/predictor_module.cpp
  src/iap/predictor/rolling_spatial_advisory_window.cpp
  # integrity monitoring (IAP-RQ-200/210/220)
  src/iap/integrity/integrity_monitor.cpp
  src/iap/integrity/integrity_fusion_policy.cpp
  # odometry
  src/iap/odometry/callbacks.cpp
  src/iap/odometry/estimation_frame.cpp
  src/iap/odometry/initial_state_estimation.cpp
  src/iap/odometry/loose_initial_state_estimation.cpp
  src/iap/odometry/odometry_estimation_base.cpp
  src/iap/odometry/async_odometry_estimation.cpp
  src/iap/odometry/odometry_estimation_imu.cpp
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

# Repository-local, non-ROS P0 provider profiler (IAP-RQ-320).
add_executable(iap_predictor_offline_profile
  apps/iap_predictor_offline_profile.cpp
)
target_link_libraries(iap_predictor_offline_profile
  iap iap_predictor iap_planner_risk_grid
)

# Create shared libraries for estimation modules
list(APPEND module_srcs
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
    find_package(traj_utils REQUIRED)
    find_package(quadrotor_msgs REQUIRED)

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
    ament_target_dependencies(integrity_extension rclcpp std_msgs visualization_msgs)
    list(APPEND iap_LIBRARIES integrity_extension)
    install(TARGETS integrity_extension DESTINATION lib)

    ## Simulation bridge extension module (IAP <-> EGO sim adapter)
    add_library(sim_extension SHARED
      src/iap/sim/sim_extension.cpp
    )
    target_include_directories(sim_extension PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(sim_extension iap)
    ament_target_dependencies(sim_extension rclcpp nav_msgs)
    list(APPEND iap_LIBRARIES sim_extension)
    install(TARGETS sim_extension DESTINATION lib)

    ## Demo8 truth ARAIM comparison extension
    add_library(demo8_truth_araim_extension SHARED
      src/iap/sim/demo8_truth_araim_extension.cpp
    )
    target_include_directories(demo8_truth_araim_extension PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(demo8_truth_araim_extension iap)
    ament_target_dependencies(demo8_truth_araim_extension rclcpp nav_msgs visualization_msgs gnss_comm)
    list(APPEND iap_LIBRARIES demo8_truth_araim_extension)
    install(TARGETS demo8_truth_araim_extension DESTINATION lib)

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

    ## Demo3 odometry mux (truth bootstrap -> IAP odom lock)
    add_executable(demo3_odom_mux apps/demo3_odom_mux.cpp)
    target_link_libraries(demo3_odom_mux iap)
    ament_target_dependencies(demo3_odom_mux rclcpp nav_msgs)
    list(APPEND iap_LIBRARIES demo3_odom_mux)

    ## Demo3 IAP bridge (default glim_rosnode topics -> /iap_rosnode/* aliases)
    add_executable(demo3_iap_bridge apps/demo3_iap_bridge.cpp)
    target_link_libraries(demo3_iap_bridge iap)
    ament_target_dependencies(demo3_iap_bridge rclcpp nav_msgs sensor_msgs)
    list(APPEND iap_LIBRARIES demo3_iap_bridge)

    ## Demo4 LiDAR bridge (map-frame simulated cloud -> body/lidar-frame scan for IAP)
    add_executable(demo4_lidar_body_bridge apps/demo4_lidar_body_bridge.cpp)
    target_link_libraries(demo4_lidar_body_bridge iap)
    ament_target_dependencies(demo4_lidar_body_bridge rclcpp nav_msgs sensor_msgs)
    list(APPEND iap_LIBRARIES demo4_lidar_body_bridge)

    ## Demo10 Phase 2 planner integrity evaluator (C++ GNSS geometry ARAIM)
    add_executable(phase2_planner_integrity_evaluator
      apps/phase2_planner_integrity_evaluator.cpp
    )
    target_link_libraries(phase2_planner_integrity_evaluator iap "${iap_msgs_cpp_ts}")
    target_compile_definitions(phase2_planner_integrity_evaluator PRIVATE GLIM_ROS2)
    ament_target_dependencies(phase2_planner_integrity_evaluator
      rclcpp
      nav_msgs
      sensor_msgs
      geometry_msgs
      visualization_msgs
      gnss_comm
      traj_utils
      quadrotor_msgs
    )
    list(APPEND iap_LIBRARIES phase2_planner_integrity_evaluator)

    ## Demo11 forest scenario helpers
    add_executable(demo_takeoff_cmd_publisher
      apps/demo_takeoff_cmd_publisher.cpp
    )
    target_link_libraries(demo_takeoff_cmd_publisher iap)
    ament_target_dependencies(demo_takeoff_cmd_publisher rclcpp quadrotor_msgs)
    list(APPEND iap_LIBRARIES demo_takeoff_cmd_publisher)

    add_executable(demo11_corridor_map_publisher
      apps/demo11_corridor_map_publisher.cpp
    )
    target_link_libraries(demo11_corridor_map_publisher iap)
    ament_target_dependencies(demo11_corridor_map_publisher rclcpp sensor_msgs)
    list(APPEND iap_LIBRARIES demo11_corridor_map_publisher)

    add_executable(demo11_compare_paths_visualizer
      apps/demo11_compare_paths_visualizer.cpp
    )
    target_link_libraries(demo11_compare_paths_visualizer iap)
    ament_target_dependencies(demo11_compare_paths_visualizer
      rclcpp
      nav_msgs
      geometry_msgs
      sensor_msgs
    )
    list(APPEND iap_LIBRARIES demo11_compare_paths_visualizer)

    ## Predictor module system-test probe
    add_executable(test_predictor_query_probe
      apps/test_predictor_query_probe.cpp
    )
    target_link_libraries(test_predictor_query_probe iap "${iap_msgs_cpp_ts}")
    ament_target_dependencies(test_predictor_query_probe
      rclcpp
      nav_msgs
      sensor_msgs
      gnss_comm
    )
    list(APPEND iap_LIBRARIES test_predictor_query_probe)

    install(TARGETS
      iap_status
      iap_experiment
      iap_rosnode
      demo3_odom_mux
      demo3_iap_bridge
      demo4_lidar_body_bridge
      phase2_planner_integrity_evaluator
      demo_takeoff_cmd_publisher
      demo11_corridor_map_publisher
      demo11_compare_paths_visualizer
      test_predictor_query_probe
      DESTINATION lib/iap
    )
    install(PROGRAMS
      scripts/test_araim_validator.py
      scripts/planner_evidence_provenance_publisher.py
      scripts/planner_bag_recorder_with_finalizer.py
      scripts/dev_planner/finalize_planner_evidence_manifest.py
      scripts/dev_planner/verify_planner_recorder_smoke.py
      scripts/dev_planner/verify_safety_planner_evidence_bundle.py
      scripts/dev_planner/analyze_p1_candidate_diagnostic_smoke.py
      scripts/dev_planner/archive_p1_2_retrospective.py
      scripts/dev_planner/analyze_safety_planner_run.py
      scripts/dev_planner/calibrate_p1_formal_tolerances.py
      scripts/dev_planner/p1_formal_metrics.py
      scripts/test_predictor_validator.py
      scripts/dev_predictor/analyze_predictor_system_e1.py
      scripts/dev_predictor/analyze_predictor_system_e2.py
      scripts/dev_predictor/analyze_predictor_system_e3.py
      scripts/dev_predictor/analyze_predictor_system_e4.py
      scripts/dev_predictor/analyze_predictor_system_e5.py
      scripts/dev_predictor/analyze_predictor_system_e6.py
      scripts/dev_predictor/analyze_predictor_system_e7.py
      scripts/dev_predictor/analyze_predictor_system_e8.py
      scripts/dev_predictor/analyze_predictor_system_e9.py
      scripts/dev_predictor/analyze_predictor_system_e10.py
      scripts/dev_predictor/analyze_predictor_system_e11.py
      scripts/dev_predictor/analyze_predictor_system_e12.py
      scripts/dev_planner/analyze_planner_p0_phase1.py
      scripts/dev_planner/gate0_analyzer.py
      scripts/dev_planner/gate0_capture_p0_health.py
      scripts/dev_planner/gate0_disk_audit.py
      scripts/dev_planner/run_gate0_qualification.py
      DESTINATION lib/iap
    )
    install(DIRECTORY launch DESTINATION share/iap)
    install(DIRECTORY config DESTINATION share/iap)

    ## Tests (IAP-RQ-241–246, RQ-200, RQ-131)
    if(BUILD_TESTING)
      find_package(Python3 REQUIRED COMPONENTS Interpreter)
      find_package(ament_cmake_gtest REQUIRED)
      # Step 10: tests declare narrow deps via interface targets (impl still in iap)
      ament_add_gtest(test_araim test/test_araim.cpp)
      target_link_libraries(test_araim iap iap_current_integrity)
      if(DEFINED iap_msgs_cpp_ts)
        target_link_libraries(test_araim "${iap_msgs_cpp_ts}")
      endif()

      ament_add_gtest(test_integrity_fusion_policy test/test_integrity_fusion_policy.cpp)
      target_link_libraries(test_integrity_fusion_policy iap iap_integrity_common)
      ament_add_gtest(test_predicted_araim test/test_predicted_araim.cpp)
      target_link_libraries(test_predicted_araim iap iap_prediction_integrity)
      ament_add_gtest(test_integrity_snapshot test/test_integrity_snapshot.cpp)
      target_link_libraries(test_integrity_snapshot iap iap_prediction_integrity)
      ament_add_gtest(test_lidar_observability_fim test/test_lidar_observability_fim.cpp)
      target_link_libraries(test_lidar_observability_fim iap)
      ament_add_gtest(test_pl_grid test/test_pl_grid.cpp)
      target_link_libraries(test_pl_grid iap)
      ament_add_gtest(test_local_occupancy test/test_local_occupancy.cpp)
      target_link_libraries(test_local_occupancy iap)
      ament_add_gtest(test_future_pl_field_predictor test/test_future_pl_field_predictor.cpp)
      target_link_libraries(test_future_pl_field_predictor iap iap_prediction_integrity)
      ament_add_gtest(test_predictor_module test/test_predictor_module.cpp)
      target_link_libraries(test_predictor_module iap iap_predictor)
      ament_add_gtest(test_rolling_spatial_advisory_window
        test/test_rolling_spatial_advisory_window.cpp
      )
      target_link_libraries(test_rolling_spatial_advisory_window
        iap iap_predictor
      )
      ament_add_gtest(test_predictor_risk_conversion
        test/test_predictor_risk_conversion.cpp
      )
      target_link_libraries(test_predictor_risk_conversion
        iap iap_predictor iap_planner_risk_grid
      )
      ament_add_gtest(test_risk_grid_map test/test_risk_grid_map.cpp)
      target_link_libraries(test_risk_grid_map iap iap_planner_risk_grid)
      ament_add_gtest(test_pi_cost_adapter test/test_pi_cost_adapter.cpp)
      target_link_libraries(test_pi_cost_adapter iap)
      ament_add_gtest(test_unified_risk_grid test/test_unified_risk_grid.cpp)
      target_link_libraries(test_unified_risk_grid iap)
      ament_add_gtest(test_odom_freshness test/test_odom_freshness.cpp)
      target_link_libraries(test_odom_freshness iap)
      ament_add_gtest(test_alert_limit_model test/test_alert_limit_model.cpp)
      target_link_libraries(test_alert_limit_model iap)
      ament_add_gtest(test_run_log_manager test/test_run_log_manager.cpp)
      ament_add_gtest(test_p1_fixture_geometry test/test_p1_fixture_geometry.cpp)
      target_link_libraries(test_run_log_manager iap)
      target_link_libraries(test_p1_fixture_geometry iap)
      add_test(
        NAME test_phase2_summary_schema
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_phase2_summary_schema.py
      )
      add_test(
        NAME test_icra006_provider_profile
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_icra006_provider_profile.py
      )
      add_test(
        NAME test_icra007_provider_profile
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_icra007_provider_profile.py
      )
      add_test(
        NAME test_icra011_spatial_dedup_profile
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_icra011_spatial_dedup_profile.py
      )
      add_test(
        NAME test_icra020_p0_rolling_worker_profile
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_icra020_p0_rolling_worker_profile.py
      )
      add_test(
        NAME test_verify_planner_recorder_smoke
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_verify_planner_recorder_smoke.py
      )
      add_test(
        NAME test_verify_p1_pre_admission_feedback
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_verify_p1_pre_admission_feedback.py
      )
      add_test(
        NAME test_analyze_p1_pre_admission_smoke
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_analyze_p1_pre_admission_smoke.py
      )
      add_test(
        NAME test_p1_formal_metrics
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_p1_formal_metrics.py
      )
      add_test(
        NAME test_calibrate_p1_formal_tolerances
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_calibrate_p1_formal_tolerances.py
      )
      add_test(
        NAME test_archive_p1_2_retrospective
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_archive_p1_2_retrospective.py
      )
      add_test(
        NAME test_gate0_analyzer
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gate0_analyzer.py
      )
      add_test(
        NAME test_gate0_runner
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gate0_runner.py
      )
      add_test(
        NAME test_gate0_capture_p0_health
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gate0_capture_p0_health.py
      )
      add_test(
        NAME test_test_planner_launch
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_test_planner_launch.py
      )
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
