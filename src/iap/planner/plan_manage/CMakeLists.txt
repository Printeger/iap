cmake_minimum_required(VERSION 3.8)
project(ego_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED COMPONENTS traj_utils_lib)
find_package(cv_bridge REQUIRED)
find_package(iap REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(gnss_comm REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_library(IAP_MSGS_TYPESUPPORT_CPP
  NAMES iap__rosidl_typesupport_cpp
  REQUIRED
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/gate0_qualification_writer.cpp
  src/planner_manager.cpp
  src/p0_risk_grid_runtime.cpp
  src/p0_occupancy_epoch_adapter.cpp
  src/p1_replan_admission.cpp
  src/p1_candidate_selection.cpp
  src/p2_candidate_ranking.cpp
  src/p3_reference_bias.cpp
  src/p5_runtime_integrity_gate.cpp
  src/safety_rviz_publisher.cpp
)

# target_link_libraries(ego_planner_node
#   traj_utils_lib
# )

ament_target_dependencies(ego_planner_node 
                          rclcpp nav_msgs visualization_msgs 
                          geometry_msgs quadrotor_msgs 
                          plan_env path_searching 
                          bspline_opt traj_utils 
                          cv_bridge pcl_conversions
                          std_msgs sensor_msgs gnss_comm )

find_library(TRAJ_UTILS_LIB traj_utils_lib PATHS ${traj_utils_LIBRARY_DIRS})

target_link_libraries(ego_planner_node
    ${TRAJ_UTILS_LIB}
    iap::iap
    ${IAP_MSGS_TYPESUPPORT_CPP}
)

add_executable(traj_server src/traj_server.cpp)

ament_target_dependencies(traj_server 
                          rclcpp nav_msgs 
                          visualization_msgs geometry_msgs 
                          quadrotor_msgs plan_env 
                          path_searching bspline_opt 
                          traj_utils cv_bridge pcl_conversions)

if(BUILD_TESTING)
  ament_add_gtest(test_gate0_qualification_writer
    test/test_gate0_qualification_writer.cpp
    src/gate0_qualification_writer.cpp
  )
  ament_target_dependencies(test_gate0_qualification_writer Eigen3)

  ament_add_gtest(test_p0_risk_grid_runtime
    test/test_p0_risk_grid_runtime.cpp
    src/p0_risk_grid_runtime.cpp
    src/p0_occupancy_epoch_adapter.cpp
    src/safety_rviz_publisher.cpp
  )

  ament_add_gtest(test_p0_occupancy_epoch_adapter
    test/test_p0_occupancy_epoch_adapter.cpp
    src/p0_occupancy_epoch_adapter.cpp
  )
  ament_target_dependencies(test_p0_occupancy_epoch_adapter
    Eigen3 iap
  )
  target_link_libraries(test_p0_occupancy_epoch_adapter iap::iap)

  ament_add_gtest(test_p1_replan_admission
    test/test_p1_replan_admission.cpp
    src/p1_replan_admission.cpp
  )
  ament_add_gtest(test_p4_risk_grid_planning_admission
    test/test_p4_risk_grid_planning_admission.cpp
  )
  ament_target_dependencies(test_p4_risk_grid_planning_admission
    rclcpp nav_msgs geometry_msgs std_msgs sensor_msgs visualization_msgs
    quadrotor_msgs traj_utils bspline_opt plan_env path_searching cv_bridge
    pcl_conversions gnss_comm iap
  )
  target_link_libraries(test_p4_risk_grid_planning_admission iap::iap)
  ament_add_gtest(test_p1_candidate_selection
    test/test_p1_candidate_selection.cpp
    src/p1_candidate_selection.cpp
  )
  ament_target_dependencies(test_p0_risk_grid_runtime
    rclcpp nav_msgs geometry_msgs std_msgs sensor_msgs visualization_msgs
    gnss_comm iap
  )
  target_link_libraries(test_p0_risk_grid_runtime
    iap::iap
    ${IAP_MSGS_TYPESUPPORT_CPP}
  )

  ament_add_gtest(test_p5_runtime_integrity_gate
    test/test_p5_runtime_integrity_gate.cpp
    src/p5_runtime_integrity_gate.cpp
    src/safety_rviz_publisher.cpp
  )
  ament_target_dependencies(test_p5_runtime_integrity_gate
    rclcpp nav_msgs geometry_msgs std_msgs sensor_msgs visualization_msgs
    traj_utils bspline_opt iap
  )
  target_link_libraries(test_p5_runtime_integrity_gate
    ${TRAJ_UTILS_LIB}
    iap::iap
    ${IAP_MSGS_TYPESUPPORT_CPP}
  )

  ament_add_gtest(test_p2_candidate_ranking
    test/test_p2_candidate_ranking.cpp
    src/p2_candidate_ranking.cpp
  )
  ament_target_dependencies(test_p2_candidate_ranking
    rclcpp Eigen3 nav_msgs visualization_msgs plan_env path_searching
    bspline_opt traj_utils cv_bridge pcl_conversions iap gnss_comm
  )
  target_link_libraries(test_p2_candidate_ranking
    iap::iap
  )

  ament_add_gtest(test_p3_reference_bias
    test/test_p3_reference_bias.cpp
    src/p3_reference_bias.cpp
  )
  ament_target_dependencies(test_p3_reference_bias
    rclcpp Eigen3 iap gnss_comm
  )
  target_link_libraries(test_p3_reference_bias
    iap::iap
  )

  ament_add_gtest(test_planning_risk_context
    test/test_planning_risk_context.cpp
    src/gate0_qualification_writer.cpp
    src/planner_manager.cpp
    src/p0_risk_grid_runtime.cpp
    src/p0_occupancy_epoch_adapter.cpp
    src/p1_candidate_selection.cpp
    src/p2_candidate_ranking.cpp
    src/p3_reference_bias.cpp
    src/p5_runtime_integrity_gate.cpp
    src/safety_rviz_publisher.cpp
  )
  ament_target_dependencies(test_planning_risk_context
    rclcpp nav_msgs visualization_msgs geometry_msgs quadrotor_msgs
    plan_env path_searching bspline_opt traj_utils cv_bridge pcl_conversions
    std_msgs sensor_msgs gnss_comm iap
  )
  target_link_libraries(test_planning_risk_context
    ${TRAJ_UTILS_LIB}
    iap::iap
    ${IAP_MSGS_TYPESUPPORT_CPP}
  )
endif()

# Install targets
install(
  TARGETS ego_planner_node traj_server
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
