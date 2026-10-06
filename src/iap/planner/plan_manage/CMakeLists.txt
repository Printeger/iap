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
find_package(gnss_comm REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(IAP_MSGS_TYPESUPPORT_CPP NAMES iap__rosidl_typesupport_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

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
  src/planner_manager.cpp
  src/planner_risk.cpp
  src/planner_risk_viz.cpp
)

# target_link_libraries(ego_planner_node
#   traj_utils_lib
# )

ament_target_dependencies(ego_planner_node
                          rclcpp nav_msgs visualization_msgs
                          geometry_msgs quadrotor_msgs
                          plan_env path_searching
                          bspline_opt traj_utils
                          cv_bridge pcl_conversions iap gnss_comm sensor_msgs )

find_library(TRAJ_UTILS_LIB traj_utils_lib PATHS ${traj_utils_LIBRARY_DIRS})

target_link_libraries(ego_planner_node
    ${TRAJ_UTILS_LIB}
    iap::iap
    ${IAP_MSGS_TYPESUPPORT_CPP}
)

add_executable(traj_server src/traj_server.cpp)

add_executable(failure_map_replay src/failure_map_replay.cpp)
ament_target_dependencies(failure_map_replay
  plan_env path_searching rclcpp Eigen3 nav_msgs visualization_msgs
  geometry_msgs sensor_msgs iap cv_bridge pcl_conversions)

ament_target_dependencies(traj_server
                          rclcpp nav_msgs
                          visualization_msgs geometry_msgs
                          quadrotor_msgs plan_env
                          path_searching bspline_opt
                          traj_utils cv_bridge pcl_conversions)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ZLIB REQUIRED)
  add_test(NAME test_ego_pipeline COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../test/test_ego_pipeline.py
    --planner $<TARGET_FILE:ego_planner_node> --server $<TARGET_FILE:traj_server>)
  set_tests_properties(test_ego_pipeline PROPERTIES TIMEOUT 40)
  add_test(NAME test_failure_map_tools COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../scripts/dev_planner/test_failure_map_tools.py)
  set_tests_properties(test_failure_map_tools PROPERTIES ENVIRONMENT
    "IAP_FAILURE_MAP_REPLAY_BIN=$<TARGET_FILE:failure_map_replay>")
  ament_add_gtest(test_ego_baseline test/test_ego_baseline.cpp
    src/planner_manager.cpp src/planner_risk.cpp src/planner_risk_viz.cpp src/ego_replan_fsm.cpp)
  ament_target_dependencies(test_ego_baseline rclcpp nav_msgs geometry_msgs
    visualization_msgs quadrotor_msgs plan_env path_searching bspline_opt
    traj_utils cv_bridge pcl_conversions iap gnss_comm sensor_msgs)
  target_link_libraries(test_ego_baseline iap::iap ${TRAJ_UTILS_LIB} ${IAP_MSGS_TYPESUPPORT_CPP})
  target_link_libraries(test_ego_baseline ZLIB::ZLIB)
  target_compile_definitions(test_ego_baseline PRIVATE
    IAP_CURVE_OBSERVATION_ANALYZER="${CMAKE_CURRENT_SOURCE_DIR}/../../../../scripts/dev_planner/analyze_curve_observation.py"
    IAP_FAILURE_REGRESSION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/20261005T163036Z_646_endpoint")
endif()

# Install targets
install(
  TARGETS ego_planner_node traj_server failure_map_replay
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
