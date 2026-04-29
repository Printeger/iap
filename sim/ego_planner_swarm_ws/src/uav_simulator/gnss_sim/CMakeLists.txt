cmake_minimum_required(VERSION 3.8)
project(gnss_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_LIST_DIR}/../../../../../../gnss_comm/cmake")
find_package(glog REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(gnss_sim_node src/gnss_sim_node.cpp)
target_include_directories(gnss_sim_node PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(gnss_sim_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  gnss_comm
  Eigen3
)

install(TARGETS gnss_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
