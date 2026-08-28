cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(traj_utils REQUIRED)
find_package(path_searching REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(iap REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(OpenSSL REQUIRED)
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
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

ament_export_include_directories(include)

add_library( bspline_opt 
    src/uniform_bspline.cpp 
    src/bspline_optimizer.cpp 
    src/p4_collision_guide.cpp
    src/gradient_descent_optimizer.cpp
    )

ament_target_dependencies(bspline_opt 
                          std_msgs visualization_msgs 
                          nav_msgs cv_bridge 
                          rclcpp Eigen3 
                          PCL pcl_conversions 
                          plan_env traj_utils path_searching gnss_comm iap)

target_link_libraries(bspline_opt
  iap::iap
)

if(BUILD_TESTING)
  ament_add_gtest(test_p1_integrity_cost
    test/test_p1_integrity_cost.cpp
  )
  ament_target_dependencies(test_p1_integrity_cost
    rclcpp Eigen3 gnss_comm iap
  )
  target_link_libraries(test_p1_integrity_cost
    bspline_opt
    iap::iap
  )

  ament_add_gtest(test_p4_collision_scan_contract
    test/test_p4_collision_scan_contract.cpp
  )
  ament_target_dependencies(test_p4_collision_scan_contract
    rclcpp Eigen3 plan_env path_searching
  )
  target_link_libraries(test_p4_collision_scan_contract
    bspline_opt
  )

  ament_add_gtest(test_p4_collision_guide
    test/test_p4_collision_guide.cpp
  )
  ament_target_dependencies(test_p4_collision_guide
    rclcpp Eigen3 iap path_searching
  )
  target_link_libraries(test_p4_collision_guide
    bspline_opt
    iap::iap
  )

  ament_add_gtest(test_p4_collision_guide_integration
    test/test_p4_collision_guide_integration.cpp
  )
  ament_target_dependencies(test_p4_collision_guide_integration
    rclcpp Eigen3 iap plan_env path_searching
  )
  target_link_libraries(test_p4_collision_guide_integration
    bspline_opt
    iap::iap
  )

  add_executable(icra076_repeatability_probe
    test/icra076_repeatability_probe.cpp
  )
  target_include_directories(icra076_repeatability_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../thirdparty/json/include
  )
  ament_target_dependencies(icra076_repeatability_probe
    rclcpp Eigen3 iap path_searching
  )
  target_link_libraries(icra076_repeatability_probe
    bspline_opt
    iap::iap
    OpenSSL::Crypto
  )
endif()


install(TARGETS bspline_opt
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(bspline_opt)
ament_export_dependencies(gnss_comm)
ament_export_dependencies(iap)

ament_package()
