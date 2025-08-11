cmake_minimum_required(VERSION 3.8)
project(ekf_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ekf src/ekf.cpp)
ament_target_dependencies(ekf rclcpp px4_msgs std_msgs)

add_executable(quaternion_tf src/quaternion_tf.cpp)
ament_target_dependencies(quaternion_tf rclcpp px4_msgs geometry_msgs tf2 tf2_ros Eigen3)

add_executable(px4_offboard src/px4_offboard.cpp)
ament_target_dependencies(px4_offboard rclcpp px4_msgs std_msgs)

install(TARGETS
  ekf
  quaternion_tf
  px4_offboard
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
