cmake_minimum_required(VERSION 3.8)
project(ekf_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(PKG_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(ekf src/ekf.cpp)
target_include_directories(ekf PUBLIC ${PKG_DIRS})
ament_target_dependencies(ekf Eigen3)

add_executable(estimate src/estimate.cpp)
target_include_directories(estimate PUBLIC ${PKG_DIRS})
target_link_libraries(estimate ekf)
ament_target_dependencies(estimate rclcpp geometry_msgs message_filters sensor_msgs tf2 tf2_ros Eigen3)

install(TARGETS
  ekf
  estimate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/pi_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(ahrs
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ahrs/ahrs)

ament_package()
