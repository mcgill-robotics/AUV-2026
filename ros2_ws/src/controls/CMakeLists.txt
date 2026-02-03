cmake_minimum_required(VERSION 3.8)
project(controls)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(attitude_controller
  src/attitude_controller.cpp)

add_executable(superimposer
  src/superimposer.cpp)

target_include_directories(attitude_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(superimposer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(attitude_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  Eigen3)

ament_target_dependencies(superimposer
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  Eigen3)

ament_python_install_package(controls)

install(TARGETS
  attitude_controller
  superimposer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  scripts/depth_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
