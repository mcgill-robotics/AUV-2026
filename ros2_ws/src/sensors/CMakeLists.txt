cmake_minimum_required(VERSION 3.5)
project(sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)

set(PYTHON_NODE_SOURCES
  sensors/health_monitor.py
  sensors/dvl_monitor.py
)

# Executables

add_executable(state_aggregator src/State_aggregator.cpp)
ament_target_dependencies(state_aggregator rclcpp std_msgs std_srvs geometry_msgs sensor_msgs message_filters tf2_ros tf2 nav_msgs dvl_msgs angles)
target_link_libraries(state_aggregator Eigen3::Eigen)
target_include_directories(state_aggregator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(imu_processor src/Imu_processor.cpp)
ament_target_dependencies(imu_processor rclcpp std_msgs geometry_msgs sensor_msgs message_filters nav_msgs dvl_msgs angles tf2)
target_link_libraries(imu_processor Eigen3::Eigen)
target_include_directories(imu_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(depth_processor src/depth_processor.cpp)
ament_target_dependencies(depth_processor rclcpp std_msgs std_srvs geometry_msgs sensor_msgs sensor_msgs message_filters )
target_link_libraries(depth_processor Eigen3::Eigen)
target_include_directories(depth_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dvl_processor src/dvl_processor.cpp)
ament_target_dependencies(dvl_processor rclcpp std_msgs geometry_msgs sensor_msgs message_filters dvl_msgs tf2 angles)
target_link_libraries(dvl_processor Eigen3::Eigen)
target_include_directories(dvl_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install

install(TARGETS
  state_aggregator
  imu_processor
  depth_processor
  dvl_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
        launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY 
        params/ 
        DESTINATION share/${PROJECT_NAME}/params)

install(DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs std_srvs geometry_msgs sensor_msgs message_filters dvl_msgs tf2_ros tf2 nav_msgs angles)

# Python nodes
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PYTHON_NODE_SOURCES} DESTINATION lib/${PROJECT_NAME})

ament_package()
