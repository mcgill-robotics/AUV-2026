cmake_minimum_required(VERSION 3.5)
project(sensors)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# --- Main Node: Sensor Node ---
add_executable(sensor_node
        src/Sensor_Node.cpp
        src/Imu_processor.cpp
        src/depth_processor.cpp)

ament_target_dependencies(sensor_node
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        message_filters
        Eigen3)

target_include_directories(sensor_node PUBLIC include/)

# --- Installation ---
# Install both executables
install(TARGETS
        sensor_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch/
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
        include/
        DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
