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

# --- Main Node: Movella Republisher ---
add_executable(imu 
        src/movella_republish.cpp
        src/imuprocessing2.cpp)

ament_target_dependencies(imu 
        rclcpp 
        std_msgs 
        geometry_msgs 
        sensor_msgs  
        message_filters)

target_include_directories(imu PUBLIC include/)

# --- Test Node: IMU Processor Test ---
add_executable(imu_test_node 
        src/imu_test_node.cpp 
        src/imuprocessing2.cpp)

ament_target_dependencies(imu_test_node 
        rclcpp 
        sensor_msgs 
        Eigen3)

# --- Installation ---
# Install both executables
install(TARGETS
        imu
        imu_test_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch/
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
        include/
        DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()