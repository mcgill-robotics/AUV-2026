cmake_minimum_required(VERSION 3.5)
project(sensors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 17)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(imu sensors/movella_republish.cpp)
ament_target_dependencies(imu rclcpp std_msgs geometry_msgs sensor_msgs  message_filters)

install(TARGETS
	 imu
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	 launch
	DESTINATION share/${PROJECT_NAME})

ament_package()
