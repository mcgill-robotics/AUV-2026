cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPS
    ament_cmake
    ament_cmake_python
    rclcpp
    rclpy
    auv_msgs
    geometry_msgs
    OpenCV
    # Required to ensure consistent Eigen3 environment variables
    eigen3_cmake_module 
    Eigen3
)

set(CPP_LIBS
    rclcpp
    auv_msgs
    geometry_msgs
    OpenCV
    eigen3_cmake_module 
    Eigen3
    ZED
    CUDA
)

# find dependencies
foreach(dep IN LISTS DEPS)
  find_package(${dep} REQUIRED)
endforeach()

# C++ object map
add_executable(object_map 
  vision/object_map.cpp
  vision/object_map/object_tracker.cpp
)
ament_target_dependencies(object_map 
    ${CPP_LIBS}
)
target_include_directories(object_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS object_map DESTINATION lib/${PROJECT_NAME})

# Python nodes
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    vision/down_image_enhancement.py
    vision/front_image_enhancement.py
    vision/image_collection.py
    DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()