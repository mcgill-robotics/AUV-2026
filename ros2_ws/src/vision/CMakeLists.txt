cmake_minimum_required(VERSION 3.8)

project(vision)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

set(ROS_DEPS
    rclcpp
    auv_msgs
    std_msgs
    geometry_msgs
    vision_msgs
)
set(CPP_LIBS
    OpenCV
    eigen3_cmake_module 
    Eigen3
)

# Find Dependencies
foreach(dep IN LISTS ROS_DEPS)
  find_package(${dep} REQUIRED)
endforeach()
foreach(dep IN LISTS CPP_LIBS)
  find_package(${dep} REQUIRED)
endforeach()

# Find optional ZED SDK and CUDA dependencies
set(HAS_ZED_SDK_DEPS FALSE)

# add CMake option that will be passed by the build script
option(IS_JETSON_CI "Running on a Jetson device for CI" OFF)

# Skip ZED SDK in JETSON CI environments (no GPU hardware and cannot use stubs)
if(IS_JETSON_CI)
    message(STATUS "CI Jetson environment detected - skipping ZED SDK (no NVIDIA stubs available for Jetson ZED SDK dependencies).")
else()
    find_package(CUDAToolkit QUIET)
    find_package(ZED QUIET)
    
    if(CUDAToolkit_FOUND AND ZED_FOUND)
        message(STATUS "ZED SDK and CUDA found, building object map node with ZED SDK.")
        set(HAS_ZED_SDK_DEPS TRUE)
        # Add to CMake-based dependency list for ament to handle
        list(APPEND CPP_LIBS ZED CUDAToolkit)
    else()
        message(STATUS "ZED SDK and/or CUDA not found, building object map node without ZED SDK.")
        set(HAS_ZED_SDK_DEPS FALSE)
    endif()
endif()

# ---Object Map library---
add_library(object_map_lib SHARED
    ${PROJECT_NAME}/object_map/object_tracker.cpp
    ${PROJECT_NAME}/utils/hungarian.cpp
    ${PROJECT_NAME}/utils/kalman.cpp
)

if (HAS_ZED_SDK_DEPS)
    target_sources(object_map_lib PRIVATE ${PROJECT_NAME}/object_map/zed_detection.cpp)
    # Treat ZED SDK headers as system headers to suppress warnings from them
    target_include_directories(object_map_lib SYSTEM PUBLIC ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
endif()

target_include_directories(object_map_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(object_map_lib ${CPP_LIBS})

# ---ROS2 nodes---
# C++ nodes
add_executable(object_map ${PROJECT_NAME}/object_map.cpp)
# Add compile-time flag when ZED SDK is available
if(HAS_ZED_SDK_DEPS)
    target_compile_definitions(object_map PUBLIC HAS_ZED_SDK)
endif()

target_include_directories(object_map PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_map object_map_lib)
ament_target_dependencies(object_map ${ROS_DEPS} ${CPP_LIBS})

# Python nodes
set (PYTHON_NODE_SOURCES
    ${PROJECT_NAME}/down_image_enhancement.py
    ${PROJECT_NAME}/down_cam_object_detection.py
    ${PROJECT_NAME}/front_image_enhancement.py
    ${PROJECT_NAME}/front_cam_object_detection.py
    ${PROJECT_NAME}/image_collection.py
)

# --- Installation ---
# Object Map static library
install(TARGETS object_map_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
# Object Map node executable
install(TARGETS object_map 
    DESTINATION lib/${PROJECT_NAME}
)
# Python nodes
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PYTHON_NODE_SOURCES} DESTINATION lib/${PROJECT_NAME})
# Launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()