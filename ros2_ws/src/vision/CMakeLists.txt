cmake_minimum_required(VERSION 3.8)

project(vision)

set(CMAKE_CXX_STANDARD 17)

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
    std_msgs
    sensor_msgs
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
)

# find dependencies
foreach(dep IN LISTS DEPS)
  find_package(${dep} REQUIRED)
endforeach()

set (OBJECT_MAP_LIB_SOURCES 
    ${PROJECT_NAME}/object_map/object_tracker.cpp
    ${PROJECT_NAME}/utils/hungarian.cpp
    ${PROJECT_NAME}/utils/kalman.cpp
)

set (CPP_NODE_SOURCES
    ${PROJECT_NAME}/object_map.cpp
)

set (PYTHON_NODE_SOURCES
    ${PROJECT_NAME}/down_image_enhancement.py
    ${PROJECT_NAME}/front_image_enhancement.py
    ${PROJECT_NAME}/image_collection.py
)

find_package(CUDAToolkit QUIET)
find_package(ZED QUIET)

set(HAS_ZED_SDK_DEPS FALSE)

if(CUDAToolkit_FOUND AND ZED_FOUND)
    message(STATUS "ZED SDK and CUDA found, building object map node with ZED SDK.")
    set (HAS_ZED_SDK_DEPS TRUE)
else()
    message(STATUS "ZED SDK and/or CUDA not found, building object map node without ZED SDK.")
endif()

# by default, assume we have ZED SDK, otherwise remove zed detection
if(HAS_ZED_SDK_DEPS)
    list(APPEND OBJECT_MAP_LIB_SOURCES ${PROJECT_NAME}/object_map/zed_detection.cpp)
endif()

# C++ object map
add_executable(object_map ${CPP_NODE_SOURCES})
ament_target_dependencies(object_map ${CPP_LIBS})

target_include_directories(object_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# create library to build node
add_library(object_map_lib SHARED ${OBJECT_MAP_LIB_SOURCES})

ament_target_dependencies(object_map_lib ${CPP_LIBS})


target_include_directories(object_map_lib PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


if(HAS_ZED_SDK_DEPS)
    # Define a compile-time flag to indicate ZED SDK is available
    target_compile_definitions(object_map_lib PRIVATE HAS_ZED_SDK)
    target_link_libraries(object_map_lib ${ZED_LIBRARIES} ${CUDAToolkit_LIBRARIES})
    # Treat external headers as system to suppress warnings
    target_include_directories(object_map_lib SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDAToolkit_INCLUDE_DIRS})
endif()

target_link_libraries(object_map object_map_lib)

install(TARGETS object_map_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS object_map 
    DESTINATION lib/${PROJECT_NAME}
)

# IPC Test Node
add_executable(IPC_test ${PROJECT_NAME}/IPC_test.cpp)
ament_target_dependencies(IPC_test rclcpp sensor_msgs)

install(TARGETS IPC_test 
    DESTINATION lib/${PROJECT_NAME}
)

# Python nodes
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PYTHON_NODE_SOURCES} DESTINATION lib/${PROJECT_NAME})

# Launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)


ament_package()