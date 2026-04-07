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
    tf2
    tf2_geometry_msgs
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



# ---Object Map library---
add_library(object_map_lib SHARED
    ${PROJECT_NAME}/object_map/object_tracker.cpp
    ${PROJECT_NAME}/utils/hungarian.cpp
    ${PROJECT_NAME}/utils/kalman.cpp
)



target_include_directories(object_map_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(object_map_lib ${CPP_LIBS})

# ---ROS2 nodes---
# C++ nodes
add_executable(object_map ${PROJECT_NAME}/object_map.cpp)


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
# configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
# Model files
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)

ament_package()