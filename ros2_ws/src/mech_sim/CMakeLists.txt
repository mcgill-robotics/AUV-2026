cmake_minimum_required(VERSION 3.10)
project(mech_sim)

# Treat warnings seriously in dev; relax if needed.
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(auv_msgs)

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_FILES
  src/dynamics_core.cpp
  src/auv_sim_node.cpp
)

add_executable(auv_sim_node ${SRC_FILES})
target_link_libraries(auv_sim_node Eigen3::Eigen)
ament_target_dependencies(auv_sim_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  auv_msgs
)

# Install
install(TARGETS auv_sim_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

ament_package()
