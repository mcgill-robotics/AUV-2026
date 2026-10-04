cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/hardcoded_map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
