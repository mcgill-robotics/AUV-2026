cmake_minimum_required(VERSION 3.8)
project(auv_dashboard)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install foxglove layout files
install(DIRECTORY foxglove/
  DESTINATION share/${PROJECT_NAME}/foxglove
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
