cmake_minimum_required(VERSION 3.5)
project(g1_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()