cmake_minimum_required(VERSION 3.16)
project(g1_control_cpp LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
  unitree_hg
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  g1_control_cpp
  SHARED
  src/g1_arm_sdk.cpp
  src/motor_crc_hg.cpp
)

target_compile_features(g1_control_cpp PUBLIC cxx_std_17)
target_include_directories(g1_control_cpp PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/g1_control_cpp>
$<INSTALL_INTERFACE:include/common>
)
ament_target_dependencies(
  g1_control_cpp PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface g1_control_cpp.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/g1_control_cpp
)

install(
  DIRECTORY launch
  DESTINATION share/g1_control_cpp
)

install(TARGETS g1_control_cpp
  EXPORT export_g1_control_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # find_package(ament_cmake_pytest REQUIRED)

  # ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
  # ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
  # ament_add_pytest_test(run_example_7_launch test/test_r6bot_controller_launch.py)
endif()

## EXPORTS
ament_export_targets(export_g1_control_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()
