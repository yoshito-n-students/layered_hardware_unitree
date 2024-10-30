cmake_minimum_required(VERSION 3.8)
project(layered_hardware_unitree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

## Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(layered_hardware REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_actuator_sdk_ros REQUIRED)

## Find system dependencies
find_package(yaml-cpp REQUIRED)

## Declare a C++ library
add_library(
  layered_hardware_unitree_plugins SHARED
  src/layered_hardware_unitree_plugins.cpp)
target_include_directories(
  layered_hardware_unitree_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  layered_hardware_unitree_plugins
  controller_interface
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  unitree_actuator_sdk_ros)
target_link_libraries(
  layered_hardware_unitree_plugins
  ${YAML_CPP_LIBRARIES})

## Install resources
pluginlib_export_plugin_description_file(layered_hardware plugins.xml)
install(
  TARGETS layered_hardware_unitree_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  DIRECTORY examples
  DESTINATION share/layered_hardware_unitree)

# TODO: declare tests here

ament_package()