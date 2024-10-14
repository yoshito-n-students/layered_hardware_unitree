cmake_minimum_required(VERSION 3.8)
project(layered_hardware_unitree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(layered_hardware REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/lib)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

## Declare a C++ library
add_library(
  layered_hardware_unitree_plugins SHARED
  src/layered_hardware_unitree_plugins.cpp)
target_include_directories(layered_hardware_unitree_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
  layered_hardware_unitree_plugins
  ${EXTRA_LIBS}
)
ament_target_dependencies(
  layered_hardware_unitree_plugins
  controller_interface
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  yaml_cpp_vendor)

## Declare a C++ executable

pluginlib_export_plugin_description_file(layered_hardware plugins.xml)

install(
  TARGETS layered_hardware_unitree_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
    DIRECTORY config launch urdf
    DESTINATION share/layered_hardware_unitree)

# TODO: declare tests here

ament_package()