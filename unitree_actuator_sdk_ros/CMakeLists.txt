cmake_minimum_required(VERSION 3.8)
project(unitree_actuator_sdk_ros)

# find dependencies
find_package(ament_cmake REQUIRED)

# import original library
# (we do not want to export this library since this is not in a namespace)
add_library(UnitreeMotorSDK SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set_target_properties(
    UnitreeMotorSDK PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/unitree_actuator_sdk/lib/libUnitreeMotorSDK_Arm64.so)
else()
  set_target_properties(
    UnitreeMotorSDK PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/unitree_actuator_sdk/lib/libUnitreeMotorSDK_Linux64.so)
endif()

# declare a wrapper library in a namespace
add_library(
  unitree_actuator_sdk_ros SHARED
  src/unitree_actuator_sdk_ros.cpp)
target_include_directories(
  unitree_actuator_sdk_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  unitree_actuator_sdk_ros
  UnitreeMotorSDK)

# install headers & library
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS unitree_actuator_sdk_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  IMPORTED_RUNTIME_ARTIFACTS UnitreeMotorSDK
  DESTINATION lib)

# export headers & libraries to packages which depend on this package
ament_export_include_directories(include)
ament_export_libraries(unitree_actuator_sdk_ros)

# TODO: declare tests here

ament_package()