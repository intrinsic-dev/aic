cmake_minimum_required(VERSION 3.8)
project(aic_flowstate_ros_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)

set(dep_pkgs
  absl
  aic_control_interfaces
  ament_index_cpp
  flowstate_ros_bridge
  pluginlib
  rclcpp
  std_srvs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

find_package(Protobuf REQUIRED)

#==================================

intrinsic_sdk_protobuf_generate(
  NAME agent_bridge
  TARGET agent_bridge_proto
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/proto/agent_bridge.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/proto
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/agent_bridge.desc"
)

add_library(robot_control_bridge SHARED
  src/robot_control_bridge.cpp
)
target_include_directories(robot_control_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(robot_control_bridge
  absl::log
  absl::time
  agent_bridge_proto
  ament_index_cpp::ament_index_cpp
  flowstate_ros_bridge::flowstate_ros_builtin_bridges
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${aic_control_interfaces_TARGETS}
  ${std_srvs_TARGETS}
)
target_compile_features(robot_control_bridge PUBLIC cxx_std_17)

ament_export_libraries(robot_control_bridge)

#===============================================================================

add_library(aic_camera_bridge SHARED
  src/aic_camera_bridge.cpp
)
target_include_directories(aic_camera_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(aic_camera_bridge
  absl::log
  absl::time
  ament_index_cpp::ament_index_cpp
  flowstate_ros_bridge::flowstate_ros_builtin_bridges
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)
target_compile_features(aic_camera_bridge PUBLIC cxx_std_17)

ament_export_libraries(aic_camera_bridge)

#===============================================================================

pluginlib_export_plugin_description_file(flowstate_ros_bridge plugins.xml)

#===============================================================================

set(SERVICE_NAME "aic_flowstate_ros_bridge")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)
add_executable(${SERVICE_NAME}_main
 src/${SERVICE_NAME}_main.cpp
)
target_include_directories(${SERVICE_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${SERVICE_NAME}_main
  flowstate_ros_bridge::flowstate_ros_bridge_component
  ${SERVICE_NAME}_protos
)

#===============================================================================

install(TARGETS
  robot_control_bridge
  agent_bridge_proto
  aic_camera_bridge
  ${SERVICE_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install proto descriptor file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config proto launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_robot_control_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
