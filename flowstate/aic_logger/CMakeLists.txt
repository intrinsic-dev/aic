cmake_minimum_required(VERSION 3.8)
project(aic_logger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)
find_package(aic_logger_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(absl REQUIRED)
find_package(Protobuf REQUIRED)

# Intrinsic Flowstate service manifest generation
set(SERVICE_NAME "aic_logger")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)

# Single executable binary combining node logic and main entrypoint
add_executable(${SERVICE_NAME}_main
  src/aic_logger_node.cpp
  src/aic_logger_main.cpp
)
target_include_directories(${SERVICE_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${SERVICE_NAME}_main
  absl::log
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_transport::rosbag2_transport
  rosbag2_storage::rosbag2_storage
  aic_logger_interfaces::aic_logger_interfaces__rosidl_typesupport_cpp
  ${SERVICE_NAME}_protos
)

# Installation targets
install(TARGETS
  ${SERVICE_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install proto descriptor file for inbuild bundling
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ${SERVICE_NAME}.manifest.textproto
  ${SERVICE_NAME}_default_config.pbtxt
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_transport rosbag2_storage aic_logger_interfaces)

ament_package()
