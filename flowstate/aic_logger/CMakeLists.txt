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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(absl REQUIRED)
find_package(Protobuf REQUIRED)

# Generate ROS 2 service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
)

# Define aic_logger_node library
add_library(aic_logger_node SHARED
  src/aic_logger_node.cpp
)
target_include_directories(aic_logger_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(aic_logger_node
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_transport::rosbag2_transport
  rosbag2_storage::rosbag2_storage
)

# Link generated ROS service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aic_logger_node ${cpp_typesupport_target})

# Intrinsic Flowstate service manifest generation
set(SERVICE_NAME "aic_logger")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)

add_executable(${SERVICE_NAME}_main
  src/aic_logger_main.cpp
)
target_include_directories(${SERVICE_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${SERVICE_NAME}_main
  aic_logger_node
  absl::log
  ${SERVICE_NAME}_protos
)

# Installation targets
install(TARGETS
  aic_logger_node
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
ament_export_libraries(aic_logger_node)
ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_transport rosbag2_storage)

ament_package()
