cmake_minimum_required(VERSION 3.20)
project(aic_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)

find_package(aic_scoring REQUIRED)
find_package(aic_control_interfaces REQUIRED)
find_package(aic_engine_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${PROJECT_NAME}
  MANIFEST ${PROJECT_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${PROJECT_NAME}.proto
  DEFAULT_CONFIGURATION ${PROJECT_NAME}_default_config.pbtxt
  PROTOS_TARGET ${PROJECT_NAME}_protos
)

add_executable(aic_engine
  src/aic_engine.cpp
  src/main.cpp
)
target_include_directories(aic_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(aic_engine
  aic_scoring::aic_scoring
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  yaml-cpp::yaml-cpp
  ${aic_control_interfaces_TARGETS}
  ${aic_engine_interfaces_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${PROJECT_NAME}_protos
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
)

install(
  TARGETS aic_engine
  DESTINATION lib/${PROJECT_NAME}
)

# Install proto descriptor file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

# Install directories.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
