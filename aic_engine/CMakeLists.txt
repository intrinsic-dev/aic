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

find_package(aic_scoring REQUIRED)
find_package(aic_control_interfaces REQUIRED)
find_package(aic_engine_interfaces REQUIRED)
find_package(aic_task_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(aic_engine
  src/aic_engine.cpp
	src/main.cpp
)
target_link_libraries(aic_engine
  aic_scoring::aic_scoring
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  yaml-cpp
  ${aic_control_interfaces_TARGETS}
  ${aic_engine_interfaces_TARGETS}
  ${aic_task_interfaces_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

install(
  TARGETS aic_engine
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
