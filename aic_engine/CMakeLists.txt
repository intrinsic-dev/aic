cmake_minimum_required(VERSION 3.20)
project(aic_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(aic_scoring REQUIRED)
find_package(aic_control_interfaces REQUIRED)
find_package(aic_task_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(aic_engine
  src/aic_engine.cpp
	src/main.cpp
)
target_link_libraries(aic_engine
  aic_scoring::aic_scoring
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_ros::tf2_ros
  yaml-cpp
  ${aic_control_interfaces_TARGETS}
  ${aic_task_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${simulation_interfaces_TARGETS}
  ${tf2_ros_TARGETS}
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
