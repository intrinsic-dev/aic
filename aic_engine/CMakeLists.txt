cmake_minimum_required(VERSION 3.20)
project(aic_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(aic_scoring REQUIRED)
find_package(aic_task_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(aic_engine_component SHARED
  src/aic_engine.cpp
)
target_link_libraries(aic_engine_component
  aic_scoring::aic_scoring
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  yaml-cpp
  ${aic_task_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${simulation_interfaces_TARGETS}
)
rclcpp_components_register_node(aic_engine_component
  PLUGIN "aic::Engine"
  EXECUTABLE aic_engine
  EXECUTOR SingleThreadedExecutor
)

install(
  TARGETS aic_engine_component
  EXPORT aic_engine_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
