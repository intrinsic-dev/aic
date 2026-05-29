cmake_minimum_required(VERSION 3.8)
project(aic_flowstate_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(absl REQUIRED)
find_package(aic_task_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)


add_subdirectory(insert_cable_skill)
add_subdirectory(tare_force_torque_sensor_skill)

ament_package()
