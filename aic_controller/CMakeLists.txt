cmake_minimum_required(VERSION 3.20)
project(aic_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
set(DEP_PKGS
  aic_control_interfaces
  control_msgs
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  joint_limits
  kinematics_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_eigen
  trajectory_msgs
)
foreach(pkg ${DEP_PKGS})
  find_package(${pkg} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)

generate_parameter_library(aic_controller_parameters
  src/aic_controller_parameters.yaml
)

add_library(aic_controller SHARED
  src/aic_controller.cpp
	src/cartesian_state.cpp
  src/actions/cartesian_impedance_action.cpp
)
target_compile_features(aic_controller PUBLIC cxx_std_17)
target_include_directories(aic_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/aic_controller>
)

target_link_libraries(aic_controller PUBLIC
  aic_controller_parameters
  controller_interface::controller_interface
  Eigen3::Eigen
  hardware_interface::hardware_interface
  joint_limits::joint_limits
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  tf2_eigen::tf2_eigen
  ${aic_control_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${control_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface aic_controller_plugin.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(aic_controller PRIVATE "AIC_CONTROLLERS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS aic_controller aic_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)
ament_export_dependencies(${DEP_PKGS})
ament_package()
