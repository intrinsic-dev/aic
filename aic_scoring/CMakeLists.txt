cmake_minimum_required(VERSION 3.8)
project(aic_scoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aic_control_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
set(GZ_MATH_VER ${gz-math_VERSION_MAJOR})
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create a library that can be linked against
add_library(${PROJECT_NAME} SHARED
  src/ScoringTier1.cc
  src/ScoringTier2.cc)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${aic_control_interfaces_TARGETS}
    ${geometry_msgs_TARGETS}
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp
    ${ros_gz_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${tf2_msgs_TARGETS}
    yaml-cpp
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Create executable that uses the library
add_executable(scoring_tier1_main src/ScoringTier1_main.cc)
target_link_libraries(scoring_tier1_main
  PRIVATE ${PROJECT_NAME})

# Create executable that uses the library
add_executable(scoring_tier2_main src/ScoringTier2_main.cc)
target_link_libraries(scoring_tier2_main
  PRIVATE ${PROJECT_NAME} )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install library and executable with export
install(TARGETS
  ${PROJECT_NAME}
  scoring_tier1_main
  scoring_tier2_main
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the targets and dependencies
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(aic_control_interfaces gz-math8 rclcpp rclcpp_components ros_gz_interfaces rosbag2_cpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_msgs yaml-cpp)

ament_package()
