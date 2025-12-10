cmake_minimum_required(VERSION 3.8)
project(aic_scoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create a library that can be linked against
add_library(${PROJECT_NAME} SHARED src/ScoringTier1.cc)
target_link_libraries(${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} yaml-cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Create executable that uses the library
add_executable(scoring_tier1 src/ScoringTier1.cc)
target_link_libraries(scoring_tier1
  PRIVATE ${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install library and executable with export
install(TARGETS
  ${PROJECT_NAME}
  scoring_tier1
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
ament_export_dependencies(rclcpp rclcpp_components std_msgs yaml-cpp)

ament_package()
