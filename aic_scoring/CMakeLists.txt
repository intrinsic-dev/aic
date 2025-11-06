cmake_minimum_required(VERSION 3.8)
project(aic_scoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(scoring_tier1 src/ScoringTier1.cc)
target_link_libraries(scoring_tier1 PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS})

target_include_directories(
  scoring_tier1 PRIVATE include
)

install(TARGETS
  scoring_tier1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
