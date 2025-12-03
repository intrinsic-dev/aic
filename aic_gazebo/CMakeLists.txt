cmake_minimum_required(VERSION 3.8)
project(aic_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Ionic
find_package(gz_cmake_vendor REQUIRED)
find_package(gz-cmake REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin_VERSION_MAJOR})

find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common_VERSION_MAJOR})

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Ionic")

add_library(ScoringPlugin
  SHARED
  src/ScoringPlugin.cc
)

target_include_directories(
  ScoringPlugin PRIVATE include
)

target_link_libraries(ScoringPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS ScoringPlugin
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
