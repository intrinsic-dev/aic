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

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
set(GZ_MSGS_VER ${gz-msgs_VERSION_MAJOR})

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})

find_package(Protobuf REQUIRED)

find_package(aic_engine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

message(STATUS "Compiling against Gazebo Ionic")

add_library(ScoringPlugin
  SHARED
  src/ScoringPlugin.cc
  proto/scoring.proto
)

protobuf_generate(TARGET ScoringPlugin
  IMPORT_DIRS "${gz-msgs11_PROTO_PATH}/../gz/gz-msgs${GZ_MSGS_VER}/protos")

target_include_directories(
  ScoringPlugin PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(ScoringPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  ${PROTO_SRC}
  protobuf::libprotobuf)

install(
  TARGETS ScoringPlugin
  DESTINATION lib/${PROJECT_NAME}
)

add_library(CablePlugin
  SHARED
  src/CablePlugin.cc
)

target_link_libraries(CablePlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS CablePlugin
  DESTINATION lib/${PROJECT_NAME}
)

add_library(CableModeratorPlugin
  SHARED
  src/CableModeratorPlugin.cc
)

target_link_libraries(CableModeratorPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS CableModeratorPlugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS CableModeratorPlugin
  DESTINATION lib/${PROJECT_NAME}
)

add_library(OffLimitContactsPlugin
  SHARED
  src/OffLimitContactsPlugin.cc
)

target_link_libraries(OffLimitContactsPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS OffLimitContactsPlugin
  DESTINATION lib/${PROJECT_NAME}
)

add_library(ResetJointsPlugin
  SHARED
  src/ResetJointsPlugin.cc
)

target_link_libraries(ResetJointsPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  rclcpp::rclcpp
  ${aic_engine_interfaces_TARGETS}
)

install(
  TARGETS ResetJointsPlugin
  DESTINATION lib/${PROJECT_NAME}
)

add_library(WorldSdfGeneratorPlugin
  SHARED
  src/WorldSdfGeneratorPlugin.cc
)

target_link_libraries(WorldSdfGeneratorPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS WorldSdfGeneratorPlugin
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
