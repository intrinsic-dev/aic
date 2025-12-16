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

set(protobuf_MODULE_COMPATIBLE TRUE)

message(STATUS "Compiling against Gazebo Ionic")

file(GLOB gz_msgs_proto_files
  ${CMAKE_CURRENT_SOURCE_DIR}/src/proto/aic_gazebo/msgs/*.proto
)

get_target_property(msgs_desc_file
  gz-msgs11::gz-msgs11 GZ_MSGS_DESC_FILE)

gz_msgs_generate_messages_impl(
  MSGS_GEN_SCRIPT
    ${gz-msgs11_PROTO_GENERATOR_SCRIPT}
  FACTORY_GEN_SCRIPT
    ${gz-msgs11_FACTORY_GENERATOR_SCRIPT}
  GZ_PROTOC_PLUGIN
    ${gz-msgs11_PROTO_GENERATOR_PLUGIN}
  INPUT_PROTOS
    ${gz_msgs_proto_files}
  PROTO_PACKAGE
    "aic_gazebo.msgs"
  PROTO_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proto
  OUTPUT_DIRECTORY
    ${PROJECT_BINARY_DIR}/gz_msgs_gen
  OUTPUT_SOURCES
    msgs_sources
  OUTPUT_HEADERS
    msgs_headers
  OUTPUT_DETAIL_HEADERS
    msgs_detail_headers
  OUTPUT_PYTHON
    msgs_python
  DEPENDENCY_DESCRIPTIONS ${msgs_desc_file}
)

message(PROTO_PATH="${gz-msgs11_PROTO_PATH}")
message(FACTORY_GEN_SCRIPT="${gz-msgs11_FACTORY_GENERATOR_SCRIPT}")
message(GZ_PROTOC_PLUGIN="${gz-msgs11_PROTO_GENERATOR_PLUGIN}")
message(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
message(INPUT_PROTOS="${gz_msgs_proto_files}")
message(DEPENDENCY_DESCRIPTIONS="${msgs_desc_file}")


add_library(ScoringPlugin
  SHARED
  src/ScoringPlugin.cc
  ${msgs_sources}
)

#protobuf_generate(TARGET ScoringPlugin
#  IMPORT_DIRS ${gz-msgs_INCLUDE_DIRS})

target_include_directories(
  ScoringPlugin PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/gz_msgs_gen>
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

target_link_libraries(ScoringPlugin PRIVATE
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
#  ${PROTO_SRC}
  protobuf::libprotobuf)

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
