# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME lifecycle_transition_skill
  SOURCES src/lifecycle_transition_skill.proto
  TARGET lifecycle_transition_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET lifecycle_transition_skill_config
  SKILL_NAME lifecycle_transition_skill
  MANIFEST src/lifecycle_transition_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_config.pbbin"
)

install(
  FILES
    src/lifecycle_transition_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/lifecycle_transition_skill.manifest.textproto
  HEADER_FILES src/lifecycle_transition_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_main.cc"
)

find_package(lifecycle_msgs REQUIRED)

add_executable(lifecycle_transition_skill_main
  src/lifecycle_transition_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/lifecycle_transition_skill_main.cc
)

target_include_directories(lifecycle_transition_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(lifecycle_transition_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  lifecycle_transition_skill_protos
)

ament_target_dependencies(lifecycle_transition_skill_main
  rclcpp
  lifecycle_msgs
)

add_dependencies(lifecycle_transition_skill_main lifecycle_transition_skill_config)
add_dependencies(lifecycle_transition_skill_main lifecycle_transition_skill_protos)

install(
  TARGETS lifecycle_transition_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
