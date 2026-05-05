find_package(lifecycle_msgs REQUIRED)

# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME manage_model_lifecycle_skill
  SOURCES src/manage_model_lifecycle_skill.proto
  TARGET manage_model_lifecycle_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET manage_model_lifecycle_skill_config
  SKILL_NAME manage_model_lifecycle_skill
  MANIFEST src/manage_model_lifecycle_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_config.pbbin"
)

install(
  FILES
    src/manage_model_lifecycle_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/manage_model_lifecycle_skill.manifest.textproto
  HEADER_FILES src/manage_model_lifecycle_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_main.cc"
)

add_executable(manage_model_lifecycle_skill_main
  src/manage_model_lifecycle_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/manage_model_lifecycle_skill_main.cc
)

target_include_directories(manage_model_lifecycle_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(manage_model_lifecycle_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  manage_model_lifecycle_skill_protos
  rclcpp::rclcpp
  ${lifecycle_msgs_TARGETS}
)

add_dependencies(manage_model_lifecycle_skill_main manage_model_lifecycle_skill_config)
add_dependencies(manage_model_lifecycle_skill_main manage_model_lifecycle_skill_protos)

install(
  TARGETS manage_model_lifecycle_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
