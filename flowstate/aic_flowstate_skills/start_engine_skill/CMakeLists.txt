# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME start_engine_skill
  SOURCES src/start_engine_skill.proto
  TARGET start_engine_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET start_engine_skill_config
  SKILL_NAME start_engine_skill
  MANIFEST src/start_engine_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_config.pbbin"
)

install(
  FILES
    src/start_engine_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/start_engine_skill.manifest.textproto
  HEADER_FILES src/start_engine_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_main.cc"
)

add_executable(start_engine_skill_main
  src/start_engine_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/start_engine_skill_main.cc
)

target_include_directories(start_engine_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(start_engine_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  start_engine_skill_protos
  rclcpp::rclcpp
  ${aic_engine_interfaces_TARGETS}
)

add_dependencies(start_engine_skill_main start_engine_skill_config)
add_dependencies(start_engine_skill_main start_engine_skill_protos)

install(
  TARGETS start_engine_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
