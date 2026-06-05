# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME switch_to_aic_controller_skill
  SOURCES src/switch_to_aic_controller_skill.proto
  TARGET switch_to_aic_controller_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET switch_to_aic_controller_skill_config
  SKILL_NAME switch_to_aic_controller_skill
  MANIFEST src/switch_to_aic_controller_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_config.pbbin"
)

install(
  FILES
    src/switch_to_aic_controller_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/switch_to_aic_controller_skill.manifest.textproto
  HEADER_FILES src/switch_to_aic_controller_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_main.cc"
)

add_executable(switch_to_aic_controller_skill_main
  src/switch_to_aic_controller_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/switch_to_aic_controller_skill_main.cc
)

target_include_directories(switch_to_aic_controller_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(switch_to_aic_controller_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  switch_to_aic_controller_skill_protos
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

add_dependencies(switch_to_aic_controller_skill_main switch_to_aic_controller_skill_config)
add_dependencies(switch_to_aic_controller_skill_main switch_to_aic_controller_skill_protos)

install(
  TARGETS switch_to_aic_controller_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
