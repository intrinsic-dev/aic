# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME restart_control_bridge_skill
  SOURCES src/restart_control_bridge_skill.proto
  TARGET restart_control_bridge_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET restart_control_bridge_skill_config
  SKILL_NAME restart_control_bridge_skill
  MANIFEST src/restart_control_bridge_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_config.pbbin"
)

install(
  FILES
    src/restart_control_bridge_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/restart_control_bridge_skill.manifest.textproto
  HEADER_FILES src/restart_control_bridge_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_main.cc"
)

add_executable(restart_control_bridge_skill_main
  src/restart_control_bridge_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/restart_control_bridge_skill_main.cc
)

target_include_directories(restart_control_bridge_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(restart_control_bridge_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  restart_control_bridge_skill_protos
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

add_dependencies(restart_control_bridge_skill_main restart_control_bridge_skill_config)
add_dependencies(restart_control_bridge_skill_main restart_control_bridge_skill_protos)

install(
  TARGETS restart_control_bridge_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
