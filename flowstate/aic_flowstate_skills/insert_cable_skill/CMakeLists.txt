# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME insert_cable_skill
  SOURCES src/insert_cable_skill.proto
  TARGET insert_cable_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/insert_cable_skill.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET insert_cable_skill_config
  SKILL_NAME insert_cable_skill
  MANIFEST src/insert_cable_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/insert_cable_skill.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/insert_cable_skill_config.pbbin"
)

install(
  FILES
    src/insert_cable_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/insert_cable_skill_config.pbbin"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/insert_cable_skill.manifest.textproto
  HEADER_FILES src/insert_cable_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/insert_cable_skill_main.cc"
)

add_executable(insert_cable_skill_main
  src/insert_cable_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/insert_cable_skill_main.cc
)

target_include_directories(insert_cable_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(insert_cable_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  insert_cable_skill_protos
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${aic_task_interfaces_TARGETS}
)

add_dependencies(insert_cable_skill_main insert_cable_skill_config)
add_dependencies(insert_cable_skill_main insert_cable_skill_protos)

install(
  TARGETS insert_cable_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
