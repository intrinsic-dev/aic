# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME stop_engine_skill
  SOURCES src/stop_engine_skill.proto
  TARGET stop_engine_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET stop_engine_skill_config
  SKILL_NAME stop_engine_skill
  MANIFEST src/stop_engine_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_config.pbbin"
)

install(
  FILES
    src/stop_engine_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/stop_engine_skill.manifest.textproto
  HEADER_FILES src/stop_engine_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_main.cc"
)

add_executable(stop_engine_skill_main
  src/stop_engine_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/stop_engine_skill_main.cc
)

target_include_directories(stop_engine_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(stop_engine_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  stop_engine_skill_protos
  rclcpp::rclcpp
  ${aic_engine_interfaces_TARGETS}
)

add_dependencies(stop_engine_skill_main stop_engine_skill_config)
add_dependencies(stop_engine_skill_main stop_engine_skill_protos)

install(
  TARGETS stop_engine_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
