# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME tare_force_torque_sensor_skill
  SOURCES src/tare_force_torque_sensor_skill.proto
  TARGET tare_force_torque_sensor_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET tare_force_torque_sensor_skill_config
  SKILL_NAME tare_force_torque_sensor_skill
  MANIFEST src/tare_force_torque_sensor_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_config.pbbin"
)

install(
  FILES
    src/tare_force_torque_sensor_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/tare_force_torque_sensor_skill.manifest.textproto
  HEADER_FILES src/tare_force_torque_sensor_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_main.cc"
)

add_executable(tare_force_torque_sensor_skill_main
  src/tare_force_torque_sensor_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/tare_force_torque_sensor_skill_main.cc
)

target_include_directories(tare_force_torque_sensor_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(tare_force_torque_sensor_skill_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  tare_force_torque_sensor_skill_protos
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

add_dependencies(tare_force_torque_sensor_skill_main tare_force_torque_sensor_skill_config)
add_dependencies(tare_force_torque_sensor_skill_main tare_force_torque_sensor_skill_protos)

install(
  TARGETS tare_force_torque_sensor_skill_main
  DESTINATION lib/${PROJECT_NAME}
)
