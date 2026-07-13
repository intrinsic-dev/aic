# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME write_to_kv_store
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/write_to_kv_store.proto"
  TARGET write_to_kv_store_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET write_to_kv_store_config
  SKILL_NAME write_to_kv_store
  MANIFEST src/write_to_kv_store.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_config.pbbin"
)

install(
  FILES
    src/write_to_kv_store.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/write_to_kv_store.manifest.textproto
  HEADER_FILES src/write_to_kv_store.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_main.cc"
)

add_executable(write_to_kv_store_main
  src/write_to_kv_store.cc
  ${CMAKE_CURRENT_BINARY_DIR}/write_to_kv_store_main.cc
)

target_include_directories(write_to_kv_store_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(write_to_kv_store_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  write_to_kv_store_protos
)

add_dependencies(write_to_kv_store_main write_to_kv_store_config)
add_dependencies(write_to_kv_store_main write_to_kv_store_protos)

install(
  TARGETS write_to_kv_store_main
  DESTINATION lib/${PROJECT_NAME}
)
