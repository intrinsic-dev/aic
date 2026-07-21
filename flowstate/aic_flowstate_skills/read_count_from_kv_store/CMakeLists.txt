# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME read_count_from_kv_store
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/read_count_from_kv_store.proto"
  TARGET read_count_from_kv_store_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET read_count_from_kv_store_config
  SKILL_NAME read_count_from_kv_store
  MANIFEST src/read_count_from_kv_store.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_config.pbbin"
)

install(
  FILES
    src/read_count_from_kv_store.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/read_count_from_kv_store.manifest.textproto
  HEADER_FILES src/read_count_from_kv_store.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_main.cc"
)

add_executable(read_count_from_kv_store_main
  src/read_count_from_kv_store.cc
  ${CMAKE_CURRENT_BINARY_DIR}/read_count_from_kv_store_main.cc
)

target_include_directories(read_count_from_kv_store_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src
)

target_link_libraries(read_count_from_kv_store_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  read_count_from_kv_store_protos
)

add_dependencies(read_count_from_kv_store_main read_count_from_kv_store_config)
add_dependencies(read_count_from_kv_store_main read_count_from_kv_store_protos)

install(
  TARGETS read_count_from_kv_store_main
  DESTINATION lib/${PROJECT_NAME}
)
