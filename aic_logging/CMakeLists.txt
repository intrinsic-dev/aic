cmake_minimum_required(VERSION 3.8)
project(aic_logging)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS record.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    recording_config.yaml
    zenoh_router_config.json5
    zenoh_session_config.json5
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
