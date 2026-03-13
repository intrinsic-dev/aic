cmake_minimum_required(VERSION 3.8)
project(aic_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and MJCF models
install(DIRECTORY
  launch mjcf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/add_cable_plugin.py
  scripts/view_scene.py
  scripts/load_aic_world.py
  scripts/sim_comparison_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
