FROM ros:kilted-ros-base AS build

# Build the workspace
COPY aic_model /ws_aic/src/aic/aic_model
COPY aic_interfaces /ws_aic/src/aic/aic_interfaces
SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  # undeclared deps
  apt install python3-numpy && \
  cd /ws_aic && \
  # Install ROS dependencies using rosdep.
  rosdep install --ignore-src --rosdistro kilted -yr --from-paths src && \
  source /opt/ros/kilted/setup.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --executor sequential --packages-up-to aic_model

COPY --chmod=+x <<EOF /entrypoint.sh
#!/bin/bash
set -e

source /ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOHD_PID=\$!
trap "kill -SIGINT -\$ZENOHD_PID" EXIT
ros2 run aic_model model_node
EOF

ENTRYPOINT ["/entrypoint.sh"]
