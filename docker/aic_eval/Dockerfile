FROM ros:kilted-ros-base AS build

# Add `packages.osrfoundation.org` to the apt sources list:
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Build the workspace
COPY --chmod=u=rwX,go=rX --exclude=docker . /ws_aic/src/aic
SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  # undeclared deps
  apt install python3-numpy && \
  cd /ws_aic/src && \
  vcs import . < /ws_aic/src/aic/aic.repos --recursive && \
  # Install Gazebo dependencies.
  apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') && \
  cd /ws_aic && \
  # Install ROS dependencies using rosdep.
  rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev" && \
  source /opt/ros/kilted/setup.bash && \
  GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --executor sequential && \
  rm -rf /ws_aic/build /ws_aic/log /ws_aic/src

COPY --chmod=644 docker/aic_eval/zenoh_router_config.json5 /zenoh_router_config.json5
COPY --chmod=755 <<EOF /entrypoint.sh
#!/bin/bash
set -e

source /ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ZENOH_ROUTER_CONFIG_URI=/zenoh_router_config.json5 ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOHD_PID=\$!
trap "kill -SIGINT -\$ZENOHD_PID" EXIT
exec ros2 launch aic_bringup aic_gz_bringup.launch.py "\$@"
EOF

ENTRYPOINT ["/entrypoint.sh"]
