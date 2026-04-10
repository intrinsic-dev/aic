FROM ros:kilted-ros-base

# Add `packages.osrfoundation.org` to the apt sources list:
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install dependencies
RUN mkdir -p /ws_aic/src/aic
COPY --chmod=644 aic.repos /ws_aic/src/aic/
COPY --chmod=u=rwX,go=rX --parents **/package.xml /ws_aic/src/aic
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  # undeclared deps
  apt -y install python3-numpy python3-toml && \
  cd /ws_aic/src && \
  vcs import . < /ws_aic/src/aic/aic.repos --recursive && \
  # Install Gazebo dependencies.
  apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') && \
  cd /ws_aic && \
  # Install ROS dependencies using rosdep.
  rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev"

# Build the workspace
COPY --chmod=u=rwX,go=rX --exclude=docker . /ws_aic/src/aic
SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/ws_aic/build,sharing=locked \
  cd /ws_aic \
  && . /opt/ros/kilted/setup.bash \
  && GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --executor sequential \
  && rm -rf /ws_aic/log /ws_aic/src

COPY --chmod=644 docker/aic_eval/aic_zenoh_config.json5 /aic_zenoh_config.json5
COPY --chmod=755 <<"EOF" /entrypoint.sh
#!/bin/bash
set -e

. /ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

should_enable_acl() {
  [[ "$AIC_ENABLE_ACL" == "true" || "$AIC_ENABLE_ACL" == "1" ]]
}

# prepare the credentials file
if should_enable_acl; then
  if [[ ! (-n "$AIC_EVAL_PASSWD" && -n "$AIC_MODEL_PASSWD") ]]; then
    echo "AIC_EVAL_PASSWD, AIC_MODEL_PASSWD must be provided"
    exit 1
  fi
  echo "eval:$AIC_EVAL_PASSWD" >> /credentials.txt
  echo "model:$AIC_MODEL_PASSWD" >> /credentials.txt
fi

# Both the router and peer processes will use the same config, regardless
# of whether the ACL is enabled.
export ZENOH_ROUTER_CONFIG_URI=/aic_zenoh_config.json5

# start the model router
ZENOH_CONFIG_OVERRIDE='mode="router"'
ZENOH_CONFIG_OVERRIDE+=';listen/endpoints=["tcp/[::]:7447"]'
ZENOH_CONFIG_OVERRIDE+=';connect/endpoints=[]'
ZENOH_CONFIG_OVERRIDE+=';routing/router/peers_failover_brokering=true'
ZENOH_CONFIG_OVERRIDE+=';transport/shared_memory/enabled=false'
if should_enable_acl; then
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="eval"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_EVAL_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
fi
export ZENOH_CONFIG_OVERRIDE
echo "ZENOH_CONFIG_OVERRIDE_ROUTER=$ZENOH_CONFIG_OVERRIDE"
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOH_ROUTER_PID=$!

trap "kill -SIGINT -$ZENOH_EVAL_ROUTER_PID -$ZENOH_ROUTER_PID" EXIT

# start the evaluator
ZENOH_CONFIG_OVERRIDE=';transport/shared_memory/enabled=false'
if should_enable_acl; then
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="eval"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_EVAL_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
fi
export ZENOH_CONFIG_OVERRIDE
echo "ZENOH_CONFIG_OVERRIDE_PEER=$ZENOH_CONFIG_OVERRIDE"
exec ros2 launch aic_bringup aic_gz_bringup.launch.py "$@"
EOF

# Contary to the name, `/entrypoint.sh` is not actually the entrypoint.
# The name is kept to maintain compatibility as the old entrypoint script does not have ACL.
ENTRYPOINT ["/entrypoint.sh"]
