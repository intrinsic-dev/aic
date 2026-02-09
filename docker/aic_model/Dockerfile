FROM ros:kilted-ros-core AS build

# Install pixi
RUN apt update && apt install -y git && curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/root/.pixi/bin:$PATH"

# Build the workspace
COPY aic_example_policies /ws_aic/src/aic/aic_example_policies
COPY aic_model /ws_aic/src/aic/aic_model
COPY aic_interfaces /ws_aic/src/aic/aic_interfaces
COPY aic_utils /ws_aic/src/aic/aic_utils
COPY pixi.toml pixi.lock /ws_aic/src/aic

# Add other local dependencies

SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/root/.cache/rattler/cache --mount=type=cache,target=/ws_aic/src/aic/.pixi/build \
    cd /ws_aic/src/aic && pixi install --frozen

WORKDIR /ws_aic/src/aic
ENTRYPOINT [ \
    "pixi", "run", "--as-is", \
    "ros2", "run", "aic_model", "aic_model" \
]
