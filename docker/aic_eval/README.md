## About

This contains Dockerfile for building the `aic_eval` image.

## Running

### Start `rmw_zenohd`

```bash
. /opt/ros/kilted/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Run `aic_eval`

It is recommended to use [distrobox](https://distrobox.it/#installation) for easy gui, gpu and network setup.

```bash
distrobox create -r -i ghcr.io/intrinsic-dev/aic_eval:latest aic_eval
distrobox enter -r aic_eval
/entrypoint.sh
```

## Building

```bash
docker buildx build -t ghcr.io/intrinsic-dev/aic_eval -f docker/aic_eval/Dockerfile .
```

> [!NOTE]
> Newer versions of docker uses buildx by default. For older versions, you may need to install buildx separately. See the official [docker docs](https://docs.docker.com/engine/install/) for more information.
