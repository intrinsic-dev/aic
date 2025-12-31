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
docker network create --internal aic
distrobox create -r -i ghcr.io/intrinsic-dev/aic_eval:latest --unshare-all -a --network=aic aic_eval
distrobox enter -r aic_eval
/entrypoint.sh
```

> [!TIP]
> You may omit `--unshare-all -a --network=aic` flags to let the container use the host network. Note that during evaluation, there will be no external network access so it is recommended to test with a private network.

## Building

```bash
docker buildx build -t ghcr.io/intrinsic-dev/aic_eval -f docker/aic_eval/Dockerfile .
```

> [!NOTE]
> Newer versions of docker uses buildx by default. For older versions, you may need to install buildx separately. See the official [docker docs](https://docs.docker.com/engine/install/) for more information.
