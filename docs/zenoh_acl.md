## Updating zenoh router ACL

Prepare output directory

```bash
mkdir <workspace-root>/sros # or any directory you want
```

Run the evaluator

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py launch_rviz:=false gazebo_gui:=false
```

Generate SROS2 policy file

```bash
ros2 security generate_policy aic_policy.xml
```

Generate base zenoh config

```bash
# apt install ros-kilted-zenoh-security-tools
ros2 run zenoh_security_tools generate_configs -p aic_policy.xml -r ../src/aic/docker/aic_eval/zenoh_router_config.json5 -c ../src/aic/docker/aic_eval/zenoh_router_config.json5 -d 0
```

This will generate a bunch of zenoh config files, one for each node in the SROS2 policy. What you need to do now is to use the generated config files as a reference to update the [config](../docker/aic_eval/zenoh_router_config.json5) used in `aic_eval` image.

The generated config contains settings to allow outgoing publications and incoming subscriptions. But because we are puting the acl on the router with peer brokering. We actually need incoming publications and outgoing subscriptions as well.

Add the topics that you want to allow to one or more of the following

* outgoing_publications_all
* incoming_subscriptions_all
* incoming_publications_all
* outgoing_subscriptions_all
* incoming_publications_eval
* outgoing_subscriptions_eval

If you want allow the evaluator to publish a topic to the participant, put it in `outgoing_publications_all`, `incoming_subscriptions_all`, `incoming_publications_eval`, `outgoing_subscriptions_eval`.

If you want to allow the evaluator to subscribe to a publication from the participant, put it in `outgoing_publications_all`, `incoming_subscriptions_all`, `incoming_publications_all`, `outgoing_subscriptions_all`.

For services and actions, use `outgoing_publications_all`, `incoming_subscriptions_all`, `incoming_publications_all`, `outgoing_subscriptions_all` as they are bidirectional.

How it works is that the router essentially acts as a proxy for the evaluator. When you want to allow the evaluator to publish a topic, the router will *subscribe* to the topic, and *publish* it to the participant. 

So it will:

1. Receive an incoming subscription from the participant.
2. Send an outgoing subscription to the evaluator.
3. Receive an incoming publication from the evaluator.
4. Send an outgoing publication to the participant.

Limiting `incoming_publications_eval` and `outgoing_subscriptions_eval` to only the evaluator allows us to *reject* any publications from the participant side which attempts to trick the evaluator.

## Quick Testing

> [!warning]
> This is not replace a full test with `docker compose up`.

Start zenoh router

```bash
export ZENOH_ROUTER_CONFIG_URI=$(pwd)/src/aic/docker/aic_eval/zenoh_router_config.json5
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Start evaluator

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py launch_rviz:=false gazebo_gui:=false
```

Find the host docker bridge ip

```bash
$ ip link
...
6: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:95:de:3a:bd:4c brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
    inet6 fe80::95:deff:fe3a:bd4c/64 scope link 
       valid_lft forever preferred_lft forever
```

By default it should be `172.17.0.1`.

Start a container

```bash
docker run -it --rm -v="$(pwd)/src/aic/docker/acl:/acl:ro" --entrypoint=bash ghcr.io/intrinsic-dev/aic/aic_eval
# Can also use any image with ROS2 and the aic interface messages.
```

Export env vars and source AIC workspace

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
HOST_IP=172.17.0.1
export ZENOH_CONFIG_OVERRIDE=connect/endpoints='["tcp/'"$HOST_IP"':7447"];transport/auth/usrpwd/user="participant";transport/auth/usrpwd/password="CHANGE_IN_PROD";transport/auth/usrpwd/dictionary_file="/acl/credentials.txt"'
. /ws_aic/install/setup.bash
```

Test topic echo

```bash
ros2 topic echo --once /aic_controller/controller_state
```

Quickly test new changes by updating the router config and restart the router.
