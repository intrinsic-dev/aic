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

Start eval router

```bash
. src/aic/docker/acl/zenoh_config_eval_router.sh
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Start evaluator

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py launch_rviz:=false gazebo_gui:=false
```

Start model router

```bash
. src/aic/docker/acl/zenoh_config_model_router.sh
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Test with ros2 cli

```bash
. src/aic/docker/acl/zenoh_config_model.sh
ros2 topic echo --no-daemon --once /aic_controller/controller_state
```
