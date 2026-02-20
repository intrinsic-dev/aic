# Challenge Rules

## aic_model

- The submission container must start a process with a ROS 2 Lifecycle node with name `aic_model`.
- The `aic_model` node must exhibit the following behavior:
  - Start in `unconfigured` lifecycle state.
	  - In `unconfigured` state, no topics should be published by node, especially to command the robot.
	- A transition to `configured` state must succeed within a timeout of 60s. Loading of any models should occur within this window.
	  - In `configured` state, no topics should be published by node, especially to command the robot.
		- In `configured` state, the node should reject any goals requests sent to `/insert_cable` Action Server.
	- A transition to `active` state must succeed within a timeout of 60s.
		- In `active` state, the node should accept any goal request sent to `/insert_cable` and the goals should be cancellable.
        - Goal requests should complete within the `time_limit` field of the requested [Task](../aic_interfaces/aic_task_interfaces/msg/Task.msg). When running evaluation, this field will be set according to the `time_limit` specified in the [config](https://github.com/intrinsic-dev/aic/blob/main/aic_engine/config/sample_config.yaml) passed to the `aic_engine`.
	- A `deactivate` transition request should transition the node back to `configured` state successfully within a timeout of 60s.
	- A `cleanup` transition request should transition the node back to `unconfigured` state successfully within a timeout of 60s.
	- A `shutdown` transition request must succeed within a timeout of 60s.
		- In `shutdown` state, no topics should be published by node, especially to command the robot.
		- In the `shutdown` state, no robot command publishers should be present in the graph.
