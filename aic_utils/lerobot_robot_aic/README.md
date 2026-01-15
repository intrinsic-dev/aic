# lerobot_robot_aic

This package contains a [LeRobot](https://huggingface.co/lerobot) interface for the AIC robot.

## Usage

This describe some of the things you can do with LeRobot, for more information, see the official [LeRobot docs](https://huggingface.co/docs/lerobot/en/index).

The LeRobot driver is installed in a [pixi](https://prefix.dev/tools/pixi) workspace. In general, you can prefix a command with `pixi run` or enter the environment with `pixi shell`.

### Teleoperating

```bash
cd ~/ws_aic/src/aic
pixi run lerobot-teleoperate \
  --robot.type=aic_ros2_control --robot.id=aic \
  --teleop.type=aic_keyboard --teleop.id=aic \
  --display_data=true
```

Key mapping

| Key | Joint          |
| --- | -------------- |
| q   | -shoulder_pan  |
| a   | +shoulder_pan  |
| w   | -shoulder_lift |
| s   | +shoulder_lift |
| e   | -elbow         |
| d   | +elbow         |
| r   | -wrist_1       |
| f   | +wrist_1       |
| t   | -wrist_2       |
| g   | +wrist_2       |
| y   | -wrist_3       |
| h   | +wrist_3       |
| o   | -gripper       |
| l   | +gripper       |

### Recording Training Data

```bash
cd ~/ws_aic/src/aic
pixi run lerobot-record \
  --robot.type=aic_ros2_control --robot.id=aic \
  --teleop.type=aic_keyboard --teleop.id=aic \
  --dataset.repo_id=<hf-repo> \
  --dataset.single_task=<task-prompt> \
  --dataset.push_to_hub=false \
  --dataset.private=true \
  --play_sounds=false \
  --display_data=true
```
