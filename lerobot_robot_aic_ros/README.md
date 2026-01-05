# lerobot_robot_aic_ros

This is a lerobot driver for the AIC robot.

## Start the AIC simulation

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

## Teleoperate

```bash
lerobot-teleoperate \
  --robot.type=aic_ros --robot.id=aic \
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

> [!NOTE]
> KDE in wayland by default does not allow an x11 app to listen to keystrokes. You need to go to Settings -> Application Permissions -> Legacy X11App Support to disable the security feature. Even after disabling it, there needs to be an x11 app opened (rviz works) for it to work.

## Record

```bash
lerobot-record \
  --robot.type=aic_ros --robot.id=aic \
  --teleop.type=aic_keyboard --teleop.id=aic \
  --dataset.repo_id=<hf-repo> \
  --dataset.single_task=<task-prompt> \
  --dataset.push_to_hub=false \
  --dataset.private=true \
  --display_data=true
```
