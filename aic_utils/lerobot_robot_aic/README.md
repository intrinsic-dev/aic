# lerobot_robot_aic

This package contains a [LeRobot](https://huggingface.co/lerobot) interface for the AIC robot.

## Usage

This describe some of the things you can do with LeRobot, for more information, see the official [LeRobot docs](https://huggingface.co/docs/lerobot/en/index).

The LeRobot driver is installed in a [pixi](https://prefix.dev/tools/pixi) workspace. In general, you can prefix a command with `pixi run` or enter the environment with `pixi shell`.

### Teleoperating with LeRobot

```bash
cd ~/ws_aic/src/aic
pixi run lerobot-teleoperate \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=<teleop-type> --teleop.id=aic \
  --robot.teleop_target_mode=<mode> --robot.teleop_frame_id=<frame_id> \
  --display_data=true
```

Options for `--teleop.type` (and setting `--robot.teleop_target_mode` accordingly):

- `aic_keyboard_ee` for cartesian-space keyboard control (and set `--robot.teleop_target_mode=cartesian`)
- `aic_spacemouse` for cartesian-space SpaceMouse control (and set `--robot.teleop_target_mode=cartesian`)
- `aic_keyboard_joint` for joint-space control (and set `--robot.teleop_target_mode=joint`)

:warning: Note: In addition to setting `--teleop.type` you must set `--robot.teleop_target_mode` because the `AICRobotAICController` class needs to know which type of actions to send to the controller and it doesn't have access to `--teleop.type`.

#### Cartesian space control

For cartesian control, in addition to setting `--teleop.type` and `--robot.teleop_target_mode`, you can also set `teleop_frame_id` (the reference frame used for cartesian control) which sets the reference frame. Set this to either the gripper TCP (`"gripper/tcp"`, the default) or the robot base link (`"base_link"`).

##### Keyboard

> Note on using the Shift+&lt;key&gt; commands: To stop, let go of &lt;key&gt; *before* letting go of Shift. Otherwise, the robot will continue rotating even after you let go of both Shift and &lt;key&gt;.

| Key     | Cartesian      |
| ------- | ---------- |
| w       | -linear y  |
| s       | +linear y  |
| a       | -linear x  |
| d       | +linear x  |
| r       | -linear z  |
| f       | +linear z  |
| q       | -angular z |
| e       | +angular z |
| shift+w | +angular x |
| shift+s | -angular x |
| shift+a | -angular y |
| shift+d | +angular y |

Press 't' to toggle between slow and fast mode.

View and edit key mappings and speed settings in `AICKeyboardJointTeleop` and `AICKeyboardJointTeleopConfig` in `aic_teleop.py`.

##### SpaceMouse

:warning: Note: In our experience, SpaceMouse teleoperation was laggier than keyboard teleoperation.

We used a 3Dconnexion SpaceMouse with the [pyspacemouse](https://github.com/JakubAndrysek/PySpaceMouse?tab=readme-ov-file#dependencies) library. To enable USB permissions, you may need to add the following to your `/etc/udev/rules.d/99-spacemouse.rules`:
``` bash
# Apply to all hidraw nodes for 3Dconnexion devices
KERNEL=="hidraw*", ATTRS{idVendor}=="046d", MODE="0666", GROUP="plugdev"
# Apply to the USB device itself
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE="0666", GROUP="plugdev"
```
and then run
``` bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
View and edit axis mappings and speed settings in `AICSpaceMouseTeleop` and `AICSpaceMouseTeleopConfig` in `aic_teleop.py`.

#### Joint space control

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

Press 'u' to toggle between slow and fast mode.

View and edit key mappings and speed settings in `AICKeyboardEETeleop` and `AICKeyboardEETeleopConfig` in `aic_teleop.py`.

### Recording Training Data

```bash
cd ~/ws_aic/src/aic
pixi run lerobot-record \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=<teleop-type> --teleop.id=aic \
  --robot.teleop_target_mode=<mode> --robot.teleop_frame_id=<frame_id> \
  --dataset.repo_id=<hf-repo> \
  --dataset.single_task=<task-prompt> \
  --dataset.push_to_hub=false \
  --dataset.private=true \
  --play_sounds=false \
  --display_data=true
```

:warning: Note (same as with `lerobot-teleoperate` above): In addition to setting `--teleop.type` you must set `--robot.teleop_target_mode` because the `AICRobotAICController` class needs to know which type of actions to send to the controller and it doesn't have access to `--teleop.type`.

Upon starting the command, you may see `WARN   Watchdog Validator ThreadId(13) zenoh_shm::watchdog::periodic_task: Some("Watchdog Validator")` which is safe to ignore; just look for `INFO ... ls/utils.py:227 Recording episode 0`.

LeRobot recording keys:

| Key         | Command          |
| ----------- | ---------------- |
| Right Arrow | Next episode     |
| Left Arrow  | Cancel current episode and re-record |
| ESC         | Stop recording   |

<!-- TODO: lerobot-record doesn't load the hil processor to handle teleop events (lerobot bug?) -->
