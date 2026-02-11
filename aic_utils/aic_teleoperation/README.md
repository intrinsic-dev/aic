# aic_teleoperation

Keyboard-based teleoperation for the robot in both joint-space and Cartesian-space control modes.

## Prerequisites

1. Follow the [Getting Started Guide](../../docs/getting_started.md) to set up your development environment
2. X11 display server (pynput has known issues with Wayland)
3. For native builds: `sudo apt install python3-pynput` (pixi installs automatically)

## Available Scripts

### 1. Joint Space Teleoperation (`joint_keyboard_teleop`)

Control individual robot joints directly.

**Key Mappings:**
- `q/a` - Joint 1 (shoulder_pan_joint): +/-
- `w/s` - Joint 2 (shoulder_lift_joint): +/-
- `e/d` - Joint 3 (elbow_joint): +/-
- `r/f` - Joint 4 (wrist_1_joint): +/-
- `t/g` - Joint 5 (wrist_2_joint): +/-
- `y/h` - Joint 6 (wrist_3_joint): +/-

**Speed Control:**
- `k` - Slow mode (0.075 rad/s)
- `l` - Fast mode (0.2 rad/s)

**Exit:**
- `ESC` - Quit teleoperation

### 2. Cartesian Space Teleoperation (`cartesian_keyboard_teleop`)

Control end-effector pose (position and orientation).

**Linear Movement:**
- `a/d` - X axis: -/+
- `w/s` - Y axis: -/+
- `r/f` - Z axis: -/+

**Angular Movement:**
- `Shift + s/w` : -/+ Angular X
- `Shift + a/d` : -/+ Angular Y
- `q/e` : -/+ Angular Z

**Speed Control:**
- `k` - Slow mode (linear: 0.02 m/s, angular: 0.02 rad/s)
- `l` - Fast mode (linear: 0.1 m/s, angular: 0.1 rad/s)

**Frame Toggle:**
- `n` - Tool frame (`gripper/tcp`)
- `m` - Global frame (`base_link`)

**Exit:**
- `ESC` - Quit teleoperation

## Usage

Start the evaluation environment first following the [Getting Started - Quick Start](../../docs/getting_started.md#quick-start) instructions.

### With pixi (Recommended)

```bash
cd ~/ws_aic/src/aic

# Run teleoperation
pixi run ros2 run aic_teleoperation joint_keyboard_teleop
# or
pixi run ros2 run aic_teleoperation cartesian_keyboard_teleop
```

### With native ROS 2 build

See [Getting Started - Running the System](../../docs/getting_started.md#4-running-the-system) for workspace build and Zenoh setup.

```bash
# Run teleoperation
ros2 run aic_teleoperation joint_keyboard_teleop
# or
ros2 run aic_teleoperation cartesian_keyboard_teleop
```

## Notes

- The scripts automatically switch the controller to the appropriate control mode (joint or Cartesian) when started
- Press ESC to cleanly exit teleoperation
- Keyboard input will be captured regardless of whether keyboard focus is on the terminal window
