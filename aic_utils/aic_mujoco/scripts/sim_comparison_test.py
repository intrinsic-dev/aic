#!/usr/bin/env python3
"""
Simulator Comparison Test for MuJoCo vs Gazebo behavioral matching.

This script sends identical joint commands to the aic_controller and records
the resulting joint state trajectories. Run it once against each simulator,
then use the --compare mode to analyze the differences.

Usage:
  # 1. Launch Gazebo sim, then record:
  python3 sim_comparison_test.py --sim gazebo --output /tmp/gz_trajectory.csv

  # 2. Launch MuJoCo sim, then record:
  python3 sim_comparison_test.py --sim mujoco --output /tmp/mj_trajectory.csv

  # 3. Compare trajectories:
  python3 sim_comparison_test.py --compare /tmp/gz_trajectory.csv /tmp/mj_trajectory.csv

The test sends a sequence of joint position commands via the impedance controller
and records the actual joint positions/velocities at 500Hz for a configurable duration.
"""

import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from aic_control_interfaces.msg import (
    JointMotionUpdate,
    TrajectoryGenerationMode,
    TargetMode,
)
from aic_control_interfaces.srv import ChangeTargetMode


# UR5e joint names in order
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Initial configuration (from aic_ros2_controllers.yaml nullspace target)
INITIAL_CONFIG = [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]

# Test configurations: a sequence of (label, target_positions) tuples.
# Each config exercises a different type of motion to characterize
# single-joint and multi-joint (Cartesian-like) dynamics.
TEST_CONFIGS = [
    # --- Phase 1: Single-joint isolation ---
    ("shoulder_pan +0.3", [-0.1597 + 0.3, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]),
    ("return to initial", [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]),
    ("shoulder_lift +0.2", [-0.1597, -1.3542 + 0.2, -1.6648, -1.6933, 1.5710, 1.4110]),
    ("elbow -0.3", [-0.1597, -1.3542, -1.6648 - 0.3, -1.6933, 1.5710, 1.4110]),
    (
        "wrist multi-move",
        [-0.1597, -1.3542, -1.6648, -1.6933 + 0.4, 1.5710 - 0.3, 1.4110 + 0.5],
    ),
    ("return to initial", [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]),
    # --- Phase 2: Coordinated multi-joint (Cartesian-like) ---
    # EE up: shoulder_lift straightens, elbow compensates, wrist_1 keeps orientation
    (
        "EE up (coordinated)",
        [-0.1597, -1.3542 + 0.25, -1.6648 + 0.20, -1.6933 - 0.20, 1.5710, 1.4110],
    ),
    # EE down: opposite direction
    (
        "EE down (coordinated)",
        [-0.1597, -1.3542 - 0.15, -1.6648 - 0.20, -1.6933 + 0.15, 1.5710, 1.4110],
    ),
    ("return to initial", [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]),
    # --- Phase 3: Large amplitude coordinated motion ---
    # All joints shift simultaneously — stresses coupled dynamics
    (
        "large coordinated move",
        [
            -0.1597 + 0.2,
            -1.3542 + 0.3,
            -1.6648 - 0.25,
            -1.6933 + 0.3,
            1.5710 - 0.2,
            1.4110 + 0.3,
        ],
    ),
    ("return to initial", [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]),
    # --- Phase 4: Rapid back-and-forth (tests dynamic damping response) ---
    (
        "rapid EE up",
        [-0.1597, -1.3542 + 0.15, -1.6648 + 0.12, -1.6933 - 0.12, 1.5710, 1.4110],
    ),
    (
        "rapid EE down",
        [-0.1597, -1.3542 - 0.10, -1.6648 - 0.12, -1.6933 + 0.10, 1.5710, 1.4110],
    ),
    (
        "rapid EE up (2)",
        [-0.1597, -1.3542 + 0.15, -1.6648 + 0.12, -1.6933 - 0.12, 1.5710, 1.4110],
    ),
    ("return to initial", [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]),
]

# Joint impedance gains for the test (matching aic_ros2_controllers.yaml defaults)
TEST_STIFFNESS = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
TEST_DAMPING = [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]


class SimComparisonNode(Node):
    """ROS2 node for recording joint state trajectories during test commands."""

    def __init__(self, sim_name: str, output_file: str, duration_per_step: float = 3.0):
        super().__init__("sim_comparison_test")
        self.sim_name = sim_name
        self.output_file = output_file
        self.duration_per_step = duration_per_step

        # Storage for recorded data
        self.trajectory_data = []
        self.recording = False
        self.start_time = None
        self.joint_order = None  # maps joint name -> index in JointState msg

        # Subscribe to joint states
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, qos
        )

        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            JointMotionUpdate, "/aic_controller/joint_commands", 10
        )

        # Service client for changing target mode
        self.mode_client = self.create_client(
            ChangeTargetMode, "/aic_controller/change_target_mode"
        )

        self.get_logger().info(f"SimComparisonTest initialized for [{sim_name}]")

    def _joint_state_cb(self, msg: JointState):
        """Record joint state if recording is active."""
        if not self.recording:
            return

        if self.joint_order is None:
            # Build joint name -> index mapping from first message
            self.joint_order = {}
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    self.joint_order[name] = i
            if len(self.joint_order) != len(JOINT_NAMES):
                self.get_logger().warn(
                    f"Not all joints found in /joint_states. Found: {list(self.joint_order.keys())}"
                )

        # Get current time relative to start
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t = (now - self.start_time).nanoseconds / 1e9

        # Extract joint positions and velocities in canonical order
        positions = []
        velocities = []
        efforts = []
        for jname in JOINT_NAMES:
            idx = self.joint_order.get(jname)
            if idx is not None and idx < len(msg.position):
                positions.append(msg.position[idx])
                velocities.append(msg.velocity[idx] if idx < len(msg.velocity) else 0.0)
                efforts.append(msg.effort[idx] if idx < len(msg.effort) else 0.0)
            else:
                positions.append(float("nan"))
                velocities.append(float("nan"))
                efforts.append(float("nan"))

        self.trajectory_data.append(
            {
                "time": t,
                "positions": positions,
                "velocities": velocities,
                "efforts": efforts,
            }
        )

    def switch_to_joint_mode(self):
        """Switch the controller to joint impedance mode."""
        self.get_logger().info("Switching to joint impedance mode...")
        if not self.mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("ChangeTargetMode service not available!")
            return False

        req = ChangeTargetMode.Request()
        req.target_mode.mode = TargetMode.MODE_JOINT
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Switched to joint mode successfully")
            return True
        else:
            self.get_logger().error("Failed to switch to joint mode")
            return False

    def send_joint_command(self, target_positions: list):
        """Send a joint position command via the impedance controller."""
        msg = JointMotionUpdate()
        msg.target_state.positions = target_positions
        msg.target_state.velocities = [0.0] * len(target_positions)
        msg.target_stiffness = TEST_STIFFNESS
        msg.target_damping = TEST_DAMPING
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        msg.target_feedforward_torque = [0.0] * len(target_positions)
        self.joint_cmd_pub.publish(msg)

    def run_test(self):
        """Execute the full test sequence."""
        self.get_logger().info(
            f"=== Starting comparison test for [{self.sim_name}] ==="
        )

        # Wait for joint states to be available
        self.get_logger().info("Waiting for /joint_states...")
        timeout = time.time() + 15.0
        while self.joint_order is None and time.time() < timeout:
            self.recording = True  # temporarily enable to capture first msg
            rclpy.spin_once(self, timeout_sec=0.5)
        self.recording = False
        self.trajectory_data.clear()

        if self.joint_order is None:
            self.get_logger().error(
                "No joint states received! Is the simulator running?"
            )
            return False

        self.get_logger().info(
            f"Joint states available: {list(self.joint_order.keys())}"
        )

        # Switch to joint mode
        if not self.switch_to_joint_mode():
            return False

        # Brief settling time
        time.sleep(1.0)

        # Start recording
        self.recording = True
        self.start_time = None
        self.get_logger().info("Recording started")

        # Send each test configuration and wait
        for i, (label, config) in enumerate(TEST_CONFIGS):
            self.get_logger().info(
                f"Step {i+1}/{len(TEST_CONFIGS)} [{label}]: Commanding {[f'{v:.3f}' for v in config]}"
            )
            # Send command at high rate for the duration
            step_start = time.time()
            while time.time() - step_start < self.duration_per_step:
                self.send_joint_command(config)
                rclpy.spin_once(self, timeout_sec=0.01)

        # Extra settling time at end
        self.get_logger().info("Final settling period...")
        settle_start = time.time()
        while time.time() - settle_start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.01)

        self.recording = False
        self.get_logger().info(
            f"Recording complete: {len(self.trajectory_data)} samples"
        )

        # Save to CSV
        self._save_csv()
        return True

    def _save_csv(self):
        """Save trajectory data to CSV file."""
        with open(self.output_file, "w", newline="") as f:
            writer = csv.writer(f)

            # Header
            header = ["time"]
            for jname in JOINT_NAMES:
                header.extend([f"{jname}_pos", f"{jname}_vel", f"{jname}_eff"])
            writer.writerow(header)

            # Data rows
            for sample in self.trajectory_data:
                row = [f"{sample['time']:.6f}"]
                for j in range(len(JOINT_NAMES)):
                    row.append(f"{sample['positions'][j]:.8f}")
                    row.append(f"{sample['velocities'][j]:.8f}")
                    row.append(f"{sample['efforts'][j]:.8f}")
                writer.writerow(row)

        self.get_logger().info(f"Trajectory saved to {self.output_file}")


def compare_trajectories(file1: str, file2: str):
    """Compare two trajectory CSV files and report metrics."""
    print(f"\n{'='*80}")
    print(f"TRAJECTORY COMPARISON")
    print(f"  File 1 (reference): {file1}")
    print(f"  File 2 (test):      {file2}")
    print(f"{'='*80}\n")

    # Load both files
    data1 = _load_csv(file1)
    data2 = _load_csv(file2)

    if data1 is None or data2 is None:
        print("ERROR: Could not load one or both files")
        return

    # Interpolate data2 onto data1's time base for comparison
    t1, t2 = data1["time"], data2["time"]
    t_start = max(t1[0], t2[0])
    t_end = min(t1[-1], t2[-1])

    if t_end <= t_start:
        print("ERROR: No overlapping time range between the two recordings")
        return

    # Create common time base (500 Hz)
    dt = 0.002
    t_common = np.arange(t_start, t_end, dt)

    print(
        f"Comparison window: {t_start:.2f}s to {t_end:.2f}s ({len(t_common)} samples)\n"
    )

    # Per-joint analysis
    print(
        f"{'Joint':<25} {'Pos RMSE (rad)':<18} {'Pos Max Err':<15} "
        f"{'Vel RMSE':<15} {'Vel Max Err':<15}"
    )
    print("-" * 90)

    total_pos_rmse = 0.0
    total_vel_rmse = 0.0
    for jname in JOINT_NAMES:
        # Interpolate both trajectories onto common time base
        pos1 = np.interp(t_common, t1, data1[f"{jname}_pos"])
        pos2 = np.interp(t_common, t2, data2[f"{jname}_pos"])
        vel1 = np.interp(t_common, t1, data1[f"{jname}_vel"])
        vel2 = np.interp(t_common, t2, data2[f"{jname}_vel"])

        pos_err = pos1 - pos2
        vel_err = vel1 - vel2

        pos_rmse = np.sqrt(np.mean(pos_err**2))
        pos_max = np.max(np.abs(pos_err))
        vel_rmse = np.sqrt(np.mean(vel_err**2))
        vel_max = np.max(np.abs(vel_err))

        total_pos_rmse += pos_rmse**2
        total_vel_rmse += vel_rmse**2

        print(
            f"{jname:<25} {pos_rmse:<18.6f} {pos_max:<15.6f} "
            f"{vel_rmse:<15.6f} {vel_max:<15.6f}"
        )

    total_pos_rmse = np.sqrt(total_pos_rmse / len(JOINT_NAMES))
    total_vel_rmse = np.sqrt(total_vel_rmse / len(JOINT_NAMES))

    print("-" * 90)
    print(f"{'MEAN':<25} {total_pos_rmse:<18.6f} {'':15s} " f"{total_vel_rmse:<15.6f}")

    # Assessment
    print(f"\n{'='*80}")
    if total_pos_rmse < 0.001:
        print("RESULT: EXCELLENT match - position RMSE < 0.001 rad")
    elif total_pos_rmse < 0.01:
        print("RESULT: GOOD match - position RMSE < 0.01 rad")
    elif total_pos_rmse < 0.05:
        print("RESULT: MODERATE match - position RMSE < 0.05 rad")
        print(
            "  → Consider tuning MuJoCo solver parameters or adding small joint damping"
        )
    else:
        print("RESULT: POOR match - position RMSE >= 0.05 rad")
        print("  → Significant divergence detected. Tuning recommendations:")
        print("    1. Verify MuJoCo integrator='euler' in scene.xml")
        print("    2. Try increasing MuJoCo solver iterations (200+)")
        print("    3. Add matched joint damping to both simulators")
        print("    4. Check for effort clamping (UR5e limits: 150/28 Nm)")
    print(f"{'='*80}\n")

    # Per-step analysis (identify which movements cause largest divergence)
    step_duration = 3.0  # seconds per step
    print("Per-step breakdown (which movements diverge most):")
    print(f"{'Step':<8} {'Label':<28} {'Time Window':<16} {'Pos RMSE':<15}")
    print("-" * 70)

    # Group steps into phases for summary
    phase_rmses = {"single_joint": [], "coordinated": [], "large": [], "rapid": []}
    phase_names = {
        "single_joint": "Phase 1: Single-joint isolation",
        "coordinated": "Phase 2: Coordinated (Cartesian-like)",
        "large": "Phase 3: Large amplitude",
        "rapid": "Phase 4: Rapid back-and-forth",
    }

    for step_idx, (label, _config) in enumerate(TEST_CONFIGS):
        step_start_t = t_start + step_idx * step_duration
        step_end_t = step_start_t + step_duration
        mask = (t_common >= step_start_t) & (t_common < step_end_t)
        if not np.any(mask):
            continue

        step_rmse = 0.0
        for jname in JOINT_NAMES:
            pos1 = np.interp(t_common[mask], t1, data1[f"{jname}_pos"])
            pos2 = np.interp(t_common[mask], t2, data2[f"{jname}_pos"])
            step_rmse += np.mean((pos1 - pos2) ** 2)
        step_rmse = np.sqrt(step_rmse / len(JOINT_NAMES))

        # Categorize into phase
        if step_idx < 6:
            phase_rmses["single_joint"].append(step_rmse)
        elif step_idx < 9:
            phase_rmses["coordinated"].append(step_rmse)
        elif step_idx < 11:
            phase_rmses["large"].append(step_rmse)
        else:
            phase_rmses["rapid"].append(step_rmse)

        print(
            f"{step_idx+1:<8} {label:<28} "
            f"{f'{step_start_t:.1f}s-{step_end_t:.1f}s':<16} {step_rmse:<15.6f}"
        )

    # Print phase summaries
    print(f"\n{'Phase Summary':}")
    print(f"{'Phase':<45} {'Mean RMSE':<15}")
    print("-" * 60)
    for key, name in phase_names.items():
        if phase_rmses[key]:
            mean_rmse = np.mean(phase_rmses[key])
            print(f"{name:<45} {mean_rmse:<15.6f}")

    # Signed error analysis per step (ref - test, positive = test UNDERSHOOTS ref)
    print(f"\n{'='*80}")
    print("Signed error per step (ref - test): positive = test UNDERSHOOTS ref")
    print(f"{'='*80}")

    for step_idx, (label, _config) in enumerate(TEST_CONFIGS):
        step_start_t = t_start + step_idx * step_duration
        step_end_t = step_start_t + step_duration
        mask = (t_common >= step_start_t) & (t_common < step_end_t)
        if not np.any(mask):
            continue

        print(f"\nStep {step_idx+1} [{label}] ({step_start_t:.1f}s-{step_end_t:.1f}s):")
        print(
            f"  {'Joint':<25} {'Mean Err':>10} {'End Err':>10} "
            f"{'|Mean|':>10} {'Direction':>12}"
        )
        print(f"  {'-'*70}")

        for jname in JOINT_NAMES:
            pos1 = np.interp(t_common[mask], t1, data1[f"{jname}_pos"])
            pos2 = np.interp(t_common[mask], t2, data2[f"{jname}_pos"])
            signed_err = pos1 - pos2  # positive means test undershoots

            mean_err = np.mean(signed_err)
            end_err = signed_err[-1]
            abs_mean = np.mean(np.abs(signed_err))

            # Skip joints with negligible error
            if abs_mean < 0.001:
                continue

            direction = "UNDERSHOOT" if mean_err > 0 else "OVERSHOOT"
            print(
                f"  {jname:<25} {mean_err:>+10.4f} {end_err:>+10.4f} "
                f"{abs_mean:>10.4f} {direction:>12}"
            )
    print()


def _load_csv(filepath: str) -> dict:
    """Load a trajectory CSV into a dict of numpy arrays."""
    try:
        with open(filepath) as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = list(reader)
    except FileNotFoundError:
        print(f"File not found: {filepath}")
        return None

    data = {col: [] for col in header}
    for row in rows:
        for i, col in enumerate(header):
            data[col].append(float(row[i]))

    return {k: np.array(v) for k, v in data.items()}


def main():
    parser = argparse.ArgumentParser(description="MuJoCo vs Gazebo comparison test")
    parser.add_argument(
        "--sim",
        choices=["mujoco", "gazebo"],
        help="Which simulator to record from",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV file path (default: /tmp/{sim}_trajectory.csv)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Duration per step in seconds (default: 3.0)",
    )
    parser.add_argument(
        "--compare",
        nargs=2,
        metavar=("FILE1", "FILE2"),
        help="Compare two trajectory CSV files",
    )

    args = parser.parse_args()

    if args.compare:
        compare_trajectories(args.compare[0], args.compare[1])
        return

    if args.sim is None:
        parser.error("Either --sim or --compare must be specified")

    output = args.output or f"/tmp/{args.sim}_trajectory.csv"

    rclpy.init()
    node = SimComparisonNode(args.sim, output, args.duration)

    try:
        success = node.run_test()
        if success:
            print(f"\nTrajectory recorded to: {output}")
            print(f"Run with the other simulator, then compare with:")
            other = "mujoco" if args.sim == "gazebo" else "gazebo"
            print(
                f"  python3 {sys.argv[0]} --compare {output} /tmp/{other}_trajectory.csv"
            )
        else:
            print("\nTest failed! Check that the simulator and controller are running.")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
