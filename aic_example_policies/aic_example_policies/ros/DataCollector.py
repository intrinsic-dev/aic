"""
DataCollector — Monte Carlo viewpoint sampling for robust pose estimation training.

MONTE CARLO SAMPLING STRATEGY:
================================

Instead of fixed viewpoints, each trial randomly samples N poses from a
defined 6-DOF pose space around the target port. This means:

- No two trials ever produce the same set of viewpoints
- The model sees the port at truly random positions/orientations in frame
- Coverage of the pose space improves with more runs (Monte Carlo convergence)
- Robust to sim-to-real gap because the distribution covers the real workspace

POSE SPACE DEFINITION:
=======================
We define the sampling space as ranges around the initial TCP pose
(where aic_engine places the robot, a few cm from the target port):

  Translation:
    dx: [-0.08, +0.08] m  (left/right)
    dy: [-0.08, +0.08] m  (up/down)
    dz: [-0.01, +0.18] m  (close to far — asymmetric: can go slightly closer)

  Rotation (wrist tilt):
    drx: [-0.25, +0.25] rad  (~14 deg tilt)
    dry: [-0.25, +0.25] rad
    drz: [-0.15, +0.15] rad

  This gives a roughly ellipsoidal sampling space that covers:
    - Port occupying 5% to 50% of camera frame (scale diversity)
    - Port at any position in the frame (not always centered)
    - Tilted approaches (rotation diversity)
    - All of this randomized differently every single trial

ANTI-OVERFITTING DESIGN:
=========================
1. Monte Carlo viewpoints — no fixed poses ever repeated
2. 1 frame per viewpoint — no near-duplicate frames
3. Val split at RUN level — val images from completely different configs
4. Min bbox filter — skip labels too small to learn from
5. Stratified sampling — ensure close/mid/far distances are all represented

HOW TO RUN:
============
  Terminal 1 (inside distrobox):
    /entrypoint.sh ground_truth:=true start_aic_engine:=true gazebo_gui:=false

  Terminal 2 (host):
    cd ~/ws_aic/src/aic
    pixi run ros2 run aic_model aic_model --ros-args \
      -p use_sim_time:=true \
      -p policy:=aic_example_policies.ros.DataCollector
"""

import json
import os
import random

import cv2
import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException

# ─── Config ─────────────────────────────────────────────────────────────────

OUTPUT_DIR = os.path.expanduser("~/aic_perception_data")

# How many randomly sampled viewpoints per trial
# More = more data per trial but slower
VIEWPOINTS_PER_TRIAL = 15

# Val split: every VAL_EVERY_N_RUNS-th run goes entirely to val
# This ensures val images come from completely different board configs
VAL_EVERY_N_RUNS = 5

# Minimum bounding box size in pixels to include as a label
# Smaller than this is noise, not signal
MIN_BBOX_PX = 15

# ─── Monte Carlo Pose Space ───────────────────────────────────────────────
# Sampling ranges relative to the initial TCP pose.
# These are stratified into distance bins to ensure scale diversity.
# Each bin contributes a fixed fraction of viewpoints.

# Bin definitions: (dz_min, dz_max, fraction_of_viewpoints)
# dz = pullback distance from port (larger = farther away = smaller port in frame)
DISTANCE_BINS = [
    (-0.01, 0.04,  0.2),   # Very close:  port fills 30-50% of frame
    (0.04,  0.09,  0.4),   # Mid close:   port fills 15-30% of frame
    (0.09,  0.14,  0.3),   # Mid far:     port fills 5-15% of frame
    (0.14,  0.18,  0.1),   # Far:         port fills ~5% of frame, full board visible
]

# Lateral range (dx, dy) scales with distance so near views stay on target
# At dz=0: lateral range = 0.02m, at dz=0.18: lateral range = 0.08m
LATERAL_SCALE_NEAR = 0.02   # m at closest
LATERAL_SCALE_FAR  = 0.08   # m at farthest
DZ_RANGE = (DISTANCE_BINS[0][0], DISTANCE_BINS[-1][1])

# Wrist rotation range (radians)
# Small rotations — enough to change perspective without losing the port
ROT_RANGE = {
    "rx": (-0.20, 0.20),   # Tilt up/down
    "ry": (-0.20, 0.20),   # Tilt left/right
    "rz": (-0.12, 0.12),   # Roll
}

# ─── Port Geometry ───────────────────────────────────────────────────────────

SFP_PORT_BBOX_3D = np.array([
    [-0.008, -0.004, 0.0],  [0.008, -0.004, 0.0],
    [0.008,  0.004, 0.0],  [-0.008,  0.004, 0.0],
    [-0.008, -0.004, -0.02], [0.008,  0.004, -0.02],
], dtype=np.float64)

SC_PORT_BBOX_3D = np.array([
    [-0.006, -0.006, 0.0],  [0.006, -0.006, 0.0],
    [0.006,  0.006, 0.0],  [-0.006,  0.006, 0.0],
    [-0.006, -0.006, -0.01], [0.006,  0.006, -0.01],
], dtype=np.float64)

NIC_CARD_BBOX_3D = np.array([
    [-0.025, -0.04, -0.005], [0.025, -0.04, -0.005],
    [0.025,  0.005, 0.005], [-0.025,  0.005, 0.005],
], dtype=np.float64)

CLASS_MAP = {"sfp_port": 0, "sc_port": 1, "nic_card": 2}
BBOX_3D_MAP = {
    "sfp_port": SFP_PORT_BBOX_3D,
    "sc_port":  SC_PORT_BBOX_3D,
    "nic_card": NIC_CARD_BBOX_3D,
}

PORT_TF_FRAMES = {}
for n in range(5):
    for p in range(2):
        PORT_TF_FRAMES[f"task_board/nic_card_mount_{n}/sfp_port_{p}_link"] = (
            "sfp_port", f"nic_card_mount_{n}")
    PORT_TF_FRAMES[f"task_board/nic_card_mount_{n}/nic_card_link"] = (
        "nic_card", f"nic_card_mount_{n}")
for n in range(2):
    PORT_TF_FRAMES[f"task_board/sc_port_{n}/sc_port_base_link"] = (
        "sc_port", f"sc_port_{n}")

CAMERA_NAMES = ["left_camera", "center_camera", "right_camera"]


# ─── Math Helpers ────────────────────────────────────────────────────────────

def tf_to_4x4(transform):
    t = transform.translation
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def euler_to_quat(rx, ry, rz):
    """Convert euler angles to quaternion (x, y, z, w)."""
    cx, cy, cz = np.cos([rx/2, ry/2, rz/2])
    sx, sy, sz = np.sin([rx/2, ry/2, rz/2])
    return np.array([
        sx*cy*cz - cx*sy*sz,
        cx*sy*cz + sx*cy*sz,
        cx*cy*sz - sx*sy*cz,
        cx*cy*cz + sx*sy*sz,
    ])


def quat_multiply(q1, q2):
    """Multiply quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def project_points_to_image(points_3d, T_obj_to_cam, K):
    ones = np.ones((points_3d.shape[0], 1))
    pts_h = np.hstack([points_3d, ones])
    pts_cam = (T_obj_to_cam @ pts_h.T).T[:, :3]
    valid = pts_cam[:, 2] > 0.01
    if not np.any(valid):
        return None
    pts_cam = pts_cam[valid]
    pts_2d_h = (K @ pts_cam.T).T
    pts_2d = pts_2d_h[:, :2] / pts_2d_h[:, 2:3]
    return pts_2d


def points_to_yolo_bbox(pts_2d, img_w, img_h):
    if pts_2d is None or len(pts_2d) == 0:
        return None
    x_min = np.clip(pts_2d[:, 0].min(), 0, img_w)
    x_max = np.clip(pts_2d[:, 0].max(), 0, img_w)
    y_min = np.clip(pts_2d[:, 1].min(), 0, img_h)
    y_max = np.clip(pts_2d[:, 1].max(), 0, img_h)
    bw, bh = x_max - x_min, y_max - y_min
    if bw < MIN_BBOX_PX or bh < MIN_BBOX_PX:
        return None
    cx = ((x_min + x_max) / 2.0) / img_w
    cy = ((y_min + y_max) / 2.0) / img_h
    return (cx, cy, bw / img_w, bh / img_h)


def ros_image_to_cv2(img_msg):
    if img_msg.encoding == "rgb8":
        arr = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "bgr8":
        return np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, 3).copy()
    elif img_msg.encoding == "mono8":
        arr = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    else:
        try:
            arr = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception:
            return None


# ─── Monte Carlo Sampling ────────────────────────────────────────────────────

def sample_viewpoints(n_viewpoints):
    """
    Sample n_viewpoints poses from the Monte Carlo pose space.
    Uses stratified sampling across distance bins to guarantee
    scale diversity (close, mid, far views all represented).

    Returns list of (dx, dy, dz, drx, dry, drz) tuples.
    """
    viewpoints = []

    # Stratified: allocate viewpoints to bins by fraction
    bin_counts = []
    remaining = n_viewpoints
    for i, (dz_min, dz_max, frac) in enumerate(DISTANCE_BINS):
        if i == len(DISTANCE_BINS) - 1:
            count = remaining  # last bin gets the remainder
        else:
            count = max(1, round(n_viewpoints * frac))
            remaining -= count
        bin_counts.append(count)

    for (dz_min, dz_max, _), count in zip(DISTANCE_BINS, bin_counts):
        for _ in range(count):
            dz = random.uniform(dz_min, dz_max)

            # Lateral range scales with distance
            t = (dz - DZ_RANGE[0]) / (DZ_RANGE[1] - DZ_RANGE[0])
            t = max(0.0, min(1.0, t))
            lat = LATERAL_SCALE_NEAR + t * (LATERAL_SCALE_FAR - LATERAL_SCALE_NEAR)

            dx = random.uniform(-lat, lat)
            dy = random.uniform(-lat, lat)

            drx = random.uniform(*ROT_RANGE["rx"])
            dry = random.uniform(*ROT_RANGE["ry"])
            drz = random.uniform(*ROT_RANGE["rz"])

            viewpoints.append((dx, dy, dz, drx, dry, drz))

    # Shuffle so we don't always go close→far in order
    random.shuffle(viewpoints)
    return viewpoints


def compute_settle_time(dx, dy, dz):
    """Estimate settle time based on move distance."""
    dist = np.sqrt(dx**2 + dy**2 + dz**2)
    # Base 1.5s + scaling with distance for slow software-rendered sim
    return max(1.5, dist * 25.0)


# ─── The Policy ─────────────────────────────────────────────────────────────

class DataCollector(Policy):
    """
    Monte Carlo perception data collector.
    Randomly samples the 6-DOF viewpoint space each trial for
    robust, non-overfitting YOLO training data.
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self._frame_counter = 0
        self._trial_counter = 0
        self._run_counter = self._load_run_counter()

        for subdir in ["images/train", "images/val",
                       "labels/train", "labels/val", "metadata"]:
            os.makedirs(os.path.join(OUTPUT_DIR, subdir), exist_ok=True)

        self._split = "val" if (self._run_counter % VAL_EVERY_N_RUNS == 0) else "train"

        self.get_logger().info(
            f"DataCollector init | run={self._run_counter} "
            f"split={self._split} | {VIEWPOINTS_PER_TRIAL} viewpoints/trial"
        )

    def _load_run_counter(self):
        """Persist run counter so val split stays correct across restarts."""
        counter_file = os.path.join(OUTPUT_DIR, ".run_counter")
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        try:
            with open(counter_file) as f:
                count = int(f.read().strip()) + 1
        except Exception:
            count = 1
        with open(counter_file, "w") as f:
            f.write(str(count))
        return count

    def _lookup_tf(self, target, source):
        try:
            return self._parent_node._tf_buffer.lookup_transform(
                target, source, Time(), Duration(seconds=1.0))
        except TransformException:
            return None

    def _get_camera_intrinsics(self, obs, camera_name):
        if camera_name == "left_camera":
            info = obs.left_camera_info
        elif camera_name == "center_camera":
            info = obs.center_camera_info
        elif camera_name == "right_camera":
            info = obs.right_camera_info
        else:
            return None
        K = np.array(info.k).reshape(3, 3)
        return K if K[0, 0] != 0 else None

    def _get_camera_image(self, obs, camera_name):
        if camera_name == "left_camera":
            return ros_image_to_cv2(obs.left_image)
        elif camera_name == "center_camera":
            return ros_image_to_cv2(obs.center_image)
        elif camera_name == "right_camera":
            return ros_image_to_cv2(obs.right_image)
        return None

    def _capture_frame(self, obs, vp_idx, vp_params):
        """Capture all 3 cameras at current pose. Returns number of images saved."""
        frame_id = f"{self._trial_counter:03d}_{self._frame_counter:05d}"
        dx, dy, dz, drx, dry, drz = vp_params
        metadata = {
            "frame_id": frame_id,
            "trial": self._trial_counter,
            "run": self._run_counter,
            "split": self._split,
            "viewpoint_idx": vp_idx,
            "viewpoint_offset": {
                "dx": dx, "dy": dy, "dz": dz,
                "drx": drx, "dry": dry, "drz": drz,
            },
            "cameras": {}
        }
        saved = 0

        for cam_name in CAMERA_NAMES:
            image = self._get_camera_image(obs, cam_name)
            if image is None:
                continue

            K = self._get_camera_intrinsics(obs, cam_name)
            if K is None:
                continue

            cam_optical_frame = f"{cam_name}/optical"
            img_h, img_w = image.shape[:2]
            labels = []
            port_data = []

            for tf_frame, (port_type, parent_name) in PORT_TF_FRAMES.items():
                tf_obj_cam = self._lookup_tf(cam_optical_frame, tf_frame)
                if tf_obj_cam is None:
                    continue

                T = tf_to_4x4(tf_obj_cam.transform)
                pts_2d = project_points_to_image(BBOX_3D_MAP[port_type], T, K)
                yolo_bbox = points_to_yolo_bbox(pts_2d, img_w, img_h)
                if yolo_bbox is None:
                    continue

                class_id = CLASS_MAP[port_type]
                labels.append(
                    f"{class_id} {yolo_bbox[0]:.6f} {yolo_bbox[1]:.6f} "
                    f"{yolo_bbox[2]:.6f} {yolo_bbox[3]:.6f}"
                )

                tf_obj_base = self._lookup_tf("base_link", tf_frame)
                if tf_obj_base is not None:
                    t = tf_obj_base.transform.translation
                    r = tf_obj_base.transform.rotation
                    port_data.append({
                        "frame": tf_frame,
                        "type": port_type,
                        "class_id": class_id,
                        "pose_base": [t.x, t.y, t.z, r.w, r.x, r.y, r.z],
                        "bbox_yolo": list(yolo_bbox),
                    })

            if not labels:
                continue

            img_fn = f"{frame_id}_{cam_name}.png"
            lbl_fn = f"{frame_id}_{cam_name}.txt"

            cv2.imwrite(
                os.path.join(OUTPUT_DIR, "images", self._split, img_fn), image)
            with open(os.path.join(OUTPUT_DIR, "labels", self._split, lbl_fn), "w") as f:
                f.write("\n".join(labels))

            metadata["cameras"][cam_name] = {"image": img_fn, "ports": port_data}
            saved += 1

        meta_path = os.path.join(OUTPUT_DIR, "metadata", f"{frame_id}.json")
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)

        self._frame_counter += 1
        return saved

    def _move_to_offset(self, move_robot, initial_tcp, dx, dy, dz, drx, dry, drz):
        """Move robot to initial_tcp + (dx, dy, dz, drx, dry, drz)."""
        q = initial_tcp.orientation
        base_q = np.array([q.x, q.y, q.z, q.w])

        if abs(drx) + abs(dry) + abs(drz) > 1e-6:
            delta_q = euler_to_quat(drx, dry, drz)
            new_q = quat_multiply(base_q, delta_q)
            new_q = new_q / np.linalg.norm(new_q)
            orient = Quaternion(x=new_q[0], y=new_q[1], z=new_q[2], w=new_q[3])
        else:
            orient = initial_tcp.orientation

        self.set_pose_target(
            move_robot=move_robot,
            pose=Pose(
                position=Point(
                    x=initial_tcp.position.x + dx,
                    y=initial_tcp.position.y + dy,
                    z=initial_tcp.position.z + dz,
                ),
                orientation=orient,
            ),
            stiffness=[100.0, 100.0, 100.0, 50.0, 50.0, 50.0],
            damping=[70.0, 70.0, 70.0, 35.0, 35.0, 35.0],
        )

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self._trial_counter += 1
        self.get_logger().info(
            f"Trial {self._trial_counter} | {task.plug_type} → {task.port_name} "
            f"on {task.target_module_name} | run={self._run_counter} "
            f"split={self._split}"
        )
        send_feedback(f"Trial {self._trial_counter} starting (run {self._run_counter})")

        # Wait for TF to settle after board spawn
        self.sleep_for(3.0)

        obs = get_observation()
        if obs is None:
            self.get_logger().error("No observation at trial start — skipping")
            return True

        initial_tcp = obs.controller_state.tcp_pose

        # Sample random viewpoints for this trial
        viewpoints = sample_viewpoints(VIEWPOINTS_PER_TRIAL)
        self.get_logger().info(
            f"Sampled {len(viewpoints)} Monte Carlo viewpoints for trial "
            f"{self._trial_counter}"
        )

        trial_saved = 0
        for vp_idx, (dx, dy, dz, drx, dry, drz) in enumerate(viewpoints):
            # Move to sampled viewpoint
            self._move_to_offset(
                move_robot, initial_tcp, dx, dy, dz, drx, dry, drz)

            # Wait for robot to settle — longer for bigger moves
            settle = compute_settle_time(dx, dy, dz)
            self.sleep_for(settle)

            # Capture one frame (all 3 cameras)
            obs = get_observation()
            if obs is not None:
                n = self._capture_frame(obs, vp_idx, (dx, dy, dz, drx, dry, drz))
                trial_saved += n
                self.get_logger().info(
                    f"  VP {vp_idx+1}/{len(viewpoints)} "
                    f"dz={dz:.3f}m: saved {n} imgs "
                    f"(trial total: {trial_saved})"
                )
            else:
                self.get_logger().warn(f"  VP {vp_idx+1}: no observation")

        # Return to start pose
        self._move_to_offset(move_robot, initial_tcp, 0, 0, 0, 0, 0, 0)
        self.sleep_for(2.0)

        # Summary
        train_n = len([f for f in os.listdir(
            os.path.join(OUTPUT_DIR, "images", "train")) if f.endswith(".png")])
        val_n = len([f for f in os.listdir(
            os.path.join(OUTPUT_DIR, "images", "val")) if f.endswith(".png")])

        self.get_logger().info(
            f"Trial {self._trial_counter} done | "
            f"saved {trial_saved} | "
            f"total train={train_n} val={val_n}"
        )
        send_feedback(
            f"Trial {self._trial_counter} done. {trial_saved} images. "
            f"train={train_n} val={val_n}"
        )

        self._write_yolo_config(train_n, val_n)
        return True

    def _write_yolo_config(self, train_n, val_n):
        yaml_content = f"""# AIC Port Detection Dataset — Monte Carlo Collection
# Val split: every {VAL_EVERY_N_RUNS}th run → val (different board configs from train)
# Train: {train_n} images | Val: {val_n} images
path: {OUTPUT_DIR}
train: images/train
val: images/val
nc: 3
names: ['sfp_port', 'sc_port', 'nic_card']
"""
        with open(os.path.join(OUTPUT_DIR, "aic_ports.yaml"), "w") as f:
            f.write(yaml_content)