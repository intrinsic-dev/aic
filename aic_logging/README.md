# 📦 AIC Logging (`aic_logging`)

This package provides a **decoupled, external recording workflow** for logging ROS 2 topics, services, and actions from the AIC solution context to MCAP bag files over a 10GbE network connection.

---

## 🏗️ Architecture & 10GbE Network Bandwidth Math

Rather than recording bag files directly inside Kubernetes pod services (which introduces storage I/O bottlenecks and pod lifecycle constraints), recording is offloaded to a **dedicated external recording machine** connected to the Industrial PC (IPC) network over a high-speed **10GbE link**.

### Bandwidth Calculation for 3x 5MP Cameras:
- **Single 5MP Uncompressed RGB8 Frame**: ~2448 × 2048 × 3 bytes = **~15 MB per frame** (120 Megabits).
- **3 Cameras** (`wrist_camera_1`, `wrist_camera_2`, `workcell_camera_1`): **45 MB per synchronized frame set**.
- **10GbE Real-world Network Limit**: ~950–1,000 MB/s (~7.6–8.0 Gbps) safe operational ceiling.

| Frequency / Camera | Total Image Data Rate | 10GbE Network Status |
| :--- | :--- | :--- |
| **30.0 Hz** | $3 \times 15\text{ MB} \times 30 = \mathbf{1,350\text{ MB/s}}\ (10.8\text{ Gbps})$ | ❌ **Exceeds 10GbE link** (Drops frames) |
| **20.0 Hz** | $3 \times 15\text{ MB} \times 20 = \mathbf{900\text{ MB/s}}\ (7.2\text{ Gbps})$ | ✅ **Selected Operational Rate** (~72% of 10GbE) |
| **15.0 Hz** | $3 \times 15\text{ MB} \times 15 = \mathbf{675\text{ MB/s}}\ (5.4\text{ Gbps})$ | ✅ **Conservative Safe Rate** |

Over a 10GbE link, streaming 3x 5MP uncompressed cameras at **20.0 Hz** consumes **900 MB/s (7.2 Gbps)**, comfortably operating within the 10GbE network bandwidth budget.

---

## ⚙️ Configuration Files

### 1. Zenoh Router Configuration (`zenoh_router_config.json5`)
Configures 20.0 Hz rate limiting rules for wrist and workcell camera RGB streams and camera info topics:

```json5
{
  mode: "router",
  connect: {
    endpoints: [
      "tcp/ip.address.of.ipc:17447"
    ]
  },
  listen: {
    endpoints: [
      "tcp/0.0.0.0:7447"
    ]
  },
  downsampling: [
    { key_expr: "**/wrist_camera_1/rgb/**", freq: 20.0 },
    { key_expr: "**/wrist_camera_1/camera_info/**", freq: 20.0 },
    { key_expr: "**/wrist_camera_2/rgb/**", freq: 20.0 },
    { key_expr: "**/wrist_camera_2/camera_info/**", freq: 20.0 },
    { key_expr: "**/workcell_camera_1/rgb/**", freq: 20.0 },
    { key_expr: "**/workcell_camera_1/camera_info/**", freq: 20.0 }
  ]
}
```

> [!IMPORTANT]
> Make sure to update the `connect.endpoints` value in `zenoh_router_config.json5` with the actual IP address or hostname of your Industrial PC (IPC), for example: `"tcp/192.168.1.100:17447"` (replacing `"tcp/ip.address.of.ipc:17447"`).

---

### 2. Bagging Configuration (`recording_config.yaml`)
Configures MCAP storage options and the exact list of topics and actions to record:

```yaml
storage:
  identifier: "mcap"                # Storage plugin: "mcap" or "sqlite3"
  output_dir: "/tmp/aic_bags"       # Base output directory
  preset_profile: "zstd_fast"       # Storage preset: "none", "fastwrite", "zstd_fast", "zstd_small"
  max_cache_size: 104857600         # 100 MB buffer cache

topics:
  record_all: false
  include:
    - "/wrist_camera_1/rgb"
    - "/wrist_camera_1/camera_info"
    - "/wrist_camera_2/rgb"
    - "/wrist_camera_2/camera_info"
    - "/workcell_camera_1/rgb"
    - "/workcell_camera_1/camera_info"
    - "/ati/wrench"
    - "/joint_states"
    - "/robot_description"
    - "/gripper_state"
    - "/controller/cartesian_commands"
    - "/controller/joint_commands"
    - "/task_state"
    - "/rosout"
  exclude: []

services:
  record_all: false
  include: []
  exclude: []

actions:
  record_all: false
  include:
    - "/insert_cable"
  exclude: []

advanced:
  node_name: "aic_external_recorder"
```

---

## 🚀 Step-by-Step Execution Guide

### 1. Setup Environment on Recording Machine
```bash
cd src/aic
pixi install
```

### 2. Start the Local Zenoh Router (20.0 Hz Rate Limit)
```bash
ZENOH_ROUTER_CONFIG_URI=aic_logging/zenoh_router_config.json5 pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

This can be verified by downsampling further and checking that the rate reported by `ros2 topic hz` goes down.

### 3. Start Recording Session
```bash
# Preview the ros2 bag record command
pixi run python3 aic_logging/record.py --dry-run

# Start recording
pixi run python3 aic_logging/record.py --config aic_logging/recording_config.yaml --name "cable_insertion_run"
```

---

## 🔍 Inspecting and Playing Back Bags

### Inspect Bag Metadata
```bash
pixi run ros2 bag info /tmp/aic_bags/<BAG_FOLDER_NAME>
```

### Play Back Recorded Bag
```bash
pixi run ros2 bag play /tmp/aic_bags/<BAG_FOLDER_NAME>
```
