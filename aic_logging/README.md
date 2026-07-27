# 📦 AIC Logging (`aic_logging`)

This package provides a **decoupled, external recording workflow** for logging ROS 2 topics, services, and actions from the AIC solution context to MCAP bag files over a 1GbE network connection.

---

## 🏗️ Architecture & 1GbE Network Bandwidth Math

Rather than recording bag files directly inside Kubernetes pod services (which introduces storage I/O bottlenecks and pod lifecycle constraints), recording is offloaded to a **dedicated external recording machine** connected to the Industrial PC (IPC) network over a 1GbE link.

### Bandwidth Calculation for 3x 5MP Cameras:
- **Single 5MP Uncompressed RGB8 Frame**: ~2448 × 2048 × 3 bytes = **~15 MB per frame** (120 Megabits).
- **3 Cameras** (`wrist_camera_1`, `wrist_camera_2`, `workcell_camera_1`): **45 MB per synchronized frame set**.
- **1GbE Real-world Network Limit**: ~85–90 MB/s (~700–720 Mbps) safe operational ceiling.

| Frequency / Camera | Total Image Data Rate | 1GbE Network Status |
| :--- | :--- | :--- |
| **5.0 Hz** | $3 \times 15\text{ MB} \times 5 = \mathbf{225\text{ MB/s}}\ (1.8\text{ Gbps})$ | ❌ **Saturates 1GbE link** (Drops frames) |
| **2.0 Hz** | $3 \times 15\text{ MB} \times 2 = \mathbf{90\text{ MB/s}}\ (720\text{ Mbps})$ | ✅ **Selected Operational Cap** (~720 Mbps) |
| **1.5 Hz** | $3 \times 15\text{ MB} \times 1.5 = \mathbf{67.5\text{ MB/s}}\ (540\text{ Mbps})$ | ✅ **Conservative Safe Rate** |

To guarantee no network dropouts over raw 1GbE, the Zenoh router configuration downsamples the 3x 5MP uncompressed image topics to **2.0 Hz**.

---

## ⚙️ Configuration Files

### 1. Zenoh Router Configuration (`zenoh_router_config.json5`)
Configures 2.0 Hz downsampling rules for wrist and workcell camera RGB streams and camera info topics:

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
    { key_expr: "**/wrist_camera_1/rgb/**", freq: 2.0 },
    { key_expr: "**/wrist_camera_1/camera_info/**", freq: 2.0 },
    { key_expr: "**/wrist_camera_2/rgb/**", freq: 2.0 },
    { key_expr: "**/wrist_camera_2/camera_info/**", freq: 2.0 },
    { key_expr: "**/workcell_camera_1/rgb/**", freq: 2.0 },
    { key_expr: "**/workcell_camera_1/camera_info/**", freq: 2.0 }
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

### 2. Start the Local Zenoh Router (2.0 Hz Rate Limit)
```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd --config aic_logging/zenoh_router_config.json5
```

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
