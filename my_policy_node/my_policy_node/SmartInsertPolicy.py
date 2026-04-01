#
#  Copyright (C) 2026 AIC Team — Pooja & PK
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#


"""
SmartInsertPolicy — Force-guided cable insertion policy.

Key design decisions (from official docs):
  1. Robot starts within a few centimetres of the port — use initial TCP
     pose from the first Observation as the search centre, NOT hardcoded
     coordinates. The board position is randomised per trial.
  2. Three trials: Trials 1 & 2 use SFP_MODULE plug; Trial 3 uses SC_PLUG.
     task.plug_type is read to log which trial type we are in.
  3. Scoring penalties to avoid:
       - Force > 20 N for > 1 s  → -12 pts  (keep FORCE_ABORT well below 20)
       - Contact with enclosure / task board → -24 pts (never push into walls)
  4. Tier 2 (smoothness, duration, efficiency) is only awarded if the plug
     reaches close proximity to the port. Minimise unnecessary travel.
  5. The search sweep is centred on the initial TCP XY position to keep the
     path short and efficient.

Phases:
  Phase 0: Read first observation — extract initial TCP pose as search centre.
  Phase 1: Lift slightly to a safe approach height.
  Phase 2: Sweep a compact XY grid at search height, find max-Fz contact point.
  Phase 3: Descend slowly toward the port, monitoring wrist force.
           Retry with small XY nudges on excessive force.
"""

import math

import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration


class SmartInsertPolicy(Policy):
    """Force-feedback guided cable insertion — uses initial TCP pose as anchor."""

    # ── Force thresholds (Newtons) ───────────────────────────────────────────
    # Keep well below the 20 N penalty threshold.
    FORCE_CONTACT = 1.2   # Fz above this → tip touching something
    FORCE_ABORT   = 8.0   # Total force above this → retract and nudge

    # ── Heights relative to initial TCP Z (metres) ───────────────────────────
    LIFT_DELTA    = 0.05   # How much to lift above start before searching
    SEARCH_DELTA  = 0.02   # Search height above start (lower than lift)
    INSERT_DELTA  = -0.05  # How far below start to consider fully inserted

    # ── Motion parameters ────────────────────────────────────────────────────
    DESCENT_STEP  = 0.003  # 3 mm per step — small for smoothness score
    XY_NUDGE      = 0.006  # 6 mm lateral correction on retry
    MAX_RETRIES   = 8
    STEP_SLEEP    = 0.08   # seconds between descent steps
    SEARCH_SLEEP  = 0.12   # seconds to settle at each search point

    # ── Compact XY search grid (offsets in metres) ───────────────────────────
    # Tight grid — robot already starts near the port, so don't travel far.
    _SEARCH_OFFSETS = [
        ( 0.000,  0.000),
        ( 0.008,  0.000), (-0.008,  0.000),
        ( 0.000,  0.008), ( 0.000, -0.008),
        ( 0.012,  0.012), (-0.012,  0.012),
        ( 0.012, -0.012), (-0.012, -0.012),
        ( 0.016,  0.000), (-0.016,  0.000),
        ( 0.000,  0.016), ( 0.000, -0.016),
    ]

    # Cable tip pointing straight down (x=1, y=0, z=0, w=0)
    _DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("SmartInsertPolicy.__init__()")

    # ── Main entry point ─────────────────────────────────────────────────────
    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(
            f"SmartInsertPolicy.insert_cable() "
            f"plug_type={task.plug_type}  port={task.port_name}  "
            f"module={task.target_module_name}  time_limit={task.time_limit}s"
        )
        start_time = self.time_now()
        deadline = Duration(seconds=int(task.time_limit))

        # ── Phase 0: Read initial TCP pose ───────────────────────────────────
        send_feedback("Phase 0: reading initial TCP pose")
        anchor_x, anchor_y, anchor_z = self._get_initial_tcp(get_observation)
        self.get_logger().info(
            f"  anchor TCP: ({anchor_x:.4f}, {anchor_y:.4f}, {anchor_z:.4f})"
        )

        # Derived absolute Z targets
        lift_z   = anchor_z + self.LIFT_DELTA
        search_z = anchor_z + self.SEARCH_DELTA
        insert_z = anchor_z + self.INSERT_DELTA

        # ── Phase 1: Lift to safe approach height ────────────────────────────
        send_feedback("Phase 1: lifting to approach height")
        self._move_to(move_robot, anchor_x, anchor_y, lift_z)
        self.sleep_for(0.8)

        if self._past_deadline(start_time, deadline, buffer_sec=25.0):
            self.get_logger().warn("Time limit hit after Phase 1")
            return False

        # ── Phase 2: Descend to search height and sweep XY ──────────────────
        send_feedback("Phase 2: searching for port")
        self._move_to(move_robot, anchor_x, anchor_y, search_z)
        self.sleep_for(0.5)

        port_x, port_y = self._search_for_port(
            get_observation, move_robot, anchor_x, anchor_y, search_z
        )
        self.get_logger().info(
            f"  best contact at ({port_x:.4f}, {port_y:.4f})"
        )

        if self._past_deadline(start_time, deadline, buffer_sec=12.0):
            self.get_logger().warn("Time limit hit after Phase 2")
            return False

        # ── Phase 3: Guided descent ──────────────────────────────────────────
        send_feedback("Phase 3: inserting cable")
        success = self._guided_insert(
            get_observation, move_robot,
            port_x, port_y, search_z, insert_z,
            start_time, deadline,
        )

        result = "SUCCESS — cable inserted!" if success else "INCOMPLETE"
        self.get_logger().info(f"SmartInsertPolicy: {result}")
        send_feedback(result)
        return success

    # ── Phase 0: Get initial TCP pose ────────────────────────────────────────
    def _get_initial_tcp(self, get_observation: GetObservationCallback):
        """Read the initial TCP XYZ from controller_state.tcp_pose.
        Falls back to hardcoded defaults if observation is unavailable."""
        for _ in range(10):
            obs = get_observation()
            if obs is not None:
                p = obs.controller_state.tcp_pose.position
                if not (p.x == 0.0 and p.y == 0.0 and p.z == 0.0):
                    return p.x, p.y, p.z
            self.sleep_for(0.1)

        # Fallback — WaveArm board centre
        self.get_logger().warn("No valid TCP pose received — using fallback coords")
        return -0.40, 0.45, 0.25

    # ── Phase 2: XY search sweep ─────────────────────────────────────────────
    def _search_for_port(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        cx: float, cy: float, search_z: float,
    ):
        """Sweep a compact grid around the anchor XY; return the point of
        maximum downward force (best estimate of port centre)."""
        best_x, best_y = cx, cy
        best_fz = 0.0

        for dx, dy in self._SEARCH_OFFSETS:
            sx, sy = cx + dx, cy + dy
            self._move_to(move_robot, sx, sy, search_z)
            self.sleep_for(self.SEARCH_SLEEP)

            obs = get_observation()
            if obs is None:
                continue

            fz = abs(obs.wrist_wrench.wrench.force.z)
            self.get_logger().info(f"  search ({sx:.3f}, {sy:.3f})  Fz={fz:.2f}N")

            if fz > best_fz:
                best_fz = fz
                best_x, best_y = sx, sy

        return best_x, best_y

    # ── Phase 3: Descent with force monitoring ───────────────────────────────
    def _guided_insert(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        port_x: float,
        port_y: float,
        current_z: float,
        insert_z: float,
        start_time,
        deadline: Duration,
    ) -> bool:
        """Step TCP down toward insert_z, monitoring wrist force.
        Retries with XY nudges when excessive force is encountered."""
        retries = 0

        while current_z > insert_z:
            if self._past_deadline(start_time, deadline, buffer_sec=4.0):
                self.get_logger().warn("Time limit approaching — stopping descent")
                break

            current_z = max(current_z - self.DESCENT_STEP, insert_z)
            self._move_to(move_robot, port_x, port_y, current_z)
            self.sleep_for(self.STEP_SLEEP)

            obs = get_observation()
            if obs is None:
                continue

            fx = obs.wrist_wrench.wrench.force.x
            fy = obs.wrist_wrench.wrench.force.y
            fz = obs.wrist_wrench.wrench.force.z
            f_total = math.sqrt(fx**2 + fy**2 + fz**2)

            self.get_logger().info(
                f"  z={current_z:.4f}  Fz={fz:.2f}N  F_total={f_total:.2f}N"
            )

            # Abort force — hitting rim; retract and nudge XY
            if f_total > self.FORCE_ABORT and retries < self.MAX_RETRIES:
                retries += 1
                self.get_logger().info(
                    f"  Excessive force — retracting (retry {retries})"
                )
                current_z += 0.012
                self._move_to(move_robot, port_x, port_y, current_z)
                self.sleep_for(0.25)

                # Alternate nudge direction each retry
                nudge = self.XY_NUDGE * ((-1) ** retries)
                if retries % 2 == 0:
                    port_x += nudge
                else:
                    port_y += nudge
                self.get_logger().info(
                    f"  Nudged to ({port_x:.4f}, {port_y:.4f})"
                )
                continue

            # Seated: reached target depth with downward contact
            if current_z <= insert_z + 0.004 and fz > self.FORCE_CONTACT:
                self.get_logger().info(
                    f"  Cable seated: Fz={fz:.2f}N at z={current_z:.4f}"
                )
                self.sleep_for(0.5)
                return True

        return current_z <= insert_z + 0.004

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _move_to(
        self,
        move_robot: MoveRobotCallback,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """Send a Cartesian pose target with the standard downward orientation."""
        self.set_pose_target(
            move_robot=move_robot,
            pose=Pose(
                position=Point(x=x, y=y, z=z),
                orientation=self._DOWN_QUAT,
            ),
        )

    def _past_deadline(
        self,
        start_time,
        deadline: Duration,
        buffer_sec: float,
    ) -> bool:
        """True if we are within buffer_sec seconds of the time limit."""
        elapsed = self.time_now() - start_time
        limit   = Duration(nanoseconds=deadline.nanoseconds - int(buffer_sec * 1e9))
        return elapsed >= limit
