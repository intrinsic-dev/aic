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

Key fix: The FTS has a ~18N gravity/weight baseline bias.
We measure baseline force at lift height (no contact) and subtract
it everywhere so thresholds work on actual contact force only.
"""

import math

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

    # ── Force thresholds (Newtons) — relative to baseline ───────────────────
    FORCE_CONTACT = 0.8   # Contact force above this → tip touching something
    FORCE_ABORT   = 4.0   # Contact force above this → retract and nudge

    # ── Heights relative to initial TCP Z (metres) ───────────────────────────
    LIFT_DELTA    = 0.06   # Lift above start — clear of board for baseline reading
    SEARCH_DELTA  = 0.03   # Search height above start
    INSERT_DELTA  = -0.03  # Target insertion depth below start

    # ── Motion parameters ────────────────────────────────────────────────────
    DESCENT_STEP  = 0.002  # 2 mm per step
    XY_NUDGE      = 0.005  # 5 mm lateral correction on retry
    MAX_RETRIES   = 16
    STEP_SLEEP    = 0.08
    SEARCH_SLEEP  = 0.15

    # ── Compact XY search grid (offsets in metres) ───────────────────────────
    _SEARCH_OFFSETS = [
        ( 0.000,  0.000),
        ( 0.006,  0.000), (-0.006,  0.000),
        ( 0.000,  0.006), ( 0.000, -0.006),
        ( 0.010,  0.010), (-0.010,  0.010),
        ( 0.010, -0.010), (-0.010, -0.010),
        ( 0.014,  0.000), (-0.014,  0.000),
        ( 0.000,  0.014), ( 0.000, -0.014),
    ]

    _DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("SmartInsertPolicy.__init__()")

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

        lift_z   = anchor_z + self.LIFT_DELTA
        search_z = anchor_z + self.SEARCH_DELTA
        insert_z = anchor_z + self.INSERT_DELTA

        # ── Phase 1: Lift and measure force baseline ─────────────────────────
        send_feedback("Phase 1: lifting and measuring force baseline")
        self._move_to(move_robot, anchor_x, anchor_y, lift_z)
        self.sleep_for(1.0)  # settle fully before reading baseline

        baseline = self._measure_baseline(get_observation)
        self.get_logger().info(f"  Force baseline at lift height: {baseline:.2f}N")

        if self._past_deadline(start_time, deadline, buffer_sec=25.0):
            self.get_logger().warn("Time limit hit after Phase 1")
            return False

        # ── Phase 2: Search for port ─────────────────────────────────────────
        send_feedback("Phase 2: searching for port")
        self._move_to(move_robot, anchor_x, anchor_y, search_z)
        self.sleep_for(0.5)

        port_x, port_y = self._search_for_port(
            get_observation, move_robot, anchor_x, anchor_y, search_z, baseline
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
            baseline, start_time, deadline,
        )

        result = "SUCCESS — cable inserted!" if success else "INCOMPLETE"
        self.get_logger().info(f"SmartInsertPolicy: {result}")
        send_feedback(result)
        return success

    # ── Measure force baseline at lift height (no contact) ───────────────────
    def _measure_baseline(self, get_observation: GetObservationCallback) -> float:
        """Average total force over several readings at lift height (no contact)."""
        readings = []
        for _ in range(8):
            obs = get_observation()
            if obs is not None:
                fx = obs.wrist_wrench.wrench.force.x
                fy = obs.wrist_wrench.wrench.force.y
                fz = obs.wrist_wrench.wrench.force.z
                readings.append(math.sqrt(fx**2 + fy**2 + fz**2))
            self.sleep_for(0.05)
        if readings:
            return sum(readings) / len(readings)
        return 0.0

    # ── Phase 0 helper ───────────────────────────────────────────────────────
    def _get_initial_tcp(self, get_observation: GetObservationCallback):
        for _ in range(10):
            obs = get_observation()
            if obs is not None:
                p = obs.controller_state.tcp_pose.position
                if not (p.x == 0.0 and p.y == 0.0 and p.z == 0.0):
                    return p.x, p.y, p.z
            self.sleep_for(0.1)
        self.get_logger().warn("No valid TCP pose — using fallback coords")
        return -0.40, 0.45, 0.25

    # ── Phase 2: XY search sweep ─────────────────────────────────────────────
    def _search_for_port(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        cx: float, cy: float, search_z: float,
        baseline: float,
    ):
        best_x, best_y = cx, cy
        best_fz = 0.0

        for dx, dy in self._SEARCH_OFFSETS:
            sx, sy = cx + dx, cy + dy
            self._move_to(move_robot, sx, sy, search_z)
            self.sleep_for(self.SEARCH_SLEEP)

            obs = get_observation()
            if obs is None:
                continue

            raw_fz = abs(obs.wrist_wrench.wrench.force.z)
            contact_fz = max(raw_fz - baseline, 0.0)
            self.get_logger().info(
                f"  search ({sx:.3f}, {sy:.3f})  raw_Fz={raw_fz:.2f}N  contact={contact_fz:.2f}N"
            )

            if contact_fz > best_fz:
                best_fz = contact_fz
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
        baseline: float,
        start_time,
        deadline: Duration,
    ) -> bool:
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
            f_contact = max(f_total - baseline, 0.0)
            fz_contact = max(abs(fz) - baseline, 0.0)

            self.get_logger().info(
                f"  z={current_z:.4f}  raw={f_total:.2f}N  contact={f_contact:.2f}N"
            )

            # Hard stop — max retries exhausted
            if f_contact > self.FORCE_ABORT and retries >= self.MAX_RETRIES:
                self.get_logger().warn("Max retries reached — stopping")
                break

            # Excessive contact force — retract and nudge
            if f_contact > self.FORCE_ABORT and retries < self.MAX_RETRIES:
                retries += 1
                self.get_logger().info(
                    f"  Excessive contact force {f_contact:.2f}N — retracting (retry {retries})"
                )
                current_z += 0.010
                self._move_to(move_robot, port_x, port_y, current_z)
                self.sleep_for(0.3)

                angle = retries * (math.pi / 4)
                nudge_r = self.XY_NUDGE * (1 + retries // 8)
                port_x += nudge_r * math.cos(angle)
                port_y += nudge_r * math.sin(angle)
                self.get_logger().info(
                    f"  Nudged to ({port_x:.4f}, {port_y:.4f})"
                )
                continue

            # Seated: reached target depth with contact
            if current_z <= insert_z + 0.004 and fz_contact > self.FORCE_CONTACT:
                self.get_logger().info(
                    f"  Cable seated: contact_Fz={fz_contact:.2f}N at z={current_z:.4f}"
                )
                self.sleep_for(0.5)
                return True

        return current_z <= insert_z + 0.004

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _move_to(self, move_robot, x, y, z):
        self.set_pose_target(
            move_robot=move_robot,
            pose=Pose(
                position=Point(x=x, y=y, z=z),
                orientation=self._DOWN_QUAT,
            ),
        )

    def _past_deadline(self, start_time, deadline, buffer_sec):
        elapsed = self.time_now() - start_time
        limit = Duration(nanoseconds=deadline.nanoseconds - int(buffer_sec * 1e9))
        return elapsed >= limit
