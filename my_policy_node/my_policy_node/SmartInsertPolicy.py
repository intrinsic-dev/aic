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

Strategy:
  Phase 1: Move to a safe approach height above the task board.
  Phase 2: Sweep a search grid at a lower height, reading wrist force to
           identify the XY position with maximum downward contact.
  Phase 3: Slowly descend toward the port with force monitoring.
           Retry with small XY nudges if excessive lateral force is detected.
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
    """Force-feedback guided cable insertion policy."""

    # ── Tunable constants ────────────────────────────────────────────────────
    # Heights (metres, in base_link frame)
    APPROACH_Z   = 0.30    # Safe height to move to before searching
    SEARCH_Z     = 0.21    # Height during XY grid sweep
    INSERT_Z     = 0.04    # Target depth for full insertion

    # Force thresholds (Newtons)
    FORCE_CONTACT   = 1.5   # Fz above this → cable tip is touching something
    FORCE_ABORT     = 9.0   # Total force above this → retract and nudge

    # Motion parameters
    DESCENT_STEP    = 0.004  # Metres per descent step (4 mm)
    XY_NUDGE        = 0.007  # Metres lateral correction on retry (7 mm)
    MAX_RETRIES     = 8      # Maximum nudge-and-retry attempts
    STEP_SLEEP      = 0.08   # Seconds between descent steps
    SEARCH_SLEEP    = 0.15   # Seconds to settle at each search point

    # Nominal port position (centre of search area, metres in base_link frame)
    PORT_X = -0.40
    PORT_Y =  0.45

    # Cable tip pointing straight down
    _DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    # XY offsets from nominal position to sweep during search (metres)
    _SEARCH_OFFSETS = [
        ( 0.000,  0.000),
        ( 0.012,  0.000), (-0.012,  0.000),
        ( 0.000,  0.012), ( 0.000, -0.012),
        ( 0.018,  0.018), (-0.018,  0.018),
        ( 0.018, -0.018), (-0.018, -0.018),
        ( 0.024,  0.000), (-0.024,  0.000),
        ( 0.000,  0.024), ( 0.000, -0.024),
    ]

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
        self.get_logger().info(f"SmartInsertPolicy.insert_cable() task: {task}")
        start_time = self.time_now()
        # task.time_limit is uint64 seconds
        deadline = Duration(seconds=int(task.time_limit))

        send_feedback("Phase 1: moving to approach height")
        self._move_to(move_robot, self.PORT_X, self.PORT_Y, self.APPROACH_Z)
        self.sleep_for(1.0)

        if self._past_deadline(start_time, deadline, buffer_sec=20.0):
            self.get_logger().warn("Time limit hit after Phase 1")
            return False

        # ── Phase 2: Search for port ─────────────────────────────────────────
        send_feedback("Phase 2: searching for port")
        self._move_to(move_robot, self.PORT_X, self.PORT_Y, self.SEARCH_Z)
        self.sleep_for(0.6)

        port_x, port_y = self._search_for_port(get_observation, move_robot)
        self.get_logger().info(
            f"SmartInsertPolicy: best contact at ({port_x:.4f}, {port_y:.4f})"
        )

        if self._past_deadline(start_time, deadline, buffer_sec=10.0):
            self.get_logger().warn("Time limit hit after Phase 2")
            return False

        # ── Phase 3: Guided descent ──────────────────────────────────────────
        send_feedback("Phase 3: inserting cable")
        success = self._guided_insert(
            get_observation, move_robot, port_x, port_y, start_time, deadline
        )

        result = "SUCCESS" if success else "INCOMPLETE"
        self.get_logger().info(f"SmartInsertPolicy: {result}")
        send_feedback(result)
        return success

    # ── Phase 2: XY search ───────────────────────────────────────────────────
    def _search_for_port(
        self, get_observation: GetObservationCallback, move_robot: MoveRobotCallback
    ):
        """Sweep a grid of XY positions at SEARCH_Z, return the point of
        maximum downward force contact (best estimate of port centre)."""
        best_x, best_y = self.PORT_X, self.PORT_Y
        best_fz = 0.0

        for dx, dy in self._SEARCH_OFFSETS:
            cx = self.PORT_X + dx
            cy = self.PORT_Y + dy
            self._move_to(move_robot, cx, cy, self.SEARCH_Z)
            self.sleep_for(self.SEARCH_SLEEP)

            obs = get_observation()
            if obs is None:
                continue

            fz = abs(obs.wrist_wrench.wrench.force.z)
            self.get_logger().info(
                f"  search ({cx:.3f}, {cy:.3f})  Fz={fz:.2f}N"
            )

            if fz > best_fz:
                best_fz = fz
                best_x, best_y = cx, cy

        return best_x, best_y

    # ── Phase 3: Descent with force monitoring ───────────────────────────────
    def _guided_insert(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        port_x: float,
        port_y: float,
        start_time,
        deadline: Duration,
    ) -> bool:
        """Step the TCP down toward INSERT_Z, monitoring force.
        Retries with XY nudges when excessive force is encountered."""
        current_z = self.SEARCH_Z
        retries = 0

        while current_z > self.INSERT_Z:
            if self._past_deadline(start_time, deadline, buffer_sec=4.0):
                self.get_logger().warn("Time limit approaching — stopping descent")
                break

            current_z = max(current_z - self.DESCENT_STEP, self.INSERT_Z)
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

            # Too much force — retract and nudge XY
            if f_total > self.FORCE_ABORT and retries < self.MAX_RETRIES:
                retries += 1
                self.get_logger().info(
                    f"  Force limit exceeded — retracting (retry {retries})"
                )
                current_z += 0.015
                self._move_to(move_robot, port_x, port_y, current_z)
                self.sleep_for(0.3)

                # Alternate nudge directions: ±X then ±Y
                nudge = self.XY_NUDGE * ((-1) ** retries)
                if retries % 2 == 0:
                    port_x += nudge
                else:
                    port_y += nudge
                self.get_logger().info(
                    f"  Nudged to ({port_x:.4f}, {port_y:.4f})"
                )
                continue

            # Reached target depth with downward contact → insertion complete
            if current_z <= self.INSERT_Z + 0.005 and fz > self.FORCE_CONTACT:
                self.get_logger().info(
                    f"  Cable seated: Fz={fz:.2f}N at z={current_z:.4f}"
                )
                self.sleep_for(0.5)
                return True

        return current_z <= self.INSERT_Z + 0.005

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _move_to(
        self, move_robot: MoveRobotCallback, x: float, y: float, z: float
    ) -> None:
        """Send a Cartesian position target with the standard downward orientation."""
        self.set_pose_target(
            move_robot=move_robot,
            pose=Pose(
                position=Point(x=x, y=y, z=z),
                orientation=self._DOWN_QUAT,
            ),
        )

    def _past_deadline(self, start_time, deadline: Duration, buffer_sec: float) -> bool:
        """Return True if we are within buffer_sec seconds of the time limit."""
        elapsed = self.time_now() - start_time
        return elapsed >= Duration(seconds=deadline.nanoseconds / 1e9 - buffer_sec)
