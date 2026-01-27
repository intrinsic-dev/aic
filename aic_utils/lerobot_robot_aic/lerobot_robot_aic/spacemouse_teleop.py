from dataclasses import dataclass
from threading import Thread
from typing import Any

import pyspacemouse
import rclpy
from geometry_msgs.msg import Twist
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from rclpy.executors import SingleThreadedExecutor

from .types import motion_update_action_features


@TeleoperatorConfig.register_subclass("aic_spacemouse")
@dataclass(kw_only=True)
class AICSpaceMouseTeleopConfig(TeleoperatorConfig):
    operator_position_front: bool = True
    device_path: str
    command_scaling: float = 0.05


class AICSpaceMouseTeleop(Teleoperator):
    def __init__(self, config: AICSpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._target_gripper_target: float = 1.0

    @property
    def action_features(self) -> dict:
        return motion_update_action_features()

    @property
    def feedback_features(self) -> dict:
        # TODO
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def _button_callback(self, state, buttons, pressed_buttons):
        if 0 in pressed_buttons:
            print("Button 1 pressed")
            self._target_gripper_target = 0.0

        elif 1 in pressed_buttons:
            print("Button 2 pressed")
            self._target_gripper_target = 1.0

    def connect(self, calibrate: bool = True) -> None:
        if not self.is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("spacemouse_teleop")
        if calibrate:
            self._node.get_logger().warn(
                "Calibration not supported, ensure the robot is calibrated before running teleop."
            )

        self._device_open_success = pyspacemouse.open(
            dof_callback=None,
            button_callback_arr=[
                pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
                pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            ],
            path=self.config.device_path,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin)
        self._executor_thread.start()
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        # Calibration not supported
        return True

    def calibrate(self) -> None:
        # Calibration not supported
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        state = pyspacemouse.read()

        twist_msg = Twist()
        twist_msg.linear.x = -float(state.y) ** 3 * self.config.command_scaling
        twist_msg.linear.y = float(state.x) ** 3 * self.config.command_scaling
        twist_msg.linear.z = float(state.z) ** 3 * self.config.command_scaling
        twist_msg.angular.x = -float(state.roll) ** 3 * self.config.command_scaling
        twist_msg.angular.y = -float(state.pitch) ** 3 * self.config.command_scaling
        twist_msg.angular.z = -float(state.yaw) ** 3 * self.config.command_scaling

        if not self.config.operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        return {
            "linear.x": twist_msg.linear.x,
            "linear.y": twist_msg.linear.y,
            "linear.z": twist_msg.linear.z,
            "angular.x": twist_msg.angular.x,
            "angular.y": twist_msg.angular.y,
            "angular.z": twist_msg.angular.z,
            "gripper_target": self._target_gripper_target,
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        pyspacemouse.close()
        self._is_connected = False
        pass
