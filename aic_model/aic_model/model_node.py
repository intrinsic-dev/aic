import rclpy
import textwrap
import threading

from aic_model_interfaces.msg import Observation
from aic_task_interfaces.action import InsertCable
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class AicModel(Node):
    def __init__(self):
        super().__init__("aic_model")
        self.get_logger().info("Hello, world!")
        self.subscription = self.create_subscription(
            Observation, "observations", self.observation_callback, 10
        )
        self.goal_handle = None
        self.goal_handle_lock = threading.Lock()
        self.action_server = ActionServer(
            self, InsertCable, "insert_cable",
            execute_callback=self.insert_cable_execute_callback,
            goal_callback=self.insert_cable_goal_callback,
            handle_accepted_callback=self.insert_cable_accepted_goal_callback,
            cancel_callback=self.insert_cable_cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def get_seconds(self, header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def observation_callback(self, msg):
        t_cam_0 = self.get_seconds(msg.images[0].header)
        t_cam_1 = self.get_seconds(msg.images[1].header)
        t_cam_2 = self.get_seconds(msg.images[2].header)
        t_joints = self.get_seconds(msg.joint_states.header)
        t_wrench = self.get_seconds(msg.wrist_wrench.header)
        tcp_x = msg.tcp_to_world.transform.translation.x
        tcp_y = msg.tcp_to_world.transform.translation.y
        tcp_z = msg.tcp_to_world.transform.translation.z
        self.get_logger().info(
            f"times: images [{t_cam_0}, {t_cam_1}, {t_cam_2}] joints {t_joints} wrench {t_wrench} tcp: ({tcp_x:+0.4f} {tcp_y:+0.4f}, {tcp_z:+0.4f})"
        )
        with self.goal_handle_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.goal_handle.execute()

    def task_to_string(self, task):
        return textwrap.dedent(f"""
            id: {task.id}
            cable_type: {task.cable_type}
            cable_name: {task.cable_name}
            plug_type: {task.plug_type}
            plug_name: {task.plug_name}
            port_type: {task.port_type}
            port_name: {task.port_name}
            target_module_name: {task.target_module_name}
            time_limit: {task.time_limit}
        """

    def insert_cable_goal_callback(self, goal_request):
        with self.goal_handle_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().error("A goal is active and must be canceled before a new insert_cable goal can begin")
                return GoalResponse.REJECT
            else:
                self.get_logger().info("Goal accepted")
                return GoalResponse.ACCEPT

    def insert_cable_accepted_goal_callback(self, goal_handle):
        self.get_logger().info(f"Accepted insert cable goal:\n" + self.task_to_string(goal_handle.request.task))
        self.goal_handle = goal_handle
        self.goal_handle.execute()

    async def insert_cable_execute_callback(self, goal_handle):


    def finish_active_task(self):
        if not self.goal_handle:
            self.get_logger().error("finish_active_task(): No active goal")
            return
        self.goal_handle.

        result = InsertCable.Result()
        result.success = True
        result.message = "Hooray!"
        return result


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
