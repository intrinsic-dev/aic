import rclpy
from aic_model_interfaces.msg import Observation
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AicModel(Node):
    def __init__(self):
        super().__init__("aic_model")
        self.get_logger().info("Hello, world!")
        self.subscription = self.create_subscription(
            Observation, "observations", self.observation_callback, 10
        )

    def get_seconds(self, header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def observation_callback(self, msg):
        t_cam_0 = self.get_seconds(msg.images[0].header)
        t_cam_1 = self.get_seconds(msg.images[1].header)
        t_cam_2 = self.get_seconds(msg.images[2].header)
        t_joints = self.get_seconds(msg.joint_states.header)
        t_wrench = self.get_seconds(msg.wrench.header)
        self.get_logger().info(
            f"times: images [{t_cam_0}, {t_cam_1}, {t_cam_2}] joints {t_joints} wrench {t_wrench}"
        )


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
