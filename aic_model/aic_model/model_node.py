import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from aic_model_interfaces.msg import Observation


class AicModel(Node):
    def __init__(self):
        super().__init__('aic_model')
        self.get_logger().info('Hello, world!')
        self.subscription = self.create_subscription(
            Observation,
            'observations',
            self.observation_callback,
            10)

    def observation_callback(self, msg):
        t_cam_0 = msg.images[0].header.stamp.sec + msg.images[0].header.stamp.nanosec / 1e9
        t_cam_1 = msg.images[1].header.stamp.sec + msg.images[1].header.stamp.nanosec / 1e9
        t_cam_2 = msg.images[2].header.stamp.sec + msg.images[2].header.stamp.nanosec / 1e9
        t_joints = msg.joint_states.header.stamp.sec + msg.joint_states.header.stamp.nanosec / 1e9
        t_wrench = msg.wrench.header.stamp.sec + msg.wrench.header.stamp.nanosec / 1e9
        self.get_logger().info(f'observation times: images [{t_cam_0}, {t_cam_1}, {t_cam_2}] joints {t_joints} wrench {t_wrench}')


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
