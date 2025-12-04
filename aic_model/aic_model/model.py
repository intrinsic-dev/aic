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
        t0 = msg.wrist_cameras[0].header.stamp.sec + msg.wrist_cameras[0].header.stamp.nanosec / 1e9
        t1 = msg.wrist_cameras[1].header.stamp.sec + msg.wrist_cameras[1].header.stamp.nanosec / 1e9
        t2 = msg.wrist_cameras[2].header.stamp.sec + msg.wrist_cameras[2].header.stamp.nanosec / 1e9
        tj = msg.joint_states.header.stamp.sec + msg.joint_states.header.stamp.nanosec / 1e9
        self.get_logger().info(f'observation times: images [{t0}, {t1}, {t2}] joints {tj}')


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
