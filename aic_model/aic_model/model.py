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
        self.get_logger().info('observation')


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
