import rclpy
from rclpy.node import Node

class TestGymNode(Node):
    def __init__(self):
        super().__init__('test_gym_node')
        self.get_logger().info("TestGymNode has been started")

    def run_test(self):
        self.get_logger().info("Running test in TestGymNode")
        # Add any test or logic here


def main(args=None):
    rclpy.init(args=args)
    node = TestGymNode()
    node.run_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()