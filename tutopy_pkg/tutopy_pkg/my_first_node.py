#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("py_test_node")
        self._counter = 0
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self._counter += 1
        self.get_logger().info("Hello " + str(self._counter))


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
