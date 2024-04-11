#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class SubscriberNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        self.subsciber_ = self.create_subscription(
            String, "first_publisher", self.callback_publisher, 10
        )
        self.get_logger().info("Subscriber Node has been started")

    def callback_publisher(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = SubscriberNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
