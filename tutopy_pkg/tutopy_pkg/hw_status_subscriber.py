#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gonca_interfaces.msg import HardwareInfo


class HardwareInfoSubscriberNode(Node):
    def __init__(self):
        super().__init__("hw_info_subscriber")
        self.subscriber_ = self.create_subscription(HardwareInfo, "hw_info", self.subscriber_callback, 10)
        self.get_logger().info("Hardware Info Subscriber has been started.")

    def subscriber_callback(self, msg):
        self.get_logger().info(f"Temperature: {msg.temperature}")
        self.get_logger().info(f"Are motors ready: {msg.are_motors_ready}")
        self.get_logger().info(f"Debug message: {msg.debug_message}")


def main(args=None):
    rclpy.init(args=args)

    node = HardwareInfoSubscriberNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
