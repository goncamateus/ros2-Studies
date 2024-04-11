#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.publisher_ = self.create_publisher(String, "first_publisher", 10)
        self.timer_ = self.create_timer(0.5, self.publish)
        self.get_logger().info("Publisher Noder has been started")

    def publish(self):
        msg = String()
        msg.data = "Hi Python"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PublisherNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
