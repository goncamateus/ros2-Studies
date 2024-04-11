#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gonca_interfaces.msg import HardwareInfo


class HardwareInfoPublisherNode(Node):
    def __init__(self):
        super().__init__("hw_info_publisher")
        self.publisher_ = self.create_publisher(HardwareInfo, "hw_info", 10)
        self.create_timer(0.5, self.publish_callback)
        self.get_logger().info("Hardware Info Publisher has been started.")

    def publish_callback(self):
        msg = HardwareInfo()
        msg.temperature = 50
        msg.are_motors_ready = True
        msg.debug_message = "Temperature is high! Please check the cooling system."
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = HardwareInfoPublisherNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
