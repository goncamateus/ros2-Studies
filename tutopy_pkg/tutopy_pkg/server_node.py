#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class ServerNode(Node):
    def __init__(self):
        super().__init__("server_node")
        self.create_service(AddTwoInts, "add_ints", self.add_callback)
        self.get_logger().info("Server Node has been started")

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response


def main(args=None):
    rclpy.init(args=args)

    node = ServerNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
