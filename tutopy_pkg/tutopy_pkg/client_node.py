#!/usr/bin/env python3
import rclpy

from example_interfaces.srv import AddTwoInts
from functools import partial
from rclpy.node import Node


class ClientNode(Node):
    def __init__(self):
        super().__init__("client_node")
        self.call_server(6, 7)

    def call_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_done, a=a, b=b))

    def callback_done(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = ClientNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
