#!/usr/bin/env python3
import rclpy
import rclpy.time

from functools import partial
from gonca_interfaces.srv import BatteryStatus
from rclpy.node import Node


class LEDClientNode(Node):
    def __init__(self):
        super().__init__("LED_client_node")
        self.led_to_set = 3
        self.battery_full = False
        self.last_time = self.get_time_seconds()
        self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("LED Client Node has been started")

    def get_time_seconds(self):
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        return sec + nano / 1e9

    def check_battery_state(self):
        time_now = self.get_time_seconds()
        time_diff = time_now - self.last_time
        changed_state = False
        if self.battery_full and time_diff > 4.0:
            self.battery_full = False
            changed_state = True
            self.last_time = time_now
        elif not self.battery_full and time_diff > 6.0:
            self.battery_full = True
            changed_state = True
            self.last_time = time_now
        if changed_state:
            self.call_set_led(self.led_to_set, self.battery_full)

    def call_set_led(self, led, battery_full):
        client = self.create_client(BatteryStatus, "set_led")
        while not client.wait_for_service(0.5):
            self.get_logger().warn("Client waiting for server...")

        request = BatteryStatus.Request()
        request.led = led
        request.state = not battery_full

        future = client.call_async(request=request)
        future.add_done_callback(partial(self.set_led_callback, led=led, state=not battery_full))

    def set_led_callback(self, future, led, state):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"LED {led} was set {state}")
        except Exception as e:
            self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)

    node = LEDClientNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
