from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    executables = ["publisher_node", "subscriber_node"]
    for exec in executables:
        node = Node(
            package="tutocpp_pkg",
            executable=exec,
            parameters=[{"topic_name": "number"}],
        )
        ld.add_action(node)
    return ld
