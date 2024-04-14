from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    executables = ["turtle_follow", "turtle_spawn"]
    node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    ld.add_action(node)
    for exec in executables:
        node = Node(
            package="tutocpp_pkg",
            executable=exec,
        )
        ld.add_action(node)
    return ld
