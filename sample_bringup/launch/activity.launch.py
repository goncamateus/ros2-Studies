from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    robots = [
        "robot_0",
        "robot_1",
        "robot_2",
        "robot_3",
        "robot_4",
    ]
    for robot in robots:
        node = Node(
            package="tutocpp_pkg",
            executable="publisher_node",
            remappings=[
                ("__node", f"robot_news_station_{robot}"),
                ("number", "robot_news"),
            ],
        )
        ld.add_action(node)
    node = Node(
        package="tutocpp_pkg",
        executable="pubsub_node",
        parameters=[{"topic_name": "robot_news"}],
        remappings=[("__node", "smartphone"), ("number", "robot_news")],
    )
    ld.add_action(node)
    return ld
