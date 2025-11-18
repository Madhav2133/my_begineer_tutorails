#!/usr/bin/env python3
"""
Launch file to start the beginner publisher and subscriber nodes.

Accepts command-line arguments to adjust publisher frequency and subscriber tag.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for publisher and subscriber nodes."""
    publish_period_arg = DeclareLaunchArgument(
        "publish_period_ms",
        default_value="500",
        description="Timer period for beginner publisher in milliseconds.",
    )

    subscriber_tag_arg = DeclareLaunchArgument(
        "subscriber_tag",
        default_value="default",
        description="Tag appended to subscriber log output.",
    )

    publisher_node = Node(
        package="beginner_tutorials",
        executable="beginner_publisher",
        parameters=[{"publish_period_ms": LaunchConfiguration("publish_period_ms")}],
        # arguments=["--ros-args", "--log-level", "debug"],
    )

    subscriber_node = Node(
        package="beginner_tutorials",
        executable="beginner_subscriber",
        parameters=[{"node_tag": LaunchConfiguration("subscriber_tag")}],
        # arguments=["--ros-args", "--log-level", "debug"],
    )

    return LaunchDescription(
        [
            publish_period_arg,
            subscriber_tag_arg,
            publisher_node,
            subscriber_node,
        ]
    )

