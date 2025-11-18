#!/usr/bin/env python3
"""
Launch file for integration tests.
Launches the beginner_publisher node and the integration test node.
"""

from launch_catch_ros2 import Catch2IntegrationTestNode, Catch2LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch integration test for beginner_publisher."""
    # Catch2LaunchDescription wraps LaunchDescription and adds required "result_file" argument
    return Catch2LaunchDescription([
        # Launch the node under test (beginner_publisher)
        Node(
            package="beginner_tutorials",
            executable="beginner_publisher",
            parameters=[{"publish_period_ms": 500}],
        ),
        # Catch2IntegrationTestNode: wrapper around Node that passes "result_file" to Catch2
        # This node will shutdown the entire launch file when it exits
        Catch2IntegrationTestNode(
            package="beginner_tutorials",
            executable="integration_test_node",
        ),
    ])
