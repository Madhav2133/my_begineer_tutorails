#!/usr/bin/env python3
"""
Launch file to record ROS 2 bag with all topics while running the publisher.

Accepts a command-line argument to enable/disable bag recording.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for bag recording with publisher."""
    # Declare launch argument to enable/disable bag recording
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="true",
        description="Enable bag recording (true/false).",
    )

    # Get the bag name argument (optional)
    bag_name_arg = DeclareLaunchArgument(
        "bag_name",
        default_value="rosbag2_recording",
        description="Name of the bag file to record.",
    )

    # Get the bag directory argument (optional)
    # Default to package results directory
    bag_directory_arg = DeclareLaunchArgument(
        "bag_directory",
        default_value="src/beginner_tutorials/results/bags",
        description="Directory to store the bag file.",
    )

    # Launch the publisher node
    publisher_node = Node(
        package="beginner_tutorials",
        executable="beginner_publisher",
        parameters=[{"publish_period_ms": 500}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Record all topics using ros2 bag record
    # Only execute if record_bag is true
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-a",  # Record all topics
            "-o",  # Output directory
            PathJoinSubstitution([
                LaunchConfiguration("bag_directory"),
                LaunchConfiguration("bag_name"),
            ]),
        ],
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output="screen",
    )

    # Log message when recording is disabled
    log_disabled = LogInfo(
        msg="Bag recording is disabled. Set record_bag:=true to enable.",
        condition=UnlessCondition(LaunchConfiguration("record_bag")),
    )

    return LaunchDescription([
        record_bag_arg,
        bag_name_arg,
        bag_directory_arg,
        publisher_node,
        log_disabled,
        bag_recorder,
    ])

