#!/usr/bin/env python3
"""
Launch file to record ROS 2 bag with all topics while running the publisher.

Accepts a command-line argument to enable/disable bag recording.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def get_package_source_directory():
    """Get the package source directory by finding workspace src directory."""
    # Get the absolute path of this launch file
    launch_file_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(launch_file_path)
    
    # Walk up the directory tree to find workspace root (has 'src' directory)
    while current_dir != os.path.dirname(current_dir):  # Stop at filesystem root
        src_dir = os.path.join(current_dir, 'src')
        if os.path.exists(src_dir) and os.path.isdir(src_dir):
            # Found workspace root, look for package in src
            # Try to match package name from Node (beginner_tutorials) or use first found
            possible_names = ['beginner_tutorials', 'my_beginner_tutorials']
            for pkg_name in possible_names:
                pkg_dir = os.path.join(src_dir, pkg_name)
                if os.path.exists(pkg_dir) and os.path.isdir(pkg_dir):
                    return pkg_dir
            
            # If neither found, use first directory in src
            src_contents = [d for d in os.listdir(src_dir) 
                           if os.path.isdir(os.path.join(src_dir, d)) and not d.startswith('.')]
            if src_contents:
                return os.path.join(src_dir, src_contents[0])
        
        current_dir = os.path.dirname(current_dir)
    
    # Fallback: use relative path from launch file
    return os.path.dirname(os.path.dirname(launch_file_path))


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
    # Default to package source results directory (not install directory)
    pkg_source_dir = get_package_source_directory()
    default_bag_dir = os.path.join(pkg_source_dir, "results", "bags")
    
    bag_directory_arg = DeclareLaunchArgument(
        "bag_directory",
        default_value=default_bag_dir,
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

