#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _make_nodes(context):
    run_name = LaunchConfiguration("run_name").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)

    output_file = os.path.join(output_dir, f"{run_name}_floorplan.txt")

    localizer_node = Node(
        name="floorplan_localizer",
        package="challenge_tools_ros",
        executable="floorplan_localizer",
        output="screen",
        arguments=[run_name, output_file, odom_topic],
    )

    return [localizer_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_name",
                description="Run name in format floor_X_YYYY-MM-DD_run_X",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/ros2_ws/results",
                description="Directory to write output TUM file",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/loop_fusion/odometry_rect",
                description="Input odometry topic",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
