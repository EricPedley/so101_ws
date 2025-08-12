#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from pathlib import Path


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Package names - modify these to match your actual package names
    isaac_package = FindPackageShare('lerobot_isaac')
    
    # Path to your bash script relative to the package
    launch_isaac_sim = PathJoinSubstitution([
        f'{Path.home()}/IsaacLab/_isaac_sim/python.sh',
        isaac_package,
        'so101_isaac.py'
    ])
    
    maxarm_description_dir = FindPackageShare("lerobot_description")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(maxarm_description_dir, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(maxarm_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription(
        [
            model_arg,
            launch_isaac_sim,
            robot_state_publisher_node,
            rviz_node
        ]
    )