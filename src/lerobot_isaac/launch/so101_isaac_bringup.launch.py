#!/usr/bin/env python3
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package names - modify these to match your actual package names
    isaac_package = FindPackageShare('lerobot_isaac')
    maxarm_description_pkg = FindPackageShare("lerobot_description")
    
    # Path to your python script relative to the package
    isaac_so101_config_path = PathJoinSubstitution([
        isaac_package,
        'so101_isaac.py'
    ])
    
    # Launch Isaac Sim with the script
    launch_isaac_sim = ExecuteProcess(
        cmd=[str(Path.home() / 'IsaacLab' / '_isaac_sim' / 'python.sh'), isaac_so101_config_path],
        name='isaac_sim',
        output='screen',
        shell=True
    )
    
    # URDF model argument using PathJoinSubstitution
    model_path = PathJoinSubstitution([
        maxarm_description_pkg,
        "urdf",
        "so101.urdf.xacro"
    ])
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=model_path,
        description="Absolute path to robot urdf file"
    )
    
    # Robot description parameter
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), 
        value_type=str
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    # RViz node with config file
    rviz_config_path = PathJoinSubstitution([
        maxarm_description_pkg,
        "rviz",
        "display.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )
    
    return LaunchDescription([
        model_arg,
        launch_isaac_sim,
        robot_state_publisher_node,
        rviz_node
    ])