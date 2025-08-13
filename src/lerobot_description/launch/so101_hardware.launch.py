#!/usr/bin/env python3
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package names - modify these to match your actual package names
    maxarm_description_pkg = FindPackageShare("lerobot_description")
    
    # Path to your python script relative to the package
    # URDF model argument using PathJoinSubstitution
    model_path = PathJoinSubstitution([
        maxarm_description_pkg,
        "urdf",
        "so101_feetech.urdf.xacro"
    ])
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=model_path,
        description="Absolute path to robot urdf file"
    )
    
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            model_path,
            " ",
            "device_path:=",
            "/dev/ttyACM0",
            # " ",
            # "calibration_path:=",
            # '"/home/miller/.cache/huggingface/lerobot/calibration/robots/so101_follower/erics_first_so101.json"',
        ]
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

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("lerobot_controller"),
                "config",
                "so101_controllers.yaml",
            ),
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    joint_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        # rviz_node,
        controller_manager,
        arm_controller_spawner,
        gripper_controller_spawner,
        joint_broadcaster_controller_spawner
    ])