#!/usr/bin/env python3
import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory('open_manipulator_x_moveit_config')

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("open_manipulator_x_description"),
            "urdf",
            "open_manipulator_x_robot.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # SRDF
    srdf_path = os.path.join(moveit_config_pkg, "config", "open_manipulator_x_real.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Kinematics
    kinematics_yaml_path = os.path.join(moveit_config_pkg, "config", "kinematics.yaml")
    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    move_group = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_pkg, 'launch', 'move_group_real.launch.py')
            ),
        )]
    )

    pick_place_node = TimerAction(
        period=12.0,
        actions=[Node(
            package='pick_place',
            executable='pick_place',
            name='omx_pick_server',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
            ],
        )]
    )

    return LaunchDescription([
        move_group,
        pick_place_node,
    ])
