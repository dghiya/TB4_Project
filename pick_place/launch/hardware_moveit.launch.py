#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    moveit_config_pkg = get_package_share_directory('open_manipulator_x_moveit_config')

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
        )]
    )

    return LaunchDescription([
        move_group,
        pick_place_node,
    ])
