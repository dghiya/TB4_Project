#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    moveit_config_pkg = get_package_share_directory('open_manipulator_x_moveit_config')

    load_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen',
        )]
    )

    load_arm_controller = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
            output='screen',
        )]
    )

    load_gripper_controller = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
            output='screen',
        )]
    )

    move_group = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        )]
    )

    pick_place_node = TimerAction(
        period=25.0,
        actions=[Node(
            package='pick_place',
            executable='pick_place',
            name='omx_pick_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    return LaunchDescription([
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        move_group,
        pick_place_node,
    ])
