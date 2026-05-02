#!/usr/bin/env python3

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    omx_moveit_pkg = get_package_share_directory("open_manipulator_x_moveit_config")
    omx_bringup_pkg = get_package_share_directory("open_manipulator_x_bringup")
    omx_desc_pkg    = get_package_share_directory("open_manipulator_x_description")

    fake_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(omx_bringup_pkg, "launch", "fake.launch.py")
        ),
        launch_arguments={
            "use_sim": "false",       # CRITICAL: tells base.launch.py to start ros2_control_node
            "start_rviz": "false",    # we start our own RViz below
        }.items(),
    )

    robot_description_config = xacro.process_file(
        os.path.join(omx_desc_pkg, "urdf", "open_manipulator_x_robot.urdf.xacro")
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    with open(os.path.join(omx_moveit_pkg, "config", "open_manipulator_x.srdf")) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    with open(os.path.join(omx_moveit_pkg, "config", "kinematics.yaml")) as f:
        kinematics_yaml = yaml.safe_load(f)

    ompl_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    with open(os.path.join(omx_moveit_pkg, "config", "ompl_planning.yaml")) as f:
        ompl_pipeline["move_group"].update(yaml.safe_load(f))

    with open(os.path.join(omx_moveit_pkg, "config", "moveit_controllers.yaml")) as f:
        moveit_controllers_yaml = yaml.safe_load(f)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_pipeline,
            {
                "moveit_manage_controllers": True,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
            },
            {
                "moveit_simple_controller_manager": moveit_controllers_yaml,
                "moveit_controller_manager":
                    "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
            {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
            },
            {"use_sim_time": False},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(omx_moveit_pkg, "config", "moveit.rviz")],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_pipeline,
            kinematics_yaml,
            {"use_sim_time": False},
        ],
    )

    pick_place_node = Node(
        package="pick_place",
        executable="pick_place",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        fake_bringup,
        TimerAction(period=2.0, actions=[move_group_node]),
        TimerAction(period=3.0, actions=[rviz_node]),
        TimerAction(period=4.0, actions=[pick_place_node]),
    ])
