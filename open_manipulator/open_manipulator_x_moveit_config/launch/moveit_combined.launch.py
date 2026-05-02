#!/usr/bin/env python3
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    omx_moveit_pkg  = get_package_share_directory("open_manipulator_x_moveit_config")
    t4_manip_pkg    = get_package_share_directory("turtlebot4_manipulator_description")
    omx_bringup_pkg = get_package_share_directory("open_manipulator_x_bringup")

    combined_urdf = xacro.process_file(
        os.path.join(t4_manip_pkg, "urdf", "t4_manipulator.urdf.xacro"),
        mappings={
            "use_sim":              "false",
            "use_fake_hardware":    "true",
            "fake_sensor_commands": "true",
            "gazebo":               "ignition",
        }
    )
    robot_description = {"robot_description": combined_urdf.toxml()}

    omx_desc_pkg = get_package_share_directory("open_manipulator_x_description")
    omx_only_urdf = xacro.process_file(
        os.path.join(omx_desc_pkg, "urdf", "open_manipulator_x_robot.urdf.xacro"),
        mappings={
            "use_sim":              "false",
            "use_fake_hardware":    "true",
            "fake_sensor_commands": "true",
        }
    )
    robot_description_omx_only = {"robot_description": omx_only_urdf.toxml()}

    with open(os.path.join(omx_moveit_pkg, "config", "t4_manipulator.srdf")) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    with open(os.path.join(omx_moveit_pkg, "config", "kinematics.yaml")) as f:
        kinematics_yaml = yaml.safe_load(f)

    ompl_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "start_state_max_bounds_error": 0.5,
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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_omx_only,
            os.path.join(omx_bringup_pkg, "config", "hardware_controller_manager.yaml"),
        ],
        output="screen",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

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
                "start_state_max_bounds_error": 0.5,
                "jiggle_fraction": 0.05,
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
        control_node,
        rsp_node,
        jsb_spawner,
        TimerAction(period=2.0, actions=[arm_spawner, gripper_spawner]),
        TimerAction(period=3.0, actions=[move_group_node]),
        TimerAction(period=4.0, actions=[rviz_node]),
        TimerAction(period=5.0, actions=[pick_place_node]),
    ])
