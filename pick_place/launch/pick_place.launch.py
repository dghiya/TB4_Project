import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("open_manipulator_x", 
                                        package_name="open_manipulator_x_moveit_config").to_moveit_configs()
    
    moveit_node = Node(
        name="openmanipulator_pick_place",
        package="pick_place", 
        executable="pick_place",  
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )
    
    return LaunchDescription([
        moveit_node
    ])