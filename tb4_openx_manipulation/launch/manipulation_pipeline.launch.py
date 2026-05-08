import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  moveit_config = MoveItConfigsBuilder("open_manipulator_x").to_dict()

  approach_action_node = Node(
    name="approach_action_node",
    package="tb4_openx_manipulation",
    executable="approach_trash_server",
    parameters=[moveit_config, {'use_sim_time': True}]
  )
  
  aruco_params = os.path.join(
    get_package_share_directory('tb4_openx_manipulation'),
    'config', 'aruco_parameters.yaml'
  )

  aruco_node = Node(
      package='ros2_aruco',
      executable='aruco_node',
      parameters=[aruco_params]
  )
  
  pick_action_node = Node(
    name="pick_action_node",
    package="tb4_openx_manipulation",
    executable="pick_trash_server",
    parameters=[moveit_config, {'use_sim_time': True}]
  )
  
  dispose_action_node = Node(
    name="dispose_action_node",
    package="tb4_openx_manipulation",
    executable="place_trash_server",
    parameters=[moveit_config, {'use_sim_time': True}]
  )

  trash_collection_task_node = Node(
      package='tb4_openx_manipulation',
      executable='trash_collection_task.py'
  )
  
  return LaunchDescription([
    aruco_node,
    # approach_action_node,
    pick_action_node,
    dispose_action_node,
    # trash_collection_task_node # We will run this manually in a terminal to test!
  ])