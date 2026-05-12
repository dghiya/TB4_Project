import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  moveit_config = MoveItConfigsBuilder("open_manipulator_x").to_dict()
  
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

  tf_broadcaster_node = Node(
      package='tb4_openx_manipulation',  
      executable='aruco_to_tf.py',
      name='aruco_to_tf'
  )
  

  return LaunchDescription([
    aruco_node,
    pick_action_node,
    dispose_action_node,
    tf_broadcaster_node,
  ])