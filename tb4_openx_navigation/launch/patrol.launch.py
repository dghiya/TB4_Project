import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('tb4_openx_navigation'),
        'config', 'patrol_waypoints.yaml')

    patrol_node = Node(
            package='tb4_openx_navigation',
            executable='patrol_node',
            name='patrol_robot',
            parameters=[param_file],
            output='screen',
        )
    
    return LaunchDescription([
        # patrol_node
    ])