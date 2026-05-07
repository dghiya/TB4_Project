import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tb4_nav_dir = get_package_share_directory('turtlebot4_navigation')
    tb4_viz_dir = get_package_share_directory('turtlebot4_viz')
    pkg_nav = get_package_share_directory('tb4_openx_navigation')

    map_path = os.path.join(pkg_nav, 'maps', 'wyman_demo.yaml')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'namespace': '',
            'map': map_path,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_dir, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'namespace': ''
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_viz_dir, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'namespace': ''
        }.items()
    )

    return LaunchDescription([
        localization,
        nav2,
        rviz
    ])
