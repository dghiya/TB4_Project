import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    tb4_nav_dir = get_package_share_directory('turtlebot4_navigation')
    tb4_viz_dir = get_package_share_directory('turtlebot4_viz')

    # 1. Bypass the RViz2 OpenGL Map Rendering Bug
    # set_software_gl = AppendEnvironmentVariable(
    #     'LIBGL_ALWAYS_SOFTWARE', '1'
    # )

    # 2. Start Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': '', 
            'map': os.path.join(tb4_nav_dir, 'maps', 'warehouse.yaml'),
            'remap_rule': 'odom:=/diffdrive_controller/odom' 
        }.items(),
    )

    # 3. Start Nav2 
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_dir, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': ''   
        }.items(),
    )

    # 4. Start RViz Automatically
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_viz_dir, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': ''
        }.items()
    )

    return LaunchDescription([
        # set_software_gl,
        localization,
        nav2,
        rviz
    ])