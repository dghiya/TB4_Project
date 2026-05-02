import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define package paths
    pkg_description = get_package_share_directory('tb4_openx_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the XACRO file we cleaned up earlier
    xacro_file = os.path.join(pkg_description, 'urdf', 't4_manipulator.urdf.xacro')

    # 2. Setup Robot State Publisher (Processes Xacro -> URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file, ' use_sim:=true']),
            'use_sim_time': True
        }]
    )

    # 3. Start Ignition Gazebo with a Warehouse/Depot World
    # 'depot.sdf' is a standard Ignition warehouse world.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 depot.sdf'}.items(),
    )

    # 4. Spawn the robot into Ignition
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'tb4_openx',
            '-allow_renaming', 'true',
            '-z', '0.1' # Drop it slightly above the ground
        ]
    )

    # 5. Spawners for the Controllers defined in your YAML
    load_joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_diffdrive_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["diffdrive_controller", "--controller-manager", "/controller_manager"],
    )

    load_arm_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    load_gripper_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 6. Build the Launch Description
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diffdrive_controller,
        load_arm_controller,
        load_gripper_controller
    ])