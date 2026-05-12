import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('tb4_openx_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tb4_ign = get_package_share_directory('turtlebot4_ignition_bringup')

    xacro_file = os.path.join(pkg_description, 'urdf', 't4_manipulator.urdf.xacro')
    artag_xacro_file = os.path.join(pkg_description, 'models', 'artag', 'artag.urdf.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file, ' use_sim:=true gazebo:=ignition']),
            'use_sim_time': True
        }]
    )
    
    artag_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='artag_state_publisher',
        namespace='trash_block',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', artag_xacro_file]), value_type=str)
        }]
    )

    warehouse_world = os.path.join(pkg_tb4_ign, 'worlds', 'warehouse.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r -v 4 {warehouse_world}"}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'tb4_openx',
            '-allow_renaming', 'true',
            '-z', '0.1' 
        ]
    )

    # Syncs Ignition's time to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ]
    )


    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/world/warehouse/model/tb4_openx/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        remappings=[
            ('/world/warehouse/model/tb4_openx/link/rplidar_link/sensor/rplidar/scan', '/scan')
        ]
    )
    
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/world/warehouse/model/tb4_openx/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/warehouse/model/tb4_openx/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        remappings=[
            ('/world/warehouse/model/tb4_openx/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image', '/oakd/rgb/preview/image_raw'),
            ('/world/warehouse/model/tb4_openx/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info', '/oakd/rgb/preview/camera_info')
        ]
    )
    
    rplidar_stf = Node(
        name='rplidar_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'rplidar_link', 'tb4_openx/rplidar_link/rplidar'
        ]
    )
    
    footprint_stf = Node(
        name='footprint_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    spawn_artag_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/trash_block/robot_description',
            '-name', 'trash_block',
            '-x', '2.3',
            '-y', '0.5',
            '-z', '0.3',
            '-R', '1.57',
            '-P', '0.0',   
            '-Y', '-1.57',
        ],
        output='screen'
    )
    
    # Area 2
    spawn_artag_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/trash_block/robot_description',
            '-name', 'trash_block_2', 
            '-x', '-0.4',             
            '-y', '-5.0',              
            '-z', '0.3',
            '-P', '1.57',   
            '-Y', '0.0',
        ],
        output='screen'
    )

    # Area 3
    spawn_artag_3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/trash_block/robot_description',
            '-name', 'trash_block_3', 
            '-x', '-4.0',              
            '-y', '-3.0',             
            '-z', '0.2',
            '-P', '1.57',   
            '-Y', '0.0',
        ],
        output='screen'
    )

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

    artags = LaunchDescription([
        spawn_artag_1,
        spawn_artag_2,
        spawn_artag_3
    ])
    
    return LaunchDescription([
        clock_bridge,  
        lidar_bridge, 
        camera_bridge,          
        rplidar_stf,
        footprint_stf,
        gazebo,
        robot_state_publisher,
        artag_state_publisher, 
        spawn_entity,
        artags,           
        load_joint_state_broadcaster,
        load_diffdrive_controller,
        load_arm_controller,
        load_gripper_controller
    ])