"""
Gemini Robotics-ER launch — drop-in ArUco replacement.

Spawns:
  * gemini_body_tf  — body OAKD camera  -> ``marker`` TF  (was marker_gemini)
  * gemini_arm_tf   — arm RealSense     -> ``marker_arm`` TF
  * gemini_to_aruco_adapter — converts the ``marker`` TF into a
    ``/aruco_poses`` PoseArray so the mission controller works unchanged.

The ArUco node in manipulation_pipeline.launch.py should be disabled
(commented out) when using this launch file to avoid two sources
publishing ``marker`` and ``/aruco_poses`` simultaneously.

Rate-limit note:
  Free-tier Gemini Robotics-ER cap is 5 RPM / 20 RPD.  With two nodes,
  default 0.025 Hz each = 3 RPM total — under the cap.  In latching mode
  (default), each node makes ~3 API calls to latch then stops completely.

Anchor frame:
  Default anchor is "odom", which is published by Gazebo's diff-drive
  controller — works right after gazebo_sim.launch.py with NO localization.
  If navigate.launch.py is also running, pass map_frame:=map for a
  globally-consistent anchor (AMCL required).

Usage (full navigation pipeline):
  ros2 launch tb4_openx_sim gazebo_sim.launch.py
  ros2 launch tb4_openx_navigation navigate.launch.py
  export GOOGLE_API_KEY="your-free-tier-key"
  ros2 launch ros2_gemini_er gemini_debug.launch.py map_frame:=map
  ros2 run tb4_openx_navigation mission_controller.py

Re-detect (resets latch, costs 3 more API calls per node):
  ros2 topic pub --once /gemini/redetect std_msgs/msg/Empty
  ros2 topic pub --once /gemini/redetect_arm std_msgs/msg/Empty
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    target_label = LaunchConfiguration('target_label')
    model_name = LaunchConfiguration('model_name')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    publish_rate_limit_hz = LaunchConfiguration('publish_rate_limit_hz')
    latch_in_map = LaunchConfiguration('latch_in_map')
    map_frame = LaunchConfiguration('map_frame')
    republish_rate_hz = LaunchConfiguration('republish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),

        DeclareLaunchArgument('target_label', default_value='artag',
                              description='Object to detect'),

        DeclareLaunchArgument('model_name',
                              default_value='gemini-robotics-er-1.6-preview',
                              description='Gemini Robotics-ER model name'),

        DeclareLaunchArgument('confidence_threshold', default_value='0.4',
                              description='Min confidence to publish TF'),

        DeclareLaunchArgument('publish_rate_limit_hz', default_value='0.025',
                              description='Per-node Gemini call rate '
                                          '(0.025 Hz each * 2 nodes = 3 RPM, '
                                          'free-tier cap is 5 RPM)'),

        DeclareLaunchArgument('latch_in_map', default_value='true',
                              description='If true, lock marker in map frame '
                                          'after first detection and stop '
                                          'calling the API. Send Empty msg to '
                                          '/gemini/redetect to refresh.'),
        DeclareLaunchArgument('map_frame', default_value='odom',
                              description='Anchor frame for the cached marker '
                                          'pose. "odom" works with sim alone; '
                                          'pass "map" when Nav2/AMCL is running'),
        DeclareLaunchArgument('republish_rate_hz', default_value='30.0',
                              description='Rate at which camera->marker is '
                                          're-emitted from the cached pose'),

        # ---- Body camera node ---------------------------------------
        #      target_frame = "marker"  (drop-in for ArUco)
        Node(
            package='ros2_gemini_er',
            executable='gemini_body_tf_node',
            name='gemini_body_tf',
            output='screen',
            parameters=[{
                'target_label':           target_label,
                'camera_frame':           'oakd_rgb_camera_optical_frame',
                'target_frame':           'marker',
                'model_name':             model_name,
                'confidence_threshold':   confidence_threshold,
                'depth_patch_size':       5,
                'orientation_patch_size': 21,
                'min_plane_points':       12,
                'mad_outlier_threshold':  2.5,
                'min_consecutive_detections': 3,
                'latch_position_tolerance_m': 0.10,
                'publish_rate_limit_hz':  publish_rate_limit_hz,
                'use_sim_time':           use_sim_time,
                'rgb_topic':              '/oakd/rgb/preview/image_raw',
                'depth_topic':            '/oakd/depth/image_raw',
                'camera_info_topic':      '/oakd/rgb/preview/camera_info',
                'latch_in_map':           latch_in_map,
                'map_frame':              map_frame,
                'republish_rate_hz':      republish_rate_hz,
                'redetect_topic':         '/gemini/redetect',
            }],
        ),

        # ---- Arm (eye-in-hand) camera node --------------------------
        Node(
            package='ros2_gemini_er',
            executable='gemini_body_tf_node',
            name='gemini_arm_tf',
            output='screen',
            parameters=[{
                'target_label':           target_label,
                'camera_frame':           'arm_rgb_camera_optical_frame',
                'target_frame':           'marker_arm',
                'model_name':             model_name,
                'confidence_threshold':   confidence_threshold,
                'depth_patch_size':       5,
                'orientation_patch_size': 21,
                'min_plane_points':       12,
                'mad_outlier_threshold':  2.5,
                'min_consecutive_detections': 3,
                'latch_position_tolerance_m': 0.10,
                'publish_rate_limit_hz':  publish_rate_limit_hz,
                'use_sim_time':           use_sim_time,
                'rgb_topic':              '/arm/rgb/image_raw',
                'depth_topic':            '/arm/depth/image_raw',
                'camera_info_topic':      '/arm/rgb/camera_info',
                'latch_in_map':           latch_in_map,
                'map_frame':              map_frame,
                'republish_rate_hz':      republish_rate_hz,
                'redetect_topic':         '/gemini/redetect_arm',
            }],
        ),

        # ---- Adapter: TF "marker" -> /aruco_poses PoseArray ---------
        #      Bridges Gemini TF into the topic the mission controller
        #      subscribes to. No changes needed in navigation code.
        Node(
            package='ros2_gemini_er',
            executable='gemini_to_aruco_adapter',
            name='gemini_to_aruco_adapter',
            output='screen',
            parameters=[{
                'camera_frame':    'oakd_rgb_camera_optical_frame',
                'marker_frame':    'marker',
                'publish_rate_hz': 30.0,
                'use_sim_time':    use_sim_time,
            }],
        ),
    ])
