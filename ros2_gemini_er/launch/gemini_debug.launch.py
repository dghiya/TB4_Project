"""
Gemini Robotics-ER debug launch — Phase 2.

Spawns TWO Gemini Robotics-ER nodes in parallel with the existing ArUco
pipeline:
  * gemini_body_tf  — body/front OAKD camera   -> marker_gemini  TF
  * gemini_arm_tf   — arm-mounted RealSense    -> marker_gemini_arm TF

Each node publishes orientation derived from a plane fit + edge-based
in-plane recovery on the depth/RGB patches around the detected pixel.

Rate-limit note:
  Free-tier Gemini Robotics-ER cap is 5 RPM / 20 RPD.  With two nodes,
  default 0.025 Hz each = 3 RPM total — under the cap.  In latching mode
  (default), each node makes ~3 API calls to latch then stops completely.

Anchor frame:
  Default anchor is "odom", which is published by Gazebo's diff-drive
  controller — works right after gazebo_sim.launch.py with NO localization.
  If navigate.launch.py is also running, pass map_frame:=map for a
  globally-consistent anchor (AMCL required).

Usage — sim only (no Nav2):
  ros2 launch tb4_openx_sim gazebo_sim.launch.py
  export GOOGLE_API_KEY="your-free-tier-key"
  ros2 launch ros2_gemini_er gemini_debug.launch.py

Usage — with navigation:
  ros2 launch tb4_openx_sim gazebo_sim.launch.py
  ros2 launch tb4_openx_navigation navigate.launch.py
  export GOOGLE_API_KEY="your-free-tier-key"
  ros2 launch ros2_gemini_er gemini_debug.launch.py map_frame:=map

Re-detect (resets latch, costs 3 more API calls per node):
  ros2 topic pub --once /gemini/redetect std_msgs/msg/Empty
  ros2 topic pub --once /gemini/redetect_arm std_msgs/msg/Empty

Compare body-cam TFs:
  ros2 run tf2_ros tf2_echo oakd_rgb_camera_optical_frame marker
  ros2 run tf2_ros tf2_echo oakd_rgb_camera_optical_frame marker_gemini
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

        # Per-node rate. Two nodes share the 5 RPM free-tier budget,
        # so 0.025 Hz each = 3 RPM total — under the cap.
        DeclareLaunchArgument('publish_rate_limit_hz', default_value='0.025',
                              description='Per-node Gemini call rate '
                                          '(0.025 Hz each * 2 nodes = 3 RPM, '
                                          'free-tier cap is 5 RPM)'),

        # Latching adapter: one-shot Gemini -> map-frame anchor -> republish
        # camera->marker at high rate from cache. Set false for raw debug.
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
        Node(
            package='ros2_gemini_er',
            executable='gemini_body_tf_node',
            name='gemini_body_tf',
            output='screen',
            parameters=[{
                'target_label':           target_label,
                'camera_frame':           'oakd_rgb_camera_optical_frame',
                'target_frame':           'marker_gemini',
                'model_name':             model_name,
                'confidence_threshold':   confidence_threshold,
                'depth_patch_size':       5,
                'orientation_patch_size': 21,
                'min_plane_points':       12,
                'mad_outlier_threshold':  2.5,
                # Robustness: require N agreeing detections before locking
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
            executable='gemini_body_tf_node',   # same executable, different params
            name='gemini_arm_tf',
            output='screen',
            parameters=[{
                'target_label':           target_label,
                'camera_frame':           'arm_rgb_camera_optical_frame',
                'target_frame':           'marker_gemini_arm',
                'model_name':             model_name,
                'confidence_threshold':   confidence_threshold,
                'depth_patch_size':       5,
                'orientation_patch_size': 21,
                'min_plane_points':       12,
                'mad_outlier_threshold':  2.5,
                # Robustness: require N agreeing detections before locking
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
                # Separate redetect topic so body and arm can be refreshed
                # independently if needed.
                'redetect_topic':         '/gemini/redetect_arm',
            }],
        ),
    ])
