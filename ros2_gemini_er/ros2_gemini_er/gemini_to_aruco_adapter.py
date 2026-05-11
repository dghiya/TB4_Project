#!/usr/bin/env python3
"""
Gemini-to-ArUco adapter
========================
Bridges the Gemini perception TF into the ``/aruco_poses`` PoseArray topic
that the mission controller already subscribes to.

This node:
  1. Looks up the TF  ``oakd_rgb_camera_optical_frame`` -> ``marker``
     at 30 Hz (matching the Gemini republish rate).
  2. Packs the translation + rotation into a
     ``geometry_msgs/msg/PoseArray`` and publishes on ``/aruco_poses``.

Result: the navigation stack sees exactly the same interface it expects
from the ArUco pipeline, with zero changes to mission_controller.py.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros


class GeminiToArucoAdapter(Node):

    def __init__(self):
        super().__init__('gemini_to_aruco_adapter')

        # Parameters
        self.declare_parameter('camera_frame',
                               'oakd_rgb_camera_optical_frame')
        self.declare_parameter('marker_frame', 'marker')
        self.declare_parameter('publish_rate_hz', 30.0)

        self.camera_frame = (
            self.get_parameter('camera_frame')
            .get_parameter_value().string_value)
        self.marker_frame = (
            self.get_parameter('marker_frame')
            .get_parameter_value().string_value)
        rate = (self.get_parameter('publish_rate_hz')
                .get_parameter_value().double_value)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher — same topic the mission controller subscribes to
        self.pose_pub = self.create_publisher(PoseArray, '/aruco_poses', 10)

        # Timer
        self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(
            f'Adapter: TF {self.camera_frame} -> {self.marker_frame} '
            f'=> /aruco_poses @ {rate} Hz')

    # ------------------------------------------------------------------
    def _timer_cb(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, self.marker_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return  # TF not available yet — silent, no spam

        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation

        msg = PoseArray()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = self.camera_frame
        msg.poses.append(pose)

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GeminiToArucoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
