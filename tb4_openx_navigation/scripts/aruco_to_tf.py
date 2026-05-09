#!/usr/bin/env python3
"""
Subscribe to /aruco_poses (PoseArray) and broadcast the first marker
as TF frame "marker" relative to the camera optical frame.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster

CAMERA_FRAME = "tb4_openx/oakd_rgb_camera_frame/rgbd_camera"

class ArucoToTf(Node):
    def __init__(self):
        super().__init__('aruco_to_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(PoseArray, '/aruco_poses', self.cb, 10)
        self.get_logger().info(f'aruco_to_tf bridging /aruco_poses -> TF "marker" (parent: {CAMERA_FRAME})')

    def cb(self, msg: PoseArray):
        if not msg.poses:
            return
        p = msg.poses[0]
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or CAMERA_FRAME
        t.child_frame_id = 'marker'
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(ArucoToTf())

if __name__ == '__main__':
    main()
