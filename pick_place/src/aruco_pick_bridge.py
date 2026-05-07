#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from tb4_openx_interfaces.action import PickObject

MAX_X = 0.40
MIN_X = 0.15
MAX_Z = 0.30

class ArucoPickBridge(Node):
    def __init__(self):
        super().__init__('aruco_pick_bridge')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        transient_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(TFMessage, '/tf',
            lambda msg: [self.tf_buffer.set_transform(t, 'default_authority') for t in msg.transforms], 100)
        self.create_subscription(TFMessage, '/tf_static',
            lambda msg: [self.tf_buffer.set_transform_static(t, 'default_authority') for t in msg.transforms],
            transient_qos)
        self.pick_client = ActionClient(self, PickObject, 'pick_object')
        self.triggered = False
        self.timer = self.create_timer(1.0, self.check_and_pick)
        self.get_logger().info('ArucoPickBridge ready, waiting for marker_arm TF...')

    def check_and_pick(self):
        if self.triggered:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'marker_arm',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            self.get_logger().info(f'marker_arm found at: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            if not (MIN_X <= x <= MAX_X and z <= MAX_Z):
                self.get_logger().warn(
                    f'Pose out of reachable range, skipping.',
                    throttle_duration_sec=3.0)
                return
            self.triggered = True
            self.send_pick_goal(x, z)
        except Exception as e:
            self.get_logger().info(
                f'Waiting for marker_arm TF: {e}',
                throttle_duration_sec=3.0,
            )

    def send_pick_goal(self, x, z):
        goal = PickObject.Goal()
        goal.object_pose = PoseStamped()
        goal.object_pose.header.frame_id = 'base_link'
        goal.object_pose.header.stamp = self.get_clock().now().to_msg()
        goal.object_pose.pose.position.x = x
        goal.object_pose.pose.position.y = 0.0
        goal.object_pose.pose.position.z = z
        goal.object_pose.pose.orientation.w = 1.0
        goal.approach_height = 0.05
        self.get_logger().info(f'Sending pick goal: x={x:.3f}, y=0.0, z={z:.3f}')
        self.pick_client.wait_for_server()
        future = self.pick_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Phase: {feedback.feedback.current_phase}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pick goal rejected')
            return
        self.get_logger().info('Pick goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Pick succeeded: {result.message}')
        else:
            self.get_logger().error(f'Pick failed: {result.message}')
            self.triggered = False

def main():
    rclpy.init()
    node = ArucoPickBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
