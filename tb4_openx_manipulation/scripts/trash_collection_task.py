#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from tb4_openx_interfaces.action import Approach, Pick, Place

class TrashStateMachine(Node):
    def __init__(self):
        super().__init__('trash_state_machine')
        self.approach_client = ActionClient(self, Approach, 'approach_trash')
        self.pick_client = ActionClient(self, Pick, 'pick_trash')
        self.place_client = ActionClient(self, Place, 'place_trash')

    def call_approach_trash(self, marker_frame):
        self.get_logger().info('Waiting for ApproachTrash action server...')
        self.approach_client.wait_for_server()
        goal_msg = Approach.Goal()
        goal_msg.marker_frame = marker_frame
        self.get_logger().info(f'Sending ApproachTrash goal...')
        future = self.approach_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Approach goal rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def call_pick_trash(self):
        self.get_logger().info('Waiting for PickTrash action server...')
        self.pick_client.wait_for_server()
        goal_msg = Pick.Goal()
        goal_msg.marker_frame = "marker_arm"
        future = self.pick_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

def main(args=None):
    rclpy.init(args=args)
    node = TrashStateMachine()
    try:
        node.get_logger().info('Starting manipulation sequence...')
        success = node.call_approach_trash("marker")
        if success:
            node.get_logger().info('Approached marker successfully. Picking...')
            time.sleep(2.0)
            pick_success = node.call_pick_trash()
            if pick_success:
                node.get_logger().info("Pick successful!")
            else:
                node.get_logger().error("Pick failed.")
        else:
            node.get_logger().error('Failed to approach marker.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()