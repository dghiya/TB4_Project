#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Action client to talk to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher to spin the robot in place
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define our warehouse zones using Quaternions (w=1.0 means no rotation)
        self.zones = {
            'home':   {'x': 0.0,  'y': 0.0,  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'area_1': {'x': 2.0,  'y': 0.0,  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'area_2': {'x': -2.0, 'y': 2.0,  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'area_3': {'x': 1.0,  'y': -2.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
        }

    def go_to_pose(self, zone_name):
        self.get_logger().info(f'Navigating to {zone_name}...')
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Load the coordinates and Quaternions from the dictionary
        target = self.zones[zone_name]
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.x = target['qx']
        goal_msg.pose.pose.orientation.y = target['qy']
        goal_msg.pose.pose.orientation.z = target['qz']
        goal_msg.pose.pose.orientation.w = target['qw']
        
        # Send the goal and wait for it to finish
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal!')
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def spin_and_search(self):
        self.get_logger().info('Spinning to search for ArTag...')
        spin_cmd = Twist()
        spin_cmd.angular.z = 0.3  # Spin slowly to the left
        
        # Spin for 5 seconds (later, we will interrupt this when the camera sees the tag)
        start_time = time.time()
        while time.time() - start_time < 15.0:
            self.vel_pub.publish(spin_cmd)
            time.sleep(0.1)
            
        # Stop the robot
        spin_cmd.angular.z = 0.0
        self.vel_pub.publish(spin_cmd)
        self.get_logger().warn('ArTag NOT found in this area.')

    def execute_mission(self, target_area):
        # 1. Drive to the requested area
        success = self.go_to_pose(target_area)
        if not success:
            self.get_logger().error("Mission Aborted: Could not reach target area.")
            return
            
        # 2. Spin to search for the tag
        self.spin_and_search()
        
        # 3. Return Home safely
        self.get_logger().info('Returning to Safe Home Position...')
        self.go_to_pose('home')
        self.get_logger().info('Mission Complete. Robot is safely home.')

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    # Run the mission for area 1
    node.execute_mission('area_1') 
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()