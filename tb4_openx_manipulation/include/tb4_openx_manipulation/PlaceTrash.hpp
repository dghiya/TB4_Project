#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tb4_openx_interfaces/action/place.hpp"

namespace tb4_openx_manipulation {

class PlaceTrashServer : public rclcpp::Node
{
public:
  using PlaceTrash = tb4_openx_interfaces::action::Place;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlaceTrash>;

  explicit PlaceTrashServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<PlaceTrash>::SharedPtr action_server_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp::CallbackGroup::SharedPtr nav_cb_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  void initialize_moveit();
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlaceTrash::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute_place(const std::shared_ptr<GoalHandle> goal_handle);
};

}
