#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tb4_openx_interfaces/action/pick.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tb4_openx_manipulation {

class PickTrashServer : public rclcpp::Node {
public:
  using PickTrash = tb4_openx_interfaces::action::Pick;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickTrash>;

  explicit PickTrashServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<PickTrash>::SharedPtr action_server_;
  rclcpp::CallbackGroup::SharedPtr moveit_cb_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  void initialize_moveit();
  void execute_pick(const std::shared_ptr<GoalHandle> goal_handle);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PickTrash::Goal>);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>);
  void handle_accepted(const std::shared_ptr<GoalHandle>);
};

}  
