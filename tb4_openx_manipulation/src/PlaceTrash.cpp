#include "tb4_openx_manipulation/PlaceTrash.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace tb4_openx_manipulation {

PlaceTrashServer::PlaceTrashServer(const rclcpp::NodeOptions & options)
: Node("place_trash_server", options)
{
  nav_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose", nav_cb_group_);

  action_server_ = rclcpp_action::create_server<PlaceTrash>(
    this,
    "place_trash",
    std::bind(&PlaceTrashServer::handle_goal, this, _1, _2),
    std::bind(&PlaceTrashServer::handle_cancel, this, _1),
    std::bind(&PlaceTrashServer::handle_accepted, this, _1)
  );

  RCLCPP_INFO(this->get_logger(), "PlaceTrash action server ready.");
}

void PlaceTrashServer::initialize_moveit()
{
  try {
    if (!arm_group_) {
      arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->shared_from_this(), "arm");
      arm_group_->setPlanningTime(5.0);
    }

    if (!gripper_group_) {
      gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->shared_from_this(), "gripper");
      gripper_group_->setPlanningTime(2.0);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt: %s", e.what());
    arm_group_.reset();
    gripper_group_.reset();
  }
}

rclcpp_action::GoalResponse PlaceTrashServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PlaceTrash::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received place goal.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlaceTrashServer::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(this->get_logger(), "PlaceTrash goal cancelled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlaceTrashServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  this->initialize_moveit();
  std::thread{std::bind(&PlaceTrashServer::execute_place, this, goal_handle)}.detach();
}

void PlaceTrashServer::execute_place(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (!arm_group_ || !gripper_group_) {
    RCLCPP_ERROR(this->get_logger(), "MoveIt groups not initialized.");
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "MoveIt groups not initialized.";
    goal_handle->abort(result);
    return;
  }

  auto pose = goal_handle->get_goal()->trash_pose;

  nav2_msgs::action::NavigateToPose::Goal nav_goal;
  nav_goal.pose = pose;

  if (!nav_client_->wait_for_action_server(5s)) {
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "NavigateToPose server not available.";
    goal_handle->abort(result);
    return;
  }

  auto nav_goal_future = nav_client_->async_send_goal(nav_goal);
  if (nav_goal_future.wait_for(10s) != std::future_status::ready)
  {
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "Failed to send navigation goal (timeout).";
    goal_handle->abort(result);
    return;
  }

  auto nav_goal_handle = nav_goal_future.get();
  if (!nav_goal_handle) {
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "Navigation goal was rejected by server.";
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Navigating to place goal...");
  auto nav_result_future = nav_client_->async_get_result(nav_goal_handle);

  if (nav_result_future.wait_for(std::chrono::hours(1)) != std::future_status::ready)
  {
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "Navigation result timeout.";
    goal_handle->abort(result);
    return;
  }

  auto nav_result = nav_result_future.get();
  if (nav_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "Navigation failed.";
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Navigation succeeded. Moving to pre-drop pose...");
  std::vector<double> joint_group_positions_arm = {
    -0.122173, 0.558505, -0.122173, 1.09956
  };

  arm_group_->setJointValueTarget(joint_group_positions_arm);
  if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    auto result = std::make_shared<PlaceTrash::Result>();
    result->success = false;
    result->message = "Failed to move to drop pose.";
    goal_handle->abort(result);
    return;
  }

  auto feedback = std::make_shared<PlaceTrash::Feedback>();
  feedback->distance_to_goal = 0.0;  // already there
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(this->get_logger(), "Opening gripper...");
  gripper_group_->setNamedTarget("open");
  gripper_group_->move();

  RCLCPP_INFO(this->get_logger(), "Returning to home...");
  arm_group_->setNamedTarget("home");
  arm_group_->move();

  auto result = std::make_shared<PlaceTrash::Result>();
  result->success = true;
  result->message = "Place successful.";
  goal_handle->succeed(result);
}

}  // namespace tb4_openx_manipulation
