#include "tb4_openx_manipulation/PickTrash.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace tb4_openx_manipulation {

PickTrashServer::PickTrashServer(const rclcpp::NodeOptions & options)
: Node("pick_trash_server", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  moveit_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  action_server_ = rclcpp_action::create_server<PickTrash>(
    this,
    "pick_trash",
    std::bind(&PickTrashServer::handle_goal, this, _1, _2),
    std::bind(&PickTrashServer::handle_cancel, this, _1),
    std::bind(&PickTrashServer::handle_accepted, this, _1)
  );

  RCLCPP_INFO(this->get_logger(), "PickTrash action server ready.");
}

void PickTrashServer::initialize_moveit()
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

rclcpp_action::GoalResponse PickTrashServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PickTrash::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received pick goal for marker: %s", goal->marker_frame.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickTrashServer::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(this->get_logger(), "PickTrash goal cancelled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickTrashServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  this->initialize_moveit();
  this->execute_pick(goal_handle);
}

void PickTrashServer::execute_pick(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (!arm_group_ || !gripper_group_) {
    RCLCPP_ERROR(this->get_logger(), "MoveIt groups not initialized.");
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "MoveIt groups not initialized.";
    goal_handle->abort(result);
    return;
  }

  const std::string marker_frame = goal_handle->get_goal()->marker_frame;

  RCLCPP_INFO(this->get_logger(), "Moving to home...");
  arm_group_->setNamedTarget("home");
  if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "Failed to move to home.";
    goal_handle->abort(result);
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform("link1", marker_frame, tf2::TimePointZero, 1s);
  } catch (const tf2::TransformException & ex) {
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "TF lookup failed: " + std::string(ex.what());
    goal_handle->abort(result);
    return;
  }

  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;
  double z_above = 0.18;
  double z_grasp = 0.13;

  auto feedback = std::make_shared<PickTrash::Feedback>();
  feedback->distance_to_goal = std::hypot(x, y);
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(this->get_logger(), "Opening gripper...");
  gripper_group_->setNamedTarget("open");
  if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "Failed to open gripper.";
    goal_handle->abort(result);
    return;
  }

  // RCLCPP_INFO(this->get_logger(), "Approaching object...");
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // geometry_msgs::msg::Pose pose;
  // pose.position.x = x;
  // pose.position.y = y;
  // pose.position.z = z_above;
  // pose.orientation.w = 1.0;  // No rotation for 4DOF
  // waypoints.push_back(pose);

  // pose.position.z = z_grasp;
  // waypoints.push_back(pose);

  // moveit_msgs::msg::RobotTrajectory traj;
  // if (arm_group_->computeCartesianPath(waypoints, 0.01, 0.0, traj) < 0.5) {
  //   auto result = std::make_shared<PickTrash::Result>();
  //   result->success = false;
  //   result->message = "Failed to compute Cartesian path.";
  //   goal_handle->abort(result);
  //   return;
  // }

  // arm_group_->execute(traj);

  RCLCPP_INFO(this->get_logger(), "Moving to pre-grasp joint config...");

  std::vector<double> joint_group_positions_arm = {
  //   0.0349066,  // Joint 1
  //   0.279253,   // Joint 2
  // -0.628319,   // Joint 3
  //   1.88496     // Joint 4
  -0.122173,
  0.558505,
  -0.122173,
  1.09956
  };

  arm_group_->setJointValueTarget(joint_group_positions_arm);

  if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "Failed to move to pregrasp joint configuration.";
    goal_handle->abort(result);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", arm_group_->getName().c_str());
RCLCPP_INFO(this->get_logger(), "End effector link: %s", arm_group_->getEndEffectorLink().c_str());


// RCLCPP_INFO(this->get_logger(), "Refining X/Y position using Cartesian motion...");

// // Wait a bit to ensure MoveIt receives robot state
// // rclcpp::sleep_for(std::chrono::milliseconds(500));
// rclcpp::sleep_for(std::chrono::seconds(2));
// arm_group_->setStartStateToCurrentState();

// std::vector<geometry_msgs::msg::Pose> waypoints;
// rclcpp::sleep_for(std::chrono::seconds(2));

// geometry_msgs::msg::Pose current_pose = arm_group_->getCurrentPose().pose;
// waypoints.push_back(current_pose);

// geometry_msgs::msg::Pose target_pose = current_pose;
// target_pose.position.x = x;  // x from aruco
// target_pose.position.y = y;  // y from aruco
// target_pose.position.z = 0.235;  
// waypoints.push_back(target_pose);

// moveit_msgs::msg::RobotTrajectory trajectory_approach;
// double fraction = arm_group_->computeCartesianPath(waypoints, 0.001, 0.0, trajectory_approach);
// RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %.2f", fraction);

// arm_group_->execute(trajectory_approach);




  RCLCPP_INFO(this->get_logger(), "Closing gripper...");
  gripper_group_->setNamedTarget("close");
  if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "Failed to close gripper.";
    goal_handle->abort(result);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Retreating...");
  std::vector<geometry_msgs::msg::Pose> retreat;
  // pose.position.z = z_above;
  geometry_msgs::msg::Pose pose = arm_group_->getCurrentPose().pose;
  pose.position.z = z_above;

  retreat.push_back(pose);
  moveit_msgs::msg::RobotTrajectory traj_retreat;
  arm_group_->computeCartesianPath(retreat, 0.01, 0.0, traj_retreat);
  arm_group_->execute(traj_retreat);

  RCLCPP_INFO(this->get_logger(), "Returning to home...");
  arm_group_->setNamedTarget("home");
  if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    auto result = std::make_shared<PickTrash::Result>();
    result->success = false;
    result->message = "Failed to return to home.";
    goal_handle->abort(result);
    return;
  }

  auto result = std::make_shared<PickTrash::Result>();
  result->success = true;
  result->message = "Pick successful.";
  goal_handle->succeed(result);
}

} 
