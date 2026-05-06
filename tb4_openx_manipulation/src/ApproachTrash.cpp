#include "tb4_openx_manipulation/ApproachTrash.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace tb4_openx_manipulation
{

ApproachTrashServer::ApproachTrashServer(const rclcpp::NodeOptions & options)
: Node("approach_trash_server", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  action_server_ = rclcpp_action::create_server<ApproachTrash>(
    this,
    "approach_trash",
    std::bind(&ApproachTrashServer::handle_goal, this, _1, _2),
    std::bind(&ApproachTrashServer::handle_cancel, this, _1),
    std::bind(&ApproachTrashServer::handle_accepted, this, _1)
  );

  nav_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose", nav_cb_group_);

  if (!nav_to_pose_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
  } else {
    RCLCPP_INFO(this->get_logger(), "ApproachTrash action server ready.");
  }
}

rclcpp_action::GoalResponse ApproachTrashServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ApproachTrash::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal to approach marker frame: %s", goal->marker_frame.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ApproachTrashServer::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(this->get_logger(), "ApproachTrash goal cancelled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ApproachTrashServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread([this, goal_handle]() {  // This thread will now run the full sequence
    const std::string marker_frame = goal_handle->get_goal()->marker_frame;

    geometry_msgs::msg::TransformStamped tf_to_marker;
    const auto timeout = 10s;
    const auto start_time = std::chrono::steady_clock::now();
    // while (rclcpp::ok()) {
    //   try {
    //     tf_to_marker = tf_buffer_.lookupTransform("map", marker_frame, tf2::TimePointZero, 600ms);
    //     break;
    //   } catch (const tf2::TransformException & ex) {
    //     if (std::chrono::steady_clock::now() - start_time > timeout) {
    //       RCLCPP_ERROR(this->get_logger(), "TF lookup timed out: %s", ex.what());
    //       auto result = std::make_shared<ApproachTrash::Result>();
    //       result->success = false;
    //       result->message = ex.what();
    //       goal_handle->abort(result);
    //       return;
    //     }
    //   }
    //   rclcpp::sleep_for(200ms);
    // }

    // marker_x = tf_to_marker.transform.translation.x;
    // marker_y = tf_to_marker.transform.translation.y;
    
    // HARDCODED TARGET (Bypassing Perception TF lookup)
    marker_x = 2.0; // Change to your desired map X coordinate
    marker_y = 1.0; // Change to your desired map Y coordinate
    RCLCPP_INFO(this->get_logger(), "Using HARDCODED target: x=%.2f, y=%.2f", marker_x, marker_y);

    double goal_x = marker_x + 0.25;
    double goal_y = marker_y + 0.15;

    geometry_msgs::msg::PoseStamped nav_goal;
    nav_goal.header.frame_id = "map";
    nav_goal.header.stamp = this->now();
    nav_goal.pose.position.x = goal_x;
    nav_goal.pose.position.y = goal_y;
    nav_goal.pose.position.z = 0.0;
    nav_goal.pose.orientation.x = 0.0;
    nav_goal.pose.orientation.y = 0.0;
    nav_goal.pose.orientation.z = 1.0;
    nav_goal.pose.orientation.w = 0.0;

    NavigateToPose::Goal nav_goal_msg;
    nav_goal_msg.pose = nav_goal;

    // Setup feedback callback for the navigation action
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions nav_options;
    nav_options.feedback_callback =
      [this, goal_handle, goal_x, goal_y](auto, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        const auto & current = feedback->current_pose.pose;
        double dx = goal_x - current.position.x;
        double dy = goal_y - current.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        auto fb = std::make_shared<ApproachTrash::Feedback>();
        fb->distance_to_goal = distance;
        goal_handle->publish_feedback(fb);
      };

    RCLCPP_INFO(this->get_logger(), "Sending navigation goal to Nav2...");
    auto send_goal_future = nav_to_pose_client_->async_send_goal(nav_goal_msg, nav_options);

    // Wait for the goal to be accepted
    if (send_goal_future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 goal send timed out.");
      auto result = std::make_shared<ApproachTrash::Result>();
      result->success = false;
      result->message = "Nav2 goal send timed out.";
      goal_handle->abort(result);
      return;
    }

    auto nav_goal_handle = send_goal_future.get();
    if (!nav_goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected.");
      auto result = std::make_shared<ApproachTrash::Result>();
      result->success = false;
      result->message = "Nav2 goal was rejected.";
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation goal accepted. Waiting for result...");
    auto result_future = nav_to_pose_client_->async_get_result(nav_goal_handle);

    // Wait for the navigation to complete
    if (result_future.wait_for(std::chrono::minutes(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action timed out.");
      auto result = std::make_shared<ApproachTrash::Result>();
      result->success = false;
      result->message = "Nav2 action timed out.";
      goal_handle->abort(result);
      return;
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Navigation failed with code: %d", static_cast<int>(wrapped_result.code));
      auto final_result = std::make_shared<ApproachTrash::Result>();
      final_result->success = false;
      final_result->message = "Navigation failed or aborted.";
      goal_handle->abort(final_result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation succeeded. Starting PID refinement...");

    // --- PID Refinement Logic ---
    geometry_msgs::msg::TransformStamped tf_base;
    try {
      tf_base = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero, 500ms);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "TF error after nav: %s", ex.what());
      auto result = std::make_shared<ApproachTrash::Result>();
      result->success = false;
      result->message = "TF error after nav";
      goal_handle->abort(result);
      return;
    }

    double robot_x = tf_base.transform.translation.x;
    double robot_y = tf_base.transform.translation.y;

    double dx = marker_x - robot_x;
    double dy = marker_y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_yaw = std::atan2(dy, dx);

    tf2::Quaternion q(
      tf_base.transform.rotation.x,
      tf_base.transform.rotation.y,
      tf_base.transform.rotation.z,
      tf_base.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    auto vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    rclcpp::Rate rate(10);
    double Kp_lin = 0.5;
    double Kp_ang = 1.0;
    double threshold = 0.1;  

    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
      dx = marker_x - robot_x;
      dy = marker_y - robot_y;
      distance = std::sqrt(dx * dx + dy * dy);
      target_yaw = std::atan2(dy, dx);

      double yaw_error = target_yaw - yaw;
      while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = Kp_lin * distance;
      cmd.angular.z = Kp_ang * yaw_error;

      cmd.linear.x = std::clamp(cmd.linear.x, -0.2, 0.2);
      cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);

      vel_pub->publish(cmd);
      rate.sleep();

      try {
        tf_base = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero, 100ms);
        robot_x = tf_base.transform.translation.x;
        robot_y = tf_base.transform.translation.y;
        tf2::Quaternion q(
          tf_base.transform.rotation.x,
          tf_base.transform.rotation.y,
          tf_base.transform.rotation.z,
          tf_base.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      } catch (...) {
        continue;
      }

      if (distance < threshold) break;
    }

    geometry_msgs::msg::Twist stop;
    vel_pub->publish(stop);

    auto final_result = std::make_shared<ApproachTrash::Result>();
    final_result->success = true;
    final_result->message = "Refined position near marker.";
    goal_handle->succeed(final_result);

  }).detach();
}

}  