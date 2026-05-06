#ifndef TB4_OPENX_MANIPULATION__APPROACH_TRASH_HPP_ 
#define TB4_OPENX_MANIPULATION__APPROACH_TRASH_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tb4_openx_interfaces/action/approach.hpp"

#include <memory>
#include <string>

namespace tb4_openx_manipulation
{

class ApproachTrashServer : public rclcpp::Node
{
public:
  using ApproachTrash = tb4_openx_interfaces::action::Approach;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ApproachTrash>;

  explicit ApproachTrashServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double marker_x = 0.0;
  double marker_y = 0.0;
  rclcpp_action::Server<ApproachTrash>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::CallbackGroup::SharedPtr nav_cb_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ApproachTrash::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
};

}  

#endif 
