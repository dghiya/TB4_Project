#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <tb4_openx_interfaces/action/pick_object.hpp>
#include <tb4_openx_interfaces/action/place_object.hpp>

using PickObject = tb4_openx_interfaces::action::PickObject;
using PlaceObject = tb4_openx_interfaces::action::PlaceObject;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandlePick = rclcpp_action::ServerGoalHandle<PickObject>;
using GoalHandlePlace = rclcpp_action::ServerGoalHandle<PlaceObject>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("omx_pick_server");

static constexpr double GRIPPER_OPEN_POSITION = 0.025;
static constexpr double GRIPPER_CLOSE_POSITION = 0.015;
static constexpr double GRIPPER_MAX_EFFORT = 0.0;

class OmxPickServer : public rclcpp::Node
{
public:
  explicit OmxPickServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("omx_pick_server", options)
  {
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/gripper_controller/gripper_cmd");

    pick_server_ = rclcpp_action::create_server<PickObject>(
      this, "pick_object",
      std::bind(&OmxPickServer::handle_pick_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OmxPickServer::handle_pick_cancel, this, std::placeholders::_1),
      std::bind(&OmxPickServer::handle_pick_accepted, this, std::placeholders::_1));

    place_server_ = rclcpp_action::create_server<PlaceObject>(
      this, "place_object",
      std::bind(&OmxPickServer::handle_place_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OmxPickServer::handle_place_cancel, this, std::placeholders::_1),
      std::bind(&OmxPickServer::handle_place_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(LOGGER, "OMX Pick/Place action server ready");
  }

private:
  rclcpp_action::Server<PickObject>::SharedPtr pick_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_server_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  bool send_gripper_command(double position, double max_effort)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(LOGGER, "Gripper action server not available");
      return false;
    }

    auto goal = GripperCommand::Goal();
    goal.command.position = position;
    goal.command.max_effort = max_effort;

    auto send_goal_future = gripper_client_->async_send_goal(goal);
    if (send_goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Gripper goal send timeout");
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Gripper goal rejected");
      return false;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Gripper result timeout");
      return false;
    }

    auto wrapped_result = result_future.get();
    RCLCPP_INFO(LOGGER, "Gripper result: position=%.4f stalled=%d reached_goal=%d",
      wrapped_result.result->position,
      wrapped_result.result->stalled,
      wrapped_result.result->reached_goal);

    return true;
  }

  bool open_gripper()
  {
    return send_gripper_command(GRIPPER_OPEN_POSITION, GRIPPER_MAX_EFFORT);
  }

  bool close_gripper()
  {
    return send_gripper_command(GRIPPER_CLOSE_POSITION, GRIPPER_MAX_EFFORT);
  }

  bool move_to_named(moveit::planning_interface::MoveGroupInterface & mg, const std::string & name)
  {
    mg.setNamedTarget(name);
    auto result = mg.move();
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_to_pose(moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & pose)
  {
    mg.setPoseTarget(pose);
    auto result = mg.move();
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_cartesian(moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & target)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = mg.computeCartesianPath(waypoints, 0.01, 0.0, traj);
    if (fraction > 0.9) {
      mg.execute(traj);
      return true;
    }
    return false;
  }

  rclcpp_action::GoalResponse handle_pick_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickObject::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_pick_cancel(
    const std::shared_ptr<GoalHandlePick>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_pick_accepted(const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    std::thread{std::bind(&OmxPickServer::execute_pick, this, goal_handle)}.detach();
  }

  void execute_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    auto feedback = std::make_shared<PickObject::Feedback>();
    auto result   = std::make_shared<PickObject::Result>();

    const auto goal = goal_handle->get_goal();
    const geometry_msgs::msg::Pose & obj = goal->object_pose.pose;
    const double approach_h = goal->approach_height;

    auto node = shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);

    auto send_fb = [&](const std::string & phase) {
      feedback->current_phase = phase;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(LOGGER, "Phase: %s", phase.c_str());
    };

    send_fb("opening_gripper");
    if (!open_gripper()) {
      result->success = false; result->message = "Failed to open gripper";
      goal_handle->abort(result); return;
    }

    send_fb("approaching");
    geometry_msgs::msg::Pose pre_grasp = obj;
    pre_grasp.position.z += approach_h;
    if (!move_to_pose(arm, pre_grasp)) {
      result->success = false; result->message = "Failed to reach pre-grasp pose";
      goal_handle->abort(result); return;
    }

    send_fb("grasping");
    if (!move_cartesian(arm, obj)) {
      if (!move_to_pose(arm, obj)) {
        result->success = false; result->message = "Failed to reach grasp pose";
        goal_handle->abort(result); return;
      }
    }

    send_fb("closing_gripper");
    if (!close_gripper()) {
      result->success = false; result->message = "Failed to close gripper";
      goal_handle->abort(result); return;
    }

    send_fb("lifting");
    if (!move_cartesian(arm, pre_grasp)) {
      move_to_pose(arm, pre_grasp);
    }

    send_fb("returning_home");
    move_to_named(arm, "home");

    result->success = true;
    result->message = "Pick succeeded";
    goal_handle->succeed(result);
    RCLCPP_INFO(LOGGER, "Pick succeeded");
  }

  rclcpp_action::GoalResponse handle_place_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlaceObject::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_place_cancel(
    const std::shared_ptr<GoalHandlePlace>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_place_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    std::thread{std::bind(&OmxPickServer::execute_place, this, goal_handle)}.detach();
  }

  void execute_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    auto feedback = std::make_shared<PlaceObject::Feedback>();
    auto result   = std::make_shared<PlaceObject::Result>();

    const auto goal = goal_handle->get_goal();
    const geometry_msgs::msg::Pose & place = goal->place_pose.pose;
    const double retreat_h = goal->retreat_height;

    auto node = shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);

    auto send_fb = [&](const std::string & phase) {
      feedback->current_phase = phase;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(LOGGER, "Phase: %s", phase.c_str());
    };

    send_fb("approaching_place");
    geometry_msgs::msg::Pose pre_place = place;
    pre_place.position.z += retreat_h;
    if (!move_to_pose(arm, pre_place)) {
      result->success = false; result->message = "Failed to reach pre-place pose";
      goal_handle->abort(result); return;
    }

    send_fb("placing");
    if (!move_cartesian(arm, place)) {
      if (!move_to_pose(arm, place)) {
        result->success = false; result->message = "Failed to reach place pose";
        goal_handle->abort(result); return;
      }
    }

    send_fb("opening_gripper");
    if (!open_gripper()) {
      result->success = false; result->message = "Failed to open gripper";
      goal_handle->abort(result); return;
    }

    send_fb("retreating");
    if (!move_cartesian(arm, pre_place)) {
      move_to_pose(arm, pre_place);
    }

    send_fb("returning_home");
    move_to_named(arm, "home");

    result->success = true;
    result->message = "Place succeeded";
    goal_handle->succeed(result);
    RCLCPP_INFO(LOGGER, "Place succeeded");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<OmxPickServer>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
