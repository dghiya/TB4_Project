#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tb4_openx_navigation/navigator.hpp> 
#include <vector>
#include <iostream>
#include <memory>

class PatrolRobot : public rclcpp::Node {
public:
    PatrolRobot() : Node("patrol_robot") {
        this->declare_parameter<std::vector<double>>("patrol_route", std::vector<double>{});

        std::vector<double> flattened_route;
        this->get_parameter("patrol_route", flattened_route);

        if (flattened_route.size() % 6 != 0) {
          RCLCPP_ERROR(get_logger(),
            "Invalid patrol_route: length % 6 != 0! 6 values per waypoint (x,y,qx,qy,qz,qw).");
          rclcpp::shutdown();
          return;
        }

       for (size_t i = 0; i < flattened_route.size(); i += 6) {
          patrol_route_.push_back({
              flattened_route[i + 0], flattened_route[i + 1],
              flattened_route[i + 2], flattened_route[i + 3],
              flattened_route[i + 4], flattened_route[i + 5]}
          );
        }

        if (patrol_route_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No patrol waypoints found in parameters!");
            rclcpp::shutdown();
        }

        navigator_ = std::make_unique<Navigator>(true);
            
        auto initial_pose = std::make_shared<geometry_msgs::msg::Pose>();
        initial_pose->position.x = 0.0;
        initial_pose->position.y = 0.0;
        initial_pose->orientation.w = 1.0;
        navigator_->SetInitialPose(initial_pose);

        navigator_->WaitUntilNav2Active();
        patrolLoop();
    }

private:
    std::vector<std::vector<double>> patrol_route_;
    std::unique_ptr<Navigator> navigator_;

    void patrolLoop() {
        while (rclcpp::ok()) {
            std::vector<geometry_msgs::msg::PoseStamped> waypoints;
            for (const auto &point : patrol_route_) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = navigator_->get_clock()->now();
                pose.pose.position.x = point[0];
                pose.pose.position.y = point[1];
                pose.pose.orientation.x = point[2];
                pose.pose.orientation.y = point[3];
                pose.pose.orientation.z = point[4];
                pose.pose.orientation.w = point[5];
                waypoints.push_back(pose);
            }

            navigator_->FollowWaypoints(waypoints);

            while (!navigator_->IsTaskComplete()) {
                auto feedback_ptr = navigator_->GetFeedback();
                auto ptr_waypoint = std::static_pointer_cast<const nav2_msgs::action::FollowWaypoints::Feedback>(feedback_ptr);
                std::cout << "Feedback: Current Waypoint " << ptr_waypoint->current_waypoint << std::endl;
            }

            auto result = navigator_->GetResult();
            if (result == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Patrol complete! Restarting...");
            } else if (result == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_WARN(this->get_logger(), "Patrol canceled. Exiting.");
                break;
            } else if (result == rclcpp_action::ResultCode::ABORTED) {
                RCLCPP_ERROR(this->get_logger(), "Patrol failed! Restarting from the other side...");
            }

            std::reverse(patrol_route_.begin(), patrol_route_.end());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolRobot>());
    rclcpp::shutdown();
    return 0;
}