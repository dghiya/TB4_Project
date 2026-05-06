#include "rclcpp/rclcpp.hpp"
#include "tb4_openx_manipulation/ApproachTrash.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto approach_node = std::make_shared<tb4_openx_manipulation::ApproachTrashServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(approach_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
