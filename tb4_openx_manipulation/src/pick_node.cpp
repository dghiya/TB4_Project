#include "rclcpp/rclcpp.hpp"
#include "tb4_openx_manipulation/PickTrash.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto pickup_node = std::make_shared<tb4_openx_manipulation::PickTrashServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pickup_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
