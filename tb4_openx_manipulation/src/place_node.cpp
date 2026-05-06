#include "rclcpp/rclcpp.hpp"
#include "tb4_openx_manipulation/PlaceTrash.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto place_node = std::make_shared<tb4_openx_manipulation::PlaceTrashServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(place_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
