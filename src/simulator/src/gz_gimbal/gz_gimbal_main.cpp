#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gz_gimbal/gz_gimbal_node.hpp"

int main(int argc, char * argv[])
{
  // create ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gz_gimbal::GzGimbalNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
