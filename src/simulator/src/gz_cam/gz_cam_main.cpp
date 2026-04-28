#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gz_cam/gz_cam_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gz_cam::GzCamNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
