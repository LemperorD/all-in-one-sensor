#include "sensor_fusion_node/fusion_node.hpp"

namespace sensor_fusion
{

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("sensor_fusion_node_node", options)
{
  std::cout << "\033[32m" << "Starting FusionNode" << "\033[0m" << std::endl;
  onConfigure(); // 配置参数
}

FusionNode::~FusionNode()
{
}

void FusionNode::onConfigure()
{

}

} // namespace sensor_fusion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_fusion::FusionNode)