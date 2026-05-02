#include "immkf_predictor/predictor_node.hpp"

namespace immkf_predictor
{

PredictorNode::PredictorNode(const rclcpp::NodeOptions & options)
: Node("immkf_predictor_node", options)
{
  std::cout << "\033[32m" << "Starting PredictorNode" << "\033[0m" << std::endl;
  onConfigure(); // 配置参数
}

PredictorNode::~PredictorNode()
{
}

void PredictorNode::onConfigure()
{

}

} // namespace immkf_predictor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(immkf_predictor::PredictorNode)