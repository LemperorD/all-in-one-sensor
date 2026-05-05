#include "rc_gimbal/rc_gimbal_node.hpp"

namespace rc_gimbal
{

RcGimbalNode::RcGimbalNode(const rclcpp::NodeOptions & options)
  : Node("rc_gimbal_node", options)
{
  onConfigure(); // 配置参数
  rc_gimbal_main_ = std::make_shared<RcGimbalMain>(file_name_);
}

RcGimbalNode::~RcGimbalNode()
{
  rc_gimbal_main_.reset();
  std::cout << "RcGimbalNode destructor called" << std::endl;
}

void RcGimbalNode::onConfigure()
{
  this->declare_parameter<std::string>("file_name", "/dev/input/js0");
  this->get_parameter("file_name", file_name_);
}

} // namespace rc_gimbal

