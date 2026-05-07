#include "rc_gimbal/rc_gimbal_node.hpp"
#include <iostream>

namespace rc_gimbal
{

RcGimbalNode::RcGimbalNode(const rclcpp::NodeOptions & options)
  : Node("rc_gimbal_node", options)
{
  onConfigure(); // 配置参数
  rc_gimbal_main_ = std::make_shared<RcGimbalMain>(file_name_.c_str());
  test_thread_ = std::thread(&RcGimbalNode::test_thread, this);
}

RcGimbalNode::~RcGimbalNode()
{
  if (test_thread_.joinable()) test_thread_.join();
  rc_gimbal_main_.reset();
  std::cout << "RcGimbalNode destructor called" << std::endl;
}

void RcGimbalNode::onConfigure()
{
  this->declare_parameter<std::string>("file_name", "/dev/input/js0");
  this->get_parameter("file_name", file_name_);
}

void RcGimbalNode::test_thread()
{
  std::cout << "Axis1: " << rc_gimbal_main_->get_axis_state(1) << std::endl;
  std::cout << "Axis2: " << rc_gimbal_main_->get_axis_state(2) << std::endl;
}

} // namespace rc_gimbal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_gimbal::RcGimbalNode)

