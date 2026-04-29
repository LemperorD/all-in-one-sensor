#include "gz_gimbal/gz_gimbal_actuator.hpp"

#include <memory>
#include <string>

namespace gz_gimbal
{

IgnGimbalActuator::IgnGimbalActuator(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & gz_pitch_topic,
  const std::string & gz_yaw_topic)
: node_(node), gz_node_(gz_node)
{
  gz_pitch_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_pitch_topic));
  gz_yaw_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_yaw_topic));
}

void IgnGimbalActuator::set(const simulator::msg::Gimbal & data)
{
  if (!enable_) {
    return;
  }
  ignition::msgs::Double gz_msg;
  gz_msg.set_data(data.pitch);
  gz_pitch_pub_->Publish(gz_msg);
  gz_msg.set_data(data.yaw);
  gz_yaw_pub_->Publish(gz_msg);
}

}  // namespace gz_gimbal
