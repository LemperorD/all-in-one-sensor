#include "gz_gimbal/gz_gimbal_encoder.hpp"

#include <memory>
#include <string>
#include <cmath>


namespace gz_gimbal
{

IgnGimbalEncoder::IgnGimbalEncoder(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & gz_joint_state_topic)
: node_(node), gz_node_(gz_node)
{
  gz_node_->Subscribe(gz_joint_state_topic, &IgnGimbalEncoder::gz_Joint_state_cb, this);
  position_sensor_ = std::make_shared<DataSensor<simulator::msg::Gimbal>>();
  velocity_sensor_ = std::make_shared<DataSensor<simulator::msg::Gimbal>>();
}

void IgnGimbalEncoder::gz_Joint_state_cb(const ignition::msgs::Model & msg)
{
  if (!enable_) {
    return;
  }
  simulator::msg::Gimbal position, velocity;
  for (int i = 0; i < msg.joint_size(); i++) {
    if (msg.joint(i).name().find("gimbal_pitch_odom_joint") != std::string::npos) {
      position.pitch += msg.joint(i).axis1().position();
      velocity.pitch += msg.joint(i).axis1().velocity();
    }
    if (msg.joint(i).name().find("gimbal_yaw_odom_joint") != std::string::npos) {
      position.yaw += msg.joint(i).axis1().position();
      velocity.yaw += msg.joint(i).axis1().velocity();
    }
    if (msg.joint(i).name().find("gimbal_pitch_joint") != std::string::npos) {
      position.pitch += msg.joint(i).axis1().position();
      velocity.pitch += msg.joint(i).axis1().velocity();
    }
    if (msg.joint(i).name().find("gimbal_yaw_joint") != std::string::npos) {
      position.yaw += msg.joint(i).axis1().position();
      velocity.yaw += msg.joint(i).axis1().velocity();
    }
  }
  position_sensor_->update(position, node_->get_clock()->now());
  velocity_sensor_->update(velocity, node_->get_clock()->now());
}

}  // namespace gz_gimbal
