#ifndef GZ_GIMBAL_ACTUATOR_HPP_
#define GZ_GIMBAL_ACTUATOR_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "simulator/msg/gimbal.hpp"

namespace gz_gimbal
{

class IgnGimbalActuator : public Actuator<simulator::msg::Gimbal>
{
public:
  IgnGimbalActuator(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_pitch_topic,
    const std::string & gz_yaw_topic);

  void set(const simulator::msg::Gimbal & data) override;
  void enable(bool enable) {enable_ = enable;}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_pitch_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_yaw_pub_;
  bool enable_{false};
};

}  // namespace gz_gimbal

#endif  // GZ_GIMBAL_ACTUATOR_HPP_
