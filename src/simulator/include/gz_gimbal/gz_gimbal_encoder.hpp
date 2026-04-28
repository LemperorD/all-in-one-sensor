#ifndef GZ_GIMBAL_ENCODER_HPP_
#define GZ_GIMBAL_ENCODER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <map>
#include <vector>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "simulator/msg/gimbal.hpp"

namespace gz_gimbal
{

class IgnGimbalEncoder
{
public:
  IgnGimbalEncoder(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_joint_state_topic);
  ~IgnGimbalEncoder() {}

public:
  void enable(bool enable) {enable_ = enable;}
  Sensor<simulator::msg::Gimbal>::SharedPtr get_position_sensor() {return position_sensor_;}
  Sensor<simulator::msg::Gimbal>::SharedPtr get_velocity_sensor() {return velocity_sensor_;}

private:
  void gz_Joint_state_cb(const ignition::msgs::Model & msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  bool enable_{false};
  // info
  std::shared_ptr<DataSensor<simulator::msg::Gimbal>> position_sensor_;
  std::shared_ptr<DataSensor<simulator::msg::Gimbal>> velocity_sensor_;
};


}  // namespace gz_gimbal

#endif  // GZ_GIMBAL_ENCODER_HPP_
