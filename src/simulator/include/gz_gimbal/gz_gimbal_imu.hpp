#ifndef GZ_GIMBAL_IMU_HPP_
#define GZ_GIMBAL_IMU_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "rclcpp/clock.hpp"
#include "simulator/msg/gimbal.hpp"

namespace gz_gimbal
{

class IgnGimbalImu
{
public:
  IgnGimbalImu(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_gimbal_imu_topic);
  ~IgnGimbalImu() {}

  void enable(bool enable) {enable_ = enable;}
  Sensor<simulator::msg::Gimbal>::SharedPtr get_position_sensor() {return position_sensor_;}

private:
  void gz_imu_cb(const ignition::msgs::IMU & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  bool enable_{false};
  // sensor data
  double last_yaw_angle_{0};
  double continuous_yaw_angle_{0};
  simulator::msg::Gimbal cur_position_;
  std::shared_ptr<DataSensor<simulator::msg::Gimbal>> position_sensor_;
};

}  // namespace gz_gimbal

#endif  // GZ_GIMBAL_IMU_HPP_
