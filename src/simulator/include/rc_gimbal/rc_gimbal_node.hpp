#ifndef RC_GIMBAL_NODE_HPP
#define RC_GIMBAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "simulator/msg/gimbal.hpp"
#include "simulator/msg/gimbal_cmd.hpp"

#include "rc_gimbal_main.hpp"

namespace rc_gimbal
{

class RcGimbalNode : public rclcpp::Node
{
public:
  explicit RcGimbalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RcGimbalNode();

private:
  void onConfigure();

private:
  std::shared_ptr<RcGimbalMain> rc_gimbal_main_;
  char* file_name_;
};

} // namespace rc_gimbal

#endif // RC_GIMBAL_NODE_HPP