#ifndef RMUA19_ROBOT_BASE_NODE_HPP_
#define RMUA19_ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"
  
#include "gz_gimbal/gz_gimbal_actuator.hpp"
#include "gz_gimbal/gz_gimbal_encoder.hpp"
#include "gz_gimbal/gimbal_controller.hpp"
#include "gz_gimbal/pid.hpp"

namespace gz_gimbal
{
class GzGimbalNode : public rclcpp::Node
{
public:
  explicit GzGimbalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  // ign actuator moudule
  std::shared_ptr<gz_gimbal::IgnGimbalActuator> gimbal_vel_actuator_;
  // ign sensor moudule
  std::shared_ptr<gz_gimbal::IgnGimbalEncoder> gz_gimbal_encoder_;
  // ros controller/publisher wrapper
  std::shared_ptr<gz_gimbal::GimbalController> gimbal_controller_;
};

}  // namespace gz_gimbal

#endif  // RMUA19_ROBOT_BASE_NODE_HPP_
