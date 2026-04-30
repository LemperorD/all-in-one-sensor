#include "gz_gimbal/gz_gimbal_node.hpp"

#include <thread>
#include <memory>
#include <string>

namespace gz_gimbal
{

GzGimbalNode::GzGimbalNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("gz_gimbal_node", options)
{
  node_ = std::make_shared<rclcpp::Node>("robot_base", options);
  gz_node_ = std::make_shared<ignition::transport::Node>();
  // parameters
  std::string world_name, robot_name;
  bool use_odometry = false;
  node_->declare_parameter("world_name", "default");
  node_->declare_parameter("robot_name", "all_in_one_sensor");
  node_->get_parameter("world_name", world_name);
  node_->get_parameter("robot_name", robot_name);
  node_->get_parameter("use_odometry", use_odometry);
  // ign topic string
  std::string gz_pitch_cmd_topic = "/model/" + robot_name + "/joint/gimbal_pitch_joint/cmd_vel";
  std::string gz_yaw_cmd_topic = "/model/" + robot_name + "/joint/gimbal_yaw_joint/cmd_vel";
  std::string gz_joint_state_topic = "/world/" + world_name + "/model/" + robot_name +
    "/joint_state";
  std::string gz_gimbal_imu_topic = "/world/" + world_name + "/model/" + robot_name +
    "/link/gimbal_pitch/sensor/gimbal_imu/imu";
  // create hardware module
  // Actuator
  gimbal_vel_actuator_ = std::make_shared<gz_gimbal::IgnGimbalActuator>(
    node_, gz_node_, gz_pitch_cmd_topic, gz_yaw_cmd_topic);
  // sensor wrapper
  gz_gimbal_encoder_ = std::make_shared<gz_gimbal::IgnGimbalEncoder>(
    node_, gz_node_, gz_joint_state_topic);
  gz_gimbal_imu_ = std::make_shared<gz_gimbal::IgnGimbalImu>(
    node_, gz_node_, gz_gimbal_imu_topic);
  // create controller and publisher
  gimbal_controller_ = std::make_shared<gz_gimbal::GimbalController>(
    node_, gimbal_vel_actuator_, gz_gimbal_imu_->get_position_sensor());
  //
  gimbal_vel_actuator_->enable(true);
  gz_gimbal_encoder_->enable(true);
  gz_gimbal_encoder_->enable(true);
  gz_gimbal_imu_->enable(true);
}

}  // namespace gz_gimbal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gz_gimbal::GzGimbalNode)
