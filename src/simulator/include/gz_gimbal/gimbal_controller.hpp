#ifndef GZ_GIMBAL_CONTROLLER_HPP_
#define GZ_GIMBAL_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "simulator/msg/gimbal_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "simulator/msg/gimbal.hpp"
#include "pid.hpp"
#include "hardware_interface.hpp"

namespace gz_gimbal
{

class GimbalController
{
public:
  GimbalController(
    rclcpp::Node::SharedPtr node,
    Actuator<simulator::msg::Gimbal>::SharedPtr gimbal_vel_actuator,
    Sensor<simulator::msg::Gimbal>::SharedPtr gimbal_pos_sensor,
    const std::string & controller_name = "gimbal_controller");
  ~GimbalController() {}

public:
  void set_yaw_pid(struct PidParam pid_param);
  void set_pitch_pid(struct PidParam pid_param);
  // set gimbal's motor limit (TODO)
  // void set_yaw_motor_limit(double min, double max) {}
  // void set_pitch_motor_limit(double min, double max) {}
  void reset();

private:
  void gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
  void gimbal_joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void update();
  void gimbal_state_timer_cb();

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub and sub
  rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr rmoss_gimbal_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros_gimbal_cmd_sub_;
  rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr rmoss_gimbal_state_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
  // control interface
  Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator_;
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor_;
  // target data
  double target_pitch_{0};
  double target_yaw_{0};
  double cur_pitch_{0};
  double cur_yaw_{0};
  // pid and pid parameter
  PidParam picth_pid_param_;
  PidParam yaw_pid_param_;
  ignition::math::PID picth_pid_;
  ignition::math::PID yaw_pid_;
  std::chrono::nanoseconds pid_period_;
  // flag
  bool update_pid_flag_{true};
};

}  // namespace gz_gimbal

#endif  // GZ_GIMBAL_CONTROLLER_HPP_
