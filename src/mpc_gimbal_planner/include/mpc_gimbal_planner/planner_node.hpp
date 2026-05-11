#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "simulator/msg/gimbal.hpp"
#include "simulator/msg/gimbal_cmd.hpp"

#include "mpc_gimbal.hpp"

namespace mpc_gimbal_planner
{

class PlannerNode : public rclcpp::Node
{
public: // 构造函数与析构函数
  explicit PlannerNode(const rclcpp::NodeOptions & options);
  ~PlannerNode() override;

public: // 方法
  void onConfigure();

private:
  enum class ControlMode
  {
    Patrol,
    Track,
  };

  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onGimbalState(const simulator::msg::Gimbal::SharedPtr msg);
  void publishCommand();
  void publishPatrolCommand(const rclcpp::Time & now);
  std::vector<Eigen::Vector2d> buildReferenceSequence() const;
  geometry_msgs::msg::PoseStamped transformPoseToBase(
    const geometry_msgs::msg::PoseStamped & pose) const;

private: // 成员变量
  bool has_path_{false};
  bool has_state_{false};
  bool enable_patrol_{true};

  ControlMode control_mode_{ControlMode::Patrol};

  std::string input_path_topic_;
  std::string input_state_topic_;
  std::string output_cmd_topic_;
  std::string base_frame_;

  std::size_t prediction_horizon_{10};
  double prediction_dt_{0.1};
  double control_rate_hz_{20.0};
  double target_timeout_sec_{0.5};
  double yaw_min_{-1.57};
  double yaw_max_{1.57};
  double pitch_min_{-0.5};
  double pitch_max_{0.8};
  double patrol_yaw_rate_{0.3};
  double patrol_pitch_rate_amplitude_{0.25};
  double patrol_pitch_frequency_{0.15};
  double patrol_yaw_margin_{0.05};

  int patrol_yaw_direction_{1};

  nav_msgs::msg::Path latest_path_;
  Eigen::Vector2d current_angles_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d current_rates_{Eigen::Vector2d::Zero()};
  rclcpp::Time last_path_update_time_;
  rclcpp::Time patrol_start_time_;
  rclcpp::Time last_patrol_update_time_;
  Eigen::Vector2d patrol_target_angles_{Eigen::Vector2d::Zero()};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<simulator::msg::Gimbal>::SharedPtr state_sub_;
  rclcpp::Publisher<simulator::msg::GimbalCmd>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<MPCGimbal> mpc_;
  MPCGimbal::Config config_;

};

} // namespace mpc_gimbal_planner

#endif // PLANNER_NODE_HPP