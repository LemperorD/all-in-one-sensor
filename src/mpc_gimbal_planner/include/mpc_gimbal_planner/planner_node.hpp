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
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onGimbalState(const simulator::msg::Gimbal::SharedPtr msg);
  void publishCommand();
  std::vector<Eigen::Vector2d> buildReferenceSequence() const;
  geometry_msgs::msg::PoseStamped transformPoseToBase(
    const geometry_msgs::msg::PoseStamped & pose) const;

private: // 成员变量
  bool configured_{false};
  bool has_path_{false};
  bool has_state_{false};

  std::string input_path_topic_;
  std::string input_state_topic_;
  std::string output_cmd_topic_;
  std::string base_frame_;

  std::size_t prediction_horizon_{10};
  double prediction_dt_{0.1};
  double control_rate_hz_{20.0};

  nav_msgs::msg::Path latest_path_;
  Eigen::Vector2d current_angles_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d current_rates_{Eigen::Vector2d::Zero()};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<simulator::msg::Gimbal>::SharedPtr state_sub_;
  rclcpp::Publisher<simulator::msg::GimbalCmd>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<MPCGimbal> mpc_;

};

} // namespace mpc_gimbal_planner

#endif // PLANNER_NODE_HPP