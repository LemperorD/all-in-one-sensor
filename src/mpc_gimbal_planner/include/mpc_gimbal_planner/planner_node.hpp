#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

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

private: // 成员变量

};

} // namespace mpc_gimbal_planner

#endif // PLANNER_NODE_HPP