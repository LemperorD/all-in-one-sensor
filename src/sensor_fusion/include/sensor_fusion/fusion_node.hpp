#ifndef FUSION_NODE_HPP
#define FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "bytetrack.hpp"
#include "AB3DMOT.hpp"
#include "fusion_utils.hpp"

namespace sensor_fusion
{

class FusionNode : public rclcpp::Node
{
public: // 构造函数与析构函数
  explicit FusionNode(const rclcpp::NodeOptions & options);
  ~FusionNode() override;

public: // 方法
  void onConfigure();

private: // 成员变量

};

} // namespace sensor_fusion

#endif // FUSION_NODE_HPP