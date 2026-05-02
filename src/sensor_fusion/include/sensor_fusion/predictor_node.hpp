#ifndef PREDICTOR_NODE_HPP
#define PREDICTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "immkf.hpp"

namespace immkf_predictor
{

class PredictorNode : public rclcpp::Node
{
public: // 构造函数与析构函数
  explicit PredictorNode(const rclcpp::NodeOptions & options);
  ~PredictorNode() override;

public: // 方法
  void onConfigure();

private: // 成员变量

};

} // namespace immkf_predictor

#endif // PREDICTOR_NODE_HPP