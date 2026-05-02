#ifndef PREDICTOR_NODE_HPP
#define PREDICTOR_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#include "immkf_predictor/immkf.hpp"

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
  std::unique_ptr<ImmkfPredictor> predictor_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  State initial_state_ = State::Zero();
  Covariance initial_covariance_ = Covariance::Identity();
  std::string input_topic_;
  std::string output_path_topic_;
  std::string output_pose_topic_;
  double prediction_dt_ = 0.1;
  std::size_t prediction_horizon_ = 10;
  int target_detection_index_ = -1;

  bool extractMeasurement(
    const yolo_msgs::msg::DetectionArray & msg,
    Measurement & measurement,
    std::string & frame_id) const;
  void publishPredictions(const rclcpp::Time & stamp, const std::string & frame_id) const;
  void detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);

};

} // namespace immkf_predictor

#endif // PREDICTOR_NODE_HPP