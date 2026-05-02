#include "immkf_predictor/predictor_node.hpp"

#include <algorithm>
#include <functional>
#include <vector>
#include <limits>

namespace immkf_predictor
{

PredictorNode::PredictorNode(const rclcpp::NodeOptions & options)
: Node("immkf_predictor_node", options)
{
  RCLCPP_INFO(get_logger(), "Starting PredictorNode");
  onConfigure(); // 配置参数
}

PredictorNode::~PredictorNode()
{
}

void PredictorNode::onConfigure()
{
  input_topic_ = this->declare_parameter<std::string>("topics.input", "/tracks_3d");
  output_path_topic_ = this->declare_parameter<std::string>("topics.output_path", "/immkf/predicted_path");
  output_pose_topic_ = this->declare_parameter<std::string>("topics.output_pose", "/immkf/predicted_pose");
  target_detection_index_ = this->declare_parameter<int>("target_detection_index", -1);

  const auto initial_state_vector = this->declare_parameter<std::vector<double>>(
    "initial_state", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  const auto initial_covariance_diagonal = this->declare_parameter<std::vector<double>>(
    "initial_covariance_diagonal", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  prediction_dt_ = this->declare_parameter<double>("prediction_dt", 0.1);
  prediction_horizon_ = static_cast<std::size_t>(
    this->declare_parameter<int>("prediction_horizon", 10));

  ImmkfConfig config;
  config.measurement_noise.diagonal() = Eigen::Vector2d(
    this->declare_parameter<double>("measurement_noise_x", 1.0),
    this->declare_parameter<double>("measurement_noise_y", 1.0));
  config.transition_matrix = ImmkfPredictor::defaultTransitionMatrix();

  predictor_ = std::make_unique<ImmkfPredictor>(config);

  for (std::size_t i = 0; i < initial_state_vector.size() && i < 6; ++i) {
    initial_state_(static_cast<int>(i)) = initial_state_vector[i];
  }

  initial_covariance_.setZero();
  for (std::size_t i = 0; i < initial_covariance_diagonal.size() && i < 6; ++i) {
    initial_covariance_(static_cast<int>(i), static_cast<int>(i)) = initial_covariance_diagonal[i];
  }

  predictor_->reset(initial_state_, initial_covariance_);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, 10);

  detection_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
    input_topic_, 10,
    std::bind(&PredictorNode::detectionsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Topic config: input=%s, output_path=%s, output_pose=%s, target_index=%d",
    input_topic_.c_str(), output_path_topic_.c_str(), output_pose_topic_.c_str(), target_detection_index_);

  const auto predicted_trajectory = predictor_->predictTrajectory(prediction_dt_, prediction_horizon_);
  if (!predicted_trajectory.empty()) {
    const auto & state = predicted_trajectory.back();
    RCLCPP_INFO(
      get_logger(),
      "IMM predictor configured, horizon=%zu, last predicted pose=(%.3f, %.3f)",
      prediction_horizon_, state(0), state(1));
  }
}

bool PredictorNode::extractMeasurement(
  const yolo_msgs::msg::DetectionArray & msg,
  Measurement & measurement,
  std::string & frame_id) const
{
  if (msg.detections.empty()) {
    return false;
  }

  int index = target_detection_index_;
  if (index < 0 || index >= static_cast<int>(msg.detections.size())) {
    index = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < msg.detections.size(); ++i) {
      if (msg.detections[i].score > best_score) {
        best_score = msg.detections[i].score;
        index = static_cast<int>(i);
      }
    }
  }

  const auto & detection = msg.detections[static_cast<std::size_t>(index)];
  frame_id = msg.header.frame_id;

  if (!detection.bbox3d.frame_id.empty()) {
    measurement(0) = detection.bbox3d.center.position.x;
    measurement(1) = detection.bbox3d.center.position.y;
    return true;
  }

  measurement(0) = detection.bbox.center.position.x;
  measurement(1) = detection.bbox.center.position.y;
  return true;
}

void PredictorNode::publishPredictions(const rclcpp::Time & stamp, const std::string & frame_id) const
{
  if (!predictor_ || !path_pub_ || !pose_pub_ || !predictor_->isInitialized()) {
    return;
  }

  const auto trajectory = predictor_->predictTrajectory(prediction_dt_, prediction_horizon_);

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = stamp;
  path_msg.header.frame_id = frame_id;
  path_msg.poses.reserve(trajectory.size());

  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp + rclcpp::Duration::from_seconds(prediction_dt_ * static_cast<double>(i + 1));
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.position.x = trajectory[i](0);
    pose_msg.pose.position.y = trajectory[i](1);
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose_msg);
  }

  const auto & fused_state = predictor_->fusedState();
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = frame_id;
  pose_msg.pose.position.x = fused_state(0);
  pose_msg.pose.position.y = fused_state(1);
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation.w = 1.0;

  path_pub_->publish(path_msg);
  pose_pub_->publish(pose_msg);
}

void PredictorNode::detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
  if (!predictor_ || !msg) {
    return;
  }

  Measurement measurement = Measurement::Zero();
  std::string frame_id = msg->header.frame_id;

  if (!extractMeasurement(*msg, measurement, frame_id)) {
    if (predictor_->isInitialized()) {
      predictor_->predict(prediction_dt_);
      publishPredictions(msg->header.stamp, frame_id);
    }
    return;
  }

  if (!predictor_->isInitialized()) {
    State initial_state = State::Zero();
    initial_state(0) = measurement(0);
    initial_state(1) = measurement(1);
    predictor_->reset(initial_state, initial_covariance_);
  } else {
    predictor_->predict(prediction_dt_);
    predictor_->update(measurement);
  }

  publishPredictions(msg->header.stamp, frame_id);
}

} // namespace immkf_predictor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(immkf_predictor::PredictorNode)