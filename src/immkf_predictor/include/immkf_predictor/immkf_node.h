#ifndef IMMKF_PREDICTOR_IMMKF_NODE_H_
#define IMMKF_PREDICTOR_IMMKF_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <unordered_map>

#include "immkf_tracker.h"
#include "motion_models.h"
#include "data_association.h"
#include "immkf_predictor/msg/predicted_trajectory.hpp"

namespace immkf_predictor {

/**
 * ROS2 node for IMMKF-based trajectory prediction
 * Subscribes to fused detections and publishes predicted trajectories
 */
class IMMKFPredictorNode : public rclcpp::Node {
 public:
  explicit IMMKFPredictorNode(const rclcpp::NodeOptions& options =
                               rclcpp::NodeOptions());
  ~IMMKFPredictorNode() override = default;

 private:
  // ROS2 components
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr
      detection_sub_;
  rclcpp::Publisher<immkf_predictor::msg::PredictedTrajectory>::SharedPtr
      trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  // Multi-target tracking: map from object_id to tracker
  std::unordered_map<std::string, std::shared_ptr<IMMKFTracker>> active_tracks_;

  // Configuration parameters
  double prediction_horizon_s_;
  double prediction_step_hz_;
  double max_association_distance_;
  int confirmation_threshold_;
  int max_track_age_;

  // Motion model parameters
  double q_pos_, q_vel_, q_acc_;
  double r_pos_;
  double singer_decay_rate_;

  // Methods
  void detectionCallback(
      const yolo_msgs::msg::DetectionArray::SharedPtr msg);
  void updateTracks(
      const yolo_msgs::msg::DetectionArray::SharedPtr msg);
  void publishTrajectories(double current_time);
  void publishVisualization(double current_time);

  /**
   * Create IMMKF filter with three motion models
   */
  std::shared_ptr<IMMKFFilter> createIMMKFFilter();

  /**
   * Age out old tracks
   */
  void ageOutTracks();

  /**
   * Initialize marker for visualization
   */
  visualization_msgs::msg::Marker createTrajectoryMarker(
      const std::string& object_id,
      const std::vector<TrajectoryPoint>& trajectory,
      int marker_id);
};

}  // namespace immkf_predictor

RCLCPP_COMPONENTS_REGISTER_NODE(immkf_predictor::IMMKFPredictorNode)

#endif  // IMMKF_PREDICTOR_IMMKF_NODE_H_
