#include "immkf_predictor/immkf_node.h"

#include "immkf_predictor/msg/predicted_trajectory.hpp"

namespace immkf_predictor {

IMMKFPredictorNode::IMMKFPredictorNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("immkf_predictor_node", options) {
  // Declare and read parameters
  this->declare_parameter("input_topic", "/fused_detections");
  this->declare_parameter("output_topic", "/predicted_trajectories");
  this->declare_parameter("prediction.horizon_seconds", 0.5);
  this->declare_parameter("prediction.output_step_interval_hz", 10.0);
  this->declare_parameter("prediction.internal_update_hz", 100.0);
  this->declare_parameter("max_association_distance", 2.0);
  this->declare_parameter("confirmation_threshold", 5);
  this->declare_parameter("max_track_age", 100);

  this->declare_parameter("models.constant_velocity.process_noise_pos", 0.01);
  this->declare_parameter("models.constant_velocity.process_noise_vel", 0.01);
  this->declare_parameter("models.constant_acceleration.process_noise_pos", 0.01);
  this->declare_parameter("models.constant_acceleration.process_noise_vel", 0.1);
  this->declare_parameter("models.constant_acceleration.process_noise_acc", 0.01);
  this->declare_parameter("models.singer_model.process_noise_pos", 0.01);
  this->declare_parameter("models.singer_model.process_noise_vel", 0.1);
  this->declare_parameter("models.singer_model.process_noise_acc", 0.01);
  this->declare_parameter("models.singer_model.decay_rate", 0.95);

  this->declare_parameter("measurement.position_noise", 0.2);
  this->declare_parameter("measurement.velocity_prior_noise", 2.0);

  // Read parameters
  std::string input_topic = this->get_parameter("input_topic").as_string();
  std::string output_topic = this->get_parameter("output_topic").as_string();
  prediction_horizon_s_ =
      this->get_parameter("prediction.horizon_seconds").as_double();
  prediction_step_hz_ =
      this->get_parameter("prediction.output_step_interval_hz").as_double();
  max_association_distance_ =
      this->get_parameter("max_association_distance").as_double();
  confirmation_threshold_ = this->get_parameter("confirmation_threshold").as_int();
  max_track_age_ = this->get_parameter("max_track_age").as_int();

  // Motion model parameters
  double cv_q_pos =
      this->get_parameter("models.constant_velocity.process_noise_pos")
          .as_double();
  double cv_q_vel =
      this->get_parameter("models.constant_velocity.process_noise_vel")
          .as_double();
  double ca_q_pos =
      this->get_parameter("models.constant_acceleration.process_noise_pos")
          .as_double();
  double ca_q_vel =
      this->get_parameter("models.constant_acceleration.process_noise_vel")
          .as_double();
  double ca_q_acc =
      this->get_parameter("models.constant_acceleration.process_noise_acc")
          .as_double();
  double singer_q_pos =
      this->get_parameter("models.singer_model.process_noise_pos").as_double();
  double singer_q_vel =
      this->get_parameter("models.singer_model.process_noise_vel").as_double();
  double singer_q_acc =
      this->get_parameter("models.singer_model.process_noise_acc").as_double();
  singer_decay_rate_ =
      this->get_parameter("models.singer_model.decay_rate").as_double();

  r_pos_ = this->get_parameter("measurement.position_noise").as_double();

  q_pos_ = (cv_q_pos + ca_q_pos + singer_q_pos) / 3.0;
  q_vel_ = (cv_q_vel + ca_q_vel + singer_q_vel) / 3.0;
  q_acc_ = (ca_q_acc + singer_q_acc) / 2.0;

  // Create subscriptions and publishers
  detection_sub_ =
      this->create_subscription<yolo_msgs::msg::DetectionArray>(
          input_topic, rclcpp::SensorDataQoS(),
          std::bind(&IMMKFPredictorNode::detectionCallback, this,
                    std::placeholders::_1));

  trajectory_pub_ =
      this->create_publisher<immkf_predictor::msg::PredictedTrajectory>(
          output_topic, 10);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/immkf_trajectories_markers", 10);

  RCLCPP_INFO(this->get_logger(), "IMMKF Predictor Node initialized");
  RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
}

void IMMKFPredictorNode::detectionCallback(
    const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
  if (msg->detections.empty()) {
    return;
  }

  double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // Update existing tracks and create new ones
  updateTracks(msg);

  // Age out old tracks
  ageOutTracks();

  // Publish trajectories
  publishTrajectories(current_time);

  // Publish visualization
  publishVisualization(current_time);
}

void IMMKFPredictorNode::updateTracks(
    const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
  double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // Extract positions from detections
  std::vector<Eigen::Vector3d> detection_positions;
  for (const auto& det : msg->detections) {
    Eigen::Vector3d pos(det.bbox3d.center.position.x,
                        det.bbox3d.center.position.y,
                        det.bbox3d.center.position.z);
    detection_positions.push_back(pos);
  }

  // Extract track positions
  std::vector<Eigen::Vector3d> track_positions;
  std::vector<std::string> track_ids;
  for (const auto& pair : active_tracks_) {
    track_positions.push_back(pair.second->getPosition());
    track_ids.push_back(pair.first);
  }

  // Data association
  if (!track_positions.empty() && !detection_positions.empty()) {
    auto association = DataAssociation::associate(
        track_positions, detection_positions, max_association_distance_);

    // Update matched tracks
    for (size_t i = 0; i < track_ids.size(); ++i) {
      if (association.detection_indices[i] >= 0) {
        int det_idx = association.detection_indices[i];
        active_tracks_[track_ids[i]]->update(
            detection_positions[det_idx], current_time);
      }
    }

    // Create new tracks for unmatched detections
    for (size_t j = 0; j < msg->detections.size(); ++j) {
      if (association.track_indices[j] < 0) {
        const auto& det = msg->detections[j];
        Eigen::Vector3d pos(det.bbox3d.center.position.x,
                            det.bbox3d.center.position.y,
                            det.bbox3d.center.position.z);

        auto filter = createIMMKFFilter();
        filter->initialize(pos);

        auto tracker = std::make_shared<IMMKFTracker>(det.id, filter);
        tracker->update(pos, current_time);

        active_tracks_[det.id] = tracker;

        RCLCPP_DEBUG(this->get_logger(), "Created new track for object: %s",
                     det.id.c_str());
      }
    }
  } else if (!detection_positions.empty() && active_tracks_.empty()) {
    // No existing tracks, create new ones for all detections
    for (const auto& det : msg->detections) {
      Eigen::Vector3d pos(det.bbox3d.center.position.x,
                          det.bbox3d.center.position.y,
                          det.bbox3d.center.position.z);

      auto filter = createIMMKFFilter();
      filter->initialize(pos);

      auto tracker = std::make_shared<IMMKFTracker>(det.id, filter);
      tracker->update(pos, current_time);

      active_tracks_[det.id] = tracker;
    }
  }
}

void IMMKFPredictorNode::publishTrajectories(double current_time) {
  for (const auto& pair : active_tracks_) {
    const auto& tracker = pair.second;

    if (!tracker->isConfirmed(confirmation_threshold_)) {
      continue;  // Only publish confirmed tracks
    }

    // Generate trajectory
    auto trajectory = tracker->generateTrajectory(prediction_horizon_s_,
                                                   prediction_step_hz_);

    if (trajectory.empty()) {
      continue;
    }

    // Create and publish message
    auto msg = std::make_unique<immkf_predictor::msg::PredictedTrajectory>();
    msg->header.frame_id = "camera_init";
    msg->header.stamp.sec = static_cast<uint32_t>(current_time);
    msg->header.stamp.nanosec =
        static_cast<uint32_t>((current_time - msg->header.stamp.sec) * 1e9);

    msg->object_id = tracker->getObjectId();
    msg->publish_timestamp = current_time;
    msg->track_confidence = tracker->getTrackConfidence();

    // Fill trajectory points
    for (const auto& pt : trajectory) {
      immkf_predictor::msg::TrajectoryPoint tpt;
      tpt.timestamp.sec = static_cast<uint32_t>(pt.timestamp);
      tpt.timestamp.nanosec =
          static_cast<uint32_t>((pt.timestamp - tpt.timestamp.sec) * 1e9);
      tpt.x = pt.position.x();
      tpt.y = pt.position.y();
      tpt.z = pt.position.z();
      tpt.vx = pt.velocity.x();
      tpt.vy = pt.velocity.y();
      tpt.vz = pt.velocity.z();
      tpt.confidence = pt.confidence;
      tpt.model_index = pt.model_index;

      msg->trajectory.push_back(tpt);
    }

    // Fill model probabilities
    auto probs = tracker->getModelProbabilities();
    for (double p : probs) {
      msg->model_probabilities.push_back(p);
    }

    trajectory_pub_->publish(std::move(msg));

    RCLCPP_DEBUG(this->get_logger(),
                 "Published trajectory for object %s with %zu points",
                 tracker->getObjectId().c_str(), trajectory.size());
  }
}

void IMMKFPredictorNode::publishVisualization(double current_time) {
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  int marker_id = 0;

  for (const auto& pair : active_tracks_) {
    const auto& tracker = pair.second;

    if (!tracker->isConfirmed(confirmation_threshold_)) {
      continue;
    }

    auto trajectory = tracker->generateTrajectory(prediction_horizon_s_,
                                                   prediction_step_hz_);

    if (trajectory.empty()) {
      continue;
    }

    // Create line strip marker
    auto marker = createTrajectoryMarker(tracker->getObjectId(), trajectory,
                                         marker_id++);
    marker.header.stamp.sec = static_cast<uint32_t>(current_time);
    marker.header.stamp.nanosec =
        static_cast<uint32_t>((current_time - marker.header.stamp.sec) * 1e9);

    marker_array->markers.push_back(marker);
  }

  marker_pub_->publish(std::move(marker_array));
}

std::shared_ptr<IMMKFFilter> IMMKFPredictorNode::createIMMKFFilter() {
  // Create three motion models
  std::vector<std::shared_ptr<MotionModel>> models;
  models.push_back(
      std::make_shared<ConstantVelocityModel>(q_pos_, q_vel_, r_pos_));
  models.push_back(
      std::make_shared<ConstantAccelerationModel>(q_pos_, q_vel_, q_acc_, r_pos_));
  auto singer = std::make_shared<SingerModel>(q_pos_, q_vel_, q_acc_,
                                               singer_decay_rate_, r_pos_);
  models.push_back(singer);

  // Initial model probabilities
  std::vector<double> initial_probs = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};

  // Mode transition matrix (Markov chain)
  Eigen::MatrixXd mode_transitions(3, 3);
  mode_transitions << 0.95, 0.025, 0.025,  //
      0.025, 0.95, 0.025,                       //
      0.025, 0.025, 0.95;

  return std::make_shared<IMMKFFilter>(models, initial_probs, mode_transitions);
}

void IMMKFPredictorNode::ageOutTracks() {
  std::vector<std::string> to_remove;

  for (auto& pair : active_tracks_) {
    if (pair.second->getTrackAge() > max_track_age_) {
      to_remove.push_back(pair.first);
    }
  }

  for (const auto& id : to_remove) {
    active_tracks_.erase(id);
    RCLCPP_DEBUG(this->get_logger(), "Removed track: %s", id.c_str());
  }
}

visualization_msgs::msg::Marker IMMKFPredictorNode::createTrajectoryMarker(
    const std::string& object_id,
    const std::vector<TrajectoryPoint>& trajectory,
    int marker_id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "camera_init";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Green line for trajectory
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.7;
  marker.scale.x = 0.05;  // Line width

  // Add points
  for (const auto& pt : trajectory) {
    geometry_msgs::msg::Point p;
    p.x = pt.position.x();
    p.y = pt.position.y();
    p.z = pt.position.z();
    marker.points.push_back(p);
  }

  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 100000000;  // 0.1 seconds

  return marker;
}

}  // namespace immkf_predictor

RCLCPP_COMPONENTS_REGISTER_NODE(immkf_predictor::IMMKFPredictorNode)
