#include "immkf_predictor/predictor_node.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <vector>

namespace immkf_predictor
{

PredictorNode::PredictorNode(const rclcpp::NodeOptions & options)
	: Node("immkf_predictor_node", options)
{
	RCLCPP_INFO(get_logger(), "Starting PredictorNode with multi-target support");
	onConfigure();
}

PredictorNode::~PredictorNode()
{
}

void PredictorNode::onConfigure()
{
	// Topic configuration
	input_topic_ = this->declare_parameter<std::string>("topics.input", "/tracks_3d");
	output_path_topic_prefix_ = this->declare_parameter<std::string>("topics.output_path_prefix", "/immkf/tracks");
	output_pose_topic_prefix_ = this->declare_parameter<std::string>("topics.output_pose_prefix", "/immkf/poses");

	// Multi-target configuration
	track_timeout_ = this->declare_parameter<double>("track_timeout_seconds", 5.0);
	max_tracks_ = static_cast<std::size_t>(this->declare_parameter<int>("max_tracks", 100));
	min_confidence_ = this->declare_parameter<double>("min_confidence", 0.0);
	publish_per_track_ = this->declare_parameter<bool>("publish_per_track", true);
	publish_aggregated_ = this->declare_parameter<bool>("publish_aggregated", false);

	// Class filtering
	const auto allowed_class_ids_list = this->declare_parameter<std::vector<int64_t>>("allowed_class_ids", std::vector<int64_t>{});
	for (const auto id : allowed_class_ids_list) {
		allowed_class_ids_.push_back(static_cast<int>(id));
	}

	// Prediction configuration
	const auto initial_state_vector = this->declare_parameter<std::vector<double>>(
		"initial_state", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	const auto initial_covariance_diagonal = this->declare_parameter<std::vector<double>>(
		"initial_covariance_diagonal", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
	prediction_dt_ = this->declare_parameter<double>("prediction_dt", 0.1);
	prediction_horizon_ = static_cast<std::size_t>(
		this->declare_parameter<int>("prediction_horizon", 10));

	// Create TrackManager with IMM-KF configuration
	ImmkfConfig config;
	config.measurement_noise.diagonal() = Eigen::Vector2d(
		this->declare_parameter<double>("measurement_noise_x", 1.0),
		this->declare_parameter<double>("measurement_noise_y", 1.0));
	config.transition_matrix = ImmkfPredictor::defaultTransitionMatrix();

	track_manager_ = std::make_unique<TrackManager>(config);

	// Set initial state and covariance
	for (std::size_t i = 0; i < initial_state_vector.size() && i < 6; ++i) {
		initial_state_(static_cast<int>(i)) = initial_state_vector[i];
	}

	initial_covariance_.setZero();
	for (std::size_t i = 0; i < initial_covariance_diagonal.size() && i < 6; ++i) {
		initial_covariance_(static_cast<int>(i), static_cast<int>(i)) = initial_covariance_diagonal[i];
	}

	// Create subscriptions and publishers
	detection_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
		input_topic_, 10,
		std::bind(&PredictorNode::detectionsCallback, this, std::placeholders::_1));

	if (publish_aggregated_) {
		aggregated_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
			output_path_topic_prefix_ + "/all_tracks", 10);
	}

	RCLCPP_INFO(
		get_logger(),
		"Multi-target IMM-KF configured: input=%s, timeout=%.1f, max_tracks=%zu, "
		"min_confidence=%.2f, publish_per_track=%s, publish_aggregated=%s",
		input_topic_.c_str(), track_timeout_, max_tracks_, min_confidence_,
		publish_per_track_ ? "true" : "false", publish_aggregated_ ? "true" : "false");
}

std::vector<PredictorNode::Detection> PredictorNode::extractDetections(
	const yolo_msgs::msg::DetectionArray & msg,
	std::string & frame_id) const
{
	std::vector<Detection> detections;
	frame_id = msg.header.frame_id;

	for (const auto & det : msg.detections) {
		// Filter by confidence
		if (det.score < min_confidence_) {
			continue;
		}

		// Filter by class ID if configured
		if (!allowed_class_ids_.empty()) {
			if (std::find(allowed_class_ids_.begin(), allowed_class_ids_.end(), det.class_id) ==
				allowed_class_ids_.end()) {
				continue;
			}
		}

		Detection detection;
		detection.track_id = det.id;
		detection.confidence = det.score;
		detection.class_id = det.class_id;

		// Extract measurement from 3D or 2D bbox
		if (!det.bbox3d.frame_id.empty()) {
			detection.measurement(0) = det.bbox3d.center.position.x;
			detection.measurement(1) = det.bbox3d.center.position.y;
		} else {
			detection.measurement(0) = det.bbox.center.position.x;
			detection.measurement(1) = det.bbox.center.position.y;
		}

		detections.push_back(detection);
	}

	return detections;
}

void PredictorNode::publishTrackPredictions(
	const std::string & track_id,
	const rclcpp::Time & stamp,
	const std::string & frame_id)
{
	auto predictor = track_manager_->getOrCreateTrack(track_id);
	if (!predictor || !predictor->isInitialized()) {
		return;
	}

	// Create or get publishers for this track
	if (path_pubs_.find(track_id) == path_pubs_.end()) {
		std::string path_topic = output_path_topic_prefix_ + "/" + track_id + "/path";
		std::string pose_topic = output_pose_topic_prefix_ + "/" + track_id + "/current";
		path_pubs_[track_id] = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
		pose_pubs_[track_id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
	}

	// Get predicted trajectory
	const auto trajectory = track_manager_->getTrajectory(track_id, prediction_dt_, prediction_horizon_);

	// Publish path
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

	path_pubs_[track_id]->publish(path_msg);

	// Publish current pose
	const auto & fused_state = predictor->fusedState();
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.stamp = stamp;
	pose_msg.header.frame_id = frame_id;
	pose_msg.pose.position.x = fused_state(0);
	pose_msg.pose.position.y = fused_state(1);
	pose_msg.pose.position.z = 0.0;
	pose_msg.pose.orientation.w = 1.0;

	pose_pubs_[track_id]->publish(pose_msg);
}

void PredictorNode::publishAggregatedPredictions(
	const rclcpp::Time & stamp,
	const std::string & frame_id) const
{
	if (!aggregated_path_pub_) {
		return;
	}

	// Collect all track trajectories into a single path message
	nav_msgs::msg::Path aggregated_path;
	aggregated_path.header.stamp = stamp;
	aggregated_path.header.frame_id = frame_id;

	const auto all_tracks = track_manager_->getAllTracks();
	for (const auto & track_state : all_tracks) {
		const auto & trajectory = track_state.trajectory;
		for (std::size_t i = 0; i < trajectory.size(); ++i) {
			geometry_msgs::msg::PoseStamped pose_msg;
			pose_msg.header.stamp = stamp + rclcpp::Duration::from_seconds(prediction_dt_ * static_cast<double>(i + 1));
			pose_msg.header.frame_id = frame_id;
			pose_msg.pose.position.x = trajectory[i](0);
			pose_msg.pose.position.y = trajectory[i](1);
			pose_msg.pose.position.z = 0.0;
			pose_msg.pose.orientation.w = 1.0;
			aggregated_path.poses.push_back(pose_msg);
		}
	}

	aggregated_path_pub_->publish(aggregated_path);
}

void PredictorNode::detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
	std::string frame_id;
	const auto detections = extractDetections(*msg, frame_id);

	// Limit number of tracks
	if (track_manager_->size() >= max_tracks_ && detections.size() > track_manager_->size()) {
		RCLCPP_WARN(
			get_logger(),
			"Reached maximum number of tracks (%zu), ignoring new detections",
			max_tracks_);
		return;
	}

	// Update existing tracks and create new ones
	for (const auto & detection : detections) {
		auto predictor = track_manager_->getOrCreateTrack(detection.track_id);

		// Initialize on first detection
		if (!predictor->isInitialized()) {
			State initial_state = initial_state_;
			initial_state(0) = detection.measurement(0);
			initial_state(1) = detection.measurement(1);
			predictor->reset(initial_state, initial_covariance_);
		}

		// Predict and update
		predictor->predict(prediction_dt_);
		predictor->update(detection.measurement);

		// Publish predictions for this track
		if (publish_per_track_) {
			publishTrackPredictions(detection.track_id, msg->header.stamp, frame_id);
		}
	}

	// Publish aggregated predictions if enabled
	if (publish_aggregated_) {
		publishAggregatedPredictions(msg->header.stamp, frame_id);
	}

	// Clean up old tracks
	track_manager_->pruneInactiveTracks(rclcpp::Clock().now().seconds(), track_timeout_);

	RCLCPP_DEBUG(
		get_logger(),
		"Processed %zu detections, managing %zu active tracks",
		detections.size(), track_manager_->size());
}

}  // namespace immkf_predictor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(immkf_predictor::PredictorNode)