#ifndef PREDICTOR_NODE_HPP
#define PREDICTOR_NODE_HPP

#include <map>
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
public: // Constructors & Destructors
	explicit PredictorNode(const rclcpp::NodeOptions & options);
	~PredictorNode() override;

public: // Methods
	void onConfigure();

private: // Member Variables
	std::unique_ptr<TrackManager> track_manager_;
	rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
	std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
	std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
	State initial_state_ = State::Zero();
	Covariance initial_covariance_ = Covariance::Identity();
	std::string input_topic_;
	std::string output_path_topic_prefix_;
	std::string output_pose_topic_prefix_;
	double prediction_dt_ = 0.1;
	std::size_t prediction_horizon_ = 10;
	double track_timeout_ = 5.0;
	std::size_t max_tracks_ = 100;
	double min_confidence_ = 0.0;
	std::vector<int> allowed_class_ids_;
	bool publish_per_track_ = true;
	bool publish_aggregated_ = false;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr aggregated_path_pub_;

	struct Detection
	{
		std::string track_id;
		double confidence;
		int class_id;
		Measurement measurement;
	};

	std::vector<Detection> extractDetections(
		const yolo_msgs::msg::DetectionArray & msg,
		std::string & frame_id) const;
	void publishTrackPredictions(
		const std::string & track_id,
		const rclcpp::Time & stamp,
		const std::string & frame_id);
	void publishAggregatedPredictions(
		const rclcpp::Time & stamp,
		const std::string & frame_id) const;
	void detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);

};

}  // namespace immkf_predictor

#endif  // PREDICTOR_NODE_HPP
