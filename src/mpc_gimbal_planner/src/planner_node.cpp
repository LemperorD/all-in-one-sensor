#include "mpc_gimbal_planner/planner_node.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kHalfPi = 1.57079632679489661923;
}  // namespace

namespace mpc_gimbal_planner
{
PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner_node", options)
{
	onConfigure();
	// Initialize MPC controller
	mpc_ = std::make_unique<MPCGimbal>(config_);
	patrol_start_time_ = this->now();
	const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
	control_timer_ = this->create_wall_timer(
		timer_period, std::bind(&PlannerNode::publishCommand, this));
	// TF
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
	// Subscribers and publishers
	path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
		input_path_topic_, rclcpp::QoS(10),
		std::bind(&PlannerNode::onPath, this, std::placeholders::_1));
	state_sub_ = this->create_subscription<simulator::msg::Gimbal>(
		input_state_topic_, rclcpp::QoS(10),
		std::bind(&PlannerNode::onGimbalState, this, std::placeholders::_1));
	command_pub_ = this->create_publisher<simulator::msg::GimbalCmd>(output_cmd_topic_, rclcpp::QoS(10));
}

PlannerNode::~PlannerNode()
{
}

void PlannerNode::onConfigure()
{
	input_path_topic_ = this->declare_parameter<std::string>("input_path_topic", "/immkf/tracks/all_tracks");
	input_state_topic_ = this->declare_parameter<std::string>("input_state_topic", "robot_base/gimbal_state");
	output_cmd_topic_ = this->declare_parameter<std::string>("output_cmd_topic", "robot_base/gimbal_cmd");
	base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
	prediction_horizon_ = static_cast<std::size_t>(this->declare_parameter<int>("prediction_horizon", 10));
	prediction_dt_ = this->declare_parameter<double>("prediction_dt", 0.1);
	control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
	target_timeout_sec_ = this->declare_parameter<double>("target_timeout_sec", 0.5);
	
	// Patrol mode configuration
	enable_patrol_ = this->declare_parameter<bool>("enable_patrol", true);
	control_mode_ = enable_patrol_ ? ControlMode::Patrol : ControlMode::Track;
	
	yaw_min_ = this->declare_parameter<double>("yaw_min", -1.57);
	yaw_max_ = this->declare_parameter<double>("yaw_max", 1.57);
	pitch_min_ = this->declare_parameter<double>("pitch_min", -0.8);
	pitch_max_ = this->declare_parameter<double>("pitch_max", 0.8);
	patrol_yaw_rate_ = this->declare_parameter<double>("patrol_yaw_rate", 0.01);
	patrol_pitch_rate_amplitude_ = this->declare_parameter<double>("patrol_pitch_rate_amplitude", 0.015);
	patrol_pitch_frequency_ = this->declare_parameter<double>("patrol_pitch_frequency", 0.15);
	patrol_yaw_margin_ = this->declare_parameter<double>("patrol_yaw_margin", 0.05);

	config_.horizon_steps = prediction_horizon_;
	config_.prediction_dt = prediction_dt_;
	config_.track_weight = this->declare_parameter<double>("track_weight", 1.0);
	config_.smooth_weight = this->declare_parameter<double>("smooth_weight", 0.2);
	config_.control_weight = this->declare_parameter<double>("control_weight", 0.05);
	config_.yaw_min = yaw_min_;
	config_.yaw_max = yaw_max_;
	config_.pitch_min = pitch_min_;
	config_.pitch_max = pitch_max_;
	config_.max_rate = this->declare_parameter<double>("max_rate", 1.0);
}

void PlannerNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
	latest_path_ = *msg;
	has_path_ = !msg->poses.empty();
	last_path_update_time_ = this->now();
}

void PlannerNode::onGimbalState(const simulator::msg::Gimbal::SharedPtr msg)
{
	current_angles_.x() = msg->yaw;
	current_angles_.y() = msg->pitch;
	has_state_ = true;
}

geometry_msgs::msg::PoseStamped PlannerNode::transformPoseToBase(
	const geometry_msgs::msg::PoseStamped & pose) const
{
	if (pose.header.frame_id.empty() || pose.header.frame_id == base_frame_) {
		return pose;
	}

	const auto transform = tf_buffer_->lookupTransform(base_frame_, pose.header.frame_id, tf2::TimePointZero);
	geometry_msgs::msg::PoseStamped transformed_pose;
	tf2::doTransform(pose, transformed_pose, transform);
	return transformed_pose;
}

std::vector<Eigen::Vector2d> PlannerNode::buildReferenceSequence() const
{
	std::vector<Eigen::Vector2d> references;
	if (!has_path_ || latest_path_.poses.empty()) {
		return references;
	}

	references.reserve(prediction_horizon_);

	for (std::size_t i = 0; i < latest_path_.poses.size() && i < prediction_horizon_; ++i) {
		geometry_msgs::msg::PoseStamped pose;
		pose.header = latest_path_.header;
		pose.pose = latest_path_.poses[i].pose;

		geometry_msgs::msg::PoseStamped in_base;
		try {
			in_base = transformPoseToBase(pose);
		} catch (const tf2::TransformException & ex) {
			RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
			return {};
		}

		const auto & position = in_base.pose.position;
		const double yaw = std::atan2(position.y, position.x);
		const double horizontal_distance = std::hypot(position.x, position.y);
		const double pitch = std::atan2(position.z, horizontal_distance);
		references.emplace_back(yaw, pitch);
	}

	if (references.empty()) {
		return references;
	}

	while (references.size() < prediction_horizon_) {
		references.push_back(references.back());
	}

	return references;
}

void PlannerNode::publishPatrolCommand(const rclcpp::Time & now)
{
	// compute dt since last patrol update to integrate target angles
	double dt = 1.0 / control_rate_hz_;
	if (last_patrol_update_time_.nanoseconds() != 0) {
		dt = (now - last_patrol_update_time_).seconds();
		if (dt <= 0.0) {
			dt = 1.0 / control_rate_hz_;
		}
	}

	const double elapsed_sec = (now - patrol_start_time_).seconds();
	const double pitch_phase = kTwoPi * patrol_pitch_frequency_ * elapsed_sec - kHalfPi;
	const double pitch_rate = patrol_pitch_rate_amplitude_ * std::sin(pitch_phase);

	// integrate to form absolute target angles (robot doesn't accept VELOCITY cmd)
	patrol_target_angles_.x() += patrol_yaw_rate_ * dt;
	patrol_target_angles_.y() += pitch_rate * dt;
	// if (patrol_target_angles_.x() > yaw_max_) {
	// 	patrol_target_angles_.x() -= kTwoPi;  // wrap around to stay within limits
	// } else if (patrol_target_angles_.x() < yaw_min_) {
	// 	patrol_target_angles_.x() += kTwoPi;  // wrap around to stay within limits
	// }

	simulator::msg::GimbalCmd command;
	command.tid = 0;
	// send as absolute angles so underlying controller can accept it
	command.yaw_type = simulator::msg::GimbalCmd::ABSOLUTE_ANGLE;
	command.pitch_type = simulator::msg::GimbalCmd::ABSOLUTE_ANGLE;
	command.position.yaw = static_cast<float>(patrol_target_angles_.x());
	command.position.pitch = static_cast<float>(patrol_target_angles_.y());
	// fill velocity fields for informational/debug use (may be ignored)
	command.velocity.yaw = static_cast<float>(patrol_yaw_rate_);
	command.velocity.pitch = static_cast<float>(pitch_rate);

	command_pub_->publish(command);
	current_rates_.x() = patrol_yaw_rate_;
	current_rates_.y() = pitch_rate;
	last_patrol_update_time_ = now;
}

void PlannerNode::publishCommand()
{
	if (!has_state_ || !mpc_) return;

	const auto now = this->now();
	const bool target_available = has_path_ && !latest_path_.poses.empty() &&
		((now - last_path_update_time_).seconds() <= target_timeout_sec_);

	if (!target_available) {
		if (control_mode_ != ControlMode::Patrol) {
			control_mode_ = ControlMode::Patrol;
			patrol_start_time_ = now;
			// initialize patrol targets from current measured angles
			patrol_target_angles_ = current_angles_;
			last_patrol_update_time_ = now;
			patrol_yaw_direction_ = current_angles_.x() >= 0.5 * (yaw_min_ + yaw_max_) ? -1 : 1;
			RCLCPP_INFO(this->get_logger(), "Target lost, switching to patrol mode");
		}
		publishPatrolCommand(now);
		return;
	}

	if (control_mode_ != ControlMode::Track) {
		control_mode_ = ControlMode::Track;
		RCLCPP_INFO(this->get_logger(), "Target detected, switching to track mode");
	}

	const auto references = buildReferenceSequence();
	if (references.empty()) {
		return;
	}
	const auto solution = mpc_->solve(current_angles_, current_rates_, references);

	simulator::msg::GimbalCmd command;
	command.tid = 0;
	command.yaw_type = simulator::msg::GimbalCmd::ABSOLUTE_ANGLE;
	command.pitch_type = simulator::msg::GimbalCmd::ABSOLUTE_ANGLE;
	command.position.yaw = static_cast<float>(solution.command.x());
	command.position.pitch = static_cast<float>(solution.command.y());
	command.velocity.yaw = static_cast<float>(solution.rate.x());
	command.velocity.pitch = static_cast<float>(solution.rate.y());

	command_pub_->publish(command);
	current_rates_ = solution.rate;
}

} // namespace mpc_gimbal_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mpc_gimbal_planner::PlannerNode)