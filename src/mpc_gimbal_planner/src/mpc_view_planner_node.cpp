#include "mpc_gimbal_planner/mpc_view_planner_node.h"
#include <rclcpp/rclcpp.hpp>

namespace mpc_gimbal_planner {

MPCViewPlannerNode::MPCViewPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mpc_view_planner_node", options),
      gimbal_state_{0.0, 0.0, 0.0},
      trajectory_data_{{}, rclcpp::Time(0), false} {

    // Declare parameters
    this->declare_parameter("mpc_horizon", 10);
    this->declare_parameter("mpc_dt", 0.1);
    this->declare_parameter("planning_period", 0.1);
    this->declare_parameter("w_tracking", 1.0);
    this->declare_parameter("w_smoothness", 0.5);
    this->declare_parameter("w_control", 0.2);
    this->declare_parameter("max_pan_rate", 2.0);
    this->declare_parameter("max_tilt_rate", 2.0);
    this->declare_parameter("max_pan_accel", 1.0);
    this->declare_parameter("max_tilt_accel", 1.0);
    this->declare_parameter("camera_fx", 1470.0);
    this->declare_parameter("camera_fy", 1470.0);
    this->declare_parameter("camera_cx", 480.0);
    this->declare_parameter("camera_cy", 360.0);

    // Get parameters
    mpc_horizon_ = this->get_parameter("mpc_horizon").as_int();
    mpc_dt_ = this->get_parameter("mpc_dt").as_double();
    planning_period_ = this->get_parameter("planning_period").as_double();
    w_tracking_ = this->get_parameter("w_tracking").as_double();
    w_smoothness_ = this->get_parameter("w_smoothness").as_double();
    w_control_ = this->get_parameter("w_control").as_double();
    max_pan_rate_ = this->get_parameter("max_pan_rate").as_double();
    max_tilt_rate_ = this->get_parameter("max_tilt_rate").as_double();
    max_pan_accel_ = this->get_parameter("max_pan_accel").as_double();
    max_tilt_accel_ = this->get_parameter("max_tilt_accel").as_double();

    double camera_fx = this->get_parameter("camera_fx").as_double();
    double camera_fy = this->get_parameter("camera_fy").as_double();
    double camera_cx = this->get_parameter("camera_cx").as_double();
    double camera_cy = this->get_parameter("camera_cy").as_double();

    // Initialize MPC planner
    planner_ = std::make_shared<MPCViewPlanner>(mpc_horizon_, mpc_dt_);
    planner_->setWeights(w_tracking_, w_smoothness_, w_control_);
    planner_->setConstraints(max_pan_rate_, max_tilt_rate_, max_pan_accel_, max_tilt_accel_);
    planner_->setCameraParams(camera_fx, camera_fy, camera_cx, camera_cy, 960, 720);

    // Create subscriptions
    trajectory_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
        "predicted_trajectory",
        rclcpp::SensorDataQoS(),
        std::bind(&MPCViewPlannerNode::trajectoryCallback, this, std::placeholders::_1));

    gimbal_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "gimbal_state",
        rclcpp::SensorDataQoS(),
        std::bind(&MPCViewPlannerNode::gimbalStateCallback, this, std::placeholders::_1));

    // Create publisher
    gimbal_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "gimbal_command", rclcpp::SystemDefaultsQoS());

    // Create planning timer
    planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(planning_period_ * 1000)),
        std::bind(&MPCViewPlannerNode::planningTimer, this));

    RCLCPP_INFO(this->get_logger(), "MPC View Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "Horizon: %d, dt: %.3f, planning_period: %.3f",
                mpc_horizon_, mpc_dt_, planning_period_);
    RCLCPP_INFO(this->get_logger(), "Weights - tracking: %.2f, smoothness: %.2f, control: %.2f",
                w_tracking_, w_smoothness_, w_control_);
}

void MPCViewPlannerNode::trajectoryCallback(
    const yolo_msgs::msg::DetectionArray::SharedPtr trajectory_msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received trajectory with %zu points",
                 trajectory_msg->detections.size());

    trajectory_data_.points.clear();

    // Convert detection array to trajectory points
    // Each detection is treated as one point in the trajectory
    for (const auto& detection : trajectory_msg->detections) {
        TrajectoryPoint pt;
        pt.timestamp = trajectory_msg->header.stamp.sec +
                      trajectory_msg->header.stamp.nanosec * 1e-9;
        pt.x = detection.bbox3d.center.position.x;
        pt.y = detection.bbox3d.center.position.y;
        pt.z = detection.bbox3d.center.position.z;
        pt.vx = detection.bbox3d.center.position.x;  // Placeholder: assume velocity from header
        pt.vy = detection.bbox3d.center.position.y;
        pt.vz = detection.bbox3d.center.position.z;
        pt.confidence = detection.score;

        trajectory_data_.points.push_back(pt);
    }

    trajectory_data_.timestamp = this->get_clock()->now();
    trajectory_data_.updated = true;
}

void MPCViewPlannerNode::gimbalStateCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr state_msg) {
    if (state_msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Invalid gimbal state: expected at least 2 values");
        return;
    }

    gimbal_state_.pan = state_msg->data[0];
    gimbal_state_.tilt = state_msg->data[1];
    gimbal_state_.timestamp = this->get_clock()->now().seconds();

    RCLCPP_DEBUG(this->get_logger(), "Gimbal state: pan=%.3f, tilt=%.3f",
                 gimbal_state_.pan, gimbal_state_.tilt);
}

void MPCViewPlannerNode::planningTimer() {
    if (!trajectory_data_.updated) {
        RCLCPP_DEBUG(this->get_logger(), "No trajectory update received yet");
        return;
    }

    // Update planner with latest trajectory
    planner_->updateTrajectory(trajectory_data_.points);

    // Solve MPC
    GimbalCommand cmd = planner_->solve(gimbal_state_.pan, gimbal_state_.tilt);

    // Publish command
    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp = this->get_clock()->now();
    twist_msg->header.frame_id = "gimbal";

    // Use angular velocity fields
    twist_msg->twist.angular.x = cmd.tilt_rate;   // pitch rate
    twist_msg->twist.angular.y = 0.0;              // roll rate (unused)
    twist_msg->twist.angular.z = cmd.pan_rate;    // yaw rate

    // Linear velocity field for absolute angles (as backup)
    twist_msg->twist.linear.x = cmd.pan;
    twist_msg->twist.linear.y = cmd.tilt;
    twist_msg->twist.linear.z = 0.0;

    gimbal_cmd_pub_->publish(*twist_msg);

    RCLCPP_DEBUG(this->get_logger(),
                "Published gimbal command: pan=%.3f (rate=%.3f), tilt=%.3f (rate=%.3f)",
                cmd.pan, cmd.pan_rate, cmd.tilt, cmd.tilt_rate);
}

}  // namespace mpc_gimbal_planner

// ROS 2 component registration
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mpc_gimbal_planner::MPCViewPlannerNode)
