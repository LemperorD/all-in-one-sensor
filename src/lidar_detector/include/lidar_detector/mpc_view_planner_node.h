#ifndef MPC_VIEW_PLANNER_NODE_H_
#define MPC_VIEW_PLANNER_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <deque>
#include <memory>

#include "lidar_detector/mpc_view_planner.h"

namespace lidar_detector {

/**
 * ROS2 Node for MPC-based active view planning
 *
 * Subscribes to:
 *   - predicted trajectory (from IMMKF)
 *   - current PTZ state
 *
 * Publishes:
 *   - PTZ commands (pan, tilt, rates)
 */
class MPCViewPlannerNode : public rclcpp::Node {
public:
    explicit MPCViewPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MPCViewPlannerNode() override = default;

private:
    // Callbacks
    void trajectoryCallback(const yolo_msgs::msg::DetectionArray::SharedPtr trajectory_msg);
    void currentPTZCallback(const std_msgs::msg::Float32MultiArray::SharedPtr ptz_msg);

    // Timer callback for periodic planning
    void planningTimer();

    // ROS 2 components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ptz_cmd_pub_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ptz_state_sub_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // MPC planner
    std::shared_ptr<MPCViewPlanner> planner_;

    // State
    struct PTZState {
        double pan;
        double tilt;
        double timestamp;
    } ptz_state_;

    struct LatestTrajectory {
        std::vector<TrajectoryPoint> points;
        rclcpp::Time timestamp;
        bool updated;
    } latest_trajectory_;

    // Parameters
    int mpc_horizon_;
    double mpc_dt_;
    double planning_period_;
    double w_tracking_;
    double w_smoothness_;
    double w_control_;
    double max_pan_rate_;
    double max_tilt_rate_;
    double max_pan_accel_;
    double max_tilt_accel_;
};

}  // namespace lidar_detector

#endif  // MPC_VIEW_PLANNER_NODE_H_
