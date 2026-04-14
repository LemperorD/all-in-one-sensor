#ifndef MPC_VIEW_PLANNER_NODE_H_
#define MPC_VIEW_PLANNER_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include <vector>

#include "mpc_gimbal_planner/mpc_view_planner.h"

namespace mpc_gimbal_planner {

/**
 * ROS2 Node for MPC-based gimbal view planning
 *
 * Subscribes to:
 *   - predicted trajectory from IMMKF (as DetectionArray)
 *   - current gimbal state (pan, tilt angles)
 *
 * Publishes:
 *   - gimbal commands (pan, tilt, rates)
 */
class MPCViewPlannerNode : public rclcpp::Node {
public:
    explicit MPCViewPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MPCViewPlannerNode() override = default;

private:
    // Callbacks
    void trajectoryCallback(const yolo_msgs::msg::DetectionArray::SharedPtr trajectory_msg);
    void gimbalStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr state_msg);
    void planningTimer();

    // ROS 2 components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gimbal_cmd_pub_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gimbal_state_sub_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // MPC planner
    std::shared_ptr<MPCViewPlanner> planner_;

    // State variables
    struct GimbalState {
        double pan;
        double tilt;
        double timestamp;
    } gimbal_state_;

    struct TrajectoryData {
        std::vector<TrajectoryPoint> points;
        rclcpp::Time timestamp;
        bool updated;
    } trajectory_data_;

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

}  // namespace mpc_gimbal_planner

#endif  // MPC_VIEW_PLANNER_NODE_H_
