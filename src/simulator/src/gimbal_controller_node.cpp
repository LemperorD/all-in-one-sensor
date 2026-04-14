#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class GimbalControllerNode : public rclcpp::Node {
public:
    GimbalControllerNode() : rclcpp::Node("gimbal_controller_node") {
        // Declare parameters
        declare_parameter("max_pan_rate", 2.0);
        declare_parameter("max_tilt_rate", 2.0);
        declare_parameter("control_period", 0.01);

        // Get parameters
        max_pan_rate_ = get_parameter("max_pan_rate").as_double();
        max_tilt_rate_ = get_parameter("max_tilt_rate").as_double();
        control_period_ = get_parameter("control_period").as_double();

        // Subscribe to gimbal commands from MPC planner
        gimbal_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "gimbal_command",
            rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                this->gimbalCommandCallback(msg);
            });

        // Subscribe to joint states (from Gazebo)
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->jointStateCallback(msg);
            });

        // Publisher for gimbal state (pan, tilt angles)
        gimbal_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "gimbal_state",
            rclcpp::SensorDataQoS());

        // Timer for control loop
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(control_period_ * 1000)),
            [this]() { this->controlLoop(); });

        RCLCPP_INFO(get_logger(), "Gimbal Controller Node initialized");
        RCLCPP_INFO(get_logger(), "Max pan rate: %.2f rad/s, Max tilt rate: %.2f rad/s",
                    max_pan_rate_, max_tilt_rate_);

        // Initialize state
        current_pan_ = 0.0;
        current_tilt_ = 0.0;
        target_pan_ = 0.0;
        target_tilt_ = 0.0;
        pan_rate_cmd_ = 0.0;
        tilt_rate_cmd_ = 0.0;
        last_update_time_ = 0.0;
    }

private:
    // ROS subscriptions and publishers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gimbal_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gimbal_state_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Gimbal state
    double current_pan_;
    double current_tilt_;
    double target_pan_;
    double target_tilt_;
    double pan_rate_cmd_;
    double tilt_rate_cmd_;
    double last_update_time_;

    // Parameters
    double max_pan_rate_;
    double max_tilt_rate_;
    double control_period_;

    void gimbalCommandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // Extract pan and tilt rate commands from the twist message
        // angular.z = pan rate (yaw)
        // angular.x = tilt rate (pitch)
        pan_rate_cmd_ = msg->twist.angular.z;
        tilt_rate_cmd_ = msg->twist.angular.x;

        // Saturate to maximum rates
        pan_rate_cmd_ = std::clamp(pan_rate_cmd_, -max_pan_rate_, max_pan_rate_);
        tilt_rate_cmd_ = std::clamp(tilt_rate_cmd_, -max_tilt_rate_, max_tilt_rate_);

        RCLCPP_DEBUG(get_logger(), "Gimbal command: pan_rate=%.3f, tilt_rate=%.3f",
                     pan_rate_cmd_, tilt_rate_cmd_);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Update current gimbal angles from joint states
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "pan_joint") {
                current_pan_ = msg->position[i];
                if (msg->velocity.size() > i) {
                    // Could use velocity feedback if needed
                }
            } else if (msg->name[i] == "tilt_joint") {
                current_tilt_ = msg->position[i];
            }
        }
    }

    void controlLoop() {
        // Update target angles based on rate commands
        target_pan_ += pan_rate_cmd_ * control_period_;
        target_tilt_ += tilt_rate_cmd_ * control_period_;

        // Saturate angles to joint limits
        // Pan: -pi to pi
        target_pan_ = normalizeAngle(target_pan_);
        // Tilt: -pi/2 to pi/2
        target_tilt_ = std::clamp(target_tilt_, -M_PI / 2, M_PI / 2);

        // Publish gimbal state
        auto state_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        state_msg->data.resize(2);
        state_msg->data[0] = static_cast<float>(current_pan_);
        state_msg->data[1] = static_cast<float>(current_tilt_);
        gimbal_state_pub_->publish(std::move(state_msg));

        RCLCPP_DEBUG(get_logger(), "Control loop: target_pan=%.3f, target_tilt=%.3f",
                     target_pan_, target_tilt_);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
