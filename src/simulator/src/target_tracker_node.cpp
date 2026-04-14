#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <memory>

class TargetTrackerNode : public rclcpp::Node {
public:
    TargetTrackerNode() : rclcpp::Node("target_tracker_node") {
        // Declare parameters
        declare_parameter("trajectory_type", "circular");  // circular, figure_8, random_walk
        declare_parameter("trajectory_radius", 5.0);
        declare_parameter("trajectory_height", 2.0);
        declare_parameter("trajectory_period", 20.0);
        declare_parameter("publish_rate", 10.0);
        declare_parameter("simulation_fps", 30.0);

        // Get parameters
        trajectory_type_ = get_parameter("trajectory_type").as_string();
        trajectory_radius_ = get_parameter("trajectory_radius").as_double();
        trajectory_height_ = get_parameter("trajectory_height").as_double();
        trajectory_period_ = get_parameter("trajectory_period").as_double();
        publish_rate_ = get_parameter("publish_rate").as_double();
        sim_fps_ = get_parameter("simulation_fps").as_double();

        // Subscribe to model states for getting target position
        // Note: Not used in pure trajectory generation mode
        // Uncomment if you want to get actual target position from Gazebo:
        // model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
        //     "/gazebo/model_states",
        //     rclcpp::SensorDataQoS(),
        //     [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        //        this->modelStatesCallback(msg);
        //    });

        // Publisher for predicted trajectory (as YOLO DetectionArray)
        trajectory_pub_ = create_publisher<yolo_msgs::msg::DetectionArray>(
            "predicted_trajectory",
            rclcpp::SensorDataQoS());

        // Publisher for current target detection
        detection_pub_ = create_publisher<yolo_msgs::msg::DetectionArray>(
            "sim_target_detection",
            rclcpp::SensorDataQoS());

        // Timer for trajectory update
        double update_period = 1.0 / publish_rate_;
        trajectory_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(update_period * 1000)),
            [this]() { this->trajectoryUpdate(); });

        RCLCPP_INFO(get_logger(), "Target Tracker Node initialized");
        RCLCPP_INFO(get_logger(), "Trajectory type: %s, Radius: %.2f, Height: %.2f",
                    trajectory_type_.c_str(), trajectory_radius_, trajectory_height_);

        time_elapsed_ = 0.0;
        target_x_ = 0.0;
        target_y_ = 0.0;
        target_z_ = 0.0;
        target_vx_ = 0.0;
        target_vy_ = 0.0;
        target_vz_ = 0.0;
    }

private:
    // ROS components
    rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr detection_pub_;
    rclcpp::TimerBase::SharedPtr trajectory_timer_;

    // Parameters
    std::string trajectory_type_;
    double trajectory_radius_;
    double trajectory_height_;
    double trajectory_period_;
    double publish_rate_;
    double sim_fps_;

    // Target state
    double time_elapsed_;
    double target_x_, target_y_, target_z_;
    double target_vx_, target_vy_, target_vz_;

    // Constants
    static constexpr int PREDICTION_HORIZON = 10;
    static constexpr double DT = 0.1;

    void modelStatesCallback([[maybe_unused]] const geometry_msgs::msg::Pose::SharedPtr msg) {
        // Not used for pure trajectory generation
        // Target state is computed from time, not from Gazebo feedback
    }

    void trajectoryUpdate() {
        // Generate trajectory prediction
        generateTrajectoryPrediction();

        // Generate current detection
        generateCurrentDetection();

        time_elapsed_ += 1.0 / publish_rate_;
    }

    void generateTrajectoryPrediction() {
        auto msg = std::make_unique<yolo_msgs::msg::DetectionArray>();
        msg->header.stamp = get_clock()->now();
        msg->header.frame_id = "base";

        // Generate predicted trajectory points
        for (int i = 0; i < PREDICTION_HORIZON; ++i) {
            double t = time_elapsed_ + i * DT;
            geometry_msgs::msg::Point pred_pos = computeTrajectoryPoint(t);

            // Create detection message for this trajectory point
            yolo_msgs::msg::Detection det;

            det.bbox3d.center.position.x = pred_pos.x;
            det.bbox3d.center.position.y = pred_pos.y;
            det.bbox3d.center.position.z = pred_pos.z;

            det.bbox3d.size.x = 0.1;  // QR code size
            det.bbox3d.size.y = 0.1;
            det.bbox3d.size.z = 0.1;

            det.score = 0.95;  // High confidence for simulated data
            det.class_name = "qr_code";
            det.id = "target_0";

            msg->detections.push_back(det);
        }

        trajectory_pub_->publish(std::move(msg));
    }

    void generateCurrentDetection() {
        auto msg = std::make_unique<yolo_msgs::msg::DetectionArray>();
        msg->header.stamp = get_clock()->now();
        msg->header.frame_id = "base";

        yolo_msgs::msg::Detection det;

        det.bbox3d.center.position.x = target_x_;
        det.bbox3d.center.position.y = target_y_;
        det.bbox3d.center.position.z = target_z_;

        det.bbox3d.size.x = 0.1;
        det.bbox3d.size.y = 0.1;
        det.bbox3d.size.z = 0.1;

        det.score = 0.95;
        det.class_name = "qr_code";
        det.id = "target_0";

        msg->detections.push_back(det);
        detection_pub_->publish(std::move(msg));
    }

    geometry_msgs::msg::Point computeTrajectoryPoint(double t) {
        geometry_msgs::msg::Point p;

        if (trajectory_type_ == "circular") {
            // Circular trajectory around the gimbal
            double angle = 2 * M_PI * t / trajectory_period_;
            p.x = trajectory_radius_ * std::cos(angle);
            p.y = trajectory_radius_ * std::sin(angle);
            p.z = trajectory_height_;
        } else if (trajectory_type_ == "figure_8") {
            // Figure-8 trajectory
            double angle = 2 * M_PI * t / trajectory_period_;
            p.x = trajectory_radius_ * std::sin(angle);
            p.y = trajectory_radius_ * std::sin(angle) * std::cos(angle);
            p.z = trajectory_height_;
        } else if (trajectory_type_ == "spiral_up") {
            // Spiral upward trajectory
            double angle = 2 * M_PI * t / trajectory_period_;
            double height_increment = 0.5 * t / trajectory_period_;  // Rise up to 0.5m
            p.x = trajectory_radius_ * std::cos(angle);
            p.y = trajectory_radius_ * std::sin(angle);
            p.z = trajectory_height_ + height_increment;
        } else {
            // Default: moving away horizontally
            p.x = trajectory_radius_ + 0.5 * t;
            p.y = 0.0;
            p.z = trajectory_height_;
        }

        return p;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
