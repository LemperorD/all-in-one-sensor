#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>

class GazeboBridgeNode : public rclcpp::Node {
public:
    GazeboBridgeNode() : rclcpp::Node("gazebo_bridge_node") {
        // Declare parameters
        declare_parameter("camera_info_file", "/tmp/camera_info.yaml");
        declare_parameter("publish_tf", true);
        declare_parameter("camera_frame_id", "camera_optical_frame");
        declare_parameter("lidar_frame_id", "lidar_link");
        declare_parameter("base_frame_id", "base");

        // Get parameters
        camera_info_file_ = get_parameter("camera_info_file").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        camera_frame_id_ = get_parameter("camera_frame_id").as_string();
        lidar_frame_id_ = get_parameter("lidar_frame_id").as_string();
        base_frame_id_ = get_parameter("base_frame_id").as_string();

        // Create TF broadcaster for static transforms
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            publishStaticTransforms();
        }

        // Publishers for camera info
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            rclcpp::SensorDataQoS());

        // Subscribers from Gazebo
        // Note: In real setup, you would use image_transport and point cloud subscribers
        // For now, we just set up the infrastructure

        // Timer to publish static camera info
        info_timer_ = create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->publishCameraInfo(); });

        RCLCPP_INFO(get_logger(), "Gazebo Bridge Node initialized");
        RCLCPP_INFO(get_logger(), "Camera frame: %s, Lidar frame: %s",
                    camera_frame_id_.c_str(), lidar_frame_id_.c_str());
    }

private:
    // ROS components
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr info_timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // Parameters
    std::string camera_info_file_;
    bool publish_tf_;
    std::string camera_frame_id_;
    std::string lidar_frame_id_;
    std::string base_frame_id_;

    // Cached camera info
    sensor_msgs::msg::CameraInfo camera_info_;

    void publishStaticTransforms() {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // Transform from base to camera_optical_frame
        auto camera_tf = geometry_msgs::msg::TransformStamped();
        camera_tf.header.frame_id = base_frame_id_;
        camera_tf.child_frame_id = camera_frame_id_;
        camera_tf.header.stamp = get_clock()->now();

        // Camera position
        camera_tf.transform.translation.x = 0.0;
        camera_tf.transform.translation.y = 0.0;
        camera_tf.transform.translation.z = 0.15;

        // Camera rotation (Z-forward convention for optical frame)
        tf2::Quaternion q;
        q.setRPY(-1.5708, 0, -1.5708);  // -90 around X and Z
        camera_tf.transform.rotation.x = q.x();
        camera_tf.transform.rotation.y = q.y();
        camera_tf.transform.rotation.z = q.z();
        camera_tf.transform.rotation.w = q.w();

        transforms.push_back(camera_tf);

        // Transform from base to lidar
        auto lidar_tf = geometry_msgs::msg::TransformStamped();
        lidar_tf.header.frame_id = base_frame_id_;
        lidar_tf.child_frame_id = lidar_frame_id_;
        lidar_tf.header.stamp = get_clock()->now();

        lidar_tf.transform.translation.x = 0.0;
        lidar_tf.transform.translation.y = 0.05;
        lidar_tf.transform.translation.z = 0.15;

        lidar_tf.transform.rotation.w = 1.0;
        lidar_tf.transform.rotation.x = 0.0;
        lidar_tf.transform.rotation.y = 0.0;
        lidar_tf.transform.rotation.z = 0.0;

        transforms.push_back(lidar_tf);

        if (tf_broadcaster_) {
            tf_broadcaster_->sendTransform(transforms);
        }
    }

    void publishCameraInfo() {
        // Initialize camera info with default values
        camera_info_.header.stamp = get_clock()->now();
        camera_info_.header.frame_id = camera_frame_id_;

        if (camera_info_.width == 0) {
            // Set default values
            camera_info_.width = 960;
            camera_info_.height = 720;

            // Camera matrix (identity-like)
            camera_info_.k = {
                1470.0, 0.0, 480.0,
                0.0, 1470.0, 360.0,
                0.0, 0.0, 1.0
            };

            camera_info_.p = {
                1470.0, 0.0, 480.0, 0.0,
                0.0, 1470.0, 360.0, 0.0,
                0.0, 0.0, 1.0, 0.0
            };

            camera_info_.r = {
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            };

            camera_info_.distortion_model = "plumb_bob";
            camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        }

        camera_info_.header.stamp = get_clock()->now();
        camera_info_pub_->publish(camera_info_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
