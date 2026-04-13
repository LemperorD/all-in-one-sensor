#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include "sync_board.hpp"

class SyncBoardNode : public rclcpp::Node
{
public:
    SyncBoardNode() : Node("sync_board_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("device_path", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 921600);
        this->declare_parameter<int>("trigger_edge_rising", 1);
        this->declare_parameter<int>("trigger_mode", 1);
        this->declare_parameter<int>("trigger_fps", 30);

        // Get parameters
        std::string device_path = this->get_parameter("device_path").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Initialize SyncBoard
        if (!board_.connect(device_path, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to sync board at %s", device_path.c_str());
            return;
        }

        // Register callbacks
        board_.read_highres_imu();
        board_.start_recv_message();

        // Configure trigger parameters
        int trigger_edge_rising = this->get_parameter("trigger_edge_rising").as_int();
        int trigger_mode = this->get_parameter("trigger_mode").as_int();
        int trigger_fps = this->get_parameter("trigger_fps").as_int();
        board_.SetCamTrigger(trigger_edge_rising, trigger_mode, trigger_fps);

        // Create publisher
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // Create trigger service
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_signal",
            std::bind(&SyncBoardNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create timer to publish IMU data at 100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SyncBoardNode::publish_imu, this));
    }

    ~SyncBoardNode()
    {
        board_.stop_recv_message();
        board_.disconnect();
    }

private:
    void publish_imu()
    {
        ImuData imu_data;
        if (board_.getLatestImu(imu_data)) {
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp.sec = this->now().seconds();
            imu_msg.header.stamp.nanosec = (this->now().nanoseconds() % 1000000000);
            imu_msg.header.frame_id = "imu_link";

            // Fill linear acceleration
            imu_msg.linear_acceleration.x = imu_data.ax;
            imu_msg.linear_acceleration.y = imu_data.ay;
            imu_msg.linear_acceleration.z = imu_data.az;

            // Fill angular velocity
            imu_msg.angular_velocity.x = imu_data.gx;
            imu_msg.angular_velocity.y = imu_data.gy;
            imu_msg.angular_velocity.z = imu_data.gz;

            // Publish
            imu_publisher_->publish(imu_msg);
        }
    }

    void trigger_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        board_.CamOneShotTrigger();
        response->success = true;
        response->message = "Hardware trigger signal sent for sensor synchronization";
    }

    SyncBoard board_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncBoardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}