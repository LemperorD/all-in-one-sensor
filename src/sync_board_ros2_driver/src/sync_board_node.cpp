#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <memory>
#include <thread>
#include "sync_core.hpp"

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

        // Initialize Sync server and board
        server_ = std::make_unique<SyncServer>(device_path, 12080, 10);

        // Configure trigger parameters
        int trigger_edge_rising = this->get_parameter("trigger_edge_rising").as_int();
        int trigger_mode = this->get_parameter("trigger_mode").as_int();
        int trigger_fps = this->get_parameter("trigger_fps").as_int();
        if (server_) {
            server_->board.SetCamTrigger(trigger_edge_rising, trigger_mode, trigger_fps);
        }

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

        imu_thread_ = std::thread([this]() {
            while (rclcpp::ok() && run_imu_monitor_.load()) {
                if (!server_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                sync_proto::IMU_RAW imu_msg;
                imu_msg.set_time_us(imu_data_.timestamp_us);
                imu_msg.set_ax(imu_data_.ax);
                imu_msg.set_ay(imu_data_.ay);
                imu_msg.set_az(imu_data_.az);
                imu_msg.set_gx(imu_data_.gx);
                imu_msg.set_gy(imu_data_.gy);
                imu_msg.set_gz(imu_data_.gz);
                // Serialize to string
                std::string serialized_data;

                if (server_->SerializeMessage(sync_proto::MessageID::MSG_IMU_RAW, imu_msg, serialized_data)) {
                    server_->send_to_queues(serialized_data);
                }
            }
        } );

        trigger_monitor_thread_ = std::thread([this]() {
            while (rclcpp::ok() && run_trigger_monitor_.load()) {
                if (!server_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                uint64_t cam_ts;
                uint64_t recv_ts;
                if (server_->board.pop_timestamp(recv_ts, cam_ts, true)) {
                    sync_proto::TRIGGER trigger_msg;
                    trigger_msg.set_trigger_time_us(cam_ts);
                    trigger_msg.set_recv_trigger_time_us(recv_ts);
                    std::string serialized_data;
                    if (server_->SerializeMessage(sync_proto::MessageID::MSG_TRIGGER, trigger_msg, serialized_data)) {
                        server_->send_to_queues(serialized_data);
                    }
                }
            }
        });
    }

    ~SyncBoardNode()
    {
        run_imu_monitor_.store(false);
        if (imu_thread_.joinable()) {
            imu_thread_.join();
        }
        run_trigger_monitor_.store(false);
        if (trigger_monitor_thread_.joinable()) {
            trigger_monitor_thread_.join();
        }
        if (server_) {
            server_->board.stop_recv_message();
            server_->board.disconnect();
        }
    }

private:
    void publish_imu()
    {
        ImuData imu_data;
        if (server_ && server_->board.getImuData(imu_data_, true)) {
            auto imu_msg = sensor_msgs::msg::Imu();
            // Use timestamp from sync board instead of system time
            uint64_t timestamp_ns = imu_data_.timestamp_us * 1000ULL;
            imu_msg.header.stamp = rclcpp::Time(timestamp_ns);
            imu_msg.header.frame_id = "imu_link";

            // Fill linear acceleration
            imu_msg.linear_acceleration.x = imu_data_.ax;
            imu_msg.linear_acceleration.y = imu_data_.ay;
            imu_msg.linear_acceleration.z = imu_data_.az;

            // Fill angular velocity
            imu_msg.angular_velocity.x = imu_data_.gx;
            imu_msg.angular_velocity.y = imu_data_.gy;
            imu_msg.angular_velocity.z = imu_data_.gz;

            // Publish
            imu_publisher_->publish(imu_msg);
        }
    }

    void trigger_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (server_) {
            server_->board.CamOneShotTrigger();
            response->success = true;
            response->message = "Hardware trigger signal sent for sensor synchronization";
        } else {
            response->success = false;
            response->message = "Sync server not initialized";
        }
    }

    std::unique_ptr<SyncServer> server_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread trigger_monitor_thread_;
    std::atomic<bool> run_trigger_monitor_{true};
    std::thread imu_thread_;
    std::atomic<bool> run_imu_monitor_{true};

    ImuData imu_data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncBoardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}