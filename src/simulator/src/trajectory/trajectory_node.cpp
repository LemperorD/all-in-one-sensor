#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode()
  : Node("trajectory_node")
  {
    this->declare_parameter<std::string>("model_name", "iris");
    this->declare_parameter<std::string>("world_name", "gimbal_sim_world");
    this->declare_parameter<double>("rate", 50.0);
    this->declare_parameter<double>("radius", 2.0);
    this->declare_parameter<double>("height", 1.5);
    this->declare_parameter<double>("period", 20.0);

    model_name_ = this->get_parameter("model_name").as_string();
    world_name_ = this->get_parameter("world_name").as_string();
    double rate = this->get_parameter("rate").as_double();
    radius_ = this->get_parameter("radius").as_double();
    height_ = this->get_parameter("height").as_double();
    period_ = this->get_parameter("period").as_double();

    // ROS topic that will be bridged to Ignition
    std::string topic = "/" + model_name_ + "/pose_cmd"; // bridged via ros_gz_bridge
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&TrajectoryNode::on_timer, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Trajectory node started. Publishing to %s", topic.c_str());
  }

private:
  void on_timer()
  {
    rclcpp::Time t = this->now();
    double elapsed = (t - start_time_).seconds();
    double phase = fmod(elapsed, period_) / period_ * 2.0 * M_PI;
    double x = radius_ * cos(phase);
    double y = radius_ * sin(phase);
    double z = height_;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = "world";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    // simple yaw following tangent
    double yaw = phase + M_PI/2.0;
    msg.pose.orientation.w = cos(yaw/2.0);
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = sin(yaw/2.0);

    pub_->publish(msg);
  }

  std::string model_name_;
  std::string world_name_;
  double radius_;
  double height_;
  double period_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
