#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
    this->declare_parameter<double>("center_x", 0.0);
    this->declare_parameter<double>("center_y", 0.0);
    this->declare_parameter<double>("period", 20.0);

    model_name_ = this->get_parameter("model_name").as_string();
    world_name_ = this->get_parameter("world_name").as_string();
    double rate = this->get_parameter("rate").as_double();
    radius_ = this->get_parameter("radius").as_double();
    height_ = this->get_parameter("height").as_double();
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    period_ = this->get_parameter("period").as_double();

    // ROS topic that will be bridged to Ignition - use cmd_vel instead of pose_cmd
    std::string topic = "/" + model_name_ + "/cmd_vel"; // bridged via ros_gz_bridge
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

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
    double dphase_dt = 2.0 * M_PI / period_;  // angular velocity of orbit

    // Current position on circular trajectory
    double x = center_x_ + radius_ * cos(phase);
    double y = center_y_ + radius_ * sin(phase);
    
    // Velocity: derivative of position with respect to time
    // dx/dt = -radius * sin(phase) * dphase/dt
    // dy/dt = radius * cos(phase) * dphase/dt
    double vx = -radius_ * sin(phase) * dphase_dt;
    double vy = radius_ * cos(phase) * dphase_dt;
    double vz = 0.0;  // maintain constant height

    // Yaw pointing tangent to circle (perpendicular to radius)
    double yaw_rate = dphase_dt;  // angular velocity around z-axis

    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.linear.z = vz;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = yaw_rate;

    pub_->publish(msg);
  }

  std::string model_name_;
  std::string world_name_;
  double radius_;
  double height_;
  double period_;
  double center_x_;
  double center_y_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
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
