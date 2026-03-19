#include <iostream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <acado_toolkit.hpp>

namespace mpc_gimbal_planner
{

class MPCGimbalPlanner : public rclcpp::Node
{

public:
  explicit MPCGimbalPlanner(const rclcpp::NodeOptions & options);
  ~MPCGimbalPlanner() override;

private: // sub & pub
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

};

} // namespace mpc_gimbal_planner