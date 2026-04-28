#ifndef GZ_PID_HPP_
#define GZ_PID_HPP_

#include <string>

#include "ignition/math/PID.hh"
#include "rclcpp/rclcpp.hpp"

namespace gz_gimbal
{

struct PidParam
{
  double p;
  double i;
  double d;
  double imax;
  double imin;
  double cmdmin;
  double cmdmax;
  double offset;
};

void declare_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name);

void declare_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name, PidParam & pid_param);

void get_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name, PidParam & pid_param);

}  // namespace gz_gimbal

#endif  // GZ_PID_HPP_
