#ifndef GZ_CAM_NODE_HPP_
#define GZ_CAM_NODE_HPP_

#include <thread>
#include <string>
#include <memory>
#include <vector>

#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/image_transport.hpp"
#include "simulator/srv/get_camera_info.hpp"

namespace gz_cam
{
// Node wrapper for IgnCam.
class GzCamNode
{
public:
  explicit GzCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void gz_image_cb(const ignition::msgs::Image & msg);
  void get_camera_info_cb(
    const simulator::srv::GetCameraInfo::Request::SharedPtr request,
    simulator::srv::GetCameraInfo::Response::SharedPtr response);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  // default image transport
  std::shared_ptr<image_transport::Publisher> img_pub_;
  // image_transporter for camera publisher
  std::shared_ptr<image_transport::CameraPublisher> cam_pub_;
  rclcpp::Service<simulator::srv::GetCameraInfo>::SharedPtr get_camera_info_srv_;
  // params
  std::string camera_name_{"camera"};
  std::string camera_frame_id_{""};
  bool use_qos_profile_sensor_data_{false};
  bool use_image_transport_camera_publisher_{false};
  bool run_flag_{true};
  // data
  sensor_msgs::msg::CameraInfo cam_info_;
  bool cam_info_valid_{false};
};

}  // namespace gz_cam

#endif  // GZ_CAM_NODE_HPP_
