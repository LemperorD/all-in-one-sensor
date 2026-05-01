#ifndef LIDAR_DETECTOR_HPP
#define LIDAR_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "dbscan.hpp"

namespace lidar_detector
{

class LidarDetectorNode : public rclcpp::Node
{
public: // 构造函数与析构函数
  explicit LidarDetectorNode(const rclcpp::NodeOptions & options);
  ~LidarDetectorNode() override;

public: // 方法
  void onConfigure();

private: // 成员变量
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  
  // Publishers and topic names
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr objects_pub_;
  std::string input_pc_topic_;
  std::string output_objects_topic_;
  
  // DBSCAN clustering
  DBSCAN dbscan_;
  
  // Point cloud callback
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};

} // namespace lidar_detector

#endif // LIDAR_DETECTOR_HPP