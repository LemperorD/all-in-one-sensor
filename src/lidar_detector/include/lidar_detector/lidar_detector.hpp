#ifndef LIDAR_DETECTOR_HPP
#define LIDAR_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  // Subscribers, Publishers and topic names
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::string input_pc_topic_;
  std::string output_detections_topic_;
  std::string output_markers_topic_;
  
  // DBSCAN clustering
  DBSCAN dbscan_;
  double eps_;
  int min_pts_;
  double confidence_point_scale_;
  bool publish_markers_{true};
  
  // Point cloud callback
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  double computeConfidence(std::size_t point_count) const;
  yolo_msgs::msg::Detection buildDetection(const geometry_msgs::msg::Pose &center,
                                           const geometry_msgs::msg::Vector3 &size,
                                           std::size_t point_count,
                                           const std::string &frame_id,
                                           std::size_t cluster_id) const;
  visualization_msgs::msg::Marker makeBoxMarker(const geometry_msgs::msg::Pose &center,
                                                const geometry_msgs::msg::Vector3 &size,
                                                const std::string &frame_id,
                                                const std::string &ns,
                                                int marker_id,
                                                double confidence) const;

};

} // namespace lidar_detector

#endif // LIDAR_DETECTOR_HPP