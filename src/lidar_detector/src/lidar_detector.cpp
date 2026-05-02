#include "lidar_detector/lidar_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lidar_detector
{

LidarDetectorNode::LidarDetectorNode(const rclcpp::NodeOptions & options)
: Node("lidar_detector_node", options)
{
  std::cout << "\033[32m" << "Starting LidarDetectorNode" << "\033[0m" << std::endl;
  onConfigure(); // 配置参数
}

LidarDetectorNode::~LidarDetectorNode()
{
}

void LidarDetectorNode::onConfigure()
{
  // DBSCAN parameters
  this->declare_parameter<double>("dbscan_eps", 0.5);
  this->declare_parameter<int>("dbscan_min_pts", 5);
  this->declare_parameter<double>("confidence_point_scale", 25.0);

  double eps = this->get_parameter("dbscan_eps").as_double();
  int min_pts = this->get_parameter("dbscan_min_pts").as_int();
  confidence_point_scale_ = this->get_parameter("confidence_point_scale").as_double();
  
  dbscan_.setEps(eps);
  dbscan_.setMinPts(min_pts);
  
  RCLCPP_INFO(this->get_logger(), "DBSCAN configured: eps=%.2f, min_pts=%d, confidence_point_scale=%.2f",
              eps, min_pts, confidence_point_scale_);
  
  // Topic names
  this->declare_parameter<std::string>("input_pc_topic", "/cloud_registered");
  this->declare_parameter<std::string>("output_detections_topic", "/lidar_detections");

  input_pc_topic_ = this->get_parameter("input_pc_topic").as_string();
  output_detections_topic_ = this->get_parameter("output_detections_topic").as_string();

  RCLCPP_INFO(this->get_logger(), "Topic config: input='%s', detections='%s'",
              input_pc_topic_.c_str(), output_detections_topic_.c_str());

  // Subscribe to point cloud topic from fast_lio
  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_pc_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->onPointCloud(msg); }
  );

  detections_pub_ = this->create_publisher<yolo_msgs::msg::DetectionArray>(
    output_detections_topic_,
    rclcpp::SensorDataQoS());
}

void LidarDetectorNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg || msg->data.empty())
    return;

  // Run DBSCAN clustering
  dbscan_.run(*msg);
  const auto &points = dbscan_.points();
  const auto &labels = dbscan_.labels();
  
  int num_clusters = dbscan_.getNumClusters();

  struct ClusterStats
  {
    geometry_msgs::msg::Pose center;
    geometry_msgs::msg::Vector3 size;
    std::size_t point_count{0};
    double confidence{0.0};
    yolo_msgs::msg::Detection detection;
  };

  std::vector<ClusterStats> clusters;
  clusters.reserve(static_cast<std::size_t>(num_clusters));

  for (int cluster_id = 1; cluster_id <= num_clusters; ++cluster_id)
  {
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    std::size_t point_count = 0;

    for (std::size_t i = 0; i < labels.size(); ++i)
    {
      if (labels[i] != cluster_id)
        continue;

      const Point3 &p = points[i];
      min_x = std::min(min_x, static_cast<double>(p.x));
      min_y = std::min(min_y, static_cast<double>(p.y));
      min_z = std::min(min_z, static_cast<double>(p.z));
      max_x = std::max(max_x, static_cast<double>(p.x));
      max_y = std::max(max_y, static_cast<double>(p.y));
      max_z = std::max(max_z, static_cast<double>(p.z));
      ++point_count;
    }

    if (point_count == 0)
      continue;

    geometry_msgs::msg::Pose center;
    geometry_msgs::msg::Vector3 size;
    center.position.x = (min_x + max_x) * 0.5;
    center.position.y = (min_y + max_y) * 0.5;
    center.position.z = (min_z + max_z) * 0.5;
    center.orientation.w = 1.0;
    size.x = std::max(max_x - min_x, 0.05);
    size.y = std::max(max_y - min_y, 0.05);
    size.z = std::max(max_z - min_z, 0.05);

    ClusterStats stats;
    stats.center = center;
    stats.size = size;
    stats.point_count = point_count;
    stats.confidence = computeConfidence(point_count);
    stats.detection = buildDetection(center, size, point_count, msg->header.frame_id,
                                     static_cast<std::size_t>(cluster_id));
    clusters.push_back(stats);
  }

  // Publish detections with 3D boxes and confidence.
  yolo_msgs::msg::DetectionArray detection_array;
  detection_array.header = msg->header;
  detection_array.detections.reserve(clusters.size());
  for (const auto &cluster : clusters)
  {
    yolo_msgs::msg::Detection detection = cluster.detection;
    detection.score = cluster.confidence;
    detection_array.detections.push_back(detection);
  }
  detections_pub_->publish(detection_array);
  
  RCLCPP_DEBUG(this->get_logger(), "Point cloud received: %u points, %zu clusters detected",
               msg->width * msg->height, clusters.size());
  
  // Output detected object positions, confidence, and 3D bounding boxes.
  for (std::size_t i = 0; i < clusters.size(); ++i)
  {
    const auto &detection = detection_array.detections[i];
    const auto &bbox = detection.bbox3d;
    RCLCPP_INFO(this->get_logger(),
                "Object %zu: score=%.3f center=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f) points=%zu",
                i + 1,
                detection.score,
                bbox.center.position.x,
                bbox.center.position.y,
                bbox.center.position.z,
                bbox.size.x,
                bbox.size.y,
                bbox.size.z,
                clusters[i].point_count);
  }
}

double LidarDetectorNode::computeConfidence(std::size_t point_count) const
{
  const double scale = std::max(confidence_point_scale_, 1.0);
  const double score = 1.0 - std::exp(-static_cast<double>(point_count) / scale);
  return std::clamp(score, 0.0, 1.0);
}

yolo_msgs::msg::Detection LidarDetectorNode::buildDetection(
  const geometry_msgs::msg::Pose &center,
  const geometry_msgs::msg::Vector3 &size,
  std::size_t point_count,
  const std::string &frame_id,
  std::size_t cluster_id) const
{
  yolo_msgs::msg::Detection detection;
  detection.class_id = 0;
  detection.class_name = "radar_object";
  detection.score = computeConfidence(point_count);
  detection.id = std::to_string(cluster_id);
  detection.bbox3d.center = center;
  detection.bbox3d.size = size;
  detection.bbox3d.frame_id = frame_id;
  return detection;
}

} // namespace lidar_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_detector::LidarDetectorNode)