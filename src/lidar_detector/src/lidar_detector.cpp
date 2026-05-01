#include "lidar_detector/lidar_detector.hpp"

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

  double eps = this->get_parameter("dbscan_eps").as_double();
  int min_pts = this->get_parameter("dbscan_min_pts").as_int();
  
  dbscan_.setEps(eps);
  dbscan_.setMinPts(min_pts);
  
  RCLCPP_INFO(this->get_logger(), "DBSCAN configured: eps=%.2f, min_pts=%d", eps, min_pts);
  
  // Subscribe to point cloud topic from fast_lio
  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered",
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->onPointCloud(msg); }
  );
}

void LidarDetectorNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg || msg->data.empty())
    return;

  // Run DBSCAN clustering
  std::vector<int> labels = dbscan_.run(*msg);
  
  // Get cluster centroids
  std::vector<Point3> centroids = dbscan_.clusterCentroids();
  int num_clusters = dbscan_.getNumClusters();
  
  RCLCPP_DEBUG(this->get_logger(), "Point cloud received: %u points, %d clusters detected",
               msg->width * msg->height, num_clusters);
  
  // Output detected object positions (3D coordinates of cluster centroids)
  for (int i = 0; i < num_clusters; ++i)
  {
    const Point3 &centroid = centroids[i];
    RCLCPP_INFO(this->get_logger(), 
                "Object %d: x=%.3f, y=%.3f, z=%.3f",
                i + 1, centroid.x, centroid.y, centroid.z);
  }
}

} // namespace lidar_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_detector::LidarDetectorNode)