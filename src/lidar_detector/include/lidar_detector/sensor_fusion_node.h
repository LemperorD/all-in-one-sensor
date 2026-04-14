#ifndef SENSOR_FUSION_NODE_H_
#define SENSOR_FUSION_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "lidar_detector/dbscan.h"

using lidarDetector::Point;
using lidarDetector::DBSCAN;

namespace lidar_detector {

// Custom message for fused detection result
struct FusedDetection {
    std::string id;                           // tracking ID from YOLO
    std::string class_name;                  // class name from YOLO
    float confidence;                        // detection confidence
    cv::Point3f center_3d;                   // 3D center point
    cv::Point3f size_3d;                     // 3D bounding box size
    float distance;                          // distance from camera
    int matched_cluster_id;                  // matched cluster ID from DBSCAN
    float iou_2d;                           // 2D IoU with vision detection
};

class SensorFusionNode : public rclcpp::Node {
public:
    explicit SensorFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SensorFusionNode() override = default;

private:
    // Callback for synchronized point cloud and detections
    void fusionCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg);

    // Helper functions
    std::vector<Point> cloudToPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);
    std::vector<std::vector<Point>> clusterPointCloud(
        const std::vector<Point>& points);

    cv::Point2f getCluster2DCenter(
        const std::vector<Point>& cluster,
        float fx, float fy, float cx, float cy);

    float calculateIoU2D(
        const cv::Rect& bbox1,
        const cv::Rect& bbox2);

    std::vector<FusedDetection> matchDetectionsWithClusters(
        const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg,
        const std::vector<std::vector<Point>>& clusters);

    // ROS 2 components
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>> detection_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        yolo_msgs::msg::DetectionArray>>> sync_;

    rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr fused_pub_;

    // Parameters
    float dbscan_eps_;
    int dbscan_min_pts_;
    float iou_threshold_;
    float camera_fx_;
    float camera_fy_;
    float camera_cx_;
    float camera_cy_;
    std::string camera_frame_id_;
    std::string lidar_frame_id_;
};

}  // namespace lidar_detector

#endif  // SENSOR_FUSION_NODE_H_
