#include "lidar_detector/sensor_fusion_node.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_detector {

SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("sensor_fusion_node", options) {
    // Parameters
    this->declare_parameter("dbscan_eps", 0.1f);
    this->declare_parameter("dbscan_min_pts", 5);
    this->declare_parameter("iou_threshold", 0.3f);
    this->declare_parameter("camera_fx", 1470.0f);
    this->declare_parameter("camera_fy", 1470.0f);
    this->declare_parameter("camera_cx", 480.0f);
    this->declare_parameter("camera_cy", 360.0f);
    this->declare_parameter("camera_frame_id", "camera");
    this->declare_parameter("lidar_frame_id", "camera_init");

    dbscan_eps_ = this->get_parameter("dbscan_eps").as_double();
    dbscan_min_pts_ = this->get_parameter("dbscan_min_pts").as_int();
    iou_threshold_ = this->get_parameter("iou_threshold").as_double();
    camera_fx_ = this->get_parameter("camera_fx").as_double();
    camera_fy_ = this->get_parameter("camera_fy").as_double();
    camera_cx_ = this->get_parameter("camera_cx").as_double();
    camera_cy_ = this->get_parameter("camera_cy").as_double();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node initialized");
    RCLCPP_INFO(this->get_logger(), "DBSCAN eps: %.3f, min_pts: %d", dbscan_eps_, dbscan_min_pts_);
    RCLCPP_INFO(this->get_logger(), "Camera intrinsics - fx: %.1f, fy: %.1f, cx: %.1f, cy: %.1f",
                camera_fx_, camera_fy_, camera_cx_, camera_cy_);

    // Create subscriptions
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "/cloud_registered");
    detection_sub_ = std::make_shared<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>>(
        this, "detections");

    // Create synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        yolo_msgs::msg::DetectionArray>>>(
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            yolo_msgs::msg::DetectionArray>(10),
        *cloud_sub_, *detection_sub_);

    sync_->registerCallback(std::bind(&SensorFusionNode::fusionCallback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    // Publisher for fused detections
    fused_pub_ = this->create_publisher<yolo_msgs::msg::DetectionArray>("fused_detections", 10);
}

void SensorFusionNode::fusionCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg) {
    RCLCPP_DEBUG(this->get_logger(), "Fusion callback triggered with %zu detections",
                 detections_msg->detections.size());

    if (detections_msg->detections.empty()) {
        RCLCPP_WARN(this->get_logger(), "No detections received");
        return;
    }

    // Convert point cloud to vector of points
    auto points = cloudToPoints(cloud_msg);
    if (points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty point cloud received");
        return;
    }

    // Cluster point cloud using DBSCAN
    auto clusters = clusterPointCloud(points);
    RCLCPP_DEBUG(this->get_logger(), "DBSCAN found %zu clusters", clusters.size());

    // Match detections with clusters
    auto fused_detections = matchDetectionsWithClusters(detections_msg, clusters);

    // Publish results
    auto output_msg = std::make_shared<yolo_msgs::msg::DetectionArray>();
    output_msg->header = detections_msg->header;

    for (const auto& fused_det : fused_detections) {
        auto detection = std::make_shared<yolo_msgs::msg::Detection>();
        detection->id = fused_det.id;
        detection->class_name = fused_det.class_name;
        detection->score = fused_det.confidence;

        // Set 3D bounding box
        detection->bbox3d.center.position.x = fused_det.center_3d.x;
        detection->bbox3d.center.position.y = fused_det.center_3d.y;
        detection->bbox3d.center.position.z = fused_det.center_3d.z;
        detection->bbox3d.size.x = fused_det.size_3d.x;
        detection->bbox3d.size.y = fused_det.size_3d.y;
        detection->bbox3d.size.z = fused_det.size_3d.z;
        detection->bbox3d.frame_id = lidar_frame_id_;

        output_msg->detections.push_back(*detection);
    }

    fused_pub_->publish(*output_msg);
    RCLCPP_INFO(this->get_logger(), "Published %zu fused detections", fused_detections.size());
}

std::vector<Point> SensorFusionNode::cloudToPoints(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    std::vector<Point> points;

    // Convert ROS2 PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    for (const auto& pcl_point : pcl_cloud) {
        Point p;
        p.x = pcl_point.x;
        p.y = pcl_point.y;
        p.z = pcl_point.z;
        p.clusterID = UNCLASSIFIED;

        // Simple filtering: ignore points behind or very close to camera
        if (p.z > 0.1f && p.z < 50.0f) {
            points.push_back(p);
        }
    }

    return points;
}

std::vector<std::vector<Point>> SensorFusionNode::clusterPointCloud(
    const std::vector<Point>& points) {
    std::vector<Point> points_copy = points;

    // Run DBSCAN
    DBSCAN dbscan(dbscan_min_pts_, dbscan_eps_, points_copy);
    dbscan.run();

    // Organize points by cluster
    std::map<int, std::vector<Point>> cluster_map;
    for (const auto& point : dbscan.m_points) {
        if (point.clusterID > 0) {  // Ignore noise and unclassified
            cluster_map[point.clusterID].push_back(point);
        }
    }

    std::vector<std::vector<Point>> clusters;
    for (const auto& kv : cluster_map) {
        clusters.push_back(kv.second);
    }

    return clusters;
}

cv::Point2f SensorFusionNode::getCluster2DCenter(
    const std::vector<Point>& cluster,
    float fx, float fy, float cx, float cy) {
    // Project 3D cluster center to 2D image coordinates
    float center_x = 0, center_y = 0, center_z = 0;

    for (const auto& p : cluster) {
        center_x += p.x;
        center_y += p.y;
        center_z += p.z;
    }

    int n = cluster.size();
    center_x /= n;
    center_y /= n;
    center_z /= n;

    // Avoid division by zero
    if (center_z < 0.01f) {
        return cv::Point2f(cx, cy);
    }

    // Project to 2D
    float u = fx * center_x / center_z + cx;
    float v = fy * center_y / center_z + cy;

    return cv::Point2f(u, v);
}

float SensorFusionNode::calculateIoU2D(
    const cv::Rect& bbox1,
    const cv::Rect& bbox2) {
    int intersection_area = (bbox1 & bbox2).area();
    int union_area = bbox1.area() + bbox2.area() - intersection_area;

    if (union_area == 0) return 0.0f;
    return static_cast<float>(intersection_area) / union_area;
}

std::vector<FusedDetection> SensorFusionNode::matchDetectionsWithClusters(
    const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg,
    const std::vector<std::vector<Point>>& clusters) {
    std::vector<FusedDetection> results;

    // Create bounding boxes from detections
    std::vector<cv::Rect> detection_bboxes;
    for (const auto& det : detections_msg->detections) {
        int x = static_cast<int>(det.bbox.center.position.x - det.bbox.size.x / 2);
        int y = static_cast<int>(det.bbox.center.position.y - det.bbox.size.y / 2);
        int w = static_cast<int>(det.bbox.size.x);
        int h = static_cast<int>(det.bbox.size.y);
        cv::Rect bbox(x, y, w, h);
        detection_bboxes.push_back(bbox);
    }

    // For each detection, find best matching cluster
    for (size_t det_idx = 0; det_idx < detections_msg->detections.size(); ++det_idx) {
        const auto& detection = detections_msg->detections[det_idx];
        const auto& det_bbox = detection_bboxes[det_idx];

        float best_iou = 0.0f;
        int best_cluster_idx = -1;
        cv::Point3f best_center_3d(0, 0, 0);
        float best_distance = 0.0f;

        // Find cluster with best IoU overlap
        for (size_t cls_idx = 0; cls_idx < clusters.size(); ++cls_idx) {
            const std::vector<Point>& cluster = clusters[cls_idx];

            // Project cluster to 2D
            auto cluster_center_2d = getCluster2DCenter(
                cluster, camera_fx_, camera_fy_, camera_cx_, camera_cy_);

            // Create cluster bounding box in image space (approximate)
            cv::Rect cluster_bbox(
                static_cast<int>(cluster_center_2d.x - 20),
                static_cast<int>(cluster_center_2d.y - 20),
                40, 40);

            float iou = calculateIoU2D(det_bbox, cluster_bbox);

            if (iou > best_iou) {
                best_iou = iou;
                best_cluster_idx = cls_idx;

                // Calculate 3D center and distance
                float center_x = 0, center_y = 0, center_z = 0;
                for (const auto& p : cluster) {
                    center_x += p.x;
                    center_y += p.y;
                    center_z += p.z;
                }
                center_x /= cluster.size();
                center_y /= cluster.size();
                center_z /= cluster.size();

                best_center_3d = cv::Point3f(center_x, center_y, center_z);
                best_distance = std::sqrt(center_x * center_x +
                                         center_y * center_y +
                                         center_z * center_z);
            }
        }

        // If IoU exceeds threshold, add to results
        if (best_iou >= iou_threshold_ && best_cluster_idx >= 0) {
            const std::vector<Point>& cluster = clusters[best_cluster_idx];

            FusedDetection fused;
            fused.id = detection.id;
            fused.class_name = detection.class_name;
            fused.confidence = detection.score;
            fused.center_3d = best_center_3d;
            fused.distance = best_distance;
            fused.matched_cluster_id = best_cluster_idx;
            fused.iou_2d = best_iou;

            // Estimate 3D size from cluster
            float min_x = cluster[0].x;
            float max_x = cluster[0].x;
            float min_y = cluster[0].y;
            float max_y = cluster[0].y;
            float min_z = cluster[0].z;
            float max_z = cluster[0].z;

            for (const auto& p : cluster) {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
                min_z = std::min(min_z, p.z);
                max_z = std::max(max_z, p.z);
            }

            fused.size_3d = cv::Point3f(
                max_x - min_x,
                max_y - min_y,
                max_z - min_z);

            results.push_back(fused);

            RCLCPP_DEBUG(this->get_logger(),
                        "Detection '%s' matched to cluster %d with IoU %.3f, distance %.2f m",
                        fused.id.c_str(), best_cluster_idx, best_iou, best_distance);
        }
    }

    return results;
}

}  // namespace lidar_detector

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_detector::SensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}
