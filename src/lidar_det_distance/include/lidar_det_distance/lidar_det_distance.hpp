#ifndef LIDAR_DET_DISTANCE__LIDAR_DET_DISTANCE_HPP_
#define LIDAR_DET_DISTANCE__LIDAR_DET_DISTANCE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

namespace lidar_det_distance
{

class LidarDetDistanceNode : public rclcpp::Node
{
public:
	explicit LidarDetDistanceNode(const rclcpp::NodeOptions & options);
	~LidarDetDistanceNode() override = default;

private:
	struct ProjectedPoint
	{
		pcl::PointXYZ lidar_point;
		float u{0.0F};
		float v{0.0F};
	};

	void onDetections(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
	void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void processDetections();

	bool projectPoint(
		const pcl::PointXYZ & lidar_point,
		const tf2::Transform & lidar_to_camera,
		const sensor_msgs::msg::CameraInfo & camera_info,
		ProjectedPoint & projected_point) const;

	pcl::PointXYZ transformPoint(
		const pcl::PointXYZ & point,
		const tf2::Transform & transform) const;

	geometry_msgs::msg::Pose computeCenter(const pcl::PointCloud<pcl::PointXYZ> & cloud) const;
	visualization_msgs::msg::Marker makeDirectionMarker(
		const geometry_msgs::msg::Pose & center,
		const geometry_msgs::msg::Point & origin,
		const std::string & frame_id,
		int marker_id) const;

	rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr center_marker_pub_;

	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

	std::mutex data_mutex_;
	yolo_msgs::msg::DetectionArray latest_detections_;
	sensor_msgs::msg::PointCloud2 latest_cloud_;
	sensor_msgs::msg::CameraInfo latest_camera_info_;
	bool has_detections_{false};
	bool has_cloud_{false};
	bool has_camera_info_{false};

	std::string detections_topic_;
	std::string cloud_topic_;
	std::string camera_info_topic_;
	std::string output_topic_;
	std::string camera_frame_;
	std::string target_frame_;
	std::string output_cloud_topic_;
	std::string center_marker_topic_;

	double bbox_margin_pixels_{6.0};
	double min_depth_{0.1};
	double max_depth_{80.0};
	double min_box_dimension_{0.05};
	double center_marker_scale_{0.2};
	double direction_marker_scale_{0.03};
};

}  // namespace lidar_det_distance

#endif  // LIDAR_DET_DISTANCE__LIDAR_DET_DISTANCE_HPP_
