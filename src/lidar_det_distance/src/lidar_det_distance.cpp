#include "lidar_det_distance/lidar_det_distance.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace lidar_det_distance
{

namespace
{
constexpr double kDefaultFocalFallback = 1.0;
constexpr double kDefaultPrincipalFallback = 0.0;
}  // namespace

LidarDetDistanceNode::LidarDetDistanceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lidar_det_distance_node", options)
{
	detections_topic_ = this->declare_parameter<std::string>("detections_topic", "/yolo/detections");
	cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/cloud_registered");
	camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
	output_cloud_topic_ = this->declare_parameter<std::string>("output_cloud_topic", "/lidar_det_distance/cropped_cloud");
	center_marker_topic_ = this->declare_parameter<std::string>("center_marker_topic", "/lidar_det_distance/center_markers");
	camera_frame_ = this->declare_parameter<std::string>("camera_frame", "");
	target_frame_ = this->declare_parameter<std::string>("target_frame", "");

	bbox_margin_pixels_ = this->declare_parameter<double>("bbox_margin_pixels", 6.0);
	min_depth_ = this->declare_parameter<double>("min_depth", 0.1);
	max_depth_ = this->declare_parameter<double>("max_depth", 80.0);
	min_box_dimension_ = this->declare_parameter<double>("min_box_dimension", 0.05);
	center_marker_scale_ = this->declare_parameter<double>("center_marker_scale", 0.2);
	direction_marker_scale_ = this->declare_parameter<double>("direction_marker_scale", 0.03);

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

	detections_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
		detections_topic_, rclcpp::QoS(10),
		std::bind(&LidarDetDistanceNode::onDetections, this, std::placeholders::_1));

	cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		cloud_topic_, rclcpp::SensorDataQoS(),
		std::bind(&LidarDetDistanceNode::onPointCloud, this, std::placeholders::_1));

	camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic_, rclcpp::QoS(10),
		std::bind(&LidarDetDistanceNode::onCameraInfo, this, std::placeholders::_1));

	cropped_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		output_cloud_topic_, rclcpp::SensorDataQoS());
	center_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		center_marker_topic_, rclcpp::QoS(10));

	RCLCPP_INFO(
		this->get_logger(),
		"lidar_det_distance started: det=%s cloud=%s camera_info=%s cropped_cloud=%s markers=%s",
		detections_topic_.c_str(),
		cloud_topic_.c_str(),
		camera_info_topic_.c_str(),
		output_cloud_topic_.c_str(),
		center_marker_topic_.c_str());
}

void LidarDetDistanceNode::onDetections(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
	{
		std::scoped_lock<std::mutex> lock(data_mutex_);
		latest_detections_ = *msg;
		has_detections_ = true;
	}

	processDetections();
}

void LidarDetDistanceNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	std::scoped_lock<std::mutex> lock(data_mutex_);
	latest_cloud_ = *msg;
	has_cloud_ = true;
}

void LidarDetDistanceNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	std::scoped_lock<std::mutex> lock(data_mutex_);
	latest_camera_info_ = *msg;
	has_camera_info_ = true;
}

void LidarDetDistanceNode::processDetections()
{
	yolo_msgs::msg::DetectionArray detections;
	sensor_msgs::msg::PointCloud2 cloud;
	sensor_msgs::msg::CameraInfo camera_info;

	{
		std::scoped_lock<std::mutex> lock(data_mutex_);
		if (!has_detections_ || !has_cloud_ || !has_camera_info_) {
			return;
		}

		detections = latest_detections_;
		cloud = latest_cloud_;
		camera_info = latest_camera_info_;
	}

	if (detections.detections.empty() || cloud.data.empty()) {
		return;
	}

	const std::string camera_frame = !camera_frame_.empty() ? camera_frame_ : camera_info.header.frame_id;
	const std::string cloud_frame = cloud.header.frame_id;
	const std::string output_frame = !target_frame_.empty() ? target_frame_ : cloud_frame;

	if (camera_frame.empty()) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "camera frame is empty, skip processing");
		return;
	}

	if (cloud_frame.empty()) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cloud frame is empty, skip processing");
		return;
	}

	if (camera_info.k.size() < 9) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "camera_info K is invalid, skip processing");
		return;
	}

	const auto lidar_to_camera_msg = tf_buffer_->lookupTransform(camera_frame, cloud_frame, tf2::TimePointZero);
	tf2::Transform lidar_to_camera;
	tf2::fromMsg(lidar_to_camera_msg.transform, lidar_to_camera);

	tf2::Transform lidar_to_output;
	const bool need_output_transform = output_frame != cloud_frame;
	if (need_output_transform) {
		const auto lidar_to_output_msg = tf_buffer_->lookupTransform(output_frame, cloud_frame, tf2::TimePointZero);
		tf2::fromMsg(lidar_to_output_msg.transform, lidar_to_output);
	}

	geometry_msgs::msg::Point camera_origin_in_output;
	try {
		const auto output_to_camera_msg = tf_buffer_->lookupTransform(output_frame, camera_frame, tf2::TimePointZero);
		camera_origin_in_output.x = output_to_camera_msg.transform.translation.x;
		camera_origin_in_output.y = output_to_camera_msg.transform.translation.y;
		camera_origin_in_output.z = output_to_camera_msg.transform.translation.z;
	} catch (const std::exception & ex) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(), *this->get_clock(), 2000,
			"failed to lookup camera origin in %s: %s",
			output_frame.c_str(), ex.what());
		camera_origin_in_output.x = 0.0;
		camera_origin_in_output.y = 0.0;
		camera_origin_in_output.z = 0.0;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(cloud, *cloud_points);

	std::vector<ProjectedPoint> projected_points;
	projected_points.reserve(cloud_points->size());

	for (const auto & point : cloud_points->points) {
		if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
			continue;
		}

		ProjectedPoint projected_point;
		if (!projectPoint(point, lidar_to_camera, camera_info, projected_point)) {
			continue;
		}

		projected_points.push_back(projected_point);
	}

  std::cout << "Projected points: " << projected_points.size() << std::endl;
  if (projected_points.empty()) {
    RCLCPP_INFO(this->get_logger(), "No valid projected points, skip processing");
  }

	pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	visualization_msgs::msg::MarkerArray marker_array;
	visualization_msgs::msg::Marker delete_all;
	delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
	marker_array.markers.push_back(delete_all);

	int marker_id = 0;

	for (const auto & detection : detections.detections) {
		const double center_u = detection.bbox.center.position.x;
		const double center_v = detection.bbox.center.position.y;
		const double half_w = std::max(0.0, detection.bbox.size.x * 0.5);
		const double half_h = std::max(0.0, detection.bbox.size.y * 0.5);

		const double min_u = center_u - half_w - bbox_margin_pixels_;
		const double max_u = center_u + half_w + bbox_margin_pixels_;
		const double min_v = center_v - half_h - bbox_margin_pixels_;
		const double max_v = center_v + half_h + bbox_margin_pixels_;

		pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points(new pcl::PointCloud<pcl::PointXYZ>());
		candidate_points->reserve(projected_points.size());

    // std::cout << "Candidate points: " << candidate_points->size() << std::endl;

		for (const auto & projected_point : projected_points) {
			// if (projected_point.u < min_u || projected_point.u > max_u) {
			// 	continue;
			// }
			// if (projected_point.v < min_v || projected_point.v > max_v) {
			// 	continue;
			// }
			candidate_points->push_back(projected_point.lidar_point);
		}

		if (candidate_points->empty()) {
			RCLCPP_DEBUG(
				this->get_logger(),
				"No candidate points for detection %s class %s",
				detection.id.c_str(),
				detection.class_name.c_str());
			continue;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cluster(new pcl::PointCloud<pcl::PointXYZ>());
		output_cluster->reserve(candidate_points->size());

		for (const auto & point : candidate_points->points) {
			if (need_output_transform) {
				output_cluster->push_back(transformPoint(point, lidar_to_output));
			} else {
				output_cluster->push_back(point);
			}
		}

		if (output_cluster->empty()) {
			continue;
		}

		for (const auto & point : output_cluster->points) {
			cropped_cloud->push_back(point);
		}

		const auto center_pose = computeCenter(*output_cluster);

		visualization_msgs::msg::Marker center_marker;
		center_marker.header.frame_id = output_frame;
		center_marker.header.stamp = cloud.header.stamp;
		center_marker.ns = "lidar_det_distance_center";
		center_marker.id = marker_id++;
		center_marker.type = visualization_msgs::msg::Marker::SPHERE;
		center_marker.action = visualization_msgs::msg::Marker::ADD;
		center_marker.pose = center_pose;
		center_marker.scale.x = center_marker_scale_;
		center_marker.scale.y = center_marker_scale_;
		center_marker.scale.z = center_marker_scale_;
		center_marker.color.r = 0.1F;
		center_marker.color.g = 1.0F;
		center_marker.color.b = 0.1F;
		center_marker.color.a = 1.0F;
		center_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
		marker_array.markers.push_back(center_marker);
		marker_array.markers.push_back(
			makeDirectionMarker(center_pose, camera_origin_in_output, output_frame, 1000 + marker_id));

		const auto & center = center_pose.position;
		RCLCPP_INFO(
			this->get_logger(),
			"det=%s class=%s cropped_points=%zu center=(%.3f, %.3f, %.3f)",
			detection.id.c_str(),
			detection.class_name.c_str(),
			output_cluster->size(),
			center.x,
			center.y,
			center.z);
	}

	if (!cropped_cloud->empty()) {
		sensor_msgs::msg::PointCloud2 cropped_msg;
		pcl::toROSMsg(*cropped_cloud, cropped_msg);
		cropped_msg.header.frame_id = output_frame;
		cropped_msg.header.stamp = cloud.header.stamp;
		cropped_cloud_pub_->publish(cropped_msg);
	}

	if (!marker_array.markers.empty()) {
		center_marker_pub_->publish(marker_array);
	}
}

bool LidarDetDistanceNode::projectPoint(
	const pcl::PointXYZ & lidar_point,
	const tf2::Transform & lidar_to_camera,
	const sensor_msgs::msg::CameraInfo & camera_info,
	ProjectedPoint & projected_point) const
{
	const tf2::Vector3 lidar_vec(lidar_point.x, lidar_point.y, lidar_point.z);
	const tf2::Vector3 camera_vec = lidar_to_camera * lidar_vec;

	const double z = camera_vec.z();
	if (!std::isfinite(z) || z <= min_depth_ || z >= max_depth_) {
		return false;
	}

	const double fx = camera_info.k[0] != 0.0 ? camera_info.k[0] : kDefaultFocalFallback;
	const double fy = camera_info.k[4] != 0.0 ? camera_info.k[4] : kDefaultFocalFallback;
	const double cx = camera_info.k[2] != 0.0 ? camera_info.k[2] : kDefaultPrincipalFallback;
	const double cy = camera_info.k[5] != 0.0 ? camera_info.k[5] : kDefaultPrincipalFallback;

	const double u = fx * (camera_vec.x() / z) + cx;
	const double v = fy * (camera_vec.y() / z) + cy;
	if (!std::isfinite(u) || !std::isfinite(v)) {
		return false;
	}

	projected_point.lidar_point = lidar_point;
	projected_point.u = static_cast<float>(u);
	projected_point.v = static_cast<float>(v);
	return true;
}

pcl::PointXYZ LidarDetDistanceNode::transformPoint(
	const pcl::PointXYZ & point,
	const tf2::Transform & transform) const
{
	const tf2::Vector3 transformed = transform * tf2::Vector3(point.x, point.y, point.z);
	pcl::PointXYZ output_point;
	output_point.x = static_cast<float>(transformed.x());
	output_point.y = static_cast<float>(transformed.y());
	output_point.z = static_cast<float>(transformed.z());
	return output_point;
}

geometry_msgs::msg::Pose LidarDetDistanceNode::computeCenter(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
	geometry_msgs::msg::Pose center;
	if (cloud.empty()) {
		center.orientation.w = 1.0;
		return center;
	}

	pcl::PointXYZ min_point;
	pcl::PointXYZ max_point;
	pcl::getMinMax3D(cloud, min_point, max_point);

	center.position.x = 0.5 * (static_cast<double>(min_point.x) + static_cast<double>(max_point.x));
	center.position.y = 0.5 * (static_cast<double>(min_point.y) + static_cast<double>(max_point.y));
	center.position.z = 0.5 * (static_cast<double>(min_point.z) + static_cast<double>(max_point.z));
	center.orientation.w = 1.0;
	return center;
}

visualization_msgs::msg::Marker LidarDetDistanceNode::makeDirectionMarker(
	const geometry_msgs::msg::Pose & center,
	const geometry_msgs::msg::Point & origin,
	const std::string & frame_id,
	int marker_id) const
{
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = this->now();
	marker.ns = "lidar_det_distance_direction";
	marker.id = marker_id;
	marker.type = visualization_msgs::msg::Marker::ARROW;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.scale.x = direction_marker_scale_;
	marker.scale.y = direction_marker_scale_ * 2.0;
	marker.scale.z = direction_marker_scale_ * 2.0;
	marker.color.r = 1.0F;
	marker.color.g = 0.4F;
	marker.color.b = 0.1F;
	marker.color.a = 1.0F;
	marker.lifetime = rclcpp::Duration::from_seconds(0.5);
	marker.points.push_back(origin);
	marker.points.push_back(center.position);
	return marker;
}

}  // namespace lidar_det_distance

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_det_distance::LidarDetDistanceNode)