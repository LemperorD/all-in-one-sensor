#include "sensor_fusion/fusion_node.hpp"
#include "sensor_fusion/fusion_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace
{

std::uint64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
    return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

template<typename QuaternionT>
float quaternionToYaw(const QuaternionT & quaternion)
{
    const float siny_cosp = 2.0f * static_cast<float>(quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const float cosy_cosp = 1.0f - 2.0f * static_cast<float>(quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

using namespace std::chrono_literals;

namespace sensor_fusion
{

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("sensor_fusion_node_node", options)
{
    std::cout << "\033[32m" << "Starting FusionNode" << "\033[0m" << std::endl;
    onConfigure(); // 配置参数

    // Configure ByteTrack
    bytetrack_.high_conf_threshold = bytetrack_high_conf_;
    bytetrack_.low_conf_threshold = bytetrack_low_conf_;
    bytetrack_.iou_threshold = bytetrack_iou_thresh_;

    // Configure AB3DMOT
    ab3dmot_.high_conf_threshold = ab3dmot_high_conf_;
    ab3dmot_.low_conf_threshold = ab3dmot_low_conf_;
    ab3dmot_.mahal_threshold = ab3dmot_mahal_thresh_;

    tracks_2d_pub_ = this->create_publisher<yolo_msgs::msg::DetectionArray>(
        topic_tracks_2d_, rclcpp::SensorDataQoS());
    tracks_3d_pub_ = this->create_publisher<yolo_msgs::msg::DetectionArray>(
        topic_tracks_3d_, rclcpp::SensorDataQoS());

    detections_2d_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
        topic_detections_2d_, rclcpp::SensorDataQoS(),
        std::bind(&FusionNode::detections2dCallback, this, std::placeholders::_1));

    detections_3d_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
        topic_detections_3d_, rclcpp::SensorDataQoS(),
        std::bind(&FusionNode::detections3dCallback, this, std::placeholders::_1));
}

FusionNode::~FusionNode()
{
}

void FusionNode::onConfigure()
{
    // Declare and read topic names
    this->declare_parameter<std::string>("topics.detections_2d", "/detections_2d");
    this->declare_parameter<std::string>("topics.detections_3d", "/detections_3d");
    this->declare_parameter<std::string>("topics.tracks_2d", "/tracks_2d");
    this->declare_parameter<std::string>("topics.tracks_3d", "/tracks_3d");
    topic_detections_2d_ = this->get_parameter("topics.detections_2d").as_string();
    topic_detections_3d_ = this->get_parameter("topics.detections_3d").as_string();
    topic_tracks_2d_ = this->get_parameter("topics.tracks_2d").as_string();
    topic_tracks_3d_ = this->get_parameter("topics.tracks_3d").as_string();

    // Declare and read camera intrinsics parameters
    this->declare_parameter<std::vector<double>>("camera.K", std::vector<double>{640, 0, 320, 0, 640, 240, 0, 0, 1});
    auto K_vec = this->get_parameter("camera.K").as_double_array();
    
    if (K_vec.size() != 9) {
        RCLCPP_WARN(this->get_logger(), "Invalid camera.K size, using defaults");
        camera_K_ = Eigen::Matrix3f::Identity();
        camera_K_(0, 0) = 640.0f;  // fx
        camera_K_(1, 1) = 640.0f;  // fy
        camera_K_(0, 2) = 320.0f;  // cx
        camera_K_(1, 2) = 240.0f;  // cy
    } else {
        camera_K_ << K_vec[0], K_vec[1], K_vec[2],
                     K_vec[3], K_vec[4], K_vec[5],
                     K_vec[6], K_vec[7], K_vec[8];
    }
    RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded");

    // Declare and read fusion parameters
    this->declare_parameter<float>("fusion.iou_threshold", 0.3f);
    fusion_iou_threshold_ = this->get_parameter("fusion.iou_threshold").as_double();

    // ByteTrack parameters
    this->declare_parameter<float>("bytetrack.high_conf_threshold", 0.5f);
    this->declare_parameter<float>("bytetrack.low_conf_threshold", 0.1f);
    this->declare_parameter<float>("bytetrack.iou_threshold", 0.3f);
    bytetrack_high_conf_ = this->get_parameter("bytetrack.high_conf_threshold").as_double();
    bytetrack_low_conf_ = this->get_parameter("bytetrack.low_conf_threshold").as_double();
    bytetrack_iou_thresh_ = this->get_parameter("bytetrack.iou_threshold").as_double();

    // AB3DMOT parameters
    this->declare_parameter<float>("ab3dmot.high_conf_threshold", 0.5f);
    this->declare_parameter<float>("ab3dmot.low_conf_threshold", 0.1f);
    this->declare_parameter<float>("ab3dmot.mahal_threshold", 9.4877f);
    ab3dmot_high_conf_ = this->get_parameter("ab3dmot.high_conf_threshold").as_double();
    ab3dmot_low_conf_ = this->get_parameter("ab3dmot.low_conf_threshold").as_double();
    ab3dmot_mahal_thresh_ = this->get_parameter("ab3dmot.mahal_threshold").as_double();
}

void FusionNode::detections2dCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_detections_2d_ = *msg;
        has_latest_detections_2d_ = true;
    }

    tryProcessLatestFrame();
}

void FusionNode::detections3dCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_detections_3d_ = *msg;
        has_latest_detections_3d_ = true;
    }

    tryProcessLatestFrame();
}

void FusionNode::tryProcessLatestFrame()
{
    yolo_msgs::msg::DetectionArray detections_2d_msg;
    yolo_msgs::msg::DetectionArray detections_3d_msg;

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!has_latest_detections_2d_ || !has_latest_detections_3d_) {
            return;
        }

        const auto latest_2d_stamp_ns = stampToNanoseconds(latest_detections_2d_.header.stamp);
        const auto latest_3d_stamp_ns = stampToNanoseconds(latest_detections_3d_.header.stamp);
        if (latest_2d_stamp_ns == last_processed_2d_stamp_ns_ &&
            latest_3d_stamp_ns == last_processed_3d_stamp_ns_) {
            return;
        }

        detections_2d_msg = latest_detections_2d_;
        detections_3d_msg = latest_detections_3d_;
        last_processed_2d_stamp_ns_ = latest_2d_stamp_ns;
        last_processed_3d_stamp_ns_ = latest_3d_stamp_ns;
    }

    const auto dets2d = convertDetections2d(detections_2d_msg);
    const auto dets3d = convertDetections3d(detections_3d_msg);

    processFrame(++frame_counter_,
                 dets2d,
                 dets3d,
                 detections_2d_msg.header,
                 detections_3d_msg.header,
                 camera_K_);
}

std::vector<Detection> FusionNode::convertDetections2d(const yolo_msgs::msg::DetectionArray & msg) const
{
    std::vector<Detection> detections;
    detections.reserve(msg.detections.size());

    for (const auto & detection_msg : msg.detections) {
        const float center_x = static_cast<float>(detection_msg.bbox.center.position.x);
        const float center_y = static_cast<float>(detection_msg.bbox.center.position.y);
        const float width = static_cast<float>(detection_msg.bbox.size.x);
        const float height = static_cast<float>(detection_msg.bbox.size.y);

        detections.emplace_back(
            cv::Rect2f(center_x - width * 0.5f, center_y - height * 0.5f, width, height),
            static_cast<float>(detection_msg.score),
            detection_msg.class_id);
    }

    return detections;
}

std::vector<Detection3D> FusionNode::convertDetections3d(const yolo_msgs::msg::DetectionArray & msg) const
{
    std::vector<Detection3D> detections;
    detections.reserve(msg.detections.size());

    for (const auto & detection_msg : msg.detections) {
        const auto & bbox3d = detection_msg.bbox3d;
        Detection3D detection;
        detection.x = static_cast<float>(bbox3d.center.position.x);
        detection.y = static_cast<float>(bbox3d.center.position.y);
        detection.z = static_cast<float>(bbox3d.center.position.z);
        detection.length = static_cast<float>(bbox3d.size.x);
        detection.width = static_cast<float>(bbox3d.size.y);
        detection.height = static_cast<float>(bbox3d.size.z);
        detection.rotation_y = quaternionToYaw(bbox3d.center.orientation);
        detection.score = static_cast<float>(detection_msg.score);
        detection.class_id = detection_msg.class_id;
        detection.K = camera_K_;
        detections.push_back(detection);
    }

    return detections;
}

yolo_msgs::msg::DetectionArray FusionNode::buildTracks2dMessage(
    const std::vector<STrack> & tracks,
    const std_msgs::msg::Header & header) const
{
    yolo_msgs::msg::DetectionArray msg;
    msg.header = header;

    for (const auto & track : tracks) {
        if (!track.isActive()) {
            continue;
        }

        yolo_msgs::msg::Detection detection;
        detection.id = std::to_string(track.track_id);
        detection.class_id = track.detections.empty() ? -1 : track.detections.back().class_id;
        detection.score = track.detections.empty() ? 0.0f : track.detections.back().conf;
        detection.bbox.center.position.x = track.bbox.x + track.bbox.width * 0.5f;
        detection.bbox.center.position.y = track.bbox.y + track.bbox.height * 0.5f;
        detection.bbox.center.theta = 0.0;
        detection.bbox.size.x = track.bbox.width;
        detection.bbox.size.y = track.bbox.height;
        msg.detections.push_back(detection);
    }

    return msg;
}

yolo_msgs::msg::DetectionArray FusionNode::buildTracks3dMessage(
    const std::vector<STrack3D> & tracks,
    const std_msgs::msg::Header & header) const
{
    yolo_msgs::msg::DetectionArray msg;
    msg.header = header;

    for (const auto & track : tracks) {
        if (!track.isActive()) {
            continue;
        }

        yolo_msgs::msg::Detection detection;
        detection.id = std::to_string(track.track_id);
        detection.class_id = track.detections.empty() ? -1 : track.detections.back().class_id;
        detection.score = track.detections.empty() ? 0.0f : track.detections.back().score;

        Detection3D projection_source;
        projection_source.x = track.mean[0];
        projection_source.y = track.mean[1];
        projection_source.z = track.mean[2];
        projection_source.length = track.mean[3];
        projection_source.width = track.mean[4];
        projection_source.height = track.mean[5];
        projection_source.rotation_y = track.mean[6];
        projection_source.K = camera_K_;

        const auto projected_bbox = sensor_fusion::project3dTo2dBox(projection_source, camera_K_);
        detection.bbox.center.position.x = projected_bbox.x + projected_bbox.width * 0.5f;
        detection.bbox.center.position.y = projected_bbox.y + projected_bbox.height * 0.5f;
        detection.bbox.center.theta = 0.0;
        detection.bbox.size.x = projected_bbox.width;
        detection.bbox.size.y = projected_bbox.height;

        detection.bbox3d.frame_id = header.frame_id;
        detection.bbox3d.center.position.x = track.mean[0];
        detection.bbox3d.center.position.y = track.mean[1];
        detection.bbox3d.center.position.z = track.mean[2];
        detection.bbox3d.center.orientation.x = 0.0;
        detection.bbox3d.center.orientation.y = 0.0;
        detection.bbox3d.center.orientation.z = std::sin(track.mean[6] * 0.5f);
        detection.bbox3d.center.orientation.w = std::cos(track.mean[6] * 0.5f);
        detection.bbox3d.size.x = track.mean[3];
        detection.bbox3d.size.y = track.mean[4];
        detection.bbox3d.size.z = track.mean[5];

        msg.detections.push_back(detection);
    }

    return msg;
}

void FusionNode::processFrame(int frame_id,
                              const std::vector<Detection> &dets2d,
                              const std::vector<Detection3D> &dets3d,
                              const std_msgs::msg::Header &header_2d,
                              const std_msgs::msg::Header &header_3d,
                              const Eigen::Matrix3f &K) {
    // 1) Fuse detections (associate 2D<->3D by IoU and merge confidences)
    auto fused3d = sensor_fusion::fuseDetectionsByIoU(dets2d, dets3d, K, fusion_iou_threshold_);

    // 2) Update 3D tracker
    last3d_tracks_ = ab3dmot_.update(fused3d, frame_id);

    // 3) For 2D tracking, use input 2D detections directly
    last2d_tracks_ = bytetrack_.update(dets2d, frame_id);

    if (tracks_2d_pub_) {
        tracks_2d_pub_->publish(buildTracks2dMessage(last2d_tracks_, header_2d));
    }

    if (tracks_3d_pub_) {
        tracks_3d_pub_->publish(buildTracks3dMessage(last3d_tracks_, header_3d));
    }

    // 4) Log processing results
    RCLCPP_DEBUG(this->get_logger(), 
                "Frame %d: input(2d=%zu, 3d=%zu) -> fused3d=%zu, 2d_tracks=%zu (pub:%s), 3d_tracks=%zu (pub:%s)",
                frame_id, dets2d.size(), dets3d.size(), fused3d.size(),
                last2d_tracks_.size(), topic_tracks_2d_.c_str(),
                last3d_tracks_.size(), topic_tracks_3d_.c_str());
}

} // namespace sensor_fusion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_fusion::FusionNode)