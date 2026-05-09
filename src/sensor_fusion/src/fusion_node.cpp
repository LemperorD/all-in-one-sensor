#include "sensor_fusion/fusion_node.hpp"
#include "sensor_fusion/fusion_utils.hpp"
#include <chrono>

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

void FusionNode::processFrame(int frame_id,
                              const std::vector<Detection> &dets2d,
                              const std::vector<Detection3D> &dets3d,
                              const Eigen::Matrix3f &K) {
    // 1) Fuse detections (associate 2D<->3D by IoU and merge confidences)
    auto fused3d = sensor_fusion::fuseDetectionsByIoU(dets2d, dets3d, K, fusion_iou_threshold_);

    // 2) Update 3D tracker
    last3d_tracks_ = ab3dmot_.update(fused3d, frame_id);

    // 3) For 2D tracking, use input 2D detections directly
    last2d_tracks_ = bytetrack_.update(dets2d, frame_id);

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