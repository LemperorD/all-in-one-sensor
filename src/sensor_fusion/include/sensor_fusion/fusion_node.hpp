#ifndef FUSION_NODE_HPP
#define FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <std_msgs/msg/header.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#include <mutex>

#include "bytetrack.hpp"
#include "AB3DMOT.hpp"
#include "fusion_utils.hpp"

namespace sensor_fusion
{

class FusionNode : public rclcpp::Node
{
public: // 构造函数与析构函数
  explicit FusionNode(const rclcpp::NodeOptions & options);
  ~FusionNode() override;

public: // 方法
  void onConfigure();

  void detections2dCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
  void detections3dCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
  
  /**
   * @brief Process a frame: fuse 2D and 3D detections and update trackers
   * @param frame_id frame index
   * @param dets2d 2D detections
   * @param dets3d 3D detections
   * @param header_2d header for 2D input and 2D track output
   * @param header_3d header for 3D input and 3D track output
   * @param K camera intrinsics for projection
   */
  void processFrame(int frame_id,
                    const std::vector<Detection> &dets2d,
                    const std::vector<Detection3D> &dets3d,
                    const std_msgs::msg::Header &header_2d,
                    const std_msgs::msg::Header &header_3d,
                    const Eigen::Matrix3f &K);

private: // 成员变量
  // Topic names (for future subscriber/publisher setup)
  std::string topic_detections_2d_;
  std::string topic_detections_3d_;
  std::string topic_tracks_2d_;
  std::string topic_tracks_3d_;

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_2d_sub_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_3d_sub_;
  rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr tracks_2d_pub_;
  rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr tracks_3d_pub_;

  std::mutex input_mutex_;
  yolo_msgs::msg::DetectionArray latest_detections_2d_;
  yolo_msgs::msg::DetectionArray latest_detections_3d_;
  bool has_latest_detections_2d_{false};
  bool has_latest_detections_3d_{false};
  std::uint64_t last_processed_2d_stamp_ns_{0};
  std::uint64_t last_processed_3d_stamp_ns_{0};
  int frame_counter_{0};

  // Trackers
  ByteTrack bytetrack_;
  AB3DMOT ab3dmot_;

  // last outputs
  std::vector<STrack> last2d_tracks_;
  std::vector<STrack3D> last3d_tracks_;

  // Camera intrinsics (3x3 matrix)
  Eigen::Matrix3f camera_K_;

  // Fusion parameters
  float fusion_iou_threshold_;
  float bytetrack_high_conf_;
  float bytetrack_low_conf_;
  float bytetrack_iou_thresh_;
  float ab3dmot_high_conf_;
  float ab3dmot_low_conf_;
  float ab3dmot_mahal_thresh_;

  void tryProcessLatestFrame();
  std::vector<Detection> convertDetections2d(const yolo_msgs::msg::DetectionArray & msg) const;
  std::vector<Detection3D> convertDetections3d(const yolo_msgs::msg::DetectionArray & msg) const;
  yolo_msgs::msg::DetectionArray buildTracks2dMessage(const std::vector<STrack> & tracks,
                                                      const std_msgs::msg::Header & header) const;
  yolo_msgs::msg::DetectionArray buildTracks3dMessage(const std::vector<STrack3D> & tracks,
                                                      const std_msgs::msg::Header & header) const;

};

} // namespace sensor_fusion

#endif // FUSION_NODE_HPP