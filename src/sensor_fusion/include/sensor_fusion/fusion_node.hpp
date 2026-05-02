#ifndef FUSION_NODE_HPP
#define FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>

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
  
  /**
   * @brief Process a frame: fuse 2D and 3D detections and update trackers
   * @param frame_id frame index
   * @param dets2d 2D detections
   * @param dets3d 3D detections
   * @param K camera intrinsics for projection
   */
  void processFrame(int frame_id,
                    const std::vector<Detection> &dets2d,
                    const std::vector<Detection3D> &dets3d,
                    const Eigen::Matrix3f &K);

private: // 成员变量
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

};

} // namespace sensor_fusion

#endif // FUSION_NODE_HPP