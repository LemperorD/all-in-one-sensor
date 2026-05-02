#ifndef FUSION_UTILS_HPP
#define FUSION_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "bytetrack.hpp"
#include "AB3DMOT.hpp"

namespace sensor_fusion {

// Compute 2D IoU between two rectangles
float iou2d(const cv::Rect2f &a, const cv::Rect2f &b);

// Given a 3D detection and camera intrinsics K, project its 8 corners
// to image plane and return the enclosing 2D bounding box.
cv::Rect2f project3dTo2dBox(const Detection3D &det3d, const Eigen::Matrix3f &K);

// Compute IoU between a 3D detection (projected) and 2D bbox
float iou3dTo2d(const Detection3D &det3d, const cv::Rect2f &box2d, const Eigen::Matrix3f &K);

// Fuse 2D and 3D detection lists by matching projected 3D boxes to 2D boxes
// Returns fused 3D detections (matched ones gain the 2D confidence if higher)
std::vector<Detection3D> fuseDetectionsByIoU(const std::vector<Detection> &dets2d,
                                              const std::vector<Detection3D> &dets3d,
                                              const Eigen::Matrix3f &K,
                                              float iou_thresh = 0.3f);

} // namespace sensor_fusion

#endif // FUSION_UTILS_HPP

