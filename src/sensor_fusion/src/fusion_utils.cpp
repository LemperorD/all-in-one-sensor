#include "sensor_fusion/fusion_utils.hpp"

namespace sensor_fusion {

float iou2d(const cv::Rect2f &a, const cv::Rect2f &b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float w = std::max(0.0f, x2 - x1);
    float h = std::max(0.0f, y2 - y1);
    float inter = w * h;
    float u = a.width * a.height + b.width * b.height - inter;
    return u > 0.0f ? inter / u : 0.0f;
}

cv::Rect2f project3dTo2dBox(const Detection3D &det3d, const Eigen::Matrix3f &K) {
    // get 8 corners in 3D local frame
    float l = det3d.length;
    float w = det3d.width;
    float h = det3d.height;
    float x = det3d.x;
    float y = det3d.y;
    float z = det3d.z;
    float ry = det3d.rotation_y;

    float cosr = std::cos(ry);
    float sinr = std::sin(ry);

    std::vector<Eigen::Vector3f> corners;
    float dx[] = {-l/2, l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2};
    float dy[] = {-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2};
    float dz[] = {-w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2, w/2};

    for (int i = 0; i < 8; ++i) {
        float px = cosr * dx[i] - sinr * dz[i];
        float py = dy[i];
        float pz = sinr * dx[i] + cosr * dz[i];
        corners.emplace_back(x + px, y + py, z + pz);
    }

    // project
    float minx = std::numeric_limits<float>::infinity();
    float miny = std::numeric_limits<float>::infinity();
    float maxx = -std::numeric_limits<float>::infinity();
    float maxy = -std::numeric_limits<float>::infinity();

    for (const auto &pt : corners) {
        Eigen::Vector3f p = K * pt;
        if (p(2) <= 1e-6f) continue;
        float px = p(0) / p(2);
        float py = p(1) / p(2);
        minx = std::min(minx, px);
        miny = std::min(miny, py);
        maxx = std::max(maxx, px);
        maxy = std::max(maxy, py);
    }

    if (minx == std::numeric_limits<float>::infinity()) {
        return cv::Rect2f(-1, -1, 0, 0);
    }
    return cv::Rect2f(minx, miny, maxx - minx, maxy - miny);
}

float iou3dTo2d(const Detection3D &det3d, const cv::Rect2f &box2d, const Eigen::Matrix3f &K) {
    cv::Rect2f proj = project3dTo2dBox(det3d, K);
    if (proj.width <= 0 || proj.height <= 0) return 0.0f;
    return iou2d(proj, box2d);
}

std::vector<Detection3D> fuseDetectionsByIoU(const std::vector<Detection> &dets2d,
                                              const std::vector<Detection3D> &dets3d,
                                              const Eigen::Matrix3f &K,
                                              float iou_thresh) {
    std::vector<Detection3D> fused;

    std::vector<bool> used2d(dets2d.size(), false);

    for (const auto &d3 : dets3d) {
        float best_iou = 0.0f;
        int best_idx = -1;
        for (size_t i = 0; i < dets2d.size(); ++i) {
            if (used2d[i]) continue;
            float iou = iou3dTo2d(d3, dets2d[i].bbox, K);
            if (iou > best_iou) {
                best_iou = iou;
                best_idx = static_cast<int>(i);
            }
        }

        Detection3D out = d3;
        if (best_idx != -1 && best_iou >= iou_thresh) {
            // If 2D detection has higher confidence, adopt it to 3D score
            out.score = std::max(out.score, dets2d[best_idx].conf);
            used2d[best_idx] = true;
        }
        fused.push_back(out);
    }

    // Add leftover 2D detections as pseudo-3D (with z=0) if desired
    for (size_t i = 0; i < dets2d.size(); ++i) {
        if (used2d[i]) continue;
        Detection3D pseudo;
        // approximate center by backprojecting image center at z=0 (or leave z small)
        pseudo.x = 0.0f; pseudo.y = 0.0f; pseudo.z = 0.0f;
        pseudo.length = dets2d[i].bbox.width;
        pseudo.width = dets2d[i].bbox.height;
        pseudo.height = 1.0f;
        pseudo.rotation_y = 0.0f;
        pseudo.score = dets2d[i].conf;
        fused.push_back(pseudo);
    }

    return fused;
}

} // namespace sensor_fusion
