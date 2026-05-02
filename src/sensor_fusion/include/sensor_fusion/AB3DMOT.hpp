#ifndef AB3DMOT_HPP
#define AB3DMOT_HPP

#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace sensor_fusion {

/**
 * @brief 3D Detection result
 * 7-parameter representation: (x, y, z, length, width, height, rotation_y)
 */
struct Detection3D {
    // 3D position (center of bounding box)
    float x, y, z;
    
    // 3D bounding box dimensions
    float length;   // x-axis extent
    float width;    // y-axis extent  
    float height;   // z-axis extent
    
    // Rotation around y-axis (in radians)
    float rotation_y;
    
    // Detection score
    float score;
    
    // Class ID
    int class_id;
    
    // Camera intrinsics for projection (optional)
    Eigen::Matrix3f K;
    
    Detection3D() : x(0), y(0), z(0), length(0), width(0), height(0),
                   rotation_y(0), score(0), class_id(-1) {
        K = Eigen::Matrix3f::Identity();
    }
    
    Detection3D(float x_, float y_, float z_, float l, float w, float h, 
                float ry, float s, int cls = -1)
        : x(x_), y(y_), z(z_), length(l), width(w), height(h),
          rotation_y(ry), score(s), class_id(cls) {
        K = Eigen::Matrix3f::Identity();
    }
    
    // Get center point in 3D
    Eigen::Vector3f getCenter() const {
        return Eigen::Vector3f(x, y, z);
    }
    
    // Get 3D bounding box volume
    float getVolume() const {
        return length * width * height;
    }
    
    // Project to 2D using camera intrinsics
    cv::Point2f project2D() const {
        Eigen::Vector3f pos(x, y, z);
        Eigen::Vector3f proj = K * pos;
        if (proj(2) > 0) {
            return cv::Point2f(proj(0) / proj(2), proj(1) / proj(2));
        }
        return cv::Point2f(-1, -1);
    }
};

/**
 * @brief 3D Track state enum
 */
enum class Track3DState {
    New,        // Newly created track
    Tracked,    // Active track
    Lost        // Lost track
};

/**
 * @brief Single 3D tracked object
 */
class STrack3D {
public:
    int track_id;
    Track3DState state;
    
    // 3D position and dimensions
    Eigen::Vector7f mean;  // [x, y, z, length, width, height, rotation_y]
    Eigen::Matrix<float, 7, 7> covariance;
    
    // Velocity in 3D space (dx, dy, dz per frame)
    Eigen::Vector3f velocity;
    
    // Track age and history
    int frame_id;
    int time_since_update;
    int hits;
    int age;
    
    // Detection history
    std::deque<Detection3D> detections;
    static constexpr int MAX_DETECTION_HISTORY = 30;
    
    // Track parameters
    static constexpr int MAX_AGE = 30;
    static constexpr int MIN_HITS = 3;
    
    // Track ID counter
    static int next_id;
    
public:
    STrack3D();
    STrack3D(const Detection3D& det, int frame_id);
    ~STrack3D() = default;
    
    /**
     * @brief Update track with new detection
     */
    void update(const Detection3D& det, int frame_id);
    
    /**
     * @brief Update track without detection (prediction only)
     */
    void updateWithoutDetection(int frame_id);
    
    /**
     * @brief Predict next position
     */
    void predict();
    
    /**
     * @brief Calculate Mahalanobis distance to detection
     */
    float getMahalanobisDistance(const Detection3D& det) const;
    
    /**
     * @brief Calculate 3D IoU with detection
     */
    float getIoU3D(const Detection3D& det) const;
    
    /**
     * @brief Check if track is active
     */
    bool isActive() const;
    
    /**
     * @brief Check if track is confirmed
     */
    bool isConfirmed() const {
        return state == Track3DState::Tracked;
    }
    
    /**
     * @brief Get bounding box corners in 3D
     */
    std::vector<Eigen::Vector3f> getBBoxCorners() const;
    
private:
    /**
     * @brief Initialize Kalman filter
     */
    void initKalmanFilter(const Detection3D& det);
    
    /**
     * @brief Kalman predict step
     */
    void kalmanPredict();
    
    /**
     * @brief Kalman update step
     */
    void kalmanUpdate(const Detection3D& det);
};

/**
 * @brief AB3DMOT - 3D Multi-Object Tracking
 */
class AB3DMOT {
public:
    // Matching parameters
    float high_conf_threshold = 0.5f;
    float low_conf_threshold = 0.1f;
    float mahal_threshold = 9.4877f;  // chi-squared threshold for 7-DOF
    float max_age = 30;
    float max_iou_distance = 1.0f;
    
public:
    AB3DMOT();
    ~AB3DMOT() = default;
    
    /**
     * @brief Update tracker with 3D detections
     */
    std::vector<STrack3D> update(const std::vector<Detection3D>& detections,
                                 int frame_id);
    
    /**
     * @brief Get all confirmed tracks
     */
    const std::vector<STrack3D>& getConfirmedTracks() const {
        return confirmed_tracks;
    }
    
    /**
     * @brief Get all active tracks
     */
    const std::vector<STrack3D>& getActiveTracks() const {
        return active_tracks;
    }
    
    /**
     * @brief Reset tracker
     */
    void reset();
    
private:
    // Track management
    std::vector<STrack3D> confirmed_tracks;
    std::vector<STrack3D> lost_tracks;
    std::vector<STrack3D> active_tracks;
    
    int frame_id_;
    int track_id_counter;
    
    /**
     * @brief Match detections with tracks
     */
    std::vector<std::pair<int, int>> matchDetectionsWithTracks(
        const std::vector<Detection3D>& detections,
        const std::vector<STrack3D>& tracks,
        float threshold);
    
    /**
     * @brief Build cost matrix (Mahalanobis distance)
     */
    Eigen::MatrixXf buildCostMatrix(
        const std::vector<Detection3D>& detections,
        const std::vector<STrack3D>& tracks);
    
    /**
     * @brief Greedy matching
     */
    std::vector<std::pair<int, int>> greedyMatching(
        const Eigen::MatrixXf& cost_matrix,
        float threshold);
};

}  // namespace sensor_fusion

#endif  // AB3DMOT_HPP

