#ifndef BYTETRACK_HPP
#define BYTETRACK_HPP

#include <vector>
#include <deque>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace sensor_fusion {

/**
 * @brief Detection result from object detector
 */
struct Detection {
    cv::Rect2f bbox;      // Bounding box (x, y, w, h)
    float conf;           // Confidence score
    int class_id;         // Class ID
    
    Detection() : conf(0.0f), class_id(-1) {}
    Detection(const cv::Rect2f& b, float c, int cls = -1) 
        : bbox(b), conf(c), class_id(cls) {}
    
    // Get center point
    cv::Point2f getCenter() const {
        return cv::Point2f(bbox.x + bbox.width / 2.0f, 
                          bbox.y + bbox.height / 2.0f);
    }
    
    // Get area
    float getArea() const {
        return bbox.width * bbox.height;
    }
};

/**
 * @brief Single tracked object state
 */
enum class TrackState {
    New,        // Newly created track
    Tracked,    // Currently being tracked
    Lost        // Lost but may be recovered
};

/**
 * @brief Tracked object (trajectory)
 */
class STrack {
public:
    int track_id;
    TrackState state;
    
    // Bounding box and center
    cv::Rect2f bbox;
    cv::Point2f center;
    
    // Kalman filter state (simplified: pos_x, pos_y, velocity_x, velocity_y)
    Eigen::Vector4f mean;        // [cx, cy, vx, vy]
    Eigen::Matrix4f covariance;  // Position and velocity covariance
    
    // Track history
    int frame_id;           // Last frame where this track was updated
    int time_since_update;  // Frames since last update
    int hits;               // Number of consecutive hits
    int age;                // Total age of the track
    
    // Detection history
    std::deque<Detection> detections;
    static constexpr int MAX_DETECTION_HISTORY = 30;
    
    // Track parameters
    static constexpr int MAX_AGE = 30;           // Max frames to keep lost track
    static constexpr int MIN_HITS = 3;           // Min hits to become tracked
    
    // Track ID counter
    static int next_id;
    
public:
    STrack();
    STrack(const Detection& det, int frame_id);
    ~STrack() = default;
    
    /**
     * @brief Update track with new detection
     */
    void update(const Detection& det, int frame_id);
    
    /**
     * @brief Update track state without detection (prediction only)
     */
    void updateWithoutDetection(int frame_id);
    
    /**
     * @brief Predict next position using Kalman filter
     */
    void predict();
    
    /**
     * @brief Get IoU (Intersection over Union) with another detection
     */
    float getIoU(const Detection& det) const;
    
    /**
     * @brief Get distance (1 - IoU) for matching
     */
    float getDistance(const Detection& det) const;
    
    /**
     * @brief Check if track is active (should be output)
     */
    bool isActive() const;
    
    /**
     * @brief Check if track is confirmed
     */
    bool isConfirmed() const {
        return state == TrackState::Tracked;
    }
    
    /**
     * @brief Get track status string
     */
    std::string getStateStr() const;
    
private:
    /**
     * @brief Initialize Kalman filter
     */
    void initKalmanFilter(const Detection& det);
    
    /**
     * @brief Predict Kalman filter state
     */
    void kalmanPredict();
    
    /**
     * @brief Update Kalman filter with measurement
     */
    void kalmanUpdate(const Detection& det);
};

/**
 * @brief ByteTrack - Simple yet effective tracker
 */
class ByteTrack {
public:
    // Matching parameters
    float high_conf_threshold = 0.5f;    // High confidence detection threshold
    float low_conf_threshold = 0.1f;     // Low confidence detection threshold
    float iou_threshold = 0.3f;          // IoU threshold for matching
    float max_time_lost = 30;             // Max frames to keep lost tracks
    
public:
    ByteTrack();
    ~ByteTrack() = default;
    
    /**
     * @brief Update tracker with new detections
     * @param detections Vector of detected objects
     * @param frame_id Current frame ID
     * @return Vector of active tracked objects
     */
    std::vector<STrack> update(const std::vector<Detection>& detections, 
                               int frame_id);
    
    /**
     * @brief Get all active tracks
     */
    const std::vector<STrack>& getActiveTracks() const {
        return confirmed_tracks;
    }
    
    /**
     * @brief Get all tracks (including unconfirmed)
     */
    const std::vector<STrack>& getAllTracks() const {
        return all_tracks;
    }
    
    /**
     * @brief Reset tracker
     */
    void reset();
    
    /**
     * @brief Set frame ID
     */
    void setFrameId(int frame_id) { frame_id_ = frame_id; }
    
private:
    // Track management
    std::vector<STrack> confirmed_tracks;  // Confirmed tracks
    std::vector<STrack> lost_tracks;       // Lost tracks (may be recovered)
    std::vector<STrack> all_tracks;        // All tracks for output
    
    int frame_id_;
    int track_id_counter;
    
    /**
     * @brief Match detections with tracks using IoU
     * @param detections Input detections
     * @param tracks Tracks to match against
     * @param iou_threshold Matching threshold
     * @return Matched pairs (track_idx, detection_idx), -1 means unmatched
     */
    std::vector<std::pair<int, int>> matchDetectionsWithTracks(
        const std::vector<Detection>& detections,
        const std::vector<STrack>& tracks,
        float iou_threshold);
    
    /**
     * @brief Match unmatched detections with unmatched tracks
     */
    std::vector<std::pair<int, int>> matchUnmatchedDetectionsWithUnmatchedTracks(
        const std::vector<Detection>& detections,
        const std::vector<int>& unmatched_detection_indices,
        const std::vector<STrack>& tracks,
        const std::vector<int>& unmatched_track_indices,
        float iou_threshold);
    
    /**
     * @brief Build cost matrix for bipartite matching
     */
    Eigen::MatrixXf buildCostMatrix(
        const std::vector<Detection>& detections,
        const std::vector<int>& detection_indices,
        const std::vector<STrack>& tracks,
        const std::vector<int>& track_indices);
    
    /**
     * @brief Solve bipartite matching with Hungarian algorithm (simple greedy approach)
     */
    std::vector<std::pair<int, int>> linearAssignment(
        const Eigen::MatrixXf& cost_matrix,
        const std::vector<int>& detection_indices,
        const std::vector<int>& track_indices,
        float threshold);
    
    /**
     * @brief Update confirmed tracks
     */
    void updateConfirmedTracks(const std::vector<STrack>& updated_tracks);
    
    /**
     * @brief Update lost tracks
     */
    void updateLostTracks();
};

}  // namespace sensor_fusion

#endif  // BYTETRACK_HPP

