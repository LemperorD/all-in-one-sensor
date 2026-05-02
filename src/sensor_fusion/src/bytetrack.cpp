#include "sensor_fusion/bytetrack.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace sensor_fusion {

// ======================= STrack Implementation =======================

int STrack::next_id = 0;

STrack::STrack() 
    : track_id(-1), state(TrackState::New), frame_id(-1), 
      time_since_update(0), hits(0), age(0) {
    mean = Eigen::Vector4f::Zero();
    covariance = Eigen::Matrix4f::Identity() * 10.0f;
}

STrack::STrack(const Detection& det, int frame_id_val)
    : track_id(next_id++), state(TrackState::New), bbox(det.bbox), center(det.getCenter()),
      frame_id(frame_id_val), time_since_update(0), hits(1), age(1) {
    
    center = det.getCenter();
    initKalmanFilter(det);
    detections.push_back(det);
}

void STrack::initKalmanFilter(const Detection& det) {
    float cx = det.bbox.x + det.bbox.width / 2.0f;
    float cy = det.bbox.y + det.bbox.height / 2.0f;
    
    mean[0] = cx;
    mean[1] = cy;
    mean[2] = 0.0f;  // vx
    mean[3] = 0.0f;  // vy
    
    // Initial covariance
    covariance = Eigen::Matrix4f::Identity();
    covariance(0, 0) = det.bbox.width * det.bbox.width;
    covariance(1, 1) = det.bbox.height * det.bbox.height;
    covariance(2, 2) = 10.0f;
    covariance(3, 3) = 10.0f;
}

void STrack::predict() {
    kalmanPredict();
    
    // Update bounding box based on predicted center
    bbox.x = mean[0] - bbox.width / 2.0f;
    bbox.y = mean[1] - bbox.height / 2.0f;
    center = cv::Point2f(mean[0], mean[1]);
}

void STrack::kalmanPredict() {
    // Simple constant velocity model
    // x' = x + vx
    // y' = y + vy
    // vx' = vx
    // vy' = vy
    
    const float dt = 1.0f;
    
    // Transition matrix
    Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    
    // Process noise
    float q = 0.01f;
    Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
    Q(0, 0) = q;
    Q(1, 1) = q;
    Q(2, 2) = q;
    Q(3, 3) = q;
    
    // Predict mean and covariance
    mean = F * mean;
    covariance = F * covariance * F.transpose() + Q;
}

void STrack::kalmanUpdate(const Detection& det) {
    float cx = det.bbox.x + det.bbox.width / 2.0f;
    float cy = det.bbox.y + det.bbox.height / 2.0f;
    
    Eigen::Vector4f z;
    z[0] = cx;
    z[1] = cy;
    z[2] = mean[2];  // Use predicted velocity
    z[3] = mean[3];
    
    // Observation matrix: we observe position only
    Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;
    
    // Measurement noise
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.1f;
    
    // Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
    Eigen::Matrix<float, 4, 2> K = covariance * H.transpose() * 
        (H * covariance * H.transpose() + R).inverse();
    
    // Update mean and covariance
    Eigen::Vector2f y;
    y[0] = z[0] - mean[0];
    y[1] = z[1] - mean[1];
    
    mean = mean + K * y;
    covariance = (Eigen::Matrix4f::Identity() - K * H) * covariance;
}

void STrack::update(const Detection& det, int frame_id_val) {
    frame_id = frame_id_val;
    time_since_update = 0;
    hits++;
    
    // Update with Kalman filter
    kalmanUpdate(det);
    
    // Update bounding box
    bbox = det.bbox;
    center = det.getCenter();
    
    // Store detection history
    detections.push_back(det);
    if (detections.size() > MAX_DETECTION_HISTORY) {
        detections.pop_front();
    }
    
    // Update state
    if (state == TrackState::New && hits >= MIN_HITS) {
        state = TrackState::Tracked;
    } else if (state == TrackState::Lost && hits >= 1) {
        state = TrackState::Tracked;
    }
}

void STrack::updateWithoutDetection(int frame_id_val) {
    frame_id = frame_id_val;
    time_since_update++;
    age++;
    
    predict();
    
    // Update state
    if (state == TrackState::Tracked && time_since_update > 1) {
        state = TrackState::Lost;
    }
}

float STrack::getIoU(const Detection& det) const {
    // Intersection over Union
    float inter_w = std::max(0.0f, std::min(bbox.x + bbox.width, det.bbox.x + det.bbox.width) - 
                                   std::max(bbox.x, det.bbox.x));
    float inter_h = std::max(0.0f, std::min(bbox.y + bbox.height, det.bbox.y + det.bbox.height) - 
                                   std::max(bbox.y, det.bbox.y));
    
    float intersection = inter_w * inter_h;
    float union_area = bbox.width * bbox.height + det.bbox.width * det.bbox.height - intersection;
    
    return union_area > 0 ? intersection / union_area : 0.0f;
}

float STrack::getDistance(const Detection& det) const {
    // Distance is 1 - IoU
    return 1.0f - getIoU(det);
}

bool STrack::isActive() const {
    return state == TrackState::Tracked && time_since_update <= 1;
}

std::string STrack::getStateStr() const {
    switch (state) {
        case TrackState::New:
            return "New";
        case TrackState::Tracked:
            return "Tracked";
        case TrackState::Lost:
            return "Lost";
        default:
            return "Unknown";
    }
}

// ======================= ByteTrack Implementation =======================

ByteTrack::ByteTrack() 
    : frame_id_(0), track_id_counter(0) {
}

std::vector<STrack> ByteTrack::update(const std::vector<Detection>& detections,
                                      int frame_id) {
    frame_id_ = frame_id;
    
    // Separate detections by confidence
    std::vector<Detection> high_conf_detections;
    std::vector<Detection> low_conf_detections;
    
    std::vector<Detection> sorted_dets = detections;
    std::sort(sorted_dets.begin(), sorted_dets.end(),
              [](const Detection& a, const Detection& b) {
                  return a.conf > b.conf;
              });
    
    for (const auto& det : sorted_dets) {
        if (det.conf >= high_conf_threshold) {
            high_conf_detections.push_back(det);
        } else if (det.conf >= low_conf_threshold) {
            low_conf_detections.push_back(det);
        }
    }
    
    // Update lost tracks
    for (auto& track : lost_tracks) {
        track.updateWithoutDetection(frame_id_);
    }
    
    // Update confirmed tracks
    for (auto& track : confirmed_tracks) {
        track.predict();
    }
    
    // Match high confidence detections with confirmed tracks
    auto matched_high = matchDetectionsWithTracks(
        high_conf_detections, confirmed_tracks, iou_threshold);
    
    // Collect unmatched indices
    std::vector<bool> det_matched(high_conf_detections.size(), false);
    std::vector<bool> track_matched(confirmed_tracks.size(), false);
    
    std::vector<STrack> updated_confirmed;
    for (const auto& match : matched_high) {
        int track_idx = match.first;
        int det_idx = match.second;
        
        confirmed_tracks[track_idx].update(high_conf_detections[det_idx], frame_id_);
        updated_confirmed.push_back(confirmed_tracks[track_idx]);
        
        track_matched[track_idx] = true;
        det_matched[det_idx] = true;
    }
    
    // Collect unmatched indices for second stage
    std::vector<int> unmatched_det_indices;
    std::vector<int> unmatched_track_indices;
    
    for (size_t i = 0; i < det_matched.size(); i++) {
        if (!det_matched[i]) {
            unmatched_det_indices.push_back(i);
        }
    }
    
    for (size_t i = 0; i < track_matched.size(); i++) {
        if (!track_matched[i]) {
            unmatched_track_indices.push_back(i);
        }
    }
    
    // Combine unmatched detections
    std::vector<Detection> all_unmatched_dets;
    for (int idx : unmatched_det_indices) {
        all_unmatched_dets.push_back(high_conf_detections[idx]);
    }
    for (const auto& det : low_conf_detections) {
        all_unmatched_dets.push_back(det);
    }
    
    // Match unmatched detections with lost tracks
    auto matched_secondary = matchDetectionsWithTracks(
        all_unmatched_dets, lost_tracks, iou_threshold);
    
    std::vector<bool> lost_track_matched(lost_tracks.size(), false);
    for (const auto& match : matched_secondary) {
        int track_idx = match.first;
        int det_idx = match.second;
        
        lost_tracks[track_idx].update(all_unmatched_dets[det_idx], frame_id_);
        if (lost_tracks[track_idx].isConfirmed()) {
            updated_confirmed.push_back(lost_tracks[track_idx]);
        }
        lost_track_matched[track_idx] = true;
    }
    
    // Update unmatched confirmed tracks
    for (size_t i = 0; i < confirmed_tracks.size(); i++) {
        if (!track_matched[i]) {
            confirmed_tracks[i].updateWithoutDetection(frame_id_);
        }
    }
    
    // Update unmatched lost tracks
    for (size_t i = 0; i < lost_tracks.size(); i++) {
        if (!lost_track_matched[i]) {
            lost_tracks[i].updateWithoutDetection(frame_id_);
        }
    }
    
    // Manage track lists
    confirmed_tracks = updated_confirmed;
    
    // Remove old lost tracks
    std::vector<STrack> new_lost;
    for (const auto& track : lost_tracks) {
        if (track.time_since_update < max_time_lost) {
            new_lost.push_back(track);
        }
    }
    lost_tracks = new_lost;
    
    // Create new tracks for unmatched detections
    for (size_t i = 0; i < all_unmatched_dets.size(); i++) {
        bool matched = false;
        for (const auto& match : matched_secondary) {
            if (match.second == static_cast<int>(i)) {
                matched = true;
                break;
            }
        }
        
        if (!matched) {
            STrack new_track(all_unmatched_dets[i], frame_id_);
            confirmed_tracks.push_back(new_track);
        }
    }
    
    // Build output: only active tracks
    all_tracks.clear();
    for (const auto& track : confirmed_tracks) {
        if (track.isActive()) {
            all_tracks.push_back(track);
        }
    }
    
    // Also add recent lost tracks that might be recovered
    for (const auto& track : lost_tracks) {
        if (track.time_since_update <= 3) {
            all_tracks.push_back(track);
        }
    }
    
    return all_tracks;
}

std::vector<std::pair<int, int>> ByteTrack::matchDetectionsWithTracks(
    const std::vector<Detection>& detections,
    const std::vector<STrack>& tracks,
    float iou_threshold) {
    
    std::vector<std::pair<int, int>> matches;
    
    if (detections.empty() || tracks.empty()) {
        return matches;
    }
    
    // Build cost matrix
    Eigen::MatrixXf cost_matrix(tracks.size(), detections.size());
    for (size_t i = 0; i < tracks.size(); i++) {
        for (size_t j = 0; j < detections.size(); j++) {
            cost_matrix(i, j) = tracks[i].getDistance(detections[j]);
        }
    }
    
    // Simple greedy matching
    for (size_t i = 0; i < tracks.size(); i++) {
        float min_cost = iou_threshold;
        int best_det_idx = -1;
        
        for (size_t j = 0; j < detections.size(); j++) {
            if (cost_matrix(i, j) < min_cost) {
                min_cost = cost_matrix(i, j);
                best_det_idx = j;
            }
        }
        
        if (best_det_idx != -1) {
            matches.push_back({static_cast<int>(i), best_det_idx});
            // Mark as matched by setting high cost
            for (size_t j = 0; j < detections.size(); j++) {
                cost_matrix(i, j) = 2.0f;
            }
            for (size_t k = 0; k < tracks.size(); k++) {
                cost_matrix(k, best_det_idx) = 2.0f;
            }
        }
    }
    
    return matches;
}

std::vector<std::pair<int, int>> ByteTrack::matchUnmatchedDetectionsWithUnmatchedTracks(
    const std::vector<Detection>& detections,
    const std::vector<int>& unmatched_detection_indices,
    const std::vector<STrack>& tracks,
    const std::vector<int>& unmatched_track_indices,
    float iou_threshold) {
    
    std::vector<std::pair<int, int>> matches;
    
    if (unmatched_detection_indices.empty() || unmatched_track_indices.empty()) {
        return matches;
    }
    
    Eigen::MatrixXf cost_matrix(unmatched_track_indices.size(), 
                                unmatched_detection_indices.size());
    
    for (size_t i = 0; i < unmatched_track_indices.size(); i++) {
        for (size_t j = 0; j < unmatched_detection_indices.size(); j++) {
            int track_idx = unmatched_track_indices[i];
            int det_idx = unmatched_detection_indices[j];
            cost_matrix(i, j) = tracks[track_idx].getDistance(detections[det_idx]);
        }
    }
    
    // Greedy matching
    for (size_t i = 0; i < unmatched_track_indices.size(); i++) {
        float min_cost = iou_threshold;
        int best_j = -1;
        
        for (size_t j = 0; j < unmatched_detection_indices.size(); j++) {
            if (cost_matrix(i, j) < min_cost) {
                min_cost = cost_matrix(i, j);
                best_j = j;
            }
        }
        
        if (best_j != -1) {
            matches.push_back({unmatched_track_indices[i], 
                              unmatched_detection_indices[best_j]});
            // Mark as matched
            for (size_t j = 0; j < unmatched_detection_indices.size(); j++) {
                cost_matrix(i, j) = 2.0f;
            }
            for (size_t k = 0; k < unmatched_track_indices.size(); k++) {
                cost_matrix(k, best_j) = 2.0f;
            }
        }
    }
    
    return matches;
}

Eigen::MatrixXf ByteTrack::buildCostMatrix(
    const std::vector<Detection>& detections,
    const std::vector<int>& detection_indices,
    const std::vector<STrack>& tracks,
    const std::vector<int>& track_indices) {
    
    Eigen::MatrixXf cost_matrix(track_indices.size(), detection_indices.size());
    
    for (size_t i = 0; i < track_indices.size(); i++) {
        for (size_t j = 0; j < detection_indices.size(); j++) {
            cost_matrix(i, j) = tracks[track_indices[i]].getDistance(
                detections[detection_indices[j]]);
        }
    }
    
    return cost_matrix;
}

std::vector<std::pair<int, int>> ByteTrack::linearAssignment(
    const Eigen::MatrixXf& cost_matrix,
    const std::vector<int>& detection_indices,
    const std::vector<int>& track_indices,
    float threshold) {
    
    std::vector<std::pair<int, int>> matches;
    
    // Simple greedy assignment
    Eigen::MatrixXf temp_cost = cost_matrix;
    
    for (size_t i = 0; i < track_indices.size(); i++) {
        float min_cost = threshold;
        int best_j = -1;
        
        for (size_t j = 0; j < detection_indices.size(); j++) {
            if (temp_cost(i, j) < min_cost) {
                min_cost = temp_cost(i, j);
                best_j = j;
            }
        }
        
        if (best_j != -1) {
            matches.push_back({track_indices[i], detection_indices[best_j]});
            temp_cost.row(i).setConstant(2.0f);
            temp_cost.col(best_j).setConstant(2.0f);
        }
    }
    
    return matches;
}

void ByteTrack::updateConfirmedTracks(const std::vector<STrack>& updated_tracks) {
    confirmed_tracks = updated_tracks;
}

void ByteTrack::updateLostTracks() {
    std::vector<STrack> remaining_lost;
    for (const auto& track : lost_tracks) {
        if (track.time_since_update < max_time_lost) {
            remaining_lost.push_back(track);
        }
    }
    lost_tracks = remaining_lost;
}

void ByteTrack::reset() {
    confirmed_tracks.clear();
    lost_tracks.clear();
    all_tracks.clear();
    frame_id_ = 0;
    track_id_counter = 0;
    STrack::next_id = 0;
}

}  // namespace sensor_fusion
