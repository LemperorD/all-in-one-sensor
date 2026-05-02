#include "sensor_fusion/AB3DMOT.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace sensor_fusion {

// ======================= STrack3D Implementation =======================

int STrack3D::next_id = 0;

STrack3D::STrack3D()
    : track_id(-1), state(Track3DState::New), frame_id(-1),
      time_since_update(0), hits(0), age(0) {
        mean = Eigen::Matrix<float, 7, 1>::Zero();
    covariance = Eigen::Matrix<float, 7, 7>::Identity() * 10.0f;
    velocity = Eigen::Vector3f::Zero();
}

STrack3D::STrack3D(const Detection3D& det, int frame_id_val)
    : track_id(next_id++), state(Track3DState::New), frame_id(frame_id_val),
      time_since_update(0), hits(1), age(1) {
    
    initKalmanFilter(det);
    velocity = Eigen::Vector3f::Zero();
    detections.push_back(det);
}

void STrack3D::initKalmanFilter(const Detection3D& det) {
    // Initialize state: [x, y, z, length, width, height, rotation_y]
    mean[0] = det.x;
    mean[1] = det.y;
    mean[2] = det.z;
    mean[3] = det.length;
    mean[4] = det.width;
    mean[5] = det.height;
    mean[6] = det.rotation_y;
    
    // Initialize covariance
    covariance = Eigen::Matrix<float, 7, 7>::Identity();
    covariance(0, 0) = det.length * det.length;
    covariance(1, 1) = det.width * det.width;
    covariance(2, 2) = det.height * det.height;
    covariance(3, 3) = 0.01f;  // length variance
    covariance(4, 4) = 0.01f;  // width variance
    covariance(5, 5) = 0.01f;  // height variance
    covariance(6, 6) = 0.01f;  // rotation variance
}

void STrack3D::predict() {
    kalmanPredict();
}

void STrack3D::kalmanPredict() {
    const float dt = 1.0f;
    
    // Update position with velocity
    mean[0] += velocity[0] * dt;
    mean[1] += velocity[1] * dt;
    mean[2] += velocity[2] * dt;
    
    // Dimensions and rotation stay the same
    // mean[3-6] unchanged
    
    // Add process noise to covariance
    float q = 0.1f;
    for (int i = 0; i < 3; i++) {
        covariance(i, i) += q;
    }
}

void STrack3D::kalmanUpdate(const Detection3D& det) {
    Eigen::Matrix<float, 7, 1> z;
    z[0] = det.x;
    z[1] = det.y;
    z[2] = det.z;
    z[3] = det.length;
    z[4] = det.width;
    z[5] = det.height;
    z[6] = det.rotation_y;
    
    // Observation matrix (we observe all 7 states)
    Eigen::Matrix<float, 7, 7> H = Eigen::Matrix<float, 7, 7>::Identity();
    
    // Measurement noise
    Eigen::Matrix<float, 7, 7> R = Eigen::Matrix<float, 7, 7>::Identity();
    R(0, 0) = 0.5f;   // x
    R(1, 1) = 0.5f;   // y
    R(2, 2) = 0.5f;   // z
    R(3, 3) = 0.01f;  // length
    R(4, 4) = 0.01f;  // width
    R(5, 5) = 0.01f;  // height
    R(6, 6) = 0.01f;  // rotation
    
    // Kalman gain
    Eigen::Matrix<float, 7, 7> S = H * covariance * H.transpose() + R;
    Eigen::Matrix<float, 7, 7> K = covariance * H.transpose() * S.inverse();
    
    // Innovation
    Eigen::Matrix<float, 7, 1> y = z - mean;
    
    // Update mean and covariance
    mean = mean + K * y;
    covariance = (Eigen::Matrix<float, 7, 7>::Identity() - K * H) * covariance;
    
    // Update velocity estimate
    Eigen::Vector3f new_pos = mean.head<3>();
    if (hits > 1) {
        // Simple velocity estimation
        velocity = new_pos - Eigen::Vector3f(detections.back().x, 
                                            detections.back().y,
                                            detections.back().z);
    }
}

void STrack3D::update(const Detection3D& det, int frame_id_val) {
    frame_id = frame_id_val;
    time_since_update = 0;
    hits++;
    
    // Update Kalman filter
    kalmanUpdate(det);
    
    // Store detection history
    detections.push_back(det);
    if (detections.size() > MAX_DETECTION_HISTORY) {
        detections.pop_front();
    }
    
    // Update state
    if (state == Track3DState::New && hits >= MIN_HITS) {
        state = Track3DState::Tracked;
    } else if (state == Track3DState::Lost && hits >= 1) {
        state = Track3DState::Tracked;
    }
}

void STrack3D::updateWithoutDetection(int frame_id_val) {
    frame_id = frame_id_val;
    time_since_update++;
    age++;
    
    predict();
    
    // Update state
    if (state == Track3DState::Tracked && time_since_update > 1) {
        state = Track3DState::Lost;
    }
}

float STrack3D::getMahalanobisDistance(const Detection3D& det) const {
    Eigen::Matrix<float, 7, 1> z;
    z[0] = det.x;
    z[1] = det.y;
    z[2] = det.z;
    z[3] = det.length;
    z[4] = det.width;
    z[5] = det.height;
    z[6] = det.rotation_y;
    
    Eigen::Matrix<float, 7, 1> y = z - mean;

    // Mahalanobis distance
    float mahal = y.transpose() * covariance.inverse() * y;
    return std::sqrt(mahal);
}

float STrack3D::getIoU3D(const Detection3D& det) const {
    // Simplified 3D IoU calculation
    // Get corners of both boxes
    auto corners1 = getBBoxCorners();
    
    // Calculate intersection volume (simplified)
    float dx = std::max(0.0f, std::min(mean[0] + mean[3]/2, det.x + det.length/2) - 
                                std::max(mean[0] - mean[3]/2, det.x - det.length/2));
    float dy = std::max(0.0f, std::min(mean[1] + mean[4]/2, det.y + det.width/2) - 
                                std::max(mean[1] - mean[4]/2, det.y - det.width/2));
    float dz = std::max(0.0f, std::min(mean[2] + mean[5]/2, det.z + det.height/2) - 
                                std::max(mean[2] - mean[5]/2, det.z - det.height/2));
    
    float intersection = dx * dy * dz;
    float vol1 = mean[3] * mean[4] * mean[5];
    float vol2 = det.length * det.width * det.height;
    float union_vol = vol1 + vol2 - intersection;
    
    return union_vol > 0 ? intersection / union_vol : 0.0f;
}

bool STrack3D::isActive() const {
    return state == Track3DState::Tracked && time_since_update <= 1;
}

std::vector<Eigen::Vector3f> STrack3D::getBBoxCorners() const {
    std::vector<Eigen::Vector3f> corners;
    
    float l = mean[3];
    float w = mean[4];
    float h = mean[5];
    float x = mean[0];
    float y = mean[1];
    float z = mean[2];
    float ry = mean[6];
    
    // Rotation matrix around y-axis
    float cos_ry = std::cos(ry);
    float sin_ry = std::sin(ry);
    
    // 8 corners in local frame
    float dx[] = {-l/2, l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2};
    float dy[] = {-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2};
    float dz[] = {-w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2, w/2};
    
    for (int i = 0; i < 8; i++) {
        float px = cos_ry * dx[i] - sin_ry * dz[i];
        float py = dy[i];
        float pz = sin_ry * dx[i] + cos_ry * dz[i];
        
        corners.push_back(Eigen::Vector3f(x + px, y + py, z + pz));
    }
    
    return corners;
}

// ======================= AB3DMOT Implementation =======================

AB3DMOT::AB3DMOT()
    : frame_id_(0), track_id_counter(0) {
}

std::vector<STrack3D> AB3DMOT::update(const std::vector<Detection3D>& detections,
                                      int frame_id) {
    frame_id_ = frame_id;
    
    // Separate detections by confidence
    std::vector<Detection3D> high_conf_dets;
    std::vector<Detection3D> low_conf_dets;
    
    std::vector<Detection3D> sorted_dets = detections;
    std::sort(sorted_dets.begin(), sorted_dets.end(),
              [](const Detection3D& a, const Detection3D& b) {
                  return a.score > b.score;
              });
    
    for (const auto& det : sorted_dets) {
        if (det.score >= high_conf_threshold) {
            high_conf_dets.push_back(det);
        } else if (det.score >= low_conf_threshold) {
            low_conf_dets.push_back(det);
        }
    }
    
    // Predict for all tracks
    for (auto& track : confirmed_tracks) {
        track.predict();
    }
    
    for (auto& track : lost_tracks) {
        track.updateWithoutDetection(frame_id_);
    }
    
    // First stage: match high confidence detections with confirmed tracks
    auto matched_high = matchDetectionsWithTracks(high_conf_dets, confirmed_tracks,
                                                   mahal_threshold);
    
    std::vector<bool> det_matched(high_conf_dets.size(), false);
    std::vector<bool> track_matched(confirmed_tracks.size(), false);
    
    std::vector<STrack3D> updated_confirmed;
    for (const auto& match : matched_high) {
        int track_idx = match.first;
        int det_idx = match.second;
        
        confirmed_tracks[track_idx].update(high_conf_dets[det_idx], frame_id_);
        updated_confirmed.push_back(confirmed_tracks[track_idx]);
        
        track_matched[track_idx] = true;
        det_matched[det_idx] = true;
    }
    
    // Collect unmatched indices
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
    
    // Second stage: match unmatched detections with lost tracks
    std::vector<Detection3D> all_unmatched_dets;
    for (int idx : unmatched_det_indices) {
        all_unmatched_dets.push_back(high_conf_dets[idx]);
    }
    for (const auto& det : low_conf_dets) {
        all_unmatched_dets.push_back(det);
    }
    
    auto matched_secondary = matchDetectionsWithTracks(all_unmatched_dets, lost_tracks,
                                                        mahal_threshold);
    
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
    
    // Create new tracks for unmatched high-confidence detections
    for (size_t i = 0; i < high_conf_dets.size(); i++) {
        bool matched = false;
        for (const auto& match : matched_high) {
            if (match.second == static_cast<int>(i)) {
                matched = true;
                break;
            }
        }
        
        if (!matched) {
            STrack3D new_track(high_conf_dets[i], frame_id_);
            updated_confirmed.push_back(new_track);
        }
    }
    
    confirmed_tracks = updated_confirmed;
    
    // Manage lost tracks list
    std::vector<STrack3D> new_lost;
    for (const auto& track : lost_tracks) {
        if (track.time_since_update < max_age) {
            new_lost.push_back(track);
        }
    }
    lost_tracks = new_lost;
    
    // Build output
    active_tracks.clear();
    for (const auto& track : confirmed_tracks) {
        if (track.isActive()) {
            active_tracks.push_back(track);
        }
    }
    
    return active_tracks;
}

std::vector<std::pair<int, int>> AB3DMOT::matchDetectionsWithTracks(
    const std::vector<Detection3D>& detections,
    const std::vector<STrack3D>& tracks,
    float threshold) {
    
    std::vector<std::pair<int, int>> matches;
    
    if (detections.empty() || tracks.empty()) {
        return matches;
    }
    
    // Build cost matrix
    Eigen::MatrixXf cost_matrix = buildCostMatrix(detections, tracks);
    
    // Greedy matching
    matches = greedyMatching(cost_matrix, threshold);
    
    return matches;
}

Eigen::MatrixXf AB3DMOT::buildCostMatrix(
    const std::vector<Detection3D>& detections,
    const std::vector<STrack3D>& tracks) {
    
    Eigen::MatrixXf cost_matrix(tracks.size(), detections.size());
    
    for (size_t i = 0; i < tracks.size(); i++) {
        for (size_t j = 0; j < detections.size(); j++) {
            // Use Mahalanobis distance
            cost_matrix(i, j) = tracks[i].getMahalanobisDistance(detections[j]);
        }
    }
    
    return cost_matrix;
}

std::vector<std::pair<int, int>> AB3DMOT::greedyMatching(
    const Eigen::MatrixXf& cost_matrix,
    float threshold) {
    
    std::vector<std::pair<int, int>> matches;
    Eigen::MatrixXf temp_cost = cost_matrix;
    
    // Greedy assignment
    while (true) {
        // Find minimum cost
        float min_cost = threshold;
        int best_i = -1, best_j = -1;
        
        for (int i = 0; i < temp_cost.rows(); i++) {
            for (int j = 0; j < temp_cost.cols(); j++) {
                if (temp_cost(i, j) < min_cost) {
                    min_cost = temp_cost(i, j);
                    best_i = i;
                    best_j = j;
                }
            }
        }
        
        if (best_i == -1) break;  // No more matches
        
        matches.push_back({best_i, best_j});
        
        // Mark row and column as used
        temp_cost.row(best_i).setConstant(threshold + 1);
        temp_cost.col(best_j).setConstant(threshold + 1);
    }
    
    return matches;
}

void AB3DMOT::reset() {
    confirmed_tracks.clear();
    lost_tracks.clear();
    active_tracks.clear();
    frame_id_ = 0;
    track_id_counter = 0;
    STrack3D::next_id = 0;
}

}  // namespace sensor_fusion
