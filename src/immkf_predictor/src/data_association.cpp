#include "immkf_predictor/data_association.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace immkf_predictor {

// Simple Hungarian algorithm implementation for minimal bipartite matching
std::vector<int> DataAssociation::hungarianAlgorithm(
    const Eigen::MatrixXd& cost_matrix) {
  int n_tracks = cost_matrix.rows();
  int n_detections = cost_matrix.cols();
  int n = std::max(n_tracks, n_detections);

  // Pad cost matrix to square if needed
  Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n_tracks; ++i) {
    for (int j = 0; j < n_detections; ++j) {
      cost(i, j) = cost_matrix(i, j);
    }
  }

  // Fill remaining with large cost (unassigned)
  double large_cost = 1e6;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i >= n_tracks || j >= n_detections) {
        cost(i, j) = large_cost;
      }
    }
  }

  // Initialize labels
  std::vector<int> u(n + 1), v(n + 1), p(n + 1), way(n + 1);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, 1e9);
    std::vector<bool> used(n + 1, false);

    do {
      used[j0] = true;
      int i0 = p[j0], delta = 1e9, j1;

      for (int j = 1; j <= n; ++j) {
        if (!used[j]) {
          double cur = cost(i0 - 1, j - 1) - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }

      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }

      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> ans(n + 1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] != 0) {
      ans[p[j]] = j;
    }
  }

  // Resize to original size
  std::vector<int> result(n_tracks, -1);
  for (int i = 0; i < n_tracks; ++i) {
    if (ans[i + 1] <= n_detections && ans[i + 1] > 0) {
      result[i] = ans[i + 1] - 1;
    }
  }

  return result;
}

AssociationResult DataAssociation::associate(
    const std::vector<Eigen::Vector3d>& track_positions,
    const std::vector<Eigen::Vector3d>& detection_positions,
    double max_distance) {
  AssociationResult result;
  int n_tracks = track_positions.size();
  int n_detections = detection_positions.size();

  result.track_indices.assign(n_detections, -1);
  result.detection_indices.assign(n_tracks, -1);
  result.costs.clear();

  // Create cost matrix based on Euclidean distance
  Eigen::MatrixXd cost_matrix(n_tracks, n_detections);

  for (int i = 0; i < n_tracks; ++i) {
    for (int j = 0; j < n_detections; ++j) {
      double distance =
          (track_positions[i] - detection_positions[j]).norm();

      if (distance > max_distance) {
        cost_matrix(i, j) = 1e6;  // Prohibitively large cost
      } else {
        cost_matrix(i, j) = distance;
      }
    }
  }

  // Run Hungarian algorithm
  std::vector<int> assignment = hungarianAlgorithm(cost_matrix);

  // Process assignment results
  for (int i = 0; i < n_tracks; ++i) {
    int j = assignment[i];
    if (j >= 0 && j < n_detections &&
        cost_matrix(i, j) < 1e5) {  // Valid assignment
      result.detection_indices[i] = j;
      result.track_indices[j] = i;
      result.costs.push_back(cost_matrix(i, j));
    }
  }

  return result;
}

}  // namespace immkf_predictor
