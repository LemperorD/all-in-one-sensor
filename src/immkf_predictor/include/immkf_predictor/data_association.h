#ifndef IMMKF_PREDICTOR_DATA_ASSOCIATION_H_
#define IMMKF_PREDICTOR_DATA_ASSOCIATION_H_

#include <Eigen/Dense>
#include <vector>

namespace immkf_predictor {

/**
 * Association result: detection -> track mapping
 */
struct AssociationResult {
  std::vector<int> track_indices;  // index of track for each detection (-1 = no match)
  std::vector<int> detection_indices;  // index of detection for each track (-1 = no match)
  std::vector<double> costs;       // cost of association
};

/**
 * Data association using Hungarian algorithm for optimal assignment
 */
class DataAssociation {
 public:
  /**
   * Match detections to tracks based on 3D position distance
   * @param track_positions Current track positions
   * @param detection_positions New detection positions
   * @param max_distance Maximum allowed distance threshold
   * @return Association result
   */
  static AssociationResult associate(
      const std::vector<Eigen::Vector3d>& track_positions,
      const std::vector<Eigen::Vector3d>& detection_positions,
      double max_distance = 2.0);

 private:
  /**
   * Hungarian algorithm for optimal assignment
   * @param cost_matrix Cost matrix (N_tracks x N_detections)
   * @return Assignment indices
   */
  static std::vector<int> hungarianAlgorithm(
      const Eigen::MatrixXd& cost_matrix);
};

}  // namespace immkf_predictor

#endif  // IMMKF_PREDICTOR_DATA_ASSOCIATION_H_
