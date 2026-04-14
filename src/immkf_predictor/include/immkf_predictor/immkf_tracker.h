#ifndef IMMKF_PREDICTOR_IMMKF_TRACKER_H_
#define IMMKF_PREDICTOR_IMMKF_TRACKER_H_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "immkf_filter.h"

namespace immkf_predictor {

/**
 * Trajectory point for output
 */
struct TrajectoryPoint {
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  double confidence;
  int model_index;
};

/**
 * Single object tracker using IMMKF
 */
class IMMKFTracker {
 public:
  /**
   * Constructor
   * @param object_id Unique object identifier
   * @param immkf_filter IMMKF filter instance
   */
  IMMKFTracker(const std::string& object_id,
               std::shared_ptr<IMMKFFilter> immkf_filter);

  /**
   * Predict to next time step
   */
  void predict(double dt);

  /**
   * Update with measurement
   */
  void update(const Eigen::Vector3d& measurement, double timestamp);

  /**
   * Generate predicted trajectory
   * @param horizon_seconds Total prediction time
   * @param step_hz Internal prediction step rate
   * @return Vector of predicted trajectory points
   */
  std::vector<TrajectoryPoint> generateTrajectory(double horizon_seconds,
                                                   double step_hz = 100.0);

  /**
   * Get object ID
   */
  const std::string& getObjectId() const { return object_id_; }

  /**
   * Get track age (number of updates)
   */
  int getTrackAge() const { return track_age_; }

  /**
   * Get track confidence (0-1)
   */
  double getTrackConfidence() const;

  /**
   * Get current position
   */
  Eigen::Vector3d getPosition() const;

  /**
   * Get current velocity
   */
  Eigen::Vector3d getVelocity() const;

  /**
   * Get model probabilities
   */
  std::vector<double> getModelProbabilities() const;

  /**
   * Get most likely model
   */
  int getMostLikelyModel() const;

  /**
   * Check if track is confirmed (enough observations)
   */
  bool isConfirmed(int confirmation_threshold = 5) const;

 private:
  std::string object_id_;
  std::shared_ptr<IMMKFFilter> filter_;
  int track_age_;
  double last_update_time_;

  // Predict single step internally
  TrajectoryPoint predictStep(double dt, int model_hint = -1);
};

}  // namespace immkf_predictor

#endif  // IMMKF_PREDICTOR_IMMKF_TRACKER_H_
