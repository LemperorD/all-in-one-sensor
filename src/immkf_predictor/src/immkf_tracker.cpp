#include "immkf_predictor/immkf_tracker.h"

#include <cmath>

namespace immkf_predictor {

IMMKFTracker::IMMKFTracker(const std::string& object_id,
                           std::shared_ptr<IMMKFFilter> immkf_filter)
    : object_id_(object_id), filter_(immkf_filter), track_age_(0),
      last_update_time_(0.0) {}

void IMMKFTracker::predict(double dt) { filter_->predict(dt); }

void IMMKFTracker::update(const Eigen::Vector3d& measurement,
                          double timestamp) {
  filter_->update(measurement);
  track_age_++;
  last_update_time_ = timestamp;
}

std::vector<TrajectoryPoint> IMMKFTracker::generateTrajectory(
    double horizon_seconds, double step_hz) {
  std::vector<TrajectoryPoint> trajectory;

  double dt = 1.0 / step_hz;
  int n_steps = static_cast<int>(horizon_seconds * step_hz);

  // Get current state for prediction
  Eigen::VectorXd current_state = filter_->getMixedState();

  for (int i = 0; i < n_steps; ++i) {
    TrajectoryPoint pt = predictStep(dt, filter_->getMostLikelyModelIndex());
    trajectory.push_back(pt);
  }

  return trajectory;
}

TrajectoryPoint IMMKFTracker::predictStep(double dt, int model_hint) {
  TrajectoryPoint pt;

  int model_idx = model_hint >= 0 ? model_hint : filter_->getMostLikelyModelIndex();

  // Get current state
  Eigen::VectorXd state = filter_->getMixedState();

  // Predict position using the model
  Eigen::VectorXd predicted_state = state;  // Start with current state
  // Note: The actual model prediction happens externally
  // Here we just extract the current state

  pt.position = state.head(3);

  // Extract velocity if available
  if (state.size() >= 6) {
    pt.velocity = state.segment(3, 3);
  } else {
    pt.velocity = Eigen::Vector3d::Zero();
  }

  pt.timestamp = last_update_time_;
  pt.confidence = getTrackConfidence();
  pt.model_index = model_idx;

  return pt;
}

Eigen::Vector3d IMMKFTracker::getPosition() const {
  return filter_->getMixedState().head(3);
}

Eigen::Vector3d IMMKFTracker::getVelocity() const {
  Eigen::VectorXd state = filter_->getMixedState();
  if (state.size() >= 6) {
    return state.segment(3, 3);
  }
  return Eigen::Vector3d::Zero();
}

std::vector<double> IMMKFTracker::getModelProbabilities() const {
  return filter_->getModelProbabilities();
}

int IMMKFTracker::getMostLikelyModel() const {
  return filter_->getMostLikelyModelIndex();
}

double IMMKFTracker::getTrackConfidence() const {
  // Confidence increases with age, plateaus at confirmed
  double age_confidence = std::min(1.0, track_age_ / 10.0);

  // Also consider model probability confidence
  auto probs = filter_->getModelProbabilities();
  double max_prob = 0.0;
  for (double p : probs) {
    max_prob = std::max(max_prob, p);
  }

  // Blend both factors
  return 0.6 * age_confidence + 0.4 * max_prob;
}

bool IMMKFTracker::isConfirmed(int confirmation_threshold) const {
  return track_age_ >= confirmation_threshold;
}

}  // namespace immkf_predictor
