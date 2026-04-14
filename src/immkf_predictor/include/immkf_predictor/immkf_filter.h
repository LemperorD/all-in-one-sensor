#ifndef IMMKF_PREDICTOR_IMMKF_FILTER_H_
#define IMMKF_PREDICTOR_IMMKF_FILTER_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "motion_models.h"

namespace immkf_predictor {

/**
 * Interacting Multiple Model Kalman Filter
 * Maintains N motion models and intelligently switches between them
 */
class IMMKFFilter {
 public:
  /**
   * Constructor
   * @param models Vector of motion models to use
   * @param initial_probs Initial model probabilities (must sum to 1.0)
   * @param mode_transitions Mode transition matrix (NxN stochastic matrix)
   */
  IMMKFFilter(const std::vector<std::shared_ptr<MotionModel>>& models,
              const std::vector<double>& initial_probs,
              const Eigen::MatrixXd& mode_transitions);

  /**
   * Initialize filter with observed position
   * @param position Observed 3D position [x, y, z]
   * @param init_covariance Covariance for position initialization
   */
  void initialize(const Eigen::Vector3d& position,
                   double init_position_covariance = 1.0,
                   double init_velocity_covariance = 10.0);

  /**
   * Predict step: move to next time step (without measurement)
   */
  void predict(double dt);

  /**
   * Update step: incorporate measurement
   * @param measurement Position measurement [x, y, z]
   */
  void update(const Eigen::Vector3d& measurement);

  /**
   * Get current mixed (fused) state estimate
   */
  Eigen::VectorXd getMixedState() const;

  /**
   * Get current mixed covariance estimate
   */
  Eigen::MatrixXd getMixedCovariance() const;

  /**
   * Get model probabilities
   */
  std::vector<double> getModelProbabilities() const;

  /**
   * Get most likely model index
   */
  int getMostLikelyModelIndex() const;

  /**
   * Get number of models
   */
  int getNumModels() const { return models_.size(); }

  /**
   * Get state from specific model (for debugging)
   */
  Eigen::VectorXd getModelState(int model_idx) const;

  /**
   * Get covariance from specific model (for debugging)
   */
  Eigen::MatrixXd getModelCovariance(int model_idx) const;

 private:
  std::vector<std::shared_ptr<MotionModel>> models_;
  std::vector<Eigen::VectorXd> states_;         // State for each model
  std::vector<Eigen::MatrixXd> covariances_;    // Covariance for each model
  std::vector<double> model_probs_;             // Model probabilities
  Eigen::MatrixXd mode_transitions_;            // Mode transition matrix

  // Internal computations
  std::vector<Eigen::VectorXd> mixed_states_;   // Mixed states for each model
  std::vector<Eigen::MatrixXd> mixed_covs_;     // Mixed covariances

  /**
   * Model mixing step
   */
  void modelMixing();

  /**
   * Model probability update based on innovation likelihood
   */
  void updateModelProbabilities(const Eigen::Vector3d& measurement);

  /**
   * Fuse estimates from all models
   */
  void fuseEstimates();

  int max_state_dim_; // Maximum state dimension across all models
};

}  // namespace immkf_predictor

#endif  // IMMKF_PREDICTOR_IMMKF_FILTER_H_
