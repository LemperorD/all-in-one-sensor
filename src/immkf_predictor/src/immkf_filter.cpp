#include "immkf_predictor/immkf_filter.h"

#include <cmath>
#include <iostream>

namespace immkf_predictor {

IMMKFFilter::IMMKFFilter(const std::vector<std::shared_ptr<MotionModel>>& models,
                         const std::vector<double>& initial_probs,
                         const Eigen::MatrixXd& mode_transitions)
    : models_(models),
      model_probs_(initial_probs),
      mode_transitions_(mode_transitions) {
  int n_models = models.size();

  // Initialize states and covariances
  max_state_dim_ = 0;
  for (size_t i = 0; i < models.size(); ++i) {
    int state_dim = models[i]->getStateDim();
    max_state_dim_ = std::max(max_state_dim_, state_dim);
    states_.push_back(Eigen::VectorXd::Zero(state_dim));
    covariances_.push_back(Eigen::MatrixXd::Zero(state_dim, state_dim));
    mixed_states_.push_back(Eigen::VectorXd::Zero(state_dim));
    mixed_covs_.push_back(Eigen::MatrixXd::Zero(state_dim, state_dim));
  }
}

void IMMKFFilter::initialize(const Eigen::Vector3d& position,
                              double init_position_covariance,
                              double init_velocity_covariance) {
  for (size_t i = 0; i < models_.size(); ++i) {
    int state_dim = models_[i]->getStateDim();
    states_[i] = Eigen::VectorXd::Zero(state_dim);
    states_[i].head(3) = position;
    // Initialize velocity high variance initially
    if (state_dim >= 6) {
      states_[i](3) = 0.0;  // vx
      states_[i](4) = 0.0;  // vy
      states_[i](5) = 0.0;  // vz
    }
    // Initialize acceleration
    if (state_dim >= 9) {
      states_[i](6) = 0.0;  // ax
      states_[i](7) = 0.0;  // ay
      states_[i](8) = 0.0;  // az
    }

    // Initialize covariance
    covariances_[i] = Eigen::MatrixXd::Identity(state_dim, state_dim);
    covariances_[i].block(0, 0, 3, 3) *= init_position_covariance;
    if (state_dim >= 6) {
      covariances_[i].block(3, 3, 3, 3) *= init_velocity_covariance;
    }
    if (state_dim >= 9) {
      covariances_[i].block(6, 6, 3, 3) *= 10.0;  // High acceleration uncertainty
    }
  }
}

void IMMKFFilter::predict(double dt) {
  modelMixing();

  for (size_t i = 0; i < models_.size(); ++i) {
    // Predict state: x_{k|k-1} = F*x_{k-1|k-1}
    states_[i] = models_[i]->predict(mixed_states_[i], dt);

    // Predict covariance: P_{k|k-1} = F*P_{k-1|k-1}*F^T + Q
    Eigen::MatrixXd F = models_[i]->getF(dt);
    Eigen::MatrixXd Q = models_[i]->getQ(dt);
    int state_dim = models_[i]->getStateDim();

    covariances_[i] = F.block(0, 0, state_dim, state_dim) *
                      mixed_covs_[i] *
                      F.block(0, 0, state_dim, state_dim).transpose();
    covariances_[i] += Q;

    // Ensure symmetry
    covariances_[i] = (covariances_[i] + covariances_[i].transpose()) * 0.5;
  }
}

void IMMKFFilter::update(const Eigen::Vector3d& measurement) {
  for (size_t i = 0; i < models_.size(); ++i) {
    Eigen::MatrixXd H = models_[i]->getH();
    Eigen::MatrixXd R = models_[i]->getR();
    int state_dim = models_[i]->getStateDim();

    // Innovation: y = z - H*x
    Eigen::Vector3d y = measurement - H.block(0, 0, 3, state_dim) * states_[i];

    // Innovation covariance: S = H*P*H^T + R
    Eigen::MatrixXd S = H.block(0, 0, 3, state_dim) * covariances_[i] *
                        H.block(0, 0, 3, state_dim).transpose();
    S += R;

    // Kalman gain: K = P*H^T*S^{-1}
    Eigen::MatrixXd K = covariances_[i] * H.block(0, 0, state_dim, 3).transpose() *
                        S.inverse();

    // Update state: x = x + K*y
    states_[i] += K * y;

    // Update covariance: P = (I - K*H)*P
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    covariances_[i] = (I - K * H.block(0, 0, 3, state_dim)) * covariances_[i];

    // Ensure symmetry
    covariances_[i] = (covariances_[i] + covariances_[i].transpose()) * 0.5;
  }

  updateModelProbabilities(measurement);
  fuseEstimates();
}

std::vector<double> IMMKFFilter::getModelProbabilities() const {
  return model_probs_;
}

int IMMKFFilter::getMostLikelyModelIndex() const {
  int best_idx = 0;
  double best_prob = model_probs_[0];
  for (size_t i = 1; i < model_probs_.size(); ++i) {
    if (model_probs_[i] > best_prob) {
      best_prob = model_probs_[i];
      best_idx = i;
    }
  }
  return best_idx;
}

Eigen::VectorXd IMMKFFilter::getModelState(int model_idx) const {
  return states_[model_idx];
}

Eigen::MatrixXd IMMKFFilter::getModelCovariance(int model_idx) const {
  return covariances_[model_idx];
}

Eigen::VectorXd IMMKFFilter::getMixedState() const {
  int max_dim = models_[0]->getStateDim();
  for (size_t i = 1; i < models_.size(); ++i) {
    max_dim = std::max(max_dim, models_[i]->getStateDim());
  }

  Eigen::VectorXd mixed = Eigen::VectorXd::Zero(max_dim);
  for (size_t i = 0; i < models_.size(); ++i) {
    int state_dim = models_[i]->getStateDim();
    mixed.head(state_dim) += model_probs_[i] * states_[i];
  }
  return mixed;
}

Eigen::MatrixXd IMMKFFilter::getMixedCovariance() const {
  Eigen::VectorXd mixed_state = getMixedState();
  int max_dim = models_[0]->getStateDim();
  for (size_t i = 1; i < models_.size(); ++i) {
    max_dim = std::max(max_dim, models_[i]->getStateDim());
  }

  Eigen::MatrixXd mixed_cov = Eigen::MatrixXd::Zero(max_dim, max_dim);
  for (size_t i = 0; i < models_.size(); ++i) {
    int state_dim = models_[i]->getStateDim();
    Eigen::VectorXd state_diff = mixed_state.head(state_dim) - states_[i];
    mixed_cov.block(0, 0, state_dim, state_dim) +=
        model_probs_[i] * (covariances_[i] + state_diff * state_diff.transpose());
  }
  return mixed_cov;
}

void IMMKFFilter::modelMixing() {
  int n_models = models_.size();

  // Compute normalization constants first
  std::vector<double> c_bar(n_models, 0.0);
  for (int j = 0; j < n_models; ++j) {
    for (int i = 0; i < n_models; ++i) {
      c_bar[j] += mode_transitions_(i, j) * model_probs_[j];
    }
  }

  // Compute mixing probabilities
  std::vector<std::vector<double>> mu(n_models,
                                       std::vector<double>(n_models));
  for (int j = 0; j < n_models; ++j) {
    for (int i = 0; i < n_models; ++i) {
      if (c_bar[j] > 1e-9) {
        mu[i][j] = mode_transitions_(i, j) * model_probs_[j] / c_bar[j];
      } else {
        mu[i][j] = 0.0;
      }
    }
  }

  // Initialize mixed states and covariances
  for (int i = 0; i < n_models; ++i) {
    int state_dim = models_[i]->getStateDim();
    mixed_states_[i] = Eigen::VectorXd::Zero(state_dim);
    mixed_covs_[i] = Eigen::MatrixXd::Zero(state_dim, state_dim);

    for (int j = 0; j < n_models; ++j) {
      // Check dimension compatibility for mixing
      if (models_[j]->getStateDim() == state_dim && mu[i][j] > 1e-9) {
        mixed_states_[i] += mu[i][j] * states_[j];
        mixed_covs_[i] += mu[i][j] * covariances_[j];
      }
    }
  }
}

void IMMKFFilter::updateModelProbabilities(const Eigen::Vector3d& measurement) {
  int n_models = models_.size();
  std::vector<double> likelihoods(n_models);
  double max_likelihood = 0.0;

  for (int i = 0; i < n_models; ++i) {
    Eigen::MatrixXd H = models_[i]->getH();
    Eigen::MatrixXd R = models_[i]->getR();
    int state_dim = models_[i]->getStateDim();

    // Innovation: y = z - H*x
    Eigen::Vector3d y = measurement - H.block(0, 0, 3, state_dim) * states_[i];

    // Innovation covariance: S = H*P*H^T + R
    Eigen::MatrixXd S = H.block(0, 0, 3, state_dim) * covariances_[i] *
                        H.block(0, 0, 3, state_dim).transpose();
    S += R;

    // Likelihood: L = exp(-0.5*y^T*S^{-1}*y) / sqrt(|S|)
    double det_S = S.determinant();
    if (det_S > 1e-10) {
      double mahal_dist = y.transpose() * S.inverse() * y;
      likelihoods[i] =
          std::exp(-0.5 * mahal_dist) / (std::sqrt(det_S) + 1e-9);
    } else {
      likelihoods[i] = 1e-3;
    }

    max_likelihood = std::max(max_likelihood, likelihoods[i]);
  }

  // Prevent underflow by normalizing with max
  if (max_likelihood < 1e-10) {
    max_likelihood = 1.0;
  }

  // Update model probabilities
  std::vector<double> new_probs(n_models);
  double prob_sum = 0.0;

  for (int i = 0; i < n_models; ++i) {
    new_probs[i] = (likelihoods[i] / max_likelihood) * model_probs_[i];
    prob_sum += new_probs[i];
  }

  // Normalize
  for (int i = 0; i < n_models; ++i) {
    model_probs_[i] = new_probs[i] / (prob_sum + 1e-9);
  }
}

void IMMKFFilter::fuseEstimates() {
  Eigen::VectorXd mixed_state = getMixedState();
  // Just store the fused state - it's already computed in models
}

}  // namespace immkf_predictor
