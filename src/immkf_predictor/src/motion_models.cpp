#include "immkf_predictor/motion_models.h"

#include <cmath>

namespace immkf_predictor {

// ============================================================================
// ConstantVelocityModel
// ============================================================================

ConstantVelocityModel::ConstantVelocityModel(double q_pos, double q_vel,
                                             double r_pos)
    : q_pos_(q_pos), q_vel_(q_vel), r_pos_(r_pos) {
  H_ = Eigen::MatrixXd::Zero(3, 6);
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  H_(2, 2) = 1.0;

  R_ = r_pos * Eigen::MatrixXd::Identity(3, 3);
}

Eigen::VectorXd ConstantVelocityModel::predict(const Eigen::VectorXd& state,
                                                double dt) {
  Eigen::VectorXd predicted = state;
  // x += vx * dt
  predicted(0) += state(3) * dt;
  // y += vy * dt
  predicted(1) += state(4) * dt;
  // z += vz * dt
  predicted(2) += state(5) * dt;
  // velocity constant
  return predicted;
}

Eigen::MatrixXd ConstantVelocityModel::getF(double dt) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 3) = dt;  // x += vx * dt
  F(1, 4) = dt;  // y += vy * dt
  F(2, 5) = dt;  // z += vz * dt
  return F;
}

Eigen::MatrixXd ConstantVelocityModel::getQ(double dt) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);

  // Position noise: integrating over dt
  double dt2 = dt * dt;
  Q(0, 0) = q_pos_ * dt2;
  Q(1, 1) = q_pos_ * dt2;
  Q(2, 2) = q_pos_ * dt2;

  // Velocity noise
  Q(3, 3) = q_vel_;
  Q(4, 4) = q_vel_;
  Q(5, 5) = q_vel_;

  return Q;
}

Eigen::MatrixXd ConstantVelocityModel::getH() { return H_; }

Eigen::MatrixXd ConstantVelocityModel::getR() { return R_; }

Eigen::Vector3d ConstantVelocityModel::extractPosition(
    const Eigen::VectorXd& state) const {
  return state.head(3);
}

std::unique_ptr<MotionModel> ConstantVelocityModel::clone() const {
  return std::make_unique<ConstantVelocityModel>(q_pos_, q_vel_, r_pos_);
}

// ============================================================================
// ConstantAccelerationModel
// ============================================================================

ConstantAccelerationModel::ConstantAccelerationModel(
    double q_pos, double q_vel, double q_acc, double r_pos)
    : q_pos_(q_pos), q_vel_(q_vel), q_acc_(q_acc), r_pos_(r_pos) {
  H_ = Eigen::MatrixXd::Zero(3, 9);
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  H_(2, 2) = 1.0;

  R_ = r_pos * Eigen::MatrixXd::Identity(3, 3);
}

Eigen::VectorXd ConstantAccelerationModel::predict(
    const Eigen::VectorXd& state, double dt) {
  Eigen::VectorXd predicted(9);
  // x += vx*dt + 0.5*ax*dt^2
  predicted(0) = state(0) + state(3) * dt + 0.5 * state(6) * dt * dt;
  // y += vy*dt + 0.5*ay*dt^2
  predicted(1) = state(1) + state(4) * dt + 0.5 * state(7) * dt * dt;
  // z += vz*dt + 0.5*az*dt^2
  predicted(2) = state(2) + state(5) * dt + 0.5 * state(8) * dt * dt;
  // vx += ax*dt
  predicted(3) = state(3) + state(6) * dt;
  // vy += ay*dt
  predicted(4) = state(4) + state(7) * dt;
  // vz += az*dt
  predicted(5) = state(5) + state(8) * dt;
  // acceleration constant
  predicted(6) = state(6);
  predicted(7) = state(7);
  predicted(8) = state(8);
  return predicted;
}

Eigen::MatrixXd ConstantAccelerationModel::getF(double dt) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
  double dt2 = dt * dt;
  double dt2_2 = 0.5 * dt2;

  // Position terms
  F(0, 3) = dt;          // x += vx*dt
  F(0, 6) = dt2_2;       // x += 0.5*ax*dt^2
  F(1, 4) = dt;          // y += vy*dt
  F(1, 7) = dt2_2;       // y += 0.5*ay*dt^2
  F(2, 5) = dt;          // z += vz*dt
  F(2, 8) = dt2_2;       // z += 0.5*az*dt^2

  // Velocity terms
  F(3, 6) = dt;  // vx += ax*dt
  F(4, 7) = dt;  // vy += ay*dt
  F(5, 8) = dt;  // vz += az*dt

  return F;
}

Eigen::MatrixXd ConstantAccelerationModel::getQ(double dt) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);

  double dt2 = dt * dt;
  double dt3_6 = dt2 * dt / 6.0;
  double dt2_2 = dt2 / 2.0;

  // Position noise terms (affected by acceleration)
  Q(0, 0) = q_acc_ * dt2 * dt2_2;
  Q(1, 1) = q_acc_ * dt2 * dt2_2;
  Q(2, 2) = q_acc_ * dt2 * dt2_2;

  // Velocity noise terms (affected by acceleration)
  Q(3, 3) = q_acc_ * dt2;
  Q(4, 4) = q_acc_ * dt2;
  Q(5, 5) = q_acc_ * dt2;

  // Acceleration noise (direct)
  Q(6, 6) = q_acc_;
  Q(7, 7) = q_acc_;
  Q(8, 8) = q_acc_;

  return Q;
}

Eigen::MatrixXd ConstantAccelerationModel::getH() { return H_; }

Eigen::MatrixXd ConstantAccelerationModel::getR() { return R_; }

Eigen::Vector3d ConstantAccelerationModel::extractPosition(
    const Eigen::VectorXd& state) const {
  return state.head(3);
}

std::unique_ptr<MotionModel> ConstantAccelerationModel::clone() const {
  return std::make_unique<ConstantAccelerationModel>(q_pos_, q_vel_, q_acc_,
                                                     r_pos_);
}

// ============================================================================
// SingerModel
// ============================================================================

SingerModel::SingerModel(double q_pos, double q_vel, double q_acc,
                         double decay_rate, double r_pos)
    : q_pos_(q_pos),
      q_vel_(q_vel),
      q_acc_(q_acc),
      r_pos_(r_pos),
      alpha_(decay_rate) {
  H_ = Eigen::MatrixXd::Zero(3, 9);
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  H_(2, 2) = 1.0;

  R_ = r_pos * Eigen::MatrixXd::Identity(3, 3);
}

Eigen::VectorXd SingerModel::predict(const Eigen::VectorXd& state,
                                      double dt) {
  Eigen::VectorXd predicted(9);

  // Position: x = x + vx*dt + ax*(1-e^(-alpha*dt))/alpha * dt
  double exp_alpha_dt = std::exp(-alpha_ * dt);
  double a_factor = (1.0 - exp_alpha_dt) / alpha_;

  predicted(0) = state(0) + state(3) * dt + state(6) * a_factor * dt;
  predicted(1) = state(1) + state(4) * dt + state(7) * a_factor * dt;
  predicted(2) = state(2) + state(5) * dt + state(8) * a_factor * dt;

  // Velocity: v = v + a*(1 - e^(-alpha*dt))
  double v_factor = 1.0 - exp_alpha_dt;
  predicted(3) = state(3) + state(6) * v_factor;
  predicted(4) = state(4) + state(7) * v_factor;
  predicted(5) = state(5) + state(8) * v_factor;

  // Acceleration decays: a = a * e^(-alpha*dt)
  predicted(6) = state(6) * exp_alpha_dt;
  predicted(7) = state(7) * exp_alpha_dt;
  predicted(8) = state(8) * exp_alpha_dt;

  return predicted;
}

Eigen::MatrixXd SingerModel::getF(double dt) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);

  double exp_alpha_dt = std::exp(-alpha_ * dt);
  double a_factor = (1.0 - exp_alpha_dt) / alpha_;
  double v_factor = 1.0 - exp_alpha_dt;

  // Position terms
  F(0, 3) = dt;              // x += vx*dt
  F(0, 6) = a_factor * dt;   // x += ax*a_factor*dt
  F(1, 4) = dt;              // y += vy*dt
  F(1, 7) = a_factor * dt;   // y += ay*a_factor*dt
  F(2, 5) = dt;              // z += vz*dt
  F(2, 8) = a_factor * dt;   // z += az*a_factor*dt

  // Velocity terms
  F(3, 6) = v_factor;  // vx += ax*v_factor
  F(4, 7) = v_factor;  // vy += ay*v_factor
  F(5, 8) = v_factor;  // vz += az*v_factor

  // Acceleration terms (exponential decay)
  F(6, 6) = exp_alpha_dt;
  F(7, 7) = exp_alpha_dt;
  F(8, 8) = exp_alpha_dt;

  return F;
}

Eigen::MatrixXd SingerModel::getQ(double dt) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);

  double exp_alpha_dt = std::exp(-alpha_ * dt);
  double exp_2alpha_dt = exp_alpha_dt * exp_alpha_dt;

  // Singer model process noise (from literature)
  // Q_aa term
  double q_aa = 2.0 * q_acc_ * alpha_ *
                (1.0 - exp_2alpha_dt - 2.0 * alpha_ * dt * exp_alpha_dt) /
                (alpha_ * alpha_);

  // Q_av term
  double q_av = q_acc_ * (1.0 - exp_2alpha_dt) / (alpha_ * alpha_) -
                2.0 * q_acc_ * dt * exp_alpha_dt / alpha_;

  // Q_vv term
  double q_vv = q_acc_ * (1.0 - exp_2alpha_dt) / (2.0 * alpha_);

  // Q_xx term
  double q_xx =
      q_acc_ * dt / (2.0 * alpha_) -
      q_acc_ * (4.0 * exp_alpha_dt - 3.0 - exp_2alpha_dt) / (4.0 * alpha_ * alpha_);

  // Q_xv term
  double q_xv = q_acc_ * (1.0 - exp_2alpha_dt) / (2.0 * alpha_ * alpha_) -
                q_acc_ * dt * exp_alpha_dt / alpha_;

  Q(0, 0) = q_xx;
  Q(1, 1) = q_xx;
  Q(2, 2) = q_xx;

  Q(3, 3) = q_vv;
  Q(4, 4) = q_vv;
  Q(5, 5) = q_vv;

  Q(6, 6) = q_aa;
  Q(7, 7) = q_aa;
  Q(8, 8) = q_aa;

  Q(0, 3) = Q(3, 0) = q_xv;
  Q(1, 4) = Q(4, 1) = q_xv;
  Q(2, 5) = Q(5, 2) = q_xv;

  Q(3, 6) = Q(6, 3) = q_av;
  Q(4, 7) = Q(7, 4) = q_av;
  Q(5, 8) = Q(8, 5) = q_av;

  return Q;
}

Eigen::MatrixXd SingerModel::getH() { return H_; }

Eigen::MatrixXd SingerModel::getR() { return R_; }

Eigen::Vector3d SingerModel::extractPosition(const Eigen::VectorXd& state) const {
  return state.head(3);
}

void SingerModel::setDecayRate(double alpha) { alpha_ = alpha; }

std::unique_ptr<MotionModel> SingerModel::clone() const {
  return std::make_unique<SingerModel>(q_pos_, q_vel_, q_acc_, alpha_, r_pos_);
}

}  // namespace immkf_predictor
