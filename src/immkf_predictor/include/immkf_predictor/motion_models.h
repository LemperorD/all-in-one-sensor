#ifndef IMMKF_PREDICTOR_MOTION_MODELS_H_
#define IMMKF_PREDICTOR_MOTION_MODELS_H_

#include <Eigen/Dense>
#include <memory>

namespace immkf_predictor {

/**
 * Abstract base class for motion models
 */
class MotionModel {
 public:
  virtual ~MotionModel() = default;

  /**
   * Get state dimension
   */
  virtual int getStateDim() const = 0;

  /**
   * Get measurement dimension (always 3 for position)
   */
  virtual int getMeasurementDim() const { return 3; }

  /**
   * Predict state forward in time
   */
  virtual Eigen::VectorXd predict(const Eigen::VectorXd& state, double dt) = 0;

  /**
   * Get state transition matrix F for time step dt
   */
  virtual Eigen::MatrixXd getF(double dt) = 0;

  /**
   * Get process noise covariance Q
   */
  virtual Eigen::MatrixXd getQ(double dt) = 0;

  /**
   * Get measurement matrix H (maps state to measurement)
   */
  virtual Eigen::MatrixXd getH() = 0;

  /**
   * Get measurement noise covariance R
   */
  virtual Eigen::MatrixXd getR() = 0;

  /**
   * Extract position from state [x, y, z]
   */
  virtual Eigen::Vector3d extractPosition(const Eigen::VectorXd& state) const = 0;

  /**
   * Clone this model
   */
  virtual std::unique_ptr<MotionModel> clone() const = 0;
};

/**
 * Constant Velocity Model
 * State: [x, y, z, vx, vy, vz]^T (6D)
 * Dynamics: x_{k+1} = F*x_k + w_k
 */
class ConstantVelocityModel : public MotionModel {
 public:
  ConstantVelocityModel(double q_pos = 0.01, double q_vel = 0.01,
                        double r_pos = 0.2);
  ~ConstantVelocityModel() override = default;

  int getStateDim() const override { return 6; }

  Eigen::VectorXd predict(const Eigen::VectorXd& state, double dt) override;
  Eigen::MatrixXd getF(double dt) override;
  Eigen::MatrixXd getQ(double dt) override;
  Eigen::MatrixXd getH() override;
  Eigen::MatrixXd getR() override;
  Eigen::Vector3d extractPosition(const Eigen::VectorXd& state) const override;
  std::unique_ptr<MotionModel> clone() const override;

 private:
  double q_pos_, q_vel_;  // Process noise variances
  double r_pos_;           // Measurement noise variance
  Eigen::MatrixXd H_;      // Cached measurement matrix
  Eigen::MatrixXd R_;      // Cached measurement covariance
};

/**
 * Constant Acceleration Model
 * State: [x, y, z, vx, vy, vz, ax, ay, az]^T (9D)
 * Dynamics: x_{k+1} = F*x_k + w_k
 */
class ConstantAccelerationModel : public MotionModel {
 public:
  ConstantAccelerationModel(double q_pos = 0.01, double q_vel = 0.1,
                            double q_acc = 0.01, double r_pos = 0.2);
  ~ConstantAccelerationModel() override = default;

  int getStateDim() const override { return 9; }

  Eigen::VectorXd predict(const Eigen::VectorXd& state, double dt) override;
  Eigen::MatrixXd getF(double dt) override;
  Eigen::MatrixXd getQ(double dt) override;
  Eigen::MatrixXd getH() override;
  Eigen::MatrixXd getR() override;
  Eigen::Vector3d extractPosition(const Eigen::VectorXd& state) const override;
  std::unique_ptr<MotionModel> clone() const override;

 private:
  double q_pos_, q_vel_, q_acc_;  // Process noise variances
  double r_pos_;                   // Measurement noise variance
  Eigen::MatrixXd H_;              // Cached measurement matrix
  Eigen::MatrixXd R_;              // Cached measurement covariance
};

/**
 * Singer Model (Random Acceleration Model)
 * State: [x, y, z, vx, vy, vz, ax, ay, az]^T (9D)
 * Acceleration decays exponentially: a_k = -alpha*a_{k-1} + w_k
 * This is more realistic for maneuvering targets
 */
class SingerModel : public MotionModel {
 public:
  SingerModel(double q_pos = 0.01, double q_vel = 0.1, double q_acc = 0.01,
              double decay_rate = 0.95, double r_pos = 0.2);
  ~SingerModel() override = default;

  int getStateDim() const override { return 9; }

  Eigen::VectorXd predict(const Eigen::VectorXd& state, double dt) override;
  Eigen::MatrixXd getF(double dt) override;
  Eigen::MatrixXd getQ(double dt) override;
  Eigen::MatrixXd getH() override;
  Eigen::MatrixXd getR() override;
  Eigen::Vector3d extractPosition(const Eigen::VectorXd& state) const override;
  std::unique_ptr<MotionModel> clone() const override;

  /**
   * Set the maneuver decay rate (alpha)
   * Higher values mean acceleration decays slower
   */
  void setDecayRate(double alpha);

 private:
  double q_pos_, q_vel_, q_acc_;  // Process noise variances
  double r_pos_;                   // Measurement noise variance
  double alpha_;                   // Maneuver decay rate (0 < alpha <= 1)
  Eigen::MatrixXd H_;              // Cached measurement matrix
  Eigen::MatrixXd R_;              // Cached measurement covariance
};

}  // namespace immkf_predictor

#endif  // IMMKF_PREDICTOR_MOTION_MODELS_H_
